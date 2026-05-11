import sys
import struct
import time
import queue
import datetime
import numpy as np
from scipy.optimize import curve_fit
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QLineEdit, QPushButton, 
                             QGroupBox, QFormLayout, QMessageBox, QScrollArea, QFileDialog)
from PyQt5.QtCore import QThread, pyqtSignal, Qt
import pyqtgraph as pg
import serial

# ==========================================
# CONSTANTS & ISOLATED REGISTER MAPPING
# ==========================================
MODBUS_SLAVE_ID = 21

REG_STEP_CMD   = 10  
REG_TARGET_POS = 11
REG_KP         = 12
REG_KI         = 13
REG_KD         = 14
REG_APPLY      = 15  

# Advanced Trajectory Registers
REG_TRAJ_TYPE  = 16  # 0=Trapz, 1=S-Curve, 2=Direct
REG_V_MAX      = 17  # rad/s * 100
REG_A_MAX      = 18  # rad/s^2 * 100
REG_J_MAX      = 19  # rad/s^3 * 100

def crc16(data: bytes) -> bytes:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return struct.pack('<H', crc)

# ==========================================
# UNIFIED SERIAL THREAD
# ==========================================
class SerialThread(QThread):
    data_received = pyqtSignal(float, float, float)
    error_signal = pyqtSignal(str)

    def __init__(self, port, baudrate=115200):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.serial_port = None
        self.is_running = False
        self.tx_queue = queue.Queue()

    def send_register(self, addr, val):
        self.tx_queue.put((addr, val))

    def run(self):
        try:
            self.serial_port = serial.Serial(self.port, self.baudrate, timeout=0.01)
            self.is_running = True
            
            buffer = bytearray()
            while self.is_running:
                if self.serial_port.in_waiting > 0:
                    buffer.extend(self.serial_port.read(self.serial_port.in_waiting))
                    while len(buffer) >= 16:
                        if buffer[0] == 0x7E and buffer[1] == 0x7E:
                            if buffer[14] == 0x03 and buffer[15] == 0x03:
                                payload = buffer[2:14]
                                pos, vel, acc = struct.unpack('<fff', payload)
                                self.data_received.emit(pos, vel, acc)
                                buffer = buffer[16:] 
                            else:
                                buffer.pop(0)
                        else:
                            buffer.pop(0)
                
                while not self.tx_queue.empty():
                    addr, val = self.tx_queue.get()
                    payload = struct.pack('>BBHH', MODBUS_SLAVE_ID, 6, addr, int(val) & 0xFFFF)
                    payload += crc16(payload)
                    self.serial_port.write(payload)
                    time.sleep(0.02) # Give STM32 20ms to process each Modbus command safely
                    
                time.sleep(0.001)
        except Exception as e:
            self.error_signal.emit(str(e))

    def stop(self):
        self.is_running = False
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()

# ==========================================
# ROBUST EVOLUTIONARY TUNER THREAD
# ==========================================
class SmartTunerThread(QThread):
    log_signal = pyqtSignal(str)
    gains_signal = pyqtSignal(float, float, float)
    finished_signal = pyqtSignal()
    
    def __init__(self, main_window):
        super().__init__()
        self.mw = main_window 
        self.is_running = True
        
    def write_reg(self, addr, val):
        if self.mw.serial_thread and self.mw.serial_thread.is_running:
            self.mw.serial_thread.send_register(addr, val)

    def set_parameters(self, kp, ki, kd, target_rad):
        # STM32 C Code divides these by 100.0
        self.write_reg(REG_KP, min(max(int(kp * 100), 0), 65535))
        self.write_reg(REG_KI, min(max(int(ki * 100), 0), 65535))
        self.write_reg(REG_KD, min(max(int(kd * 100), 0), 65535))
        
        # Hardcode Trajectory to S-Curve Fast (Very smooth for rods)
        self.write_reg(REG_TRAJ_TYPE, 1) # S-Curve
        self.write_reg(REG_V_MAX, int(6.28 * 100))
        self.write_reg(REG_A_MAX, int(6.28 * 100))
        self.write_reg(REG_J_MAX, int(12.56 * 100))
        
        self.write_reg(REG_TARGET_POS, int(target_rad * 1000) & 0xFFFF)
        self.write_reg(REG_APPLY, 1)

    def measure_performance(self, target_rad, timeout=3.5):
        time.sleep(timeout)
            
        self.mw.is_tuning = False
        time.sleep(0.2) 
        
        p = np.array(self.mw.tune_pos_data)
        v = np.array(self.mw.tune_vel_data)
        
        if len(p) < 20: 
            return None
        
        start_pos = p[0]
        ss_pos = np.mean(p[-15:])
        ss_error = abs(target_rad - ss_pos)
        
        if target_rad >= start_pos:
            overshoot = max(np.max(p) - target_rad, 0.0)
        else:
            overshoot = max(target_rad - np.min(p), 0.0)
            
        dist = abs(target_rad - start_pos)
        overshoot_pct = (overshoot / dist) * 100.0 if dist > 0.05 else 0.0
        
        # Vibration Analysis
        v_abs = np.abs(v)
        peak_idx = np.argmax(v_abs)
        v_tail = v[peak_idx:]
        
        stb_score = np.sum(np.abs(np.diff(v_tail))) / (np.max(v_abs) + 0.001)
        
        return {
            'ss_error': ss_error,
            'overshoot': overshoot_pct,
            'stability': stb_score,
        }

    def run(self):
        if not self.mw.serial_thread or not self.mw.serial_thread.is_running:
            self.log_signal.emit("❌ Port not connected.")
            self.finished_signal.emit()
            return
            
        try:
            self.log_signal.emit("\n" + "="*50)
            self.log_signal.emit(f"🔬 EXPERT ROD TUNER INITIATED")
            self.log_signal.emit(f"Time: {datetime.datetime.now().strftime('%H:%M:%S')}")
            self.log_signal.emit("="*50 + "\n")

            # --- 1. HOMING ---
            self.log_signal.emit("🤖 [1/5] Safe Homing...")
            self.set_parameters(5.0, 0.0, 0.0, 0.0) # Move to 0
            time.sleep(3.0)
            
            # --- 2. SYSTEM ID ---
            self.log_signal.emit("\n🤖 [2/5] Physics Modeling (Fast Auto-Detect)...")
            
            test_pwm = 15.0 # Start safely at 15%
            step_success = False
            
            for attempt in range(6): # Try up to 40%
                self.log_signal.emit(f"   ↳ Testing PWM at {test_pwm}%...")
                
                self.mw.tune_time_data, self.mw.tune_pos_data, self.mw.tune_vel_data = [], [], []
                self.mw.is_tuning = True
                
                self.write_reg(REG_STEP_CMD, int(test_pwm * 100))
                time.sleep(0.3) # Very short burst to check friction
                self.write_reg(REG_STEP_CMD, 0)
                self.mw.is_tuning = False
                time.sleep(0.1)
                
                p = np.array(self.mw.tune_pos_data)
                v = np.array(self.mw.tune_vel_data)
                
                # Extremely sensitive movement check (0.05 rad = ~3 degrees)
                if len(p) > 5 and (np.max(np.abs(v)) > 0.1 or abs(p[-1] - p[0]) > 0.05):
                    step_success = True
                    self.log_signal.emit(f"   ✅ Movement confirmed at {test_pwm}%!")
                    
                    self.log_signal.emit(f"   ↳ Gathering math data...")
                    self.set_parameters(5.0, 0.0, 0.0, 0.0) # Reset home
                    time.sleep(1.0)
                    
                    self.mw.tune_time_data, self.mw.tune_pos_data, self.mw.tune_vel_data = [], [], []
                    self.mw.is_tuning = True
                    
                    self.write_reg(REG_STEP_CMD, int(test_pwm * 100))
                    time.sleep(0.8) # Full step for calculation
                    self.write_reg(REG_STEP_CMD, 0)
                    self.mw.is_tuning = False
                    time.sleep(0.5)
                    break
                else:
                    self.log_signal.emit(f"   ⚠️ Too weak. Increasing power...")
                    test_pwm += 5.0
                    
            if not step_success:
                raise Exception("Motor failed to move! Check E-STOP or Hardware.")
                
            pwm_input_pct = test_pwm
            t = np.array(self.mw.tune_time_data)
            y = np.array(self.mw.tune_vel_data)
            
            if len(t) < 10: raise Exception("No Telemetry! Is the robot turned on?")
            y = y - y[0] 
            
            def step_response(t, K_sys, tau):
                # K_sys output is in rad/s per 100% PWM
                return K_sys * (pwm_input_pct / 100.0) * (1 - np.exp(-t / max(tau, 0.001)))
            
            popt, _ = curve_fit(step_response, t, y, bounds=(0, np.inf))
            K_sys, tau = popt
            
            if tau > 10.0 or K_sys < 0.005: 
                raise Exception("Motor didn't move! Increase PWM Power or check E-STOP.")
            
            self.log_signal.emit(f"   ↳ System DC Gain = {K_sys:.4f}")
            self.log_signal.emit(f"   ↳ Time Constant  = {tau:.4f} s")
            
            # THE MATHEMATICAL FIX: K_sys maps Velocity to 0.0-1.0 PWM
            # STM32 C Code expects PID output in 0.0-100.0 PWM
            # Therefore we MUST multiply Kp by 100!
            Tc_base = tau
            Kp_base = (tau / (K_sys * Tc_base)) * 100.0
            Ki_base = Kp_base / min(tau, 4 * Tc_base)
            
            # Sanity Check Clamps
            Kp_base = min(max(Kp_base, 1.0), 100.0)
            
            self.log_signal.emit(f"   ↳ Base Kp calculated: {Kp_base:.2f}")

            # --- 3. P-GAIN SWEEP ---
            self.log_signal.emit("\n🤖 [3/5] Iterative P-Gain Sweep (Finding rigid stability)...")
            
            kp_multipliers = [0.6, 1.0, 1.5]
            best_kp_score = float('inf')
            best_kp = Kp_base
            
            for mult in kp_multipliers:
                test_kp = Kp_base * mult
                self.log_signal.emit(f"\n--- Testing Kp = {test_kp:.2f} ({mult}x) ---")
                
                avg_err = 0; avg_os = 0; avg_stb = 0
                
                # Forward
                self.mw.tune_time_data, self.mw.tune_pos_data, self.mw.tune_vel_data = [], [], []
                self.mw.is_tuning = True
                self.set_parameters(test_kp, 0.0, 0.0, 1.57)
                res1 = self.measure_performance(1.57)
                
                # Reverse
                self.mw.tune_time_data, self.mw.tune_pos_data, self.mw.tune_vel_data = [], [], []
                self.mw.is_tuning = True
                self.set_parameters(test_kp, 0.0, 0.0, 0.0)
                res2 = self.measure_performance(0.0)
                
                if not res1 or not res2: continue
                
                avg_err = (res1['ss_error'] + res2['ss_error']) / 2.0
                avg_os  = (res1['overshoot'] + res2['overshoot']) / 2.0
                avg_stb = (res1['stability'] + res2['stability']) / 2.0
                
                self.log_signal.emit(f"   ↳ Avg Error: {avg_err:.3f} rad")
                self.log_signal.emit(f"   ↳ Avg Overshoot: {avg_os:.1f} %")
                self.log_signal.emit(f"   ↳ Avg Vibration: {avg_stb:.1f}")
                
                # High penalty for Error since I is 0, but vibration is worst
                score = (avg_err * 200) + (avg_os * 100) + (avg_stb * 300)
                self.log_signal.emit(f"   ★ P-Score: {score:.1f} (Lower=Better)")
                
                if score < best_kp_score:
                    best_kp_score = score
                    best_kp = test_kp
            
            self.log_signal.emit(f"\n✅ Locked Optimal P-Gain: {best_kp:.2f}")

            # --- 4. I-GAIN SWEEP ---
            self.log_signal.emit("\n🤖 [4/5] Iterative I-Gain Sweep (Removing error)...")
            
            ki_multipliers = [0.0, 0.25, 0.5, 1.0]
            best_ki_score = float('inf')
            best_ki = 0.0
            
            for mult in ki_multipliers:
                test_ki = Ki_base * mult
                if test_ki < 0.01 and mult > 0: test_ki = 0.01 # ensure minimal I
                
                self.log_signal.emit(f"\n--- Testing Ki = {test_ki:.2f} ({mult}x) ---")
                
                # Reverse (to -1.57)
                self.mw.tune_time_data, self.mw.tune_pos_data, self.mw.tune_vel_data = [], [], []
                self.mw.is_tuning = True
                self.set_parameters(best_kp, test_ki, 0.0, -1.57)
                res1 = self.measure_performance(-1.57)
                
                # Home
                self.mw.tune_time_data, self.mw.tune_pos_data, self.mw.tune_vel_data = [], [], []
                self.mw.is_tuning = True
                self.set_parameters(best_kp, test_ki, 0.0, 0.0)
                res2 = self.measure_performance(0.0)
                
                if not res1 or not res2: continue
                
                avg_err = (res1['ss_error'] + res2['ss_error']) / 2.0
                avg_os  = (res1['overshoot'] + res2['overshoot']) / 2.0
                avg_stb = (res1['stability'] + res2['stability']) / 2.0
                
                self.log_signal.emit(f"   ↳ Avg Error: {avg_err:.3f} rad")
                self.log_signal.emit(f"   ↳ Avg Overshoot: {avg_os:.1f} %")
                self.log_signal.emit(f"   ↳ Avg Vibration: {avg_stb:.1f}")
                
                # Here error penalty is HUGE because I-gain should remove it
                score = (avg_err * 2000) + (avg_os * 300) + (avg_stb * 300)
                self.log_signal.emit(f"   ★ I-Score: {score:.1f}")
                
                if avg_os > 3.0:
                    self.log_signal.emit("   ⚠️ REJECTED: High Overshoot.")
                elif score < best_ki_score:
                    best_ki_score = score
                    best_ki = test_ki

            self.log_signal.emit(f"\n✅ Locked Optimal I-Gain: {best_ki:.2f}")

            # --- 5. FINALIZE ---
            self.log_signal.emit("\n" + "="*50)
            self.log_signal.emit(f"🏆 TUNING COMPLETION REPORT")
            self.log_signal.emit(f"   Final Kp = {best_kp:.3f}")
            self.log_signal.emit(f"   Final Ki = {best_ki:.3f}")
            self.log_signal.emit("="*50 + "\n")
            
            self.gains_signal.emit(best_kp, best_ki, 0.0)
            
            # Park at home smoothly with final gains
            self.set_parameters(best_kp, best_ki, 0.0, 0.0)
            
        except Exception as e:
            self.log_signal.emit(f"❌ Tuner Crash: {e}")
            
        self.finished_signal.emit()

# ==========================================
# MAIN GUI APPLICATION
# ==========================================
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("1-DOF STM32: The Expert AI Tuner (Rod Pick & Place)")
        self.resize(1200, 750)

        self.serial_thread = None
        self.tuner_thread = None
        
        self.is_tuning = False
        self.tune_start_time = 0
        self.tune_time_data = []
        self.tune_pos_data = []
        self.tune_vel_data = []

        self.setup_ui()

    def setup_ui(self):
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout(main_widget)

        left_panel = QVBoxLayout()
        main_layout.addLayout(left_panel, stretch=3)

        conn_group = QGroupBox("Single-Cable Connection")
        conn_layout = QFormLayout()
        self.port_input = QLineEdit("COM14")
        self.btn_connect = QPushButton("Connect System")
        self.btn_connect.clicked.connect(self.toggle_connection)
        conn_layout.addRow("COM Port (ST-Link):", self.port_input)
        conn_layout.addRow(self.btn_connect)
        conn_group.setLayout(conn_layout)
        left_panel.addWidget(conn_group)

        ctrl_group = QGroupBox("Isolated Manual Control")
        ctrl_layout = QFormLayout()
        self.target_pos_input = QLineEdit("0.0")
        self.kp_input = QLineEdit("20.0")
        self.ki_input = QLineEdit("0.0")
        self.kd_input = QLineEdit("0.0")
        self.btn_send_ctrl = QPushButton("Apply to STM32")
        self.btn_send_ctrl.clicked.connect(self.send_control_settings)
        ctrl_layout.addRow("Target Pos (rad):", self.target_pos_input)
        ctrl_layout.addRow("Kp Vel:", self.kp_input)
        ctrl_layout.addRow("Ki Vel:", self.ki_input)
        ctrl_layout.addRow("Kd Vel:", self.kd_input)
        ctrl_layout.addRow(self.btn_send_ctrl)
        ctrl_group.setLayout(ctrl_layout)
        left_panel.addWidget(ctrl_group)

        tune_group = QGroupBox("Robust Sequential Tuner (P then I)")
        tune_layout = QVBoxLayout()
        
        btn_layout = QHBoxLayout()
        self.btn_tune = QPushButton("▶ RUN EXPERT TUNE")
        self.btn_tune.setStyleSheet("font-weight: bold; padding: 12px; background-color: #8b0000; color: white; font-size: 14px;")
        self.btn_tune.clicked.connect(self.start_autotune)
        
        self.btn_save_log = QPushButton("💾 Save Log")
        self.btn_save_log.setStyleSheet("padding: 12px;")
        self.btn_save_log.clicked.connect(self.save_log)
        
        btn_layout.addWidget(self.btn_tune)
        btn_layout.addWidget(self.btn_save_log)
        
        self.lbl_tune_res = QLabel("Ready.\nThis uses the robust sequential sweep method.\nIt will test P-Gains first, then I-Gains.")
        self.lbl_tune_res.setWordWrap(True)
        self.lbl_tune_res.setAlignment(Qt.AlignTop | Qt.AlignLeft)
        self.lbl_tune_res.setStyleSheet("background-color: #0c0c0c; color: #00ff00; padding: 10px; font-family: Consolas; font-size: 12px;")
        
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setWidget(self.lbl_tune_res)
        scroll_area.setMinimumHeight(350)

        tune_layout.addLayout(btn_layout)
        tune_layout.addWidget(scroll_area)
        tune_group.setLayout(tune_layout)
        left_panel.addWidget(tune_group)

        self.plot_widget = pg.GraphicsLayoutWidget()
        main_layout.addWidget(self.plot_widget, stretch=4)
        
        self.p1 = self.plot_widget.addPlot(title="Position (rad)")
        self.curve_pos = self.p1.plot(pen=pg.mkPen('y', width=2))
        self.plot_widget.nextRow()
        self.p2 = self.plot_widget.addPlot(title="Velocity (rad/s) [Vibration Detection]")
        self.curve_vel = self.p2.plot(pen=pg.mkPen('c', width=2))

        self.plot_data_size = 500
        self.pos_data = np.zeros(self.plot_data_size)
        self.vel_data = np.zeros(self.plot_data_size)

    def toggle_connection(self):
        if self.btn_connect.text() == "Connect System":
            self.serial_thread = SerialThread(self.port_input.text(), baudrate=115200)
            self.serial_thread.data_received.connect(self.update_telemetry)
            self.serial_thread.error_signal.connect(lambda e: QMessageBox.critical(self, "Serial Error", e))
            self.serial_thread.start()
            self.btn_connect.setText("Disconnect")
        else:
            if self.serial_thread:
                self.serial_thread.stop()
                self.serial_thread.wait()
                self.serial_thread = None
            self.btn_connect.setText("Connect System")

    def update_telemetry(self, pos, vel, acc):
        self.pos_data = np.roll(self.pos_data, -1)
        self.vel_data = np.roll(self.vel_data, -1)
        self.pos_data[-1] = pos
        self.vel_data[-1] = vel

        self.curve_pos.setData(self.pos_data)
        self.curve_vel.setData(self.vel_data)

        if self.is_tuning:
            current_t = time.time() - self.tune_start_time
            self.tune_time_data.append(current_t)
            self.tune_pos_data.append(pos) 
            self.tune_vel_data.append(vel)

    def send_control_settings(self):
        if not self.serial_thread or not self.serial_thread.is_running:
            QMessageBox.warning(self, "Warning", "Port not connected.")
            return

        try:
            t_pos = int(float(self.target_pos_input.text()) * 1000)
            kp = int(float(self.kp_input.text()) * 100)
            ki = int(float(self.ki_input.text()) * 100)
            kd = int(float(self.kd_input.text()) * 100)

            # Apply hardcoded fast S-Curve for manual moves too
            self.serial_thread.send_register(REG_TRAJ_TYPE, 1)
            self.serial_thread.send_register(REG_V_MAX, int(6.28 * 100))
            self.serial_thread.send_register(REG_A_MAX, int(6.28 * 100))
            self.serial_thread.send_register(REG_J_MAX, int(12.56 * 100))

            self.serial_thread.send_register(REG_TARGET_POS, t_pos)
            self.serial_thread.send_register(REG_KP, kp)
            self.serial_thread.send_register(REG_KI, ki)
            self.serial_thread.send_register(REG_KD, kd)
            self.serial_thread.send_register(REG_APPLY, 1)
            
        except ValueError:
            QMessageBox.warning(self, "Invalid Input", "Please enter valid numbers.")

    def start_autotune(self):
        if not self.serial_thread or not self.serial_thread.is_running:
            QMessageBox.warning(self, "Warning", "Connect to the System first.")
            return

        self.btn_tune.setEnabled(False)
        self.lbl_tune_res.setText("Starting EXPERT AI Tuner...\n")
        
        self.tuner_thread = SmartTunerThread(self)
        self.tuner_thread.log_signal.connect(self.update_tuner_log)
        self.tuner_thread.gains_signal.connect(self.receive_tuned_gains)
        self.tuner_thread.finished_signal.connect(lambda: self.btn_tune.setEnabled(True))
        self.tuner_thread.start()

    def update_tuner_log(self, text):
        current_text = self.lbl_tune_res.text()
        self.lbl_tune_res.setText(current_text + text + "\n")
        scroll = self.lbl_tune_res.parent().parent().verticalScrollBar()
        scroll.setValue(scroll.maximum())

    def receive_tuned_gains(self, kp, ki, kd):
        self.kp_input.setText(f"{kp:.3f}")
        self.ki_input.setText(f"{ki:.3f}")
        self.kd_input.setText("0.0")

    def save_log(self):
        log_text = self.lbl_tune_res.text()
        if not log_text or log_text.startswith("Ready"):
            QMessageBox.information(self, "Info", "No log to save yet.")
            return
            
        options = QFileDialog.Options()
        filename, _ = QFileDialog.getSaveFileName(self, "Save Tuning Log", "tuning_log.txt", "Text Files (*.txt);;All Files (*)", options=options)
        if filename:
            try:
                with open(filename, 'w', encoding='utf-8') as f:
                    f.write(log_text)
                QMessageBox.information(self, "Success", f"Log saved to {filename}")
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Could not save file:\n{e}")

    def closeEvent(self, event):
        if self.serial_thread:
            self.serial_thread.stop()
            self.serial_thread.wait()
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())