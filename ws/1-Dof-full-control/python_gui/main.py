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

REG_TRAJ_TYPE  = 16  
REG_V_MAX      = 17  
REG_A_MAX      = 18  
REG_J_MAX      = 19  

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
                    time.sleep(0.015) 
                    
                time.sleep(0.001)
        except Exception as e:
            self.error_signal.emit(str(e))

    def stop(self):
        self.is_running = False
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()

# ==========================================
# MASTERPIECE TUNER (MATH + KD DAMPING)
# ==========================================
class MasterpieceTunerThread(QThread):
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
        self.write_reg(REG_KP, min(max(int(kp * 100), 0), 65535))
        self.write_reg(REG_KI, min(max(int(ki * 100), 0), 65535))
        self.write_reg(REG_KD, min(max(int(kd * 100), 0), 65535))
        
        try:
            v_max = int(float(self.mw.vmax_input.text()) * 100)
            a_max = int(float(self.mw.amax_input.text()) * 100)
            j_max = int(float(self.mw.jmax_input.text()) * 100)
        except:
            v_max, a_max, j_max = 628, 628, 1256

        self.write_reg(REG_TRAJ_TYPE, 1) # S-Curve
        self.write_reg(REG_V_MAX, v_max)
        self.write_reg(REG_A_MAX, a_max)
        self.write_reg(REG_J_MAX, j_max)
        
        self.write_reg(REG_TARGET_POS, int(target_rad * 1000) & 0xFFFF)
        self.write_reg(REG_APPLY, 1)

    def measure_performance(self, target_rad, timeout=3.0):
        time.sleep(timeout)
        self.mw.is_tuning = False
        time.sleep(0.1) 
        
        p = np.array(self.mw.tune_pos_data)
        v = np.array(self.mw.tune_vel_data)
        a = np.array(self.mw.tune_acc_data)
        
        if len(p) < 20: 
            return None
        
        start_pos = p[0]
        ss_pos = np.mean(p[-15:])
        ss_error = abs(target_rad - ss_pos)
        
        if target_rad >= start_pos: overshoot = max(np.max(p) - target_rad, 0.0)
        else:                       overshoot = max(target_rad - np.min(p), 0.0)
            
        # การสั่นของความเร่งตอนจบ (ยิ่งน้อยแปลว่าเบรกนิ่ม)
        a_tail = a[-25:]
        vibration = np.std(a_tail)
        
        return {
            'ss_error': ss_error,
            'overshoot': overshoot,
            'vibration': vibration,
        }

    def run(self):
        if not self.mw.serial_thread or not self.mw.serial_thread.is_running:
            self.log_signal.emit("❌ Port not connected.")
            self.finished_signal.emit()
            return
            
        try:
            self.log_signal.emit("\n" + "="*50)
            self.log_signal.emit(f"🏆 MASTERPIECE TUNER INITIATED (Precision Stop)")
            self.log_signal.emit("="*50 + "\n")

            # --- 1. HOMING ---
            self.log_signal.emit("🤖 [1/6] Safe Homing...")
            self.set_parameters(5.0, 0.0, 0.0, 0.0) 
            time.sleep(2.0)
            
            # --- 2. MATH ESTIMATION (FAST SYSTEM ID) ---
            self.log_signal.emit("\n🤖 [2/6] Physics Modeling (Math Estimation)...")
            test_pwm = 20.0 
            
            self.mw.tune_time_data, self.mw.tune_pos_data, self.mw.tune_vel_data = [], [], []
            self.mw.is_tuning = True
            
            self.write_reg(REG_STEP_CMD, int(test_pwm * 100))
            time.sleep(0.8) 
            self.write_reg(REG_STEP_CMD, 0)
            self.mw.is_tuning = False
            time.sleep(0.5)

            t = np.array(self.mw.tune_time_data)
            y = np.array(self.mw.tune_vel_data)
            if len(t) < 10: raise Exception("No Telemetry!")
            y = y - y[0] 
            t = t - t[0]
            
            def step_response(t, K_sys, tau):
                return K_sys * (test_pwm / 100.0) * (1 - np.exp(-t / max(tau, 0.001)))
            
            popt, _ = curve_fit(step_response, t, y, bounds=(0, np.inf))
            K_sys, tau = popt
            
            if tau > 10.0 or K_sys < 0.005: 
                raise Exception("Motor failed to move correctly. Check Power.")
            
            Tc_base = tau
            Kp_base = (tau / (K_sys * Tc_base)) * 100.0
            Ki_base = Kp_base / min(tau, 4 * Tc_base)
            Kp_base = min(max(Kp_base, 5.0), 60.0)
            
            self.log_signal.emit(f"   ↳ Base Kp calculated: {Kp_base:.2f}")
            self.log_signal.emit(f"   ↳ Base Ki calculated: {Ki_base:.2f}")

            # --- 3. P-GAIN SWEEP (SPEED) ---
            self.log_signal.emit("\n🤖 [3/6] P-Gain Sweep (Responsiveness)...")
            kp_multipliers = [0.8, 1.0, 1.2]
            best_kp_score = float('inf')
            best_kp = Kp_base
            
            for mult in kp_multipliers:
                if not self.is_running: break
                test_kp = Kp_base * mult
                self.log_signal.emit(f"   Testing Kp = {test_kp:.1f} ({mult}x)")
                
                self.mw.tune_time_data, self.mw.tune_pos_data, self.mw.tune_vel_data, self.mw.tune_acc_data = [], [], [], []
                self.mw.is_tuning = True
                self.set_parameters(test_kp, 0.0, 0.0, 1.57)
                res = self.measure_performance(1.57)
                
                self.set_parameters(test_kp, 0.0, 0.0, 0.0) # Return home
                time.sleep(2.0)
                
                if not res: continue
                
                # เน้นที่ Error และลดการสั่น
                score = (res['ss_error'] * 500) + (res['vibration'] * 100)
                if score < best_kp_score:
                    best_kp_score = score
                    best_kp = test_kp
            
            self.log_signal.emit(f"   ✅ Locked Optimal P-Gain: {best_kp:.2f}")

            # --- 4. I-GAIN SWEEP (ERROR ELIMINATION) ---
            self.log_signal.emit("\n🤖 [4/6] I-Gain Sweep (Precision Targeting)...")
            ki_multipliers = [0.5, 1.0, 1.5]
            best_ki_score = float('inf')
            best_ki = Ki_base * 0.5
            
            for mult in ki_multipliers:
                if not self.is_running: break
                test_ki = Ki_base * mult
                self.log_signal.emit(f"   Testing Ki = {test_ki:.2f} ({mult}x)")
                
                self.mw.tune_time_data, self.mw.tune_pos_data, self.mw.tune_vel_data, self.mw.tune_acc_data = [], [], [], []
                self.mw.is_tuning = True
                self.set_parameters(best_kp, test_ki, 0.0, 1.57)
                res = self.measure_performance(1.57)
                
                self.set_parameters(best_kp, test_ki, 0.0, 0.0)
                time.sleep(2.0)
                
                if not res: continue
                
                # หักคะแนนหนักมากถ้า Overshoot > 0.015
                if res['overshoot'] > 0.015:
                    self.log_signal.emit("      ⚠️ REJECTED: Caused Overshoot.")
                    continue
                    
                score = res['ss_error']
                if score < best_ki_score:
                    best_ki_score = score
                    best_ki = test_ki

            self.log_signal.emit(f"   ✅ Locked Optimal I-Gain: {best_ki:.2f}")

            # --- 5. D-GAIN SWEEP (THE PERFECT STOP) ---
            self.log_signal.emit("\n🤖 [5/6] D-Gain Sweep (Shock Absorber for Perfect Stop)...")
            # กวาดหา D เพื่อเบรกให้นิ่งที่สุด
            kd_tests = [0.0, 0.1, 0.3, 0.5]
            best_kd_score = float('inf')
            best_kd = 0.0
            
            for test_kd in kd_tests:
                if not self.is_running: break
                self.log_signal.emit(f"   Testing Kd = {test_kd:.2f}")
                
                self.mw.tune_time_data, self.mw.tune_pos_data, self.mw.tune_vel_data, self.mw.tune_acc_data = [], [], [], []
                self.mw.is_tuning = True
                self.set_parameters(best_kp, best_ki, test_kd, 1.57)
                res = self.measure_performance(1.57)
                
                self.set_parameters(best_kp, best_ki, test_kd, 0.0)
                time.sleep(2.0)
                
                if not res: continue
                
                # โฟกัสไปที่ Vibration ตอนจบเพียวๆ
                score = res['vibration'] + (res['overshoot'] * 100)
                self.log_signal.emit(f"      Vibration Score: {res['vibration']:.2f}")
                
                if score < best_kd_score:
                    best_kd_score = score
                    best_kd = test_kd

            self.log_signal.emit(f"   ✅ Locked Optimal D-Gain: {best_kd:.2f}")

            # --- 6. FINALIZE ---
            self.log_signal.emit("\n" + "="*50)
            self.log_signal.emit(f"🏆 MASTERPIECE TUNING COMPLETE")
            self.log_signal.emit(f"   Final Kp = {best_kp:.3f} (Speed)")
            self.log_signal.emit(f"   Final Ki = {best_ki:.3f} (Precision)")
            self.log_signal.emit(f"   Final Kd = {best_kd:.3f} (Perfect Stop)")
            self.log_signal.emit("="*50 + "\n")
            
            self.gains_signal.emit(best_kp, best_ki, best_kd)
            self.set_parameters(best_kp, best_ki, best_kd, 0.0)
            
        except Exception as e:
            self.log_signal.emit(f"❌ Tuner Crash: {e}")
            
        self.finished_signal.emit()

    def stop(self):
        self.is_running = False

# ==========================================
# MAIN GUI APPLICATION
# ==========================================
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("STM32 Masterpiece Tuner (Speed + Precision Stop)")
        self.resize(1300, 850)

        self.serial_thread = None
        self.tuner_thread = None
        self.is_tuning = False
        self.tune_time_data = []
        self.tune_pos_data = []
        self.tune_vel_data = []
        self.tune_acc_data = []

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

        ctrl_group = QGroupBox("Trajectory Limits & Control")
        ctrl_layout = QFormLayout()
        self.target_pos_input = QLineEdit("0.0")
        
        # UI สำหรับความเร็วสูงสุด
        self.vmax_input = QLineEdit("6.28") 
        self.amax_input = QLineEdit("6.28")
        self.jmax_input = QLineEdit("12.56")

        self.kp_input = QLineEdit("20.0")
        self.ki_input = QLineEdit("0.0")
        self.kd_input = QLineEdit("0.0")
        self.btn_send_ctrl = QPushButton("Apply to STM32")
        self.btn_send_ctrl.setStyleSheet("font-weight: bold; background-color: #ffc107; color: black; padding: 10px;")
        self.btn_send_ctrl.clicked.connect(self.send_control_settings)
        
        ctrl_layout.addRow("Target Pos (rad):", self.target_pos_input)
        ctrl_layout.addRow("V Max (rad/s):", self.vmax_input)
        ctrl_layout.addRow("A Max (rad/s²):", self.amax_input)
        ctrl_layout.addRow("Jerk Max (rad/s³):", self.jmax_input)
        ctrl_layout.addRow("Kp Vel:", self.kp_input)
        ctrl_layout.addRow("Ki Vel:", self.ki_input)
        ctrl_layout.addRow("Kd Vel (Damper):", self.kd_input)
        ctrl_layout.addRow(self.btn_send_ctrl)
        ctrl_group.setLayout(ctrl_layout)
        left_panel.addWidget(ctrl_group)

        tune_group = QGroupBox("Masterpiece Auto-Tuner (Fast & Exact Stop)")
        tune_layout = QVBoxLayout()
        btn_layout = QHBoxLayout()
        
        self.btn_tune = QPushButton("▶ RUN MASTERPIECE TUNE")
        self.btn_tune.setStyleSheet("font-weight: bold; padding: 12px; background-color: #8b0000; color: white; font-size: 14px;")
        self.btn_tune.clicked.connect(self.start_autotune)
        
        self.btn_save_log = QPushButton("💾 Save Log")
        self.btn_save_log.setStyleSheet("padding: 12px;")
        self.btn_save_log.clicked.connect(self.save_log)
        
        btn_layout.addWidget(self.btn_tune)
        btn_layout.addWidget(self.btn_save_log)
        
        self.lbl_tune_res = QLabel("Ready.\nThis algorithm uses your original fast math,\nthen adds a strict D-Gain (Kd) sweep to ensure\nit stops DEAD precisely on target.")
        self.lbl_tune_res.setWordWrap(True)
        self.lbl_tune_res.setAlignment(Qt.AlignTop | Qt.AlignLeft)
        self.lbl_tune_res.setStyleSheet("background-color: #0c0c0c; color: #00ff00; padding: 10px; font-family: Consolas; font-size: 12px;")
        
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setWidget(self.lbl_tune_res)
        scroll_area.setMinimumHeight(300)

        tune_layout.addLayout(btn_layout)
        tune_layout.addWidget(scroll_area)
        tune_group.setLayout(tune_layout)
        left_panel.addWidget(tune_group)

        self.plot_widget = pg.GraphicsLayoutWidget()
        main_layout.addWidget(self.plot_widget, stretch=4)
        
        self.p1 = self.plot_widget.addPlot(title="Position (rad)")
        self.curve_pos = self.p1.plot(pen=pg.mkPen('y', width=2))
        self.plot_widget.nextRow()
        self.p2 = self.plot_widget.addPlot(title="Velocity (rad/s)")
        self.curve_vel = self.p2.plot(pen=pg.mkPen('c', width=2))
        self.plot_widget.nextRow()
        self.p3 = self.plot_widget.addPlot(title="Acceleration (rad/s²) [Vibration Detection]")
        self.curve_acc = self.p3.plot(pen=pg.mkPen('m', width=2))

        self.plot_data_size = 500
        self.pos_data = np.zeros(self.plot_data_size)
        self.vel_data = np.zeros(self.plot_data_size)
        self.acc_data = np.zeros(self.plot_data_size)

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
        self.acc_data = np.roll(self.acc_data, -1)
        self.pos_data[-1] = pos
        self.vel_data[-1] = vel
        self.acc_data[-1] = acc

        self.curve_pos.setData(self.pos_data)
        self.curve_vel.setData(self.vel_data)
        self.curve_acc.setData(self.acc_data)

        if self.is_tuning:
            current_t = time.time() - self.tune_start_time
            self.tune_time_data.append(current_t)
            self.tune_pos_data.append(pos) 
            self.tune_vel_data.append(vel)
            self.tune_acc_data.append(acc)

    def send_control_settings(self):
        if not self.serial_thread or not self.serial_thread.is_running:
            QMessageBox.warning(self, "Warning", "Port not connected.")
            return

        try:
            t_pos = float(self.target_pos_input.text())
            kp = float(self.kp_input.text())
            ki = float(self.ki_input.text())
            kd = float(self.kd_input.text())

            tuner = MasterpieceTunerThread(self)
            tuner.set_parameters(kp, ki, kd, t_pos)
            
        except ValueError:
            QMessageBox.warning(self, "Invalid Input", "Please enter valid numbers.")

    def start_autotune(self):
        if not self.serial_thread or not self.serial_thread.is_running:
            QMessageBox.warning(self, "Warning", "Connect to the System first.")
            return

        self.btn_tune.setEnabled(False)
        self.lbl_tune_res.setText("Starting MASTERPIECE Tuner...\n")
        
        self.tune_start_time = time.time()
        self.tune_time_data = []
        
        self.tuner_thread = MasterpieceTunerThread(self)
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
        self.kd_input.setText(f"{kd:.3f}")

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