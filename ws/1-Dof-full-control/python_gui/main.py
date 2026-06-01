"""
STM32 S-Curve Tuner — 1-DOF Pendulum Rod
Single-cable via ST-Link VCP (LPUART1, 19200 8E1)

Telemetry (24 bytes):
  [0x7E 0x7E][pos f32][vel f32][acc f32][pos_ref f32][vel_ref f32][0x03 0x03]

Modbus RTU FC06, Slave ID=21:
  Reg 10: STEP_CMD   open-loop PWM step (value × 10000 = motor%)
  Reg 11: TARGET_POS target rad × 1000 (int16 signed)
  Reg 12: KP_VEL     kp_vel × 100
  Reg 13: KI_VEL     ki_vel × 100
  Reg 14: KD_VEL     kd_vel × 100
  Reg 15: APPLY      1 = apply all + trigger move (auto-clears)
  Reg 16: TRAJ_TYPE  1 = S-Curve (forced)
  Reg 17: V_MAX      v_max × 100
  Reg 18: A_MAX      a_max × 100
  Reg 19: J_MAX      j_max × 100
  Reg 20: KP_POS     kp_pos × 100
  Reg 21: SET_HOME   1 = set home (auto-clears)
  Reg 22: KD_POS     kd_pos × 100  (D via -velocity: pendulum damping)
  Reg 23: KI_POS     ki_pos × 100
"""

import sys, struct, time, queue, threading, math
from collections import deque
import numpy as np
from scipy.optimize import curve_fit
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QLineEdit, QPushButton, QGroupBox, QFormLayout, QMessageBox, QScrollArea,
    QFileDialog, QTabWidget, QCheckBox, QComboBox
)
from PyQt5.QtCore import QThread, pyqtSignal, Qt, QTimer
import pyqtgraph as pg
import serial

# ── Register Map ───────────────────────────────────────────────────────────────
SLAVE_ID  = 21
R_STEP    = 10
R_TARGET  = 11
R_KP_VEL  = 12
R_KI_VEL  = 13
R_KD_VEL  = 14
R_APPLY   = 15
R_TRAJ    = 16
R_VMAX    = 17
R_AMAX    = 18
R_JMAX    = 19
R_KP_POS  = 20
R_SETHOME = 21
R_KD_POS  = 22
R_KI_POS  = 23
R_STOP    = 25   # หยุดทันที ค้างตำแหน่งปัจจุบัน

TELEM_BYTES = 24
MAX_HIST    = 6000   # 120 s @ 50 Hz
R2D         = 180.0 / math.pi

# ── Defaults: ตรงกับ firmware dev_dash init ────────────────────────────────────
DEF = dict(
    v_max=3.14, a_max=6.28, j_max=10.0,
    kp_pos=1.0, ki_pos=0.0, kd_pos=0.0,
    kp_vel=20.0, ki_vel=0.0, kd_vel=0.0,
    target_deg=0.0,          # องศา เหมือน firmware
)


# ── Modbus helpers ─────────────────────────────────────────────────────────────
def _crc16(data: bytes) -> bytes:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return struct.pack('<H', crc)

def _mb_write(reg: int, val: int) -> bytes:
    pkt = struct.pack('>BBHH', SLAVE_ID, 6, reg, int(val) & 0xFFFF)
    return pkt + _crc16(pkt)


# ── Serial Thread ──────────────────────────────────────────────────────────────
BAUD_CONFIGS = {
    "115200 8N1  (firmware เก่า)": (115200, 'N'),
    "19200  8E1  (firmware ใหม่)": (19200,  'E'),
}

class SerialThread(QThread):
    telem   = pyqtSignal(float, float, float, float, float)
    error   = pyqtSignal(str)
    pkt_cnt = pyqtSignal(int)   # packet counter for status display

    def __init__(self, port, baud=115200, parity='N'):
        super().__init__()
        self.port   = port
        self.baud   = baud
        self.parity = parity
        self._ser   = None
        self._running = False
        self._q = queue.Queue()

    def send(self, reg, val):
        self._q.put((reg, val))

    def run(self):
        try:
            par = serial.PARITY_EVEN if self.parity == 'E' else serial.PARITY_NONE
            self._ser = serial.Serial(
                self.port, self.baud,
                parity=par, timeout=0.01
            )
            self._running = True
            buf   = bytearray()
            count = 0
            while self._running:
                if self._ser.in_waiting:
                    buf.extend(self._ser.read(self._ser.in_waiting))

                # รองรับทั้ง 16-byte (firmware เก่า) และ 24-byte (firmware ใหม่)
                while len(buf) >= 16:
                    if buf[0] != 0x7E or buf[1] != 0x7E:
                        del buf[0:1]
                        continue

                    if len(buf) >= 24 and buf[22] == 0x03 and buf[23] == 0x03:
                        pos, vel, acc, pr, vr = struct.unpack('<5f', bytes(buf[2:22]))
                        self.telem.emit(pos, vel, acc, pr, vr)
                        del buf[:24]
                        count += 1; self.pkt_cnt.emit(count)

                    elif buf[14] == 0x03 and buf[15] == 0x03:
                        pos, vel, acc = struct.unpack('<3f', bytes(buf[2:14]))
                        self.telem.emit(pos, vel, acc, pos, vel)
                        del buf[:16]
                        count += 1; self.pkt_cnt.emit(count)

                    elif len(buf) < 24:
                        break
                    else:
                        del buf[0:1]

                while not self._q.empty():
                    r, v = self._q.get()
                    self._ser.write(_mb_write(r, v))
                    time.sleep(0.018)
                time.sleep(0.001)
        except Exception as e:
            self.error.emit(str(e))

    def stop(self):
        self._running = False
        if self._ser and self._ser.is_open:
            self._ser.close()


# ── Safe Incremental Tuner ────────────────────────────────────────────────────
class TunerThread(QThread):
    log    = pyqtSignal(str)
    result = pyqtSignal(dict)
    done   = pyqtSignal()

    def __init__(self, mw):
        super().__init__()
        self.mw          = mw
        self._alive      = True
        self._stop_event = threading.Event()

    # ── Helpers ────────────────────────────────────────────────────────────────
    def _sleep(self, s):
        self._stop_event.wait(timeout=s)

    def _send(self, reg, val):
        if self.mw.serial and self.mw.serial._running:
            self.mw.serial._q.put((reg, val))
            time.sleep(0.02)

    def _wait_vel_zero(self, timeout=4.0):
        """รอจน encoder velocity ≈ 0 (motor หยุดจริง)"""
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self._stop_event.wait(0.05):
                return
            try:
                if abs(self.mw.h_vel[-1]) < 0.15:
                    return
            except (IndexError, TypeError):
                pass

    def _estop(self):
        """Emergency stop: flush queue → STOP firmware → รอ motor หยุดจริง"""
        s = self.mw.serial
        if s and s._running:
            # ล้างคำสั่งที่รออยู่ทั้งหมด
            while not s._q.empty():
                try:
                    s._q.get_nowait()
                except Exception:
                    break
            # ส่ง STOP ตรงๆ โดยไม่ผ่านคิว
            try:
                import struct as _s
                pkt = _s.pack('>BBHH', SLAVE_ID, 6, R_STOP, 1)
                crc = _crc16(pkt)
                s._ser.write(pkt + crc)
            except Exception:
                s._q.put((R_STOP, 1))
        self._wait_vel_zero(timeout=4.0)
        self._sleep(0.3)

    def _apply(self, d, target_rad):
        self._send(R_KP_VEL, int(d['kp_vel'] * 100))
        self._send(R_KI_VEL, int(d.get('ki_vel', 0.0) * 100))
        self._send(R_KD_VEL, int(d.get('kd_vel', 0.0) * 100))
        self._send(R_KP_POS, int(d.get('kp_pos', 1.0) * 100))
        self._send(R_KD_POS, int(d.get('kd_pos', 0.0) * 100))
        self._send(R_KI_POS, int(d.get('ki_pos', 0.0) * 100))
        self._send(R_TRAJ,   1)
        self._send(R_VMAX,   int(d['v_max'] * 100))
        self._send(R_AMAX,   int(d['a_max'] * 100))
        self._send(R_JMAX,   int(d['j_max'] * 100))
        self._send(R_TARGET, int(target_rad * 1000) & 0xFFFF)
        self._send(R_APPLY,  1)

    def _go_home(self, d, from_rad):
        """กลับ start position ช้าๆ ปลอดภัย"""
        safe = {**d, 'kp_vel': min(d['kp_vel'], 8.0),
                'ki_vel': 0.0, 'kd_pos': max(d.get('kd_pos', 0.0), 0.2)}
        self._apply(safe, from_rad)
        self._wait_settle(from_rad * R2D, max_wait=4.0)

    def _wait_settle(self, target_deg, max_wait=5.0):
        """รอจนหยุดนิ่ง (อ่าน encoder จริง) หรือ timeout"""
        deadline  = time.time() + max_wait
        ok_since  = None
        while time.time() < deadline:
            if self._stop_event.wait(0.08):
                return
            try:
                pos_now = self.mw.h_pos[-1]   # degrees
                vel_now = abs(self.mw.h_vel[-1])
                near = abs(pos_now - target_deg) < 4.0
                slow = vel_now < 0.25
                if near and slow:
                    if ok_since is None:
                        ok_since = time.time()
                    elif time.time() - ok_since > 0.4:
                        return
                else:
                    ok_since = None
            except (IndexError, TypeError):
                pass

    def _measure(self, d, target_rad, max_wait=6.0):
        """สั่งหมุน รอนิ่ง แล้วเก็บข้อมูล 1 วิ"""
        self.mw.tune_buf    = {'t': [], 'pos': [], 'vel': [], 'acc': []}
        self.mw.tune_active = True
        self.mw.tune_t0     = time.time()
        self._apply(d, target_rad)
        self._wait_settle(target_rad * R2D, max_wait=max_wait)
        self._sleep(0.8)   # เก็บ tail data
        self.mw.tune_active = False
        self._sleep(0.05)
        return dict(self.mw.tune_buf)

    def _is_oscillating(self, data):
        """ตรวจ oscillation จาก velocity tail (encoder จริง)"""
        vel = np.array(data['vel'])
        acc = np.array(data['acc'])
        if len(vel) < 20:
            return False
        n        = max(15, len(vel) // 3)
        tail_v   = vel[-n:]
        tail_a   = acc[-n:]
        rms_vel  = float(np.sqrt(np.mean(tail_v ** 2)))
        std_acc  = float(np.std(tail_a))
        signs    = np.sign(tail_v)
        crossings = int(np.sum(np.diff(signs) != 0))
        # oscillating = velocity ยังสูง หรือ sign เปลี่ยนบ่อย หรือ accel สั่น
        return rms_vel > 0.8 or crossings >= 7 or std_acc > 20.0

    def _score(self, data, target_rad):
        pos = np.array(data['pos'])
        acc = np.array(data['acc'])
        vel = np.array(data['vel'])
        if len(pos) < 15:
            return None, {}
        target_deg = target_rad * R2D
        ss_err  = abs(target_deg - float(np.mean(pos[-15:])))
        if target_deg >= pos[0]:
            os_ = max(float(np.max(pos)) - target_deg, 0.0)
        else:
            os_ = max(target_deg - float(np.min(pos)), 0.0)
        vib   = float(np.std(acc[-15:]))
        score = os_ * 80 + ss_err * 30 + vib * 5   # หน่วยองศา
        return score, {'ss_err': ss_err, 'os': os_, 'vib': vib}

    # ── Main Algorithm ─────────────────────────────────────────────────────────
    def run(self):
        mw = self.mw
        if not mw.serial or not mw.serial._running:
            self.log.emit("Not connected.")
            self.done.emit()
            return

        try:
            v_max    = float(mw.e_vmax.text())
            a_max    = float(mw.e_amax.text())
            j_max    = float(mw.e_jmax.text())
            from_deg = float(mw.e_tune_from.text())
            to_deg   = float(mw.e_tune_to.text())
        except Exception:
            v_max, a_max, j_max = DEF['v_max'], DEF['a_max'], DEF['j_max']
            from_deg, to_deg = 0.0, 90.0

        from_rad = from_deg * math.pi / 180.0
        to_rad   = to_deg   * math.pi / 180.0

        traj = dict(v_max=v_max, a_max=a_max, j_max=j_max)

        self.log.emit("=" * 58)
        self.log.emit("  SAFE INCREMENTAL TUNER")
        self.log.emit(f"  {from_deg:.0f}° → {to_deg:.0f}°  |  S-Curve")
        self.log.emit(f"  เริ่มนิ่มมาก → เพิ่มทีละขั้น → หยุดก่อนสั่น")
        self.log.emit(f"  อ่าน encoder ตลอด: ถ้าสั่นหยุดทันที (STOP)")
        self.log.emit("=" * 58)

        # ── 0. Safe Home ──────────────────────────────────────────────────────
        self.log.emit(f"\n[0] Home → {from_deg:.0f}°  (ช้า Kp=5)")
        start_cfg = dict(**traj, kp_vel=5.0, kp_pos=0.5, kd_pos=0.2)
        self._apply(start_cfg, from_rad)
        self._wait_settle(from_deg, max_wait=5.0)

        if not self._alive:
            self.done.emit(); return

        # ── 1. Kp_vel search — geometric +40% each step ───────────────────────
        self.log.emit(f"\n[1/3] Kp_vel search  (Kp_pos=0.5, Kd_pos=0)")
        self.log.emit( "      เริ่ม 3 → ×1.4 ทีละขั้น → หยุดเมื่อสั่น")

        kp       = 3.0
        best_kp  = 3.0
        prev_vib = 999.0

        while kp <= 80 and self._alive:
            cfg = dict(**traj, kp_vel=kp, kp_pos=0.5, kd_pos=0.0)
            self._go_home(cfg, from_rad)
            if not self._alive: break
            data = self._measure(cfg, to_rad)
            if not self._alive: break

            osc = self._is_oscillating(data)
            sc, inf = self._score(data, to_rad)

            if osc or sc is None:
                self.log.emit(f"   Kp={kp:5.1f}  ⚠ OSCILLATION → ESTOP + รอหยุด")
                self._estop()   # flush queue + รอ velocity=0
                break

            trend = "↑ vib เพิ่ม" if inf['vib'] > prev_vib * 1.8 else "✓ ok"
            self.log.emit(
                f"   Kp={kp:5.1f}  os={inf['os']:.1f}°  "
                f"err={inf['ss_err']:.1f}°  vib={inf['vib']:.1f}  {trend}")

            best_kp  = kp
            prev_vib = inf['vib']
            kp      *= 1.4

        safe_kp = best_kp * 0.75
        self.log.emit(f"   limit≈{best_kp:.1f} → safe={safe_kp:.1f} (75%)")

        if not self._alive:
            self._estop(); self.done.emit(); return

        # ── 2. Kd_pos sweep — pendulum damping ────────────────────────────────
        self.log.emit(f"\n[2/3] Kd_pos sweep  (Kp_vel={safe_kp:.1f}, Kp_pos=1.0)")
        self.log.emit( "      motor นิ่งสนิทแล้ว — เริ่มหา Kd_pos")

        kd_steps  = [0.0, 0.1, 0.2, 0.3, 0.5, 0.7, 1.0]
        best_kd, best_kd_sc = 0.0, float('inf')

        for kd in kd_steps:
            if not self._alive: break
            cfg = dict(**traj, kp_vel=safe_kp, kp_pos=1.0, kd_pos=kd)
            self._go_home(cfg, from_rad)
            if not self._alive: break
            data = self._measure(cfg, to_rad)

            if self._is_oscillating(data):
                self.log.emit(f"   Kd_pos={kd:.2f}  ⚠ OSCILLATION → ESTOP")
                self._estop()
                break

            sc, inf = self._score(data, to_rad)
            if sc is None: continue
            tag = " ←" if sc < best_kd_sc else ""
            self.log.emit(
                f"   Kd_pos={kd:.2f}  os={inf['os']:.1f}°  "
                f"vib={inf['vib']:.1f}  err={inf['ss_err']:.1f}°{tag}")
            if sc < best_kd_sc:
                best_kd_sc, best_kd = sc, kd

        self.log.emit(f"   ✅ Best Kd_pos = {best_kd:.2f}")

        if not self._alive:
            self._estop(); self.done.emit(); return

        # ── 3. Ki_vel sweep — drift only ──────────────────────────────────────
        self.log.emit(f"\n[3/3] Ki_vel sweep  (เล็กมาก — drift เท่านั้น)")

        ki_steps = [0.0, 0.02, 0.05, 0.08, 0.12]
        best_ki, best_ki_sc = 0.0, float('inf')

        for ki in ki_steps:
            if not self._alive: break
            cfg = dict(**traj, kp_vel=safe_kp, ki_vel=ki,
                       kp_pos=1.0, kd_pos=best_kd)
            self._go_home(cfg, from_rad)
            if not self._alive: break
            data = self._measure(cfg, to_rad)

            if self._is_oscillating(data):
                self.log.emit(f"   Ki={ki:.3f}  ⚠ OSCILLATION → ESTOP")
                self._estop()
                break

            sc, inf = self._score(data, to_rad)
            if sc is None: continue
            if inf['os'] > 5.0:
                self.log.emit(f"   Ki={ki:.3f}  REJECTED (os={inf['os']:.1f}°)")
                continue

            tag = " ←" if sc < best_ki_sc else ""
            self.log.emit(
                f"   Ki={ki:.3f}  err={inf['ss_err']:.1f}°  os={inf['os']:.1f}°{tag}")
            if sc < best_ki_sc:
                best_ki_sc, best_ki = sc, ki

        self.log.emit(f"   ✅ Best Ki_vel = {best_ki:.3f}")

        # ── Final ──────────────────────────────────────────────────────────────
        result = dict(
            kp_vel=safe_kp, ki_vel=best_ki, kd_vel=0.0,
            kp_pos=1.0,     ki_pos=0.0,     kd_pos=best_kd,
            v_max=v_max, a_max=a_max, j_max=j_max,
        )
        self.log.emit("\n" + "=" * 58)
        self.log.emit("  TUNING COMPLETE")
        self.log.emit(f"  Kp_vel={safe_kp:.2f}  Ki_vel={best_ki:.3f}  Kd_vel=0")
        self.log.emit(f"  Kp_pos=1.0   Ki_pos=0   Kd_pos={best_kd:.2f}")
        self.log.emit(f"  v={v_max}  a={a_max}  j={j_max}")
        self.log.emit("=" * 58)

        # Apply final and go home
        self._apply(result, from_rad)
        self.result.emit(result)
        self.done.emit()

    def stop(self):
        self._alive = False
        self._stop_event.set()


# ── Main Window ────────────────────────────────────────────────────────────────
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("STM32 S-Curve Tuner  —  1-DOF Pendulum")
        self.resize(1500, 960)

        self.serial = None
        self.tuner  = None

        # ── Time-series history (degrees / rad/s / rad/s²) ────────────────────
        self.t_start  = None
        self.h_t      = deque(maxlen=MAX_HIST)
        self.h_pos    = deque(maxlen=MAX_HIST)   # actual position (°)
        self.h_pref   = deque(maxlen=MAX_HIST)   # reference position (°)
        self.h_vel    = deque(maxlen=MAX_HIST)   # actual velocity (rad/s)
        self.h_vref   = deque(maxlen=MAX_HIST)   # reference velocity (rad/s)
        self.h_acc    = deque(maxlen=MAX_HIST)   # acceleration (rad/s²)

        # Tuner capture
        self.tune_active = False
        self.tune_t0     = 0.0
        self.tune_buf    = {'t': [], 'pos': [], 'vel': [], 'acc': []}

        self._build_ui()

        # Graph refresh @ 25 Hz (decoupled from 50 Hz telemetry)
        self._gtimer = QTimer(self)
        self._gtimer.timeout.connect(self._refresh_graphs)
        self._gtimer.start(40)

    # ── UI ─────────────────────────────────────────────────────────────────────
    def _build_ui(self):
        root = QWidget()
        self.setCentralWidget(root)
        main = QHBoxLayout(root)
        main.setSpacing(8)

        # ── Left panel ─────────────────────────────────────────────────────────
        left = QWidget()
        left.setFixedWidth(340)
        lv = QVBoxLayout(left)
        lv.setSpacing(6)
        main.addWidget(left)

        # Connection
        cg = QGroupBox("Connection")
        cf = QFormLayout()
        self.e_port  = QLineEdit("COM14")
        self.cb_baud = QComboBox()
        for label in BAUD_CONFIGS:
            self.cb_baud.addItem(label)
        self.cb_baud.setCurrentIndex(0)
        self.lbl_pkts = QLabel("pkts: 0")
        self.lbl_pkts.setStyleSheet("color:#00cc66;font-family:Consolas;font-size:10px;")
        self.btn_conn = QPushButton("Connect")
        self.btn_conn.setStyleSheet(
            "background:#1e6b1e;color:white;font-weight:bold;padding:7px;border-radius:4px;")
        self.btn_conn.clicked.connect(self._toggle_conn)
        cf.addRow("COM Port:", self.e_port)
        cf.addRow("Baud:", self.cb_baud)
        cf.addRow(self.lbl_pkts, self.btn_conn)
        cg.setLayout(cf)
        lv.addWidget(cg)

        # STOP button — always visible
        self.btn_stop = QPushButton("⏹  STOP")
        self.btn_stop.setStyleSheet(
            "background:#cc0000;color:white;font-weight:bold;"
            "padding:12px;font-size:16px;border-radius:6px;")
        self.btn_stop.clicked.connect(self._stop)
        lv.addWidget(self.btn_stop)

        # Tabs
        tabs = QTabWidget()
        lv.addWidget(tabs, stretch=1)

        # ── Tab 1: Manual ──────────────────────────────────────────────────────
        mt = QWidget()
        mv = QVBoxLayout(mt)
        mv.setSpacing(4)

        tg = QGroupBox("S-Curve Trajectory")
        tf = QFormLayout()
        self.e_vmax = QLineEdit(str(DEF['v_max']))
        self.e_amax = QLineEdit(str(DEF['a_max']))
        self.e_jmax = QLineEdit(str(DEF['j_max']))
        tf.addRow("v_max (rad/s):",  self.e_vmax)
        tf.addRow("a_max (rad/s²):", self.e_amax)
        tf.addRow("j_max (rad/s³):", self.e_jmax)
        tg.setLayout(tf)
        mv.addWidget(tg)

        vg = QGroupBox("Velocity PID  (inner loop)")
        vf = QFormLayout()
        self.e_kp_vel = QLineEdit(str(DEF['kp_vel']))
        self.e_ki_vel = QLineEdit(str(DEF['ki_vel']))
        self.e_kd_vel = QLineEdit(str(DEF['kd_vel']))
        vf.addRow("Kp_vel:", self.e_kp_vel)
        vf.addRow("Ki_vel:", self.e_ki_vel)
        vf.addRow("Kd_vel:", self.e_kd_vel)
        self._note(vf, "Kd_vel = 0 recommended (1kHz noise)")
        vg.setLayout(vf)
        mv.addWidget(vg)

        pg_ = QGroupBox("Position PID  (outer loop)")
        pf  = QFormLayout()
        self.e_kp_pos = QLineEdit(str(DEF['kp_pos']))
        self.e_ki_pos = QLineEdit(str(DEF['ki_pos']))
        self.e_kd_pos = QLineEdit(str(DEF['kd_pos']))
        pf.addRow("Kp_pos:", self.e_kp_pos)
        pf.addRow("Ki_pos:", self.e_ki_pos)
        pf.addRow("Kd_pos (damp):", self.e_kd_pos)
        self._note(pf, "Kd_pos uses −velocity → pendulum damping")
        pg_.setLayout(pf)
        mv.addWidget(pg_)

        mg = QGroupBox("Move Command")
        mf = QFormLayout()
        self.e_target = QLineEdit(str(DEF['target_deg']))
        mf.addRow("Target (°):", self.e_target)   # องศา
        mg.setLayout(mf)
        mv.addWidget(mg)

        btn_apply = QPushButton("⚡  Apply & Move")
        btn_apply.setStyleSheet(
            "background:#9a5500;color:white;font-weight:bold;padding:10px;"
            "font-size:13px;border-radius:4px;")
        btn_apply.clicked.connect(self._manual_apply)

        btn_home = QPushButton("🏠  Set Home  (current pos = 0°)")
        btn_home.setStyleSheet(
            "background:#1e2e7a;color:white;padding:7px;border-radius:4px;")
        btn_home.clicked.connect(self._set_home)

        mv.addWidget(btn_apply)
        mv.addWidget(btn_home)
        mv.addStretch()
        tabs.addTab(mt, "Manual")

        # ── Tab 2: Auto Tune ───────────────────────────────────────────────────
        at = QWidget()
        av = QVBoxLayout(at)
        av.setSpacing(5)

        rg = QGroupBox("Test Range")
        rf = QFormLayout()
        self.e_tune_from = QLineEdit("0")
        self.e_tune_to   = QLineEdit("90")
        rf.addRow("From (°):", self.e_tune_from)
        rf.addRow("To   (°):", self.e_tune_to)
        rg.setLayout(rf)
        av.addWidget(rg)

        self.btn_tune = QPushButton("▶  RUN SAFE INCREMENTAL TUNE")
        self.btn_tune.setStyleSheet(
            "background:#7a0000;color:white;font-weight:bold;padding:13px;"
            "font-size:14px;border-radius:4px;")
        self.btn_tune.clicked.connect(self._run_tune)
        av.addWidget(self.btn_tune)

        btn_save = QPushButton("💾  Save Log")
        btn_save.clicked.connect(self._save_log)
        av.addWidget(btn_save)

        self.lbl_log = QLabel(
            "Ready.\n\n"
            "Algorithm (safe incremental):\n"
            "  [1] Kp_vel: เริ่ม 3 → ×1.4 ทีละขั้น\n"
            "       อ่าน encoder → หยุดก่อนสั่น\n"
            "       ใช้ 75% เป็น safe zone\n\n"
            "  [2] Kd_pos: 0→1.0 หา damping\n"
            "       หยุดถ้าสั่น\n\n"
            "  [3] Ki_vel: 0→0.12 drift เท่านั้น\n\n"
            "ใช้เวลา ~5 min  กด ⏹ STOP ได้ตลอด"
        )
        self.lbl_log.setWordWrap(True)
        self.lbl_log.setAlignment(Qt.AlignTop)
        self.lbl_log.setStyleSheet(
            "background:#080808;color:#00ff88;padding:8px;"
            "font-family:Consolas;font-size:11px;")
        sa = QScrollArea()
        sa.setWidgetResizable(True)
        sa.setWidget(self.lbl_log)
        av.addWidget(sa, stretch=1)
        tabs.addTab(at, "Auto Tune")

        self.lbl_values = QLabel("Applied:  (not connected)")
        self.lbl_values.setWordWrap(True)
        self.lbl_values.setStyleSheet(
            "background:#0e0e22;color:#aaaaff;padding:7px;"
            "font-family:Consolas;font-size:10px;border-radius:4px;")
        lv.addWidget(self.lbl_values)

        # ── Right panel ────────────────────────────────────────────────────────
        right = QWidget()
        rv = QVBoxLayout(right)
        rv.setSpacing(4)
        main.addWidget(right, stretch=1)

        # Signal checkboxes + controls
        ctrl = QWidget()
        cl = QHBoxLayout(ctrl)
        cl.setSpacing(12)

        self.ck_pos_a = QCheckBox("Pos actual")
        self.ck_pos_r = QCheckBox("Pos ref")
        self.ck_vel_a = QCheckBox("Vel actual")
        self.ck_vel_r = QCheckBox("Vel ref")
        self.ck_acc   = QCheckBox("Accel")
        self.ck_auto  = QCheckBox("Auto-scroll")

        for ck, on in [(self.ck_pos_a, True), (self.ck_pos_r, True),
                       (self.ck_vel_a, True), (self.ck_vel_r, True),
                       (self.ck_acc,   False), (self.ck_auto,  True)]:
            ck.setChecked(on)
            ck.setStyleSheet("color:#cccccc;")
            cl.addWidget(ck)

        btn_clear = QPushButton("🗑 Clear")
        btn_clear.setStyleSheet("padding:4px 10px;")
        btn_clear.clicked.connect(self._clear_history)
        cl.addStretch()
        cl.addWidget(btn_clear)
        rv.addWidget(ctrl)

        # Graphs (pyqtgraph)
        gw = pg.GraphicsLayoutWidget()
        rv.addWidget(gw, stretch=1)

        Y    = pg.mkPen('#ffcc00', width=2)
        Ydsh = pg.mkPen('#888888', width=1.5, style=Qt.DashLine)
        C    = pg.mkPen('#00ccff', width=2)
        Cdsh = pg.mkPen('#5588bb', width=1.5, style=Qt.DashLine)
        M    = pg.mkPen('#ff44cc', width=1.5)

        self.p1 = gw.addPlot(
            title="<b>Position</b>  "
                  "<span style='color:#ffcc00'>─ actual (°)</span>  "
                  "<span style='color:#888'>─ ─ reference (°)</span>")
        self.p1.showGrid(x=True, y=True, alpha=0.25)
        self.p1.setLabel('left', '°')
        self.p1.setLabel('bottom', 's')
        self.c_pos  = self.p1.plot(pen=Y,    name="pos actual")
        self.c_pref = self.p1.plot(pen=Ydsh, name="pos ref")
        gw.nextRow()

        self.p2 = gw.addPlot(
            title="<b>Velocity</b>  "
                  "<span style='color:#00ccff'>─ actual (rad/s)</span>  "
                  "<span style='color:#5588bb'>─ ─ reference (rad/s)</span>")
        self.p2.showGrid(x=True, y=True, alpha=0.25)
        self.p2.setLabel('left', 'rad/s')
        self.p2.setLabel('bottom', 's')
        self.p2.setXLink(self.p1)   # link X-axis → pan/zoom เคลื่อนพร้อมกัน
        self.c_vel  = self.p2.plot(pen=C,    name="vel actual")
        self.c_vref = self.p2.plot(pen=Cdsh, name="vel ref")
        gw.nextRow()

        self.p3 = gw.addPlot(title="<b>Acceleration</b>  rad/s²")
        self.p3.showGrid(x=True, y=True, alpha=0.25)
        self.p3.setLabel('left', 'rad/s²')
        self.p3.setLabel('bottom', 's')
        self.p3.setXLink(self.p1)   # link X-axis
        self.c_acc = self.p3.plot(pen=M, name="accel")

    def _note(self, layout, text):
        lbl = QLabel(text)
        lbl.setStyleSheet("color:#888;font-size:10px;font-style:italic;")
        layout.addRow("", lbl)

    # ── Slots ──────────────────────────────────────────────────────────────────
    def _toggle_conn(self):
        if self.serial is None or not self.serial._running:
            baud, parity = BAUD_CONFIGS[self.cb_baud.currentText()]
            self.serial = SerialThread(self.e_port.text(), baud=baud, parity=parity)
            self.serial.telem.connect(self._on_telem)
            self.serial.pkt_cnt.connect(lambda n: self.lbl_pkts.setText(f"pkts: {n}"))
            self.serial.error.connect(
                lambda e: QMessageBox.critical(self, "Serial Error", e))
            self.serial.start()
            self.btn_conn.setText("Disconnect")
            self.btn_conn.setStyleSheet(
                "background:#7a2020;color:white;font-weight:bold;padding:7px;border-radius:4px;")
            self.lbl_pkts.setText("pkts: 0")
            self.lbl_values.setText(f"Connected {self.cb_baud.currentText().split()[0]} — waiting…")
        else:
            self.serial.stop(); self.serial.wait(); self.serial = None
            self.btn_conn.setText("Connect")
            self.btn_conn.setStyleSheet(
                "background:#1e6b1e;color:white;font-weight:bold;padding:7px;border-radius:4px;")
            self.lbl_pkts.setText("pkts: 0")
            self.lbl_values.setText("Applied:  (disconnected)")

    def _on_telem(self, pos, vel, acc, pos_ref, vel_ref):
        now = time.time()
        if self.t_start is None:
            self.t_start = now
        t = now - self.t_start

        # เก็บ history (position เป็นองศา)
        self.h_t.append(t)
        self.h_pos.append(pos  * R2D)
        self.h_pref.append(pos_ref * R2D)
        self.h_vel.append(vel)
        self.h_vref.append(vel_ref)
        self.h_acc.append(acc)

        # tuner capture
        if self.tune_active:
            self.tune_buf['t'].append(now - self.tune_t0)
            self.tune_buf['pos'].append(pos)
            self.tune_buf['vel'].append(vel)
            self.tune_buf['acc'].append(acc)

    def _refresh_graphs(self):
        """อัปเดต graph @ 25 Hz (แยกจาก telemetry 50 Hz)"""
        if not self.h_t:
            return
        t   = np.array(self.h_t)
        pos = np.array(self.h_pos)
        pre = np.array(self.h_pref)
        vel = np.array(self.h_vel)
        vre = np.array(self.h_vref)
        acc = np.array(self.h_acc)

        self.c_pos.setData(t, pos)  if self.ck_pos_a.isChecked() else self.c_pos.setData([], [])
        self.c_pref.setData(t, pre) if self.ck_pos_r.isChecked() else self.c_pref.setData([], [])
        self.c_vel.setData(t, vel)  if self.ck_vel_a.isChecked() else self.c_vel.setData([], [])
        self.c_vref.setData(t, vre) if self.ck_vel_r.isChecked() else self.c_vref.setData([], [])
        self.c_acc.setData(t, acc)  if self.ck_acc.isChecked()   else self.c_acc.setData([], [])

        # Auto-scroll: แสดงข้อมูล 30 วินาทีล่าสุด
        if self.ck_auto.isChecked() and len(t) > 0:
            t_now = t[-1]
            self.p1.setXRange(max(0, t_now - 30), t_now, padding=0.02)

    def _clear_history(self):
        for h in (self.h_t, self.h_pos, self.h_pref,
                  self.h_vel, self.h_vref, self.h_acc):
            h.clear()
        self.t_start = None

    def _manual_apply(self):
        if not self.serial or not self.serial._running:
            QMessageBox.warning(self, "Warning", "Not connected."); return
        try:
            d = {
                'kp_vel':     float(self.e_kp_vel.text()),
                'ki_vel':     float(self.e_ki_vel.text()),
                'kd_vel':     float(self.e_kd_vel.text()),
                'kp_pos':     float(self.e_kp_pos.text()),
                'ki_pos':     float(self.e_ki_pos.text()),
                'kd_pos':     float(self.e_kd_pos.text()),
                'v_max':      float(self.e_vmax.text()),
                'a_max':      float(self.e_amax.text()),
                'j_max':      float(self.e_jmax.text()),
                'target_deg': float(self.e_target.text()),  # องศา
            }
        except ValueError:
            QMessageBox.warning(self, "Input Error", "Invalid number."); return
        target_rad = d['target_deg'] * math.pi / 180.0   # แปลงเป็น rad สำหรับ Modbus
        s = self.serial
        s.send(R_KP_VEL, int(d['kp_vel'] * 100))
        s.send(R_KI_VEL, int(d['ki_vel'] * 100))
        s.send(R_KD_VEL, int(d['kd_vel'] * 100))
        s.send(R_KP_POS, int(d['kp_pos'] * 100))
        s.send(R_KI_POS, int(d['ki_pos'] * 100))
        s.send(R_KD_POS, int(d['kd_pos'] * 100))
        s.send(R_TRAJ,   1)
        s.send(R_VMAX,   int(d['v_max']  * 100))
        s.send(R_AMAX,   int(d['a_max']  * 100))
        s.send(R_JMAX,   int(d['j_max']  * 100))
        s.send(R_TARGET, int(target_rad  * 1000) & 0xFFFF)
        s.send(R_APPLY,  1)
        self._show_values(d)

    def _stop(self):
        # หยุด tuner thread ถ้ากำลัง tune อยู่
        if self.tuner and self.tuner.isRunning():
            self.tuner.stop()
            self.lbl_log.setText(self.lbl_log.text() + "\n⏹ STOPPED by user\n")
            self.btn_tune.setEnabled(True)
        self.tune_active = False
        # ส่ง STOP ไป firmware
        if self.serial and self.serial._running:
            self.serial.send(R_STOP, 1)

    def _set_home(self):
        if not self.serial or not self.serial._running:
            QMessageBox.warning(self, "Warning", "Not connected."); return
        self.serial.send(R_STOP, 1)     # หยุดก่อนเสมอ
        self.serial.send(R_SETHOME, 1)  # แล้วค่อย zero position

    def _show_values(self, d):
        tgt = d.get('target_deg', d.get('target', '?'))
        self.lbl_values.setText(
            f"Applied:  target={tgt}°\n"
            f"  v={d.get('v_max','?')}  a={d.get('a_max','?')}  j={d.get('j_max','?')}\n"
            f"  Kp_vel={d.get('kp_vel','?')}  Ki={d.get('ki_vel','?')}  Kd={d.get('kd_vel','?')}\n"
            f"  Kp_pos={d.get('kp_pos','?')}  Ki={d.get('ki_pos','?')}  Kd={d.get('kd_pos','?')}"
        )

    def _run_tune(self):
        if not self.serial or not self.serial._running:
            QMessageBox.warning(self, "Warning", "Connect first."); return
        self.btn_tune.setEnabled(False)
        self.lbl_log.setText("Starting Pendulum Tuner…\n")
        self.tuner = TunerThread(self)
        self.tuner.log.connect(self._append_log)
        self.tuner.result.connect(self._on_result)
        self.tuner.done.connect(lambda: self.btn_tune.setEnabled(True))
        self.tuner.start()

    def _append_log(self, text):
        self.lbl_log.setText(self.lbl_log.text() + text + "\n")

    def _on_result(self, d):
        self.e_kp_vel.setText(f"{d['kp_vel']:.3f}")
        self.e_ki_vel.setText(f"{d['ki_vel']:.3f}")
        self.e_kd_vel.setText(f"{d.get('kd_vel', 0.0):.3f}")
        self.e_kp_pos.setText(f"{d['kp_pos']:.3f}")
        self.e_ki_pos.setText(f"{d.get('ki_pos', 0.0):.3f}")
        self.e_kd_pos.setText(f"{d['kd_pos']:.3f}")
        self.e_vmax.setText(f"{d['v_max']:.2f}")
        self.e_amax.setText(f"{d['a_max']:.2f}")
        self.e_jmax.setText(f"{d['j_max']:.2f}")
        d['target_deg'] = 0.0
        self._show_values(d)

    def _save_log(self):
        text = self.lbl_log.text()
        if "COMPLETE" not in text:
            QMessageBox.information(self, "Info", "No completed log yet."); return
        fn, _ = QFileDialog.getSaveFileName(
            self, "Save Log", "pendulum_tune.txt", "Text (*.txt)")
        if fn:
            with open(fn, 'w', encoding='utf-8') as f:
                f.write(text)

    def closeEvent(self, event):
        if self.tuner and self.tuner.isRunning():
            self.tuner.stop(); self.tuner.wait()
        if self.serial:
            self.serial.stop(); self.serial.wait()
        event.accept()


# ── Entry ──────────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())
