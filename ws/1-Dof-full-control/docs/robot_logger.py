#!/usr/bin/env python3
"""
Robot Real-Time Logger  —  1-DOF STM32G474RE
Reads Modbus RTU: pos_deg, vel_deg_s, accel via regs 0x28-0x30
Diagnoses: encoder vs control tracking direction

Usage:
    python robot_logger.py COM3

Requirements:
    pip install pyserial matplotlib
"""

import sys, struct, time, threading, argparse
import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

# ── Modbus ───────────────────────────────────────────────────────────────────

SLAVE  = 21
BAUD   = 19200

def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc

def fc03(reg: int, count: int) -> bytes:
    frame = struct.pack('>BBHH', SLAVE, 0x03, reg, count)
    c = crc16(frame)
    return frame + bytes([c & 0xFF, c >> 8])

def parse_fc03(raw: bytes):
    if len(raw) < 5: return None
    bc = raw[2]
    if len(raw) < 3 + bc + 2: return None
    out = []
    for i in range(0, bc, 2):
        v = struct.unpack('>H', raw[3+i:5+i])[0]
        if v > 32767: v -= 65536
        out.append(v)
    return out

# ── Shared state ──────────────────────────────────────────────────────────────

N = 300          # history points
t_buf   = deque([0.0]*N, maxlen=N)
pos_buf = deque([0.0]*N, maxlen=N)
vel_buf = deque([0.0]*N, maxlen=N)
tgt_buf = deque([0.0]*N, maxlen=N)
err_buf = deque([0.0]*N, maxlen=N)

lock      = threading.Lock()
target    = [0.0]          # user-editable
connected = [False]
status    = ['Disconnected']

# ── Poll thread ───────────────────────────────────────────────────────────────

def poll(port_name):
    try:
        ser = serial.Serial(port_name, BAUD, bytesize=8,
                            parity='E', stopbits=1, timeout=0.12)
        connected[0] = True
        status[0] = f'Connected  {port_name}  {BAUD} 8E1'
    except Exception as e:
        status[0] = f'OPEN ERROR: {e}'
        return

    t0 = time.time()
    while True:
        try:
            # Read 0x28 pos, 0x29 vel, 0x30 accel  (3 regs)
            ser.write(fc03(0x28, 3))
            raw = ser.read(11)
            vals = parse_fc03(raw)

            # Read 0x2F state, 0x31 estop  (2 regs)
            ser.write(fc03(0x2F, 4))
            raw2 = ser.read(13)
            state_vals = parse_fc03(raw2)

            if vals and len(vals) >= 2:
                pos = vals[0] / 10.0    # deg
                vel = vals[1] / 10.0    # deg/s
                tgt = target[0]
                err = tgt - pos

                with lock:
                    now = time.time() - t0
                    t_buf.append(now)
                    pos_buf.append(pos)
                    vel_buf.append(vel)
                    tgt_buf.append(tgt)
                    err_buf.append(err)

                if state_vals and len(state_vals) >= 4:
                    state_map = {0:'INIT',1:'IDLE',2:'HOMING_FAST',3:'HOMING_BACKOFF',
                                 4:'HOMING_SLOW',5:'MANUAL_SW',6:'MANUAL_MB',
                                 7:'AUTO',8:'SEQUENCE',9:'TEST',10:'EMER'}
                    st = state_map.get(state_vals[0], f'?{state_vals[0]}')
                    estop = '⚡ESTOP' if state_vals[2] else ''
                    status[0] = f'{port_name}  state={st}  pos={pos:.1f}°  vel={vel:.1f}°/s  {estop}'

        except Exception:
            pass
        time.sleep(0.05)   # 20 Hz

# ── Plot ──────────────────────────────────────────────────────────────────────

def run_plot(port_name):
    thread = threading.Thread(target=poll, args=(port_name,), daemon=True)
    thread.start()

    fig, axes = plt.subplots(3, 1, figsize=(11, 7), sharex=True)
    fig.suptitle('Robot Real-Time Logger  —  1-DOF', fontsize=12)
    plt.subplots_adjust(hspace=0.35, bottom=0.12)

    ax_pos, ax_vel, ax_diag = axes

    # ── Subplot 1: Position ──
    ax_pos.set_ylabel('deg')
    ax_pos.set_title('Position  (pos_deg = 0–360°)')
    ax_pos.set_ylim(-10, 370)
    ax_pos.axhline(0,   color='#333', lw=0.5)
    ax_pos.axhline(360, color='#333', lw=0.5)
    line_pos,  = ax_pos.plot([], [], '#4fc3f7', lw=1.5, label='pos_deg')
    line_tgt,  = ax_pos.plot([], [], '#ff8a65', lw=1.0, ls='--', label='target')
    ax_pos.legend(loc='upper right', fontsize=8)

    # ── Subplot 2: Velocity ──
    ax_vel.set_ylabel('deg/s')
    ax_vel.set_title('Velocity')
    ax_vel.axhline(0, color='#555', lw=0.5)
    line_vel,  = ax_vel.plot([], [], '#a5d6a7', lw=1.5)

    # ── Subplot 3: Tracking diagnosis ──
    ax_diag.set_ylabel('deg  /  deg/s')
    ax_diag.set_xlabel('time (s)')
    ax_diag.set_title('Tracking diagnosis  —  error vs velocity direction')
    ax_diag.axhline(0, color='#555', lw=0.5)
    line_err,  = ax_diag.plot([], [], '#ef5350', lw=1.2, label='error = target − pos')
    line_vel2, = ax_diag.plot([], [], '#80cbc4', lw=1.0, ls='--', alpha=0.7, label='velocity (scaled)')
    ax_diag.legend(loc='upper right', fontsize=8)

    # status text
    txt_status = fig.text(0.01, 0.01, '', fontsize=8, color='#aaa',
                          family='monospace')

    # target input box
    ax_inp = plt.axes([0.15, 0.02, 0.2, 0.03])
    from matplotlib.widgets import TextBox
    tbox = TextBox(ax_inp, 'Target °', initial='0')

    def on_target(text):
        try: target[0] = float(text)
        except: pass
    tbox.on_submit(on_target)

    def update(_frame):
        with lock:
            t  = list(t_buf)
            p  = list(pos_buf)
            v  = list(vel_buf)
            tg = list(tgt_buf)
            er = list(err_buf)

        if not t: return

        # scroll x-window last 8s
        t_now = t[-1]
        x_lo  = max(0, t_now - 8)
        x_hi  = t_now + 0.2

        for ax in axes:
            ax.set_xlim(x_lo, x_hi)

        line_pos.set_data(t, p)
        line_tgt.set_data(t, tg)
        line_vel.set_data(t, v)
        line_err.set_data(t, er)

        # velocity scaled to same magnitude as error for visual comparison
        scale = max(max(abs(e) for e in er), 1.0) / max(max(abs(vv) for vv in v), 0.01)
        line_vel2.set_data(t, [vv * scale for vv in v])

        # auto-scale y for velocity
        if v:
            vmax = max(abs(x) for x in v) + 10
            ax_vel.set_ylim(-vmax, vmax)

        # diagnose: last 20 points
        diag_txt = ''
        if len(er) >= 5:
            recent_err = er[-10:]
            recent_vel = v[-10:]
            same_sign  = sum(1 for e, vv in zip(recent_err, recent_vel)
                             if abs(vv) > 2 and ((e > 0) == (vv > 0)))
            opp_sign   = sum(1 for e, vv in zip(recent_err, recent_vel)
                             if abs(vv) > 2 and abs(e) > 1 and ((e > 0) != (vv > 0)))
            if opp_sign > same_sign and opp_sign > 3:
                ax_diag.set_facecolor('#1a0000')
                diag_txt = '⚠  INVERTED — vel going AWAY from target (encoder or motor direction wrong)'
            elif same_sign > opp_sign and same_sign > 3:
                ax_diag.set_facecolor('#001a00')
                diag_txt = '✓  TRACKING — vel going TOWARD target'
            else:
                ax_diag.set_facecolor('#0d0d0d')

        txt_status.set_text(f'{status[0]}\n{diag_txt}')
        return line_pos, line_tgt, line_vel, line_err, line_vel2, txt_status

    ani = animation.FuncAnimation(fig, update, interval=60, blit=False)
    plt.show()

# ── Main ──────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    ap = argparse.ArgumentParser(description='Robot real-time logger')
    ap.add_argument('port', help='Serial port, e.g. COM3')
    args = ap.parse_args()
    run_plot(args.port)
