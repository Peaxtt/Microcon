"""
server.py — WebSocket bridge between STM32 firmware and sim_gui browser frontend.

Architecture:
  Serial (Modbus RTU 19200 8E1) ──► ModbusReader thread ──► broadcast_queue
  browser (WS cmd) ──────────────► ModbusWriter thread ──► Serial

  asyncio WebSocket server listens on ws://localhost:8765
  Broadcasts robot state JSON at ~50 Hz to all connected browsers.

Usage:
  python server.py                       # simulation mode (no hardware)
  python server.py --port COM4           # connect to real STM32
  python server.py --port COM4 --baud 19200
"""

import asyncio, struct, time, threading, json, math, argparse
from collections import deque

# ── Constants ──────────────────────────────────────────────────────────────────
SLAVE_ID    = 21
WS_PORT     = 8765
POLL_HZ     = 50          # Modbus read rate (20 ms per cycle)
TELEM_HZ    = 50          # WebSocket broadcast rate

# Modbus FC03: read registers from FEEDBACK_START, count = FEEDBACK_COUNT
FEEDBACK_START = 0x23     # homed
FEEDBACK_COUNT = 24       # 0x23 → 0x3A  (covers all v1.1 status regs + 0x2F)

# Individual register offsets inside the feedback block
OFF_HOMED   = 0x23 - 0x23   # [0]  homed flag
OFF_TARGET  = 0x24 - 0x23   # [1]  P2P target hole index (write reg, but readable)
OFF_REED    = 0x26 - 0x23   # [3]  reed switches bits
OFF_STATUS  = 0x27 - 0x23   # [4]  motion status bits
OFF_POS     = 0x28 - 0x23   # [5]  position × 10 (deg, wrapped 0-360)
OFF_VEL     = 0x29 - 0x23   # [6]  velocity × 10 (deg/s)
OFF_ACCEL   = 0x30 - 0x23   # [13] acceleration × 10
OFF_ESTOP   = 0x31 - 0x23   # [14] ESTOP active
OFF_DIGIO   = 0x32 - 0x23   # [15] digital IO bits
OFF_STATE   = 0x2F - 0x23   # [12] current_state enum
OFF_CURRENT = 0x3A - 0x23   # [23] current mA

# State enum (mirrors RobotState_t in firmware)
STATE_NAMES = {
    0: "INIT", 1: "IDLE", 2: "HOMING_FAST", 3: "HOMING_BACKOFF",
    4: "HOMING_SLOW", 5: "MANUAL", 6: "MANUAL_MB", 7: "AUTO",
    8: "SEQUENCE", 9: "TEST", 10: "EMER"
}

# ── Modbus helpers (identical to main.py) ─────────────────────────────────────
def _crc16(data: bytes) -> bytes:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return struct.pack('<H', crc)

def _mb_write(reg: int, val: int) -> bytes:
    """Build Modbus FC06 Write Single Register frame."""
    pkt = struct.pack('>BBHH', SLAVE_ID, 6, reg, int(val) & 0xFFFF)
    return pkt + _crc16(pkt)

def _mb_read_req(reg: int, count: int) -> bytes:
    """Build Modbus FC03 Read Holding Registers request frame."""
    pkt = struct.pack('>BBHH', SLAVE_ID, 3, reg, count)
    return pkt + _crc16(pkt)

def _mb_read_parse(raw: bytes, count: int):
    """
    Parse FC03 response. Returns list of uint16 register values or None on error.
    Expected: [slave, 03, byte_count, data×count×2, crc×2]
    """
    expected = 3 + count * 2 + 2
    if len(raw) < expected:
        return None
    if raw[0] != SLAVE_ID or raw[1] != 0x03 or raw[2] != count * 2:
        return None
    regs = []
    for i in range(count):
        regs.append(struct.unpack('>H', raw[3 + i*2: 5 + i*2])[0])
    return regs

# ── Shared robot state ─────────────────────────────────────────────────────────
class RobotState:
    """Thread-safe snapshot of last-known robot state."""
    def __init__(self):
        self._lock = threading.Lock()
        self.pos_deg     = 0.0   # position (degrees, wrapped 0-360 for display)
        self.pos_cumul   = 0.0   # cumulative position (for arm angle, unbounded)
        self.vel_dps     = 0.0   # velocity deg/s
        self.accel_dps2  = 0.0   # acceleration deg/s²
        self.current_mA  = 0.0
        self.homed       = False
        self.state_id    = 0
        self.state_name  = "INIT"
        self.reed_up     = False
        self.reed_down   = False
        self.reed_grip   = False
        self.estop       = False
        self.status_bits = 0     # reg[0x27]: 0x01=homing, 0x02=pick, 0x04=place, 0x08=auto
        self.connected   = False
        self.sim_mode    = True

    def update_from_regs(self, regs: list):
        """Apply FC03 feedback block [0x23..0x3A] to state."""
        with self._lock:
            self.homed      = bool(regs[OFF_HOMED])
            self.reed_up    = bool(regs[OFF_REED] & 1)
            self.reed_down  = bool(regs[OFF_REED] & 2)
            self.reed_grip  = bool(regs[OFF_REED] & 4)
            self.status_bits = regs[OFF_STATUS]
            # Position: stored as uint16 (signed in firmware cast to int16 before ×10)
            raw_pos = struct.unpack('>h', struct.pack('>H', regs[OFF_POS]))[0]
            self.pos_deg    = raw_pos / 10.0
            self.pos_cumul  = raw_pos / 10.0   # wrapped; real cumul only via telemetry
            raw_vel = struct.unpack('>h', struct.pack('>H', regs[OFF_VEL]))[0]
            self.vel_dps    = raw_vel / 10.0
            raw_acc = struct.unpack('>h', struct.pack('>H', regs[OFF_ACCEL]))[0]
            self.accel_dps2 = raw_acc / 10.0
            self.estop      = bool(regs[OFF_ESTOP])
            raw_state       = regs[OFF_STATE]
            self.state_id   = raw_state
            self.state_name = STATE_NAMES.get(raw_state, f"UNK{raw_state}")
            self.current_mA = regs[OFF_CURRENT]
            self.connected  = True
            self.sim_mode   = False

    def to_json(self) -> str:
        """Serialize to JSON for WebSocket broadcast."""
        with self._lock:
            return json.dumps({
                "pos_deg":    round(self.pos_deg, 2),
                "pos_cumul":  round(self.pos_cumul, 2),
                "vel_dps":    round(self.vel_dps, 2),
                "vel_rads":   round(self.vel_dps * math.pi / 180.0, 4),
                "accel_dps2": round(self.accel_dps2, 2),
                "current_mA": round(self.current_mA, 1),
                "homed":      self.homed,
                "state_id":   self.state_id,
                "state_name": self.state_name,
                "reed_up":    self.reed_up,
                "reed_down":  self.reed_down,
                "reed_grip":  self.reed_grip,
                "estop":      self.estop,
                "status_bits": self.status_bits,
                "connected":  self.connected,
                "sim_mode":   self.sim_mode,
                "ts":         time.monotonic(),
            })

# ── Simulation state generator (offline mode) ─────────────────────────────────
class SimGenerator:
    """
    Generates fake robot telemetry for offline preview.
    Simulates a simple sine-wave oscillation so the disk arm moves visibly.
    """
    def __init__(self, state: RobotState):
        self._state = state
        self._t = 0.0
        self._pos = 0.0
        self._vel = 0.0

    def tick(self, dt: float):
        """Advance simulation by dt seconds, update shared state."""
        self._t += dt
        # Simple demo: arm sweeps between 0° and 180° at 0.25 Hz
        target = 90.0 * math.sin(2 * math.pi * 0.25 * self._t)
        err = target - self._pos
        self._vel += (err * 5.0 - self._vel * 2.0) * dt
        self._pos += self._vel * dt

        with self._state._lock:
            self._state.pos_deg    = self._pos % 360
            self._state.pos_cumul  = self._pos
            self._state.vel_dps    = self._vel
            self._state.accel_dps2 = 0.0
            self._state.state_name = "IDLE (sim)"
            self._state.state_id   = 1
            self._state.connected  = False
            self._state.sim_mode   = True

# ── Modbus reader thread (hardware mode) ──────────────────────────────────────
class ModbusReaderThread(threading.Thread):
    """
    Polls STM32 feedback registers via FC03 at POLL_HZ.
    Runs in background thread; updates shared RobotState.
    """
    def __init__(self, port: str, baud: int, state: RobotState, cmd_queue: deque):
        super().__init__(daemon=True)
        self._port      = port
        self._baud      = baud
        self._state     = state
        self._cmd_queue = cmd_queue   # deque of raw Modbus frames to send
        self._running   = True

    def stop(self):
        self._running = False

    def run(self):
        import serial as _serial
        try:
            ser = _serial.Serial(
                self._port, self._baud,
                bytesize=8, parity='E', stopbits=1,
                timeout=0.05
            )
        except Exception as e:
            print(f"[serial] open failed: {e}")
            return

        req = _mb_read_req(FEEDBACK_START, FEEDBACK_COUNT)
        interval = 1.0 / POLL_HZ

        while self._running:
            t0 = time.monotonic()

            # 1. Send any pending commands first
            while self._cmd_queue:
                frame = self._cmd_queue.popleft()
                ser.write(frame)
                ser.flush()
                time.sleep(0.003)   # brief gap before next frame
                ser.reset_input_buffer()

            # 2. Read feedback registers
            ser.reset_input_buffer()
            ser.write(req)
            resp = ser.read(3 + FEEDBACK_COUNT * 2 + 2)
            regs = _mb_read_parse(resp, FEEDBACK_COUNT)
            if regs:
                self._state.update_from_regs(regs)
            else:
                with self._state._lock:
                    self._state.connected = False

            # 3. Sleep remainder of interval
            elapsed = time.monotonic() - t0
            remaining = interval - elapsed
            if remaining > 0:
                time.sleep(remaining)

        ser.close()

# ── WebSocket server ───────────────────────────────────────────────────────────
async def ws_handler(websocket, state: RobotState, cmd_queue: deque):
    """
    Handles one browser WebSocket connection.
    Broadcasts state JSON at TELEM_HZ. Receives command JSON from browser.
    """
    import websockets
    print(f"[ws] client connected: {websocket.remote_address}")
    try:
        async def sender():
            while True:
                await websocket.send(state.to_json())
                await asyncio.sleep(1.0 / TELEM_HZ)

        async def receiver():
            async for msg in websocket:
                try:
                    cmd = json.loads(msg)
                    _handle_browser_cmd(cmd, cmd_queue)
                except json.JSONDecodeError:
                    pass

        await asyncio.gather(sender(), receiver())
    except Exception:
        pass
    finally:
        print(f"[ws] client disconnected")

def _handle_browser_cmd(cmd: dict, cmd_queue: deque):
    """
    Convert browser JSON command to Modbus FC06 frame and enqueue.

    Supported command types:
      {"type": "write_reg",  "reg": 0x01, "val": 1}
      {"type": "jog",        "deg": 10}          → reg[0x05] = deg
      {"type": "home"}                            → reg[0x01] = 1
      {"type": "set_home"}                        → reg[0x01] = 8
      {"type": "stop"}                            → reg[0x25] = 1
      {"type": "mode",       "mode": "manual"}    → reg[0x01] = 2
      {"type": "mode",       "mode": "auto"}      → reg[0x01] = 4
      {"type": "p2p",        "hole": 16}          → reg[0x24] = hole
      {"type": "gains",      "kp_vel":..., ...}   → multiple writes
      {"type": "sequence",   "pairs": [...], "n": 4}
    """
    t = cmd.get("type", "")

    if t == "write_reg":
        cmd_queue.append(_mb_write(cmd["reg"], cmd["val"]))

    elif t == "jog":
        cmd_queue.append(_mb_write(0x05, int(cmd["deg"])))

    elif t == "home":
        cmd_queue.append(_mb_write(0x01, 1))

    elif t == "set_home":
        cmd_queue.append(_mb_write(0x01, 8))

    elif t == "stop":
        cmd_queue.append(_mb_write(0x25, 1))

    elif t == "mode":
        mode_map = {"manual": 2, "auto": 4, "test": 16}
        val = mode_map.get(cmd.get("mode", ""), 0)
        if val:
            cmd_queue.append(_mb_write(0x01, val))

    elif t == "p2p":
        cmd_queue.append(_mb_write(0x24, int(cmd["hole"])))

    elif t == "gains":
        # Tuning registers (scaled × 100, matching firmware modbus_app convention)
        reg_map = {
            "kp_vel": 0x0C, "ki_vel": 0x0D, "kd_vel": 0x0E,
            "kp_pos": 0x3D, "kd_pos": 0x3E, "ki_pos": 0x3F,
            "v_max":  0x39, "a_max":  0x3B,
        }
        for key, reg in reg_map.items():
            if key in cmd:
                cmd_queue.append(_mb_write(reg, int(float(cmd[key]) * 100)))
        if "max_speed" in cmd:
            # reg[0x34] = max_speed × 100
            cmd_queue.append(_mb_write(0x34, int(float(cmd["max_speed"]) * 100)))

    elif t == "sequence":
        # pairs: list of [pick_hole, place_hole], n: number of active pairs
        pairs = cmd.get("pairs", [])
        for i, (pick, place) in enumerate(pairs[:8]):
            cmd_queue.append(_mb_write(0x12 + i * 2,     int(pick)  & 0xFFFF))
            cmd_queue.append(_mb_write(0x12 + i * 2 + 1, int(place) & 0xFFFF))
        cmd_queue.append(_mb_write(0x22, len(pairs)))
        cmd_queue.append(_mb_write(0x04, 1))     # autostart
        cmd_queue.append(_mb_write(0x01, 2))     # trigger sequence

    elif t == "gripper":
        # val: "up"/"down"/"open"/"close"
        gripper_map = {"down": 1, "up": 0, "open": 2, "close": 4}
        val = gripper_map.get(cmd.get("val", ""), 0)
        cmd_queue.append(_mb_write(0x02, val))

# ── Main entry point ───────────────────────────────────────────────────────────
async def main_async(port, baud):
    import websockets

    state     = RobotState()
    cmd_queue = deque()

    if port:
        # Hardware mode: start Modbus reader thread
        reader = ModbusReaderThread(port, baud, state, cmd_queue)
        reader.start()
        print(f"[server] hardware mode: {port} @ {baud} baud")
    else:
        # Simulation mode: tick SimGenerator from asyncio loop
        sim = SimGenerator(state)
        print("[server] simulation mode (no --port given)")

        async def sim_ticker():
            dt = 1.0 / TELEM_HZ
            while True:
                sim.tick(dt)
                await asyncio.sleep(dt)

        asyncio.ensure_future(sim_ticker())

    async def handler(ws):
        await ws_handler(ws, state, cmd_queue)

    print(f"[server] WebSocket listening on ws://localhost:{WS_PORT}")
    print(f"[server] Open:  python_gui/sim_gui/index.html  in your browser")

    async with websockets.serve(handler, "localhost", WS_PORT):
        await asyncio.Future()   # run forever

def main():
    parser = argparse.ArgumentParser(description="1-DOF Robot WebSocket bridge")
    parser.add_argument("--port",  default=None,  help="Serial port (e.g. COM4)")
    parser.add_argument("--baud",  default=19200, type=int, help="Baud rate (default 19200)")
    args = parser.parse_args()

    try:
        asyncio.run(main_async(args.port, args.baud))
    except KeyboardInterrupt:
        print("\n[server] stopped.")

if __name__ == "__main__":
    main()
