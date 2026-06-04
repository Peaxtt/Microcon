# 1-DOF Robot Simulator GUI — Full Context

> สรุปทุกอย่างที่คุยกันใน session นี้ สำหรับเอาไปคุยต่อ session อื่น
> Last updated: 2026-06-02 (session B — UI upgrade)

---

## 1. ภาพรวมโปรเจกต์

หุ่น 1-DOF pick-and-place (FRA263/264 lab) — แขนหมุนหยิบ rod จากรูหนึ่งไปวางอีกรู
บนจานหมุนที่มี **72 รู ทุก 5°** (รวม 360°)

- **Firmware**: STM32G474RE (`Core/Src/main.c` ~2900 บรรทัด, monolithic)
- **Control**: Due's cascade PID (`Core/Src/control.c`) — Position PID @500Hz → Velocity PID @1kHz
- **PC interface เดิม**: BaseSystem v1.1 (Modbus RTU, LPUART1, 19200 8E1, Slave ID=21)
- **GUI ใหม่ (session นี้)**: Web app real-time + offline simulator แทน/เสริม BaseSystem

**เป้าหมาย GUI**: ตรวจสอบว่า "สั่ง pick/place แล้ว path ถูกต้องมั้ย" แบบ offline (ไม่มีหุ่นจริง)
และดูกราฟ control response เพื่อจูน PID

---

## 2. สถาปัตยกรรม GUI (MVC)

```
python_gui/
├── server.py              ← Python backend: WebSocket bridge + Modbus RTU + SimGenerator
├── main.py                ← (เดิม) PyQt5 tuner — ยังอยู่ ไม่ได้แตะ
├── requirements.txt       ← เพิ่ม websockets
└── sim_gui/               ← Web frontend (vanilla JS ES Modules, ไม่มี build step)
    ├── index.html         ← layout shell 3 คอลัมน์
    ├── style.css          ← OKLCH design tokens (สี orange จาก BaseSystem)
    ├── app.js             ← entry point: สร้าง + wire ทุก component
    ├── PRODUCT.md         ← impeccable design brief
    ├── model/
    │   ├── robot.js       ← RobotState: pos, vel, mode, reeds + history ring buffer
    │   ├── sequence.js    ← SequenceModel: pairs, hole↔deg conversion, random gen
    │   └── simulator.js   ← SimulatorModel: wraps Web Worker
    ├── view/
    │   ├── disk.js        ← DiskView: SVG จาน 72 รู + arm animation 60fps
    │   ├── gripper.js     ← GripperView: pneumatic gripper CSS animation
    │   ├── graphs.js      ← GraphView: Chart.js streaming (pos/err/vel/pwm)
    │   ├── stats.js       ← StatsView: position/speed/mode/gripper text
    │   └── controls.js    ← ControlsView: buttons, sliders, pair table, tabs
    ├── controller/
    │   ├── ws.js          ← WebSocketController: connect + auto-reconnect
    │   └── events.js      ← EventController: wiring (user input → cmd → model)
    └── workers/
        └── sim.worker.js  ← offline cascade PID + motor model (ไม่ block UI)
```

### Data flow
```
Browser button → ControlsView (event) → EventController → WebSocketController
    → ws://localhost:8765 → server.py
        ├─ SIM mode:  SimGenerator (คำนวณ fake motion)
        └─ HW mode:   Modbus FC06 write → STM32

server.py → telemetry JSON @50Hz → WebSocketController → RobotState.applyPacket()
    → subscribers: DiskView, GripperView, StatsView, GraphView, ControlsView
```

---

## 3. วิธีรัน

```bash
pip install websockets       # ครั้งแรกเท่านั้น

# Simulation mode (ไม่มีหุ่น — ปุ่มควบคุม sim arm โดยตรง)
python python_gui/server.py

# Hardware mode (ต่อ STM32 จริง — ปิด BaseSystem ก่อน เพราะ serial ใช้ได้ทีละ process)
python python_gui/server.py --port COM4

# เปิด browser
http://localhost:8080
```

server.py เปิด **2 port**: WebSocket 8765 + HTTP static 8080 (ต้อง serve ผ่าน HTTP
เพราะ ES Modules + Web Worker โหลดจาก file:// ไม่ได้)

---

## 4. ⭐ Hole Index Logic (สำคัญที่สุด — แก้หลายรอบกว่าจะถูก)

### Convention ที่ถูกต้อง (ยืนยันจาก user)

```
จาน 72 รู, ทุกรู = 5°
hole 0 = home = 0° (ด้านบน, 12 นาฬิกา)
hole 1, 2, 3...  = ทวนเข็ม (CCW) ไปทางซ้าย
hole 71, 70, 69... = ตามเข็ม (CW) ไปทางขวา

hole index N (signed):
  |N|      = ตำแหน่งสัมบูรณ์บนจาน → target_deg = |N| × 5°
  sign(N)  = บังคับทิศ motor บนหุ่นจริง:  + = CCW,  - = CW
```

### ตัวอย่างที่ user ใช้เช็ค
> อยู่ hole 15 (75°), สั่ง place = **-69**
> → hole 69 = 345° (|−69| × 5)
> → บังคับ CW
> → motor หมุน CW 90° (15→0→71→70→69 = 18 holes × 5°)

### ⚠️ ประวัติบั๊ก (อย่าทำซ้ำ)
- **ผิดครั้งแรก**: ใช้ `(72 + raw)` wrap → -69 กลายเป็น hole 3 (ผิด!)
- **ผิดครั้งสอง**: place ใช้ relative `(pickHole + 72 + place)` → ผิด
- **ถูก**: hole = `|raw|` เสมอ (absolute), เครื่องหมายแค่บอกทิศ

---

## 5. การแยก SIM vs HARDWARE เรื่องทิศทาง

### Firmware (หุ่นจริง) — `start_move_hole(int16_t raw)` ใน main.c
บังคับทิศตามเครื่องหมายจริง เพราะ encoder เป็น cumulative:
```c
hole = |raw|;  target_abs = hole × 5° + home_proximity_offset_deg;
pos_mod = fmodf(pos_cumul, 360);
if (raw >= 0)  disp = ((target_abs - pos_mod) + 360) mod 360;     // CCW forward
else           disp = -((pos_mod - target_abs) + 360) mod 360;    // CW backward
start_move_deg(pos_cumul + disp);
```
ใช้ใน: `SEQ_MB_GOING_PICK` (step 4), `SEQ_MB_GOING_PLACE` (step 0), `STATE_AUTO` P2P (reg 0x24)

### Sim (GUI) — `_hole_target()` + tick() ใน server.py
**ไม่บังคับทิศ** — แค่ไปตำแหน่งให้ถูกด้วยทางสั้น (shortest arc) เพราะ user
ต้องการเช็คแค่ "ไปอยู่รูถูกมั้ย":
```python
def _hole_target(self, raw):
    return abs(raw) * 5.0       # absolute hole position

# tick() มี 2 mode:
#   _remaining is not None → JOG (relative, ตามเครื่องหมาย)
#   _remaining is None     → ABSOLUTE (shortest arc บนวงกลม)
err = (target - pos + 180) % 360 - 180   # shortest signed error
```

### Disk rendering — `disk.js` (แก้ทิศหลายรอบ)
```js
// hole i วาดที่มุม (-i*5 - 90)  → index เพิ่ม = CCW (ซ้าย) ในระบบ SVG y-down
const angleDeg = -i * 5 - 90;
// arm หมุน rotate(-displayAngle) → pos_deg N ชี้ตรง hole N
```

---

## 6. Pick & Place Sequence Logic

### Firmware state machine (`SEQ_MB_*` ใน main.c)
```
SEQ_MB_GOING_PICK (มี pre-check ก่อนเดิน — สำคัญ!):
  step 0: OPEN gripper
  step 1: รอ !reed_grip (SEQ_OK = dwell+timeout)
  step 2: UP cylinder
  step 3: รอ reed_up
  step 4: start_move_hole(pick_raw)  [break — race fix]
  step 5: รอ ctrl_settled → DOWN → PICKING

SEQ_MB_PICKING:
  step 0: รอ reed_down → CLOSE gripper
  step 1: รอ reed_grip → UP cylinder
  step 2: รอ reed_up   → GOING_PLACE

SEQ_MB_GOING_PLACE (ไม่ต้อง pre-check — UP+CLOSED จาก PICKING แล้ว):
  step 0: start_move_hole(place_raw) [break — race fix]
  step 1: รอ ctrl_settled → DOWN → PLACING

SEQ_MB_PLACING:
  step 0: รอ reed_down  → OPEN gripper
  step 1: รอ !reed_grip → UP cylinder
  step 2: รอ reed_up    → pair_idx++ → GOING_PICK (next) หรือ IDLE
```

**เหตุผล pre-check ก่อนหยิบ**: ถ้า cylinder ค้าง DOWN หรือ gripper ปิดจาก operation ก่อน
แล้วเดินไปหยิบใหม่ → จะชน rod อื่นและพังหมด ต้องแก้ state ให้ถูกก่อนเดินทุกครั้ง

**race fix**: ต้อง `break` หลัง start_move ไม่ให้เช็ค `ctrl_settled` ใน tick เดียวกัน
(ไม่งั้นมันจะ settled ทันทีเพราะ trajectory ยังไม่เริ่ม)

### SEQ_OK macro
```c
#define SEQ_OK(reed) ((seq_mb_timer >= seq_dwell_t && (reed)) || seq_mb_timer >= seq_timeout_t)
// รอ reed จริง แต่มี timeout กันค้าง
```

### Sim sequence (`cmd_sequence` ใน server.py)
ใช้ time-based queue เลียนแบบ: OPEN+UP → goto pick → DOWN→CLOSE→UP → goto place → DOWN→OPEN→UP
ต่อเนื่องทุก pair จนครบ

---

## 7. ⚠️ ยังไม่ได้ทำ: Path Safety (งานต่อไป)

User ชี้ว่า: **"หยิบมาแล้วไม่ควรวิ่งไปจุดอื่นที่จะต้องถูกหยิบอีก เพราะจะชนและพัง"**

BaseSystem กำหนด path มาให้แล้วเพื่อความปลอดภัย — sim ต้อง:
1. เดินตาม path ที่กำหนดเป๊ะ (ไม่ใช่ random ที่อาจชน)
2. ตรวจว่า move ระหว่าง pick→place ไม่ผ่านรูที่ยังมี rod อยู่
3. `generateRandom()` ใน sequence.js ปัจจุบันยังไม่เช็คการชน — ต้องปรับ

**สถานะ**: ยังไม่ได้แก้ — เป็นงานถัดไป

---

## 8. Reed Switch Simulation (dummy mode)

- Reed มี delay ก่อนเปลี่ยนตาม actuator (เลียนแบบ reed_dummy ใน firmware)
- **แต่ละ reed อิสระกัน**: กด DOWN → reed_up=false ทันที, reed_down=true หลัง delay
  (ไม่ coupling กัน — เคยมีบั๊กว่าตามๆ กัน แก้แล้ว)
- ปุ่ม UP/DOWN/OPEN/CLOSE เปลี่ยนสีส้มตาม reed state จริง (หลัง delay ครบ)
- **delay = `reed_delay`** ใน SimGenerator = ตรงกับ `seq_dwell_ms` ของ firmware (default 200ms)
  ปรับผ่าน Tuning panel slider "seq_dwell (ms)" → Apply

---

## 9. Gripper Visual (`gripper.js`)

Pneumatic gripper เคลื่อนไหวจริงด้วย CSS transitions (0.38s cubic-bezier):

**DOM structure (top→down):**
```
.gv-housing      ← cylinder body (dark steel gradient) + 2 pneumatic ports (::before/::after)
.gv-rod          ← piston rod (chrome gradient, visible stroke)
.gv-assembly     ← เลื่อนลง translateY(32px) เมื่อ reed_down
  .gv-flange     ← mounting flange (slightly wider than base)
  .gv-base       ← base plate + centre piston dot
  .gv-jaws
    .gv-jaw-left   translateX(-10px open, +9px closed)
    .gv-jaw-right  translateX(+10px open, -9px closed)
```

- `reed_down` → `.gv-assembly.is-down` (translateY)
- `reed_grip`  → `.gv-jaw.is-closed` (translateX inward + background เปลี่ยนส้ม = grip force)
- Jaw มี `::before` grip serration texture + `::after` tip cap
- Reed dots UP/DOWN/GRIP เขียว glow เมื่อ triggered
- ไม่มี rdot-* ใน index.html แล้ว — gripper-container เป็น single source ของ reed indicator

---

## 10. Tuning Panel (control team — minimal)

9 parameters ปรับผ่าน slider + Apply (MANUAL tab):

| Param | Default | Modbus reg | หมายเหตุ |
|-------|---------|-----------|---------|
| kp_vel | 10.0 | 0x0C | velocity P |
| ki_vel | 0.01 | 0x0D | velocity I |
| kp_pos | 2.0 | 0x3D | position P |
| kd_pos | 0.2 | 0x3E | position D (damping) |
| v_max | 3.0 | 0x39 | trajectory speed (rad/s) |
| a_max | 12.56 | 0x3B | trajectory accel (rad/s²) |
| max_speed | 0.4 | 0x34 | PWM cap (0–1) |
| seq_dwell_ms | 200 | (Live Expr / sim reed_delay) | delay ก่อนอ่าน reed |
| pos_deadband° | 2.0 | (Live Expr) | settled threshold |

ทุก gain ส่งเป็น × 100 ผ่าน Modbus FC06 (ตาม convention firmware)

---

## 11. Modbus Register Map (ที่ GUI ใช้)

### เขียน (browser → firmware)
| Reg | ความหมาย |
|-----|---------|
| 0x01 | Mode cmd (1=HOME, 2=MANUAL/seq trigger, 4=AUTO, 8=SET_HOME, 16=TEST) |
| 0x02 | Manual actuator (1=DOWN, 0=UP, 2=OPEN, 4=CLOSE) |
| 0x03 | Pick/place actuator seq (1=pick, 2=place) |
| 0x04 | Sequence autostart flag (bit0) |
| 0x05 | Jog degrees (signed) |
| 0x12+ | Pick/place pairs (pick=even, place=odd, signed hole index) |
| 0x22 | จำนวน pairs |
| 0x24 | P2P target (signed hole index) |
| 0x25 | STOP (bit0) |
| 0x0C-0x0F, 0x34, 0x39, 0x3B, 0x3D-0x3F | gains |

### อ่าน (firmware → browser, FC03 block 0x23–0x3A)
| Reg | ความหมาย |
|-----|---------|
| 0x23 | homed |
| 0x26 | reed bits (bit0=up, bit1=down, bit2=grip) |
| 0x27 | motion status (0x01=homing, 0x02=pick, 0x04=place, 0x08=auto) |
| 0x28 | position × 10 (deg, wrapped 0-360) |
| 0x29 | velocity × 10 (deg/s) |
| 0x2F | **current_state enum** (เพิ่ม session นี้ A2) |
| 0x30 | accel × 10 |
| 0x31 | ESTOP |
| 0x3A | current (mA) |

State enum: INIT=0, IDLE=1, HOMING_FAST=2, HOMING_BACKOFF=3, HOMING_SLOW=4,
MANUAL=5, MANUAL_MB=6, AUTO=7, SEQUENCE=8, TEST=9, EMER=10

---

## 12. Firmware Changes (session นี้, ยังไม่ commit หลัง v1.1-stable)

| # | สิ่งที่แก้ |
|---|----------|
| A1 | ลบ `homed=0` ตอน EMER exit (ไม่บังคับ re-home) |
| A2 | เพิ่ม `reg[0x2F] = current_state` ให้ GUI/BaseSystem อ่าน state |
| A3 | เพิ่ม `home_proximity_offset_deg` — calibrate ตำแหน่ง 0° จริง (sensor ไม่ตรงรู) |
| A4 | `finish_homing()` → วิ่งไป offset อัตโนมัติ + re-zero encoder (`homing_final_zero_pending`) |
| — | `start_move_hole()` helper บังคับทิศ CCW/CW จาก signed hole |
| — | SEQ_MB_GOING_PICK เพิ่ม pre-check (OPEN+UP ก่อนเดิน, sub-steps 0-5) |
| — | SEQ_MB_GOING_PLACE มาตรฐาน sub-steps 0-1 |

**Git**: `v1.1-stable` tag = commit 097731a (firmware fixes + web GUI baseline)
หลังจากนั้นมี firmware + sim changes ยังไม่ commit

---

## 13. State Animation (disk arm + tower lights)

- Arm interpolate 60fps ด้วย `requestAnimationFrame` + lerp (LERP_ALPHA=0.18)
- หา shortest arc: normalize diff เป็น [-180, 180] ก่อน lerp (arm ไม่หมุนรอบโลก)
- Arm มี glow layer (stroke กว้าง opacity 0.18) + centre hub dot
- Inner guide ring เส้นประ subtle แสดงพื้นที่ arm sweep
- Sim มี state names: "IDLE/AUTO/PICK n/N/PLACE n/N (sim)"
- Stats orange เฉพาะตอน moving (vel > 0.5 dps หรือ state เป็น motion state)

### Disk Hole Interaction (เพิ่ม session B)
- **Hover** hole → floating tooltip dark "Hole N · N° from home" (position: fixed, ไม่ clip)
- **Click** hole → selection ring ส้มปรากฏ + `#disk-hint` อัปเดต "Hole N (N°) selected"
- **`disk.selectedHole`** (public property) → `EventController` ส่งพ่วง `hole` index ไปกับ `set_home` command
- Hit area ใหญ่กว่า dot 2.8× เพื่อง่ายต่อการคลิก

---

## 14. งานที่เหลือ / Next Steps

1. **Path safety** (ข้อ 7) — เดินตาม path ปลอดภัย ไม่ชน rod
2. **Preview Path** บน disk — แสดงลูกศรลำดับการเคลื่อนที่ + เลขกำกับ
3. **Random generation** ปรับให้ไม่สร้าง path ที่ชน
4. **BaseSystem v1.2** — branch แยก (baud 230400, MODBUS_REG_COUNT 256)
5. Build + Flash firmware แล้วเทสจริงกับ board
6. **START SEQUENCE two-step confirm** — ปุ่มต้อง arm ก่อน (ป้องกัน misclick ตอนหุ่นอยู่จริง)
7. **TEST MODE tab** — ยังเป็น placeholder "send via BaseSystem v1.1" ต้องใส่เนื้อหา

---

## 15. Design System (`style.css` + `PRODUCT.md`)

โปรเจกต์ใช้ **Impeccable** design framework (plugin `/impeccable:impeccable`) ร่วมกับ `PRODUCT.md`

### Tokens (OKLCH ทั้งหมด)
| Token | Value | ใช้ทำอะไร |
|-------|-------|----------|
| `--primary` | `oklch(0.68 0.19 35)` | orange จาก BaseSystem — active/moving/commanded เท่านั้น |
| `--ink` | `oklch(0.17 0.015 200)` | body text |
| `--muted` | `oklch(0.47 0.008 200)` | label text (ผ่าน WCAG AA 4.5:1) |
| `--bg` | `oklch(1 0 0)` | card background |
| `--surface` | `oklch(0.975 0.005 200)` | app shell background |

### กฎสำคัญ (อย่า break)
- **"One orange"** — `--primary` ใช้เฉพาะ active/moving state ห้ามใส่ element ที่ idle
- `.stat-value` default = `var(--ink)`, เปลี่ยนเป็นส้มเฉพาะตอน `.is-active` (JS toggle)
- ห้าม `text-transform: uppercase` + `letter-spacing` บน `.card-title` (eyebrow anti-pattern)
- ห้าม `!important` บน button active state (ใช้ specificity `.btn.btn-active` แทน)
- Critique ล่าสุด (2026-06-02): score **22/40** — snapshot ใน `.impeccable/critique/`

---

## 16. Session B — UI Upgrade (2026-06-02)

### สิ่งที่ทำใน session นี้

**Impeccable critique** รัน `/impeccable:impeccable critique` บน index.html
- Score 22/40 — ปัญหาหลัก: dead reed dots, orange ทุกเวลา, flat typography, eyebrow ทุก card

**แก้ทั้งหมด 6 ไฟล์:**

| ไฟล์ | สิ่งที่เปลี่ยน |
|------|--------------|
| `view/disk.js` | Hover tooltip + click-to-select home hole + arm glow + inner guide ring + centre hub |
| `view/gripper.js` | Industrial redesign: housing, piston rod, mounting flange, serrated jaws, grip-force color |
| `style.css` | `--muted` 0.52→0.47 (WCAG AA), `.stat-value` default ink, `.is-active` orange, card-title cleanup, `.btn.btn-active` ลบ `!important` |
| `index.html` | ลบ dead `rdot-up/down/grip` elements, card labels title case (Log/Stats/Workspace/Graphs) |
| `view/stats.js` | Toggle `.is-active` บน pos/speed/accel เมื่อ robot เคลื่อนที่จริง |
| `controller/events.js` | `set_home` command แนบ `disk.selectedHole` (hole index ที่ user คลิกเลือก) |

### วิธีใช้ฟีเจอร์ใหม่ (hole select for SET HOME)
1. ดู disk workspace — ชี้ mouse ที่รูไหนก็ได้ → tooltip แสดง "Hole N · N° from home"
2. คลิก hole ที่ต้องการ → ring ส้มปรากฏ + hint text เปลี่ยนสี
3. กด **SET HOME** → server ได้รับ `{type:"set_home", hole: N}`

---

## 17. หลักการที่ user เน้นย้ำ (อย่าลืม)

- **"อยากรู้แค่ตรงมั้ย"** — sim ไม่ต้องเป๊ะเรื่อง control แค่ตำแหน่งถูก path ถูก
- **"ไม่ต้องเอาคอนโทรลเข้ามาก็ได้"** — sim ไม่ต้องจำลอง PID เป๊ะ ใช้ lerp เรียบๆ พอ
- **Path ต้องปลอดภัย** — หยิบแล้วห้ามเดินชน rod อื่นที่รออยู่
- **BaseSystem เป็นคนกำหนด path มาให้** — เพื่อความปลอดภัย ต้องเดินตาม
- **Control = ของ Due เท่านั้น** (control.c) — ห้ามเอาของคนอื่นมาปน
- **Modbus reference = Korn** แต่ดูแค่ Modbus ไม่เอา logic อื่นมาปน
