# PCB I/O Test Guide — pcb_io_test

> ทดสอบ pin assignment ใหม่ก่อน commit IOC  
> ใช้เวลาประมาณ **10–15 นาที**

---

## เตรียมอุปกรณ์

- Multimeter (วัด DC voltage)
- สาย USB ST-Link เชื่อมต่อกับ NUCLEO
- Serial terminal: **115200 8N1** (PuTTY / Tera Term / Arduino Serial Monitor)
- RP2040-Zero (เสียบ USB ไว้ก่อน แต่ยังไม่ต้องเสียบ connector บอร์ด)

---

## Flash firmware

1. เปิด STM32CubeIDE
2. **Ctrl+B** — build
3. **F11** — flash
4. เปิด terminal → เลือก COM port ของ ST-Link → **115200 8N1**

> ถ้า terminal ขึ้น `╔═══ PCB I/O VALIDATOR ═══╗` แปลว่าพร้อมแล้ว

---

## Phase 1 — Output Blink

**สถานะ connector:** เสียบหรือไม่ก็ได้ (ไม่กระทบ phase นี้)

Terminal จะบอกทีละพิน แล้ว **รอให้กด Enter** ก่อนบลิงก์:
```
  (1/10) PC7  MOTOR_DIR
  วาง probe ที่พินนี้แล้ว >>> กด Enter เพื่อดำเนินการต่อ ...
```

**สิ่งที่ต้องทำ:** วาง probe multimeter ที่พินนั้น → กด Enter → ดูเข็มสลับ

| ผลที่ถูกต้อง | แปลว่า |
|---|---|
| สลับ 3.3V ↔ 0V | ✅ พินนั้นต่อถึงบอร์ดปกติ |
| ค้างอยู่ที่ 3.3V ตลอด | ❌ VCC short |
| ค้างอยู่ที่ 0V ตลอด | ❌ GND short หรือ pin ไม่ถึง |

**พินที่ต้องวัด (ตามลำดับที่ terminal แสดง):**

| Pin | Label | Connector |
|-----|-------|-----------|
| PD2 | RESET_LED | ตาม PCB |
| PC11 | EMER_OUT | ตาม PCB |
| PB12 | POWER_LATCH | ตาม PCB |
| PB2 | GRIPPER | ตาม PCB |
| PB1 | PNEUMATIC | ตาม PCB |
| PB15 | TOWER_G | ตาม PCB |
| PB14 | TOWER_Y | ตาม PCB |
| PB13 | TOWER_R | ตาม PCB |
| PC7 | MOTOR_DIR | ตาม PCB |
| PC9 | PWM_test | ตาม PCB |

---

## Phase 2 — Input Pull

> ⚠️ **ถอด connector ทั้งหมดออกก่อน** แล้วกด Enter

Terminal จะรอให้กด Enter ก่อนอ่านค่า — ถอดให้เสร็จก่อนค่อยกด

**ผลที่ถูกต้อง:**

| กลุ่ม | Pin | ผลที่ expect |
|-------|-----|-------------|
| PULLUP (ลอยอยู่) | PA15 ESTOP | HIGH |
| | PB7 POWER_BTN | HIGH |
| | PC13 RESET_BTN | HIGH |
| | PC2 MODE | HIGH |
| | PC3 HOME_SENSOR | HIGH |
| PULLDOWN (ลอยอยู่) | PB0 REED_GRIP | LOW |
| | PA4 REED_DOWN | LOW |
| | PA7 REED_UP | LOW |

ถ้าขึ้น `FAIL` ให้วัด multimeter ที่พินนั้นเทียบ GND ขณะ firmware รัน  
- PULLUP แล้วอ่านได้ LOW → GND short บน PCB  
- PULLDOWN แล้วอ่านได้ HIGH → VCC short บน PCB

---

## Phase 3 — ADC (PA0)

Terminal รายงานแรงดันที่ PA0 (CURRENT_SENSOR)

| ผล | แปลว่า |
|----|--------|
| `1.3V – 2.0V` | ✅ PASS — sensor ต่อถูกและพินสะอาด |
| `WARN — voltage readable` | ⚠️ readable แต่ sensor อาจไม่ได้เสียบ (ไม่ถือว่า fail) |
| `FAIL — stuck HIGH >3.0V` | ❌ VCC short บน PA0 |
| `FAIL — stuck LOW <0.1V` | ❌ GND short บน PA0 |

---

## Phase 4 — RP2040 Alive

> **เสียบ connector RP2040 ให้เรียบร้อยก่อน** แล้วกด Enter

| ผล | แปลว่า |
|----|--------|
| `PB9 = HIGH → PASS` | ✅ RP2040 พร้อม, USART3 idle line ถูกต้อง |
| `PB9 = LOW  → FAIL` | ❌ RP2040 ไม่ตอบ — เช็คสาย/ไฟ |

---

## สรุปผล

Terminal แสดงผลท้าย:
```
╔═══════════════════════════════════════╗
║            TEST SUMMARY               ║
║  PASS: xx   FAIL: xx                  ║
║  ALL PASS — safe to commit IOC        ║
╚═══════════════════════════════════════╝
```

**ถ้า ALL PASS** → ไป commit IOC ได้เลย  
**ถ้ามี FAIL** → ดู phase ที่ fail และวัดพินนั้นด้วย multimeter ก่อน

---

## หลังเทสเสร็จ

firmware จะค้างอยู่ที่ `while(1)` — **ต้อง reflash ใหม่เพื่อกลับ firmware ปกติ**

ใน `Core/Src/main.c` บรรทัด ~491:
```c
// pcb_io_test(&hlpuart1);   ← comment ออก
// pin_short_test(&hlpuart1); ← ปล่อยไว้ comment
```
แล้ว **Ctrl+B → F11** อีกรอบ
