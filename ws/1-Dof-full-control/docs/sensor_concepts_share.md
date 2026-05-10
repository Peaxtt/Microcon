# Sensor Reading Concepts — STM32 Embedded

แนวคิดการอ่าน Encoder และ Current Sensor ที่ใช้ได้ดีในระบบ STM32 Industrial Control

---

## 1. Encoder — Delta Accumulation

### ปัญหาของวิธีธรรมดา

TIM counter บน STM32 เป็น 16-bit (0–65535) พอ motor หมุนข้าม boundary ค่ากระโดดทันที

```c
// ❌ ผิด — ถ้า motor หมุนเกิน ±32767 counts ค่า position พัง
position = (int32_t)(int16_t)TIM1->CNT;
// เช่น CNT ไป 65535 แล้ว wrap กลับ 0 → position กระโดดจาก +32767 เป็น -1 ทันที
```

### แนวคิด Delta Accumulation

เรียกทุก 1ms ใน Timer ISR — นับเฉพาะ "ส่วนที่เปลี่ยนไป" แล้วสะสม

```c
// ✅ ถูก — ทำงานใน 1kHz ISR
static uint16_t last_cnt = 0;
uint16_t now_cnt = (uint16_t)TIM1->CNT;

// cast เป็น (int16_t) ก่อน — ทำให้ได้ signed delta อัตโนมัติ
// ตัวอย่าง: now=5, last=65530 → (uint16_t)(5-65530) = 11 → (int16_t)11 = +11 ✓
// ตัวอย่าง: now=65530, last=5 → (uint16_t)(65530-5) = 65525 → (int16_t)65525 = -11 ✓
current_position += (int32_t)(int16_t)(now_cnt - last_cnt);
last_cnt = now_cnt;
```

### ทำไมดีกว่า

| ประเด็น | วิธีธรรมดา | Delta Accumulation |
|--------|-----------|-------------------|
| Travel range | ±32767 counts เท่านั้น | ไม่จำกัด (int32) |
| Overflow handling | กระโดด | Automatic จาก int16_t cast |
| Reset counter ต้องการไหม | ต้องการ | ไม่ต้องการ |
| Multi-turn support | ❌ | ✅ |

**ข้อแม้:** ISR ต้องเรียกถี่พอ (< 32767 counts ต่อ cycle) ซึ่งที่ 1kHz ปกติไม่มีปัญหา

---

## 2. Current Sensor — EMA Filter + Runtime Calibration

### ปัญหาของการอ่าน ADC ตรงๆ

```c
// ❌ ปัญหา: ADC noise ทำให้ค่ากระตุก + ถ้า sensor offset ไม่ตรง ต้อง flash ใหม่ทุกครั้ง
current_A = ((float)ADC_GetValue() / 4095.0f * 3.3f - 1.65f) / 0.066f;
```

### แนวคิด EMA Filter + Calibratable Parameters

```c
// Step 1: กรอง ADC noise ด้วย Exponential Moving Average
// alpha = 1/8 → smooth ดี ไม่ lag มากเกิน
static float filtered_adc = 0;
if (filtered_adc == 0) filtered_adc = raw_adc;          // init ครั้งแรก
filtered_adc = (filtered_adc * 7.0f + (float)raw_adc) / 8.0f;

// Step 2: แปลงเป็น Amperes ผ่าน parameters ที่ปรับได้
// cur_zero_v = วัด multimeter ที่ OUT pin sensor ตอนไม่มีกระแส
// cur_sens   = sensitivity จาก datasheet (V per Ampere)
float v   = (filtered_adc / 4095.0f) * 3.3f;
float i_a = (v - cur_zero_v) / cur_sens;
if (i_a < 0.0f) i_a = -i_a;   // absolute value (sensor bidirectional)
current_sensor_A = i_a;         // หน่วย Amperes, float
```

### การ Calibrate โดยไม่ต้อง Flash ใหม่

เก็บ `cur_zero_v` และ `cur_sens` เป็น global variable หรือใส่ใน struct

```c
float cur_zero_v = 1.65f;  // ค่าเริ่มต้น (ปรับได้ผ่าน debugger หรือ Live Expressions)
float cur_sens   = 0.066f; // 66mV/A สำหรับ WCS1800
```

**ขั้นตอน Calibrate:**
1. ไม่มีกระแสไหล → วัด voltage ที่ sensor output ด้วย multimeter
2. ตั้ง `cur_zero_v` = ค่าที่วัดได้ → `current_sensor_A` จะใกล้ 0
3. ปล่อยกระแส known (เช่น 5A จาก clamp meter) → ดู `current_sensor_A`
4. ถ้าไม่ตรง → ปรับ `cur_sens` จนตรง

### ทำไมดีกว่า

| ประเด็น | Hardcode ใน code | Calibratable Parameters |
|--------|-----------------|------------------------|
| Sensor offset ต่างจากทฤษฎี | Flash ใหม่ | แค่เปลี่ยนค่า variable |
| ใช้กับ sensor รุ่นอื่น | แก้ code | เปลี่ยนแค่ `cur_sens` |
| ADC noise | กระตุก | EMA กรองได้ |
| Debug real-time | ยาก | ดูผ่าน Live Expressions |

---

## ค่า Reference สำหรับ WCS1800

| VCC | Zero offset | Sensitivity |
|-----|------------|-------------|
| 3.3V | 1.65V | 66 mV/A |
| 5.0V | 2.50V | 66 mV/A |

**ตรวจสอบด้วยการวัด multimeter ที่ OUT pin ก่อนเสมอ** — ค่าจริงอาจต่างจากทฤษฎีได้ ±0.1V
