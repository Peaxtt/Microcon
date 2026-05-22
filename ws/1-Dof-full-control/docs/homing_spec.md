# Homing System Spec — 1DOF Robot, STM32G474

## Hardware
- MCU: STM32G474
- Sensor: Proximity PNP → Opto-isolator → GPIO (active HIGH = object detected)
- Target: ใบมีดคัตเตอร์บางๆ ติด 3D print บน arm (ขอบคม ตัด sensor แบบ rising edge ชัด)
- Motor มี backlash — control loop จัดการอยู่แล้ว ไม่ต้องแตะ

## Constraints
- หมุนได้แค่ ±540° (1.5 รอบ) เท่านั้น ห้ามเกิน
- สายไฟสบายที่สุดที่ 0° → Home = 0°
- Sensor อยู่ฝั่งซ้าย → homing ทำโดยหมุนซ้าย

## ตัวแปรสำคัญ
```c
float cumulative_angle;  // เก็บมุมสะสมตลอด ไม่ reset เองอัตโนมัติ
                         // หมุนซ้าย = ลบ, หมุนขวา = บวก
                         // ตัวอย่าง: หมุนซ้าย 300° → cumulative = -300.0
                         // save/load จาก EEPROM ทุกครั้ง (กันไฟดับ)
```

## Soft Limit
```c
#define MAX_ANGLE  540.0f
#define MIN_ANGLE -540.0f
// ถ้าคำสั่งจะทำให้ cumulative เกิน limit → reject ทันที อย่า move
```

## State Machine
```c
typedef enum {
    STATE_IDLE,
    STATE_HOMING_FAST,    // หมุนซ้ายเร็ว จนเจอ sensor
    STATE_HOMING_BACKOFF, // ถอยขวาออก 20° (พ้น sensor zone)
    STATE_HOMING_SLOW,    // หมุนซ้ายช้ามาก กลับเข้า sensor
    STATE_RUNNING
} RobotState;
```

## Homing Procedure (Two-Pass)
```
1. เรียก start_homing()
   → enable EXTI, state = HOMING_FAST
   → หมุนซ้าย fast speed

2. Rising edge trigger (HOMING_FAST)
   → หยุด, state = HOMING_BACKOFF
   → หมุนขวา 20° แล้วหยุด

3. Backoff เสร็จ
   → state = HOMING_SLOW
   → หมุนซ้าย ช้ามากๆ (5% ของ fast speed)

4. Rising edge trigger (HOMING_SLOW)
   → หยุด, state = RUNNING
   → cumulative_angle = 0
   → disable EXTI
   → save EEPROM
   → done ✅
```

## EXTI / Interrupt Rules
- **EXTI เปิดเฉพาะตอน homing เท่านั้น** → disable ตลอดเวลา STATE_RUNNING
- ใช้ **Rising edge** เสมอ (ขอบแรกของใบมีดเข้า sensor)
- ใน callback เช็ค state ก่อนทำอะไรทุกครั้ง
- ใช้ **Timer Input Capture** ถ้าทำได้ (แม่นกว่า EXTI เพราะ hardware จับ timestamp ไม่ผ่าน CPU)
- ถ้าใช้ EXTI ธรรมดา → software debounce 50µs กัน glitch จาก opto

```c
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin != SENSOR_PIN) return;
    if (current_state != STATE_HOMING_FAST &&
        current_state != STATE_HOMING_SLOW) return;

    // debounce: รอ 50µs แล้วเช็คว่ายัง HIGH อยู่มั้ย
    // ถ้าใช่ → proceed, ถ้าไม่ → glitch ไม่นับ

    if (current_state == STATE_HOMING_FAST) {
        stop_motor();
        current_state = STATE_HOMING_BACKOFF;
        move_right_backoff(20.0f);

    } else if (current_state == STATE_HOMING_SLOW) {
        stop_motor();
        finish_homing();
    }
}
```

## finish_homing()
```c
void finish_homing() {
    HAL_NVIC_DisableIRQ(EXTI_IRQn);
    cumulative_angle = 0.0f;
    save_to_eeprom(cumulative_angle);
    current_state = STATE_RUNNING;
}
```

## สิ่งที่ต้องระวัง / Code Review Checklist
- [ ] EXTI ต้อง disable ทันทีหลัง finish_homing() — ห้ามลืม
- [ ] approach direction ของ HOMING_SLOW ต้องเป็น ซ้าย เสมอ (same direction ทุก session)
- [ ] cumulative_angle ต้อง save EEPROM ทุกครั้งที่เปลี่ยนค่า ไม่ใช่แค่ตอน home
- [ ] soft limit ต้อง check ก่อน execute คำสั่ง move ทุกครั้ง
- [ ] ถ้าไฟดับ → boot ขึ้นมา load EEPROM → unwind กลับ 0 → home ใหม่
- [ ] debounce ต้องไม่ block interrupt handler (ใช้ timer หรือ flag แทน HAL_Delay)
