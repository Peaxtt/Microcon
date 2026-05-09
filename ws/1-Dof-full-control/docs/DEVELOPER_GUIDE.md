# 🤖 1-DOF Robot System Developer Guide

เอกสารนี้รวบรวมโครงสร้างสถาปัตยกรรม (Architecture), ลอจิกการควบคุม (State Machine), และวิธีการเทสระบบ (Live Expressions) สำหรับนักพัฒนาและทีม Control ครับ

---

## 🏗️ 1. System Architecture & Hardware Setup

ระบบถูกออกแบบมาในระดับ Industrial-Grade เน้นความเสถียรและลดภาระ CPU (Zero CPU Overhead)

*   **MCU:** STM32G474RE
*   **Timebase:** 1kHz Loop (TIM7) แยกขาดจาก SysTick เพื่อรัน PID และตรวจจับการกดปุ่มแบบ Non-blocking
*   **Safety (IWDG & EXTI):** 
    *   มี Independent Watchdog (IWDG) รีเซ็ตระบบถ้าระบบค้างเกิน 3 วินาที
    *   E-Stop (PB13) และ Selector Switch (PB5) ตอบสนองทันทีผ่าน Interrupt
*   **Modbus RTU (LPUART1 - 19200 8E1):**
    *   ทำงานผ่านวงจร DMA เต็มรูปแบบ 
    *   ใช้ Hardware Receiver Timeout (RTO) หรือ Software Timeout ใน 1kHz Loop ช่วยจับการจบเฟรม
*   **Joystick (USART3 - 460800 8N1):**
    *   ทำงานผ่าน DMA 
    *   มี Software Deadband กันมอเตอร์ครางตอนก้านอนาล็อกไม่กลับศูนย์
*   **ADC Current Sensor (PC5 / ADC2):**
    *   ใช้สมการ Moving Average Filter แบบ Exponential ในการกรองสัญญาณรบกวน (Noise)
*   **Motor PWM (PC6 / TIM3):**
    *   ตั้งความถี่ที่ `20 kHz` (Silent Motor) เพื่อไม่ให้มีเสียงความถี่สูงกวนหู

---

## 🚦 2. State Machine (ลอจิกการทำงาน)

ระบบมี 5 State หลัก ทำงานสอดคล้องกับหน้าจอ UI บน Modbus (Register `0x01` และ `0x27`):

1.  **`STATE_IDLE` (0)**: มอเตอร์หยุดทำงาน รอรับคำสั่ง (สามารถเปลี่ยนเป็น Auto, Manual, Calibrate ได้)
2.  **`STATE_CALIBRATE` (1)**: โหมดหาตำแหน่งเริ่มต้น (Homing)
3.  **`STATE_MANUAL` (0)**: ผู้ใช้ควบคุมผ่าน Joystick (ก้านอนาล็อกซ้าย `LY`) หรือ Modbus Jog (Reg `0x05`)
4.  **`STATE_AUTO` (8)**: หุ่นยนต์รันโหมด Auto (รอทีม Control ใส่สมการ PID และรับค่า Target จาก P2P `0x24`)
5.  **`STATE_EMER` (0)**: **โหมดฉุกเฉิน** ดับ PWM มอเตอร์ทันที เปิดไฟแดง (`PC8` TOWER_R) และสั่งเปิดสวิตช์ตัดไฟระบบ (`PB6` EMER_OUTPUT) การจะออกจากสเตทนี้ได้ ต้องคลายปุ่ม E-Stop และกด Reset Alarm (ปุ่ม Back บนจอย หรือส่ง 0xFF จาก Modbus) เท่านั้น

---

## 🎮 3. Joystick Mapping (การควบคุมผ่านจอย)

หากเสียบจอยสติ๊ก (ผ่านพอร์ต USB-Reader ESP32/Pico) เข้ากับบอร์ด STM32 สามารถใช้จอยคุมแบบ Override โหมดต่างๆ ได้ดังนี้:

*   **อนาล็อกซ้าย (L-Y):** บังคับมอเตอร์เดินหน้า/ถอยหลัง (ในโหมด Manual)
*   **กดก้านอนาล็อก L3 + R3:** ซอฟต์แวร์ E-Stop (เข้าโหมด `STATE_EMER`)
*   **LT (กำค้าง) + ปุ่ม X:** สั่งเข้าโหมด Homing (`STATE_CALIBRATE`)
*   **ปุ่ม Back / Select:** รีเซ็ต Alarm (ออกจาก `STATE_EMER`)
*   **RT (กำค้าง):** กดค้างไว้เพื่ออนุญาตให้ขยับแกนอนาล็อกเพื่อเลื่อนมอเตอร์ (Deadman Switch)
*   **ปุ่ม Y:** ดันกระบอกลม (PNEUMATIC)
*   **ปุ่ม A:** สั่ง Sequence (ปล่อยลม + หนีบกริปเปอร์) คล้ายคำสั่ง Pick
*   **ปุ่ม B:** สั่ง Sequence (ปล่อยลม + ปล่อยกริปเปอร์) คล้ายคำสั่ง Place
*   **ปุ่ม X (กดเดี่ยวๆ):** เทสไฟสถานะ (RESET_LED)

---

## 💻 4. Interactive Debugging & Hardware Test (Live Expressions Dashboard)

นักพัฒนาสามารถเข้าถึง Dashboard นี้ได้โดยแฟลชโค้ดลงบอร์ดด้วยปุ่ม **Debug (รูปแมลง 🐞)** และดูที่หน้าต่าง **Live Expressions** โดย Add: 👉 `dev_dash`

### 🛠️ โหมดการทำงาน (System Mode)
ตัวแปร `dev_dash.Ctrl.mode` สามารถเปลี่ยนค่าได้แบบ Real-time:

*   **`0` (`SYS_MODE_PRODUCTION`)**: **โหมดใช้งานจริง** ระบบจะรัน State Machine ตามปกติ
*   **`1` (`SYS_MODE_HARDWARE_TEST`)**: **โหมดเทสผ่านจอคอม** ตัดคำสั่งจากจอยและ Modbus และยอมให้สั่งเปิด/ปิด Relay หรือมอเตอร์ได้โดยตรงจากตัวแปรในกรุ๊ป `dev_dash.Ctrl`
*   **`2` (`SYS_MODE_JOYSTICK_TEST`)**: **โหมดเทสผ่านจอยสติ๊ก** เอาปุ่มบนจอยไปผูกกับรีเลย์แต่ละตัวตรงๆ (กดติดปล่อยดับ)
*   **`3` (`SYS_MODE_AUTO_MOTOR_TEST`)**: **โหมดเทสขยับมอเตอร์อัตโนมัติ** มอเตอร์จะสลับทิศทางทุก 1 วินาทีที่ความเร็ว 30% เพื่อเช็คความสมูทของ Hardware

### 🎛️ การสั่งงานในโหมด Hardware Test (โหมด 1)
*   `force_pneumatic`, `force_gripper` (0/1): สั่งกระบอกลม/กริปเปอร์
*   `force_tower_green`, `yellow`, `red` (0/1): สั่งไฟทาวเวอร์
*   `force_motor_speed`: สั่งมอเตอร์หมุน (ใส่ค่า `-1.0` ถึง `1.0`)

### 📊 การดูค่าเซนเซอร์แบบ Real-time
*   `Joy.connected`: ตรวจสอบสถานะการเชื่อมต่อจอย
*   `Status.state`: ดูสเตตัสปัจจุบัน
*   `Status.motor_cmd`: ความเร็วสุทธิที่ถูกสั่งไปยังบอร์ด Cytron
*   `Status.encoder`: ตำแหน่งปัจจุบัน
*   `Status.current_mA`: กระแสจาก ADC2 (ผ่าน Moving Average Filter แล้ว)
*   `In.estop`, `In.mode_switch`: สเตตัสปุ่มและสวิตช์
