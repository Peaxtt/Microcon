# 🧠 Software Logic

## 🚦 State Machine
ระบบมี 5 สถานะหลัก:
1. **IDLE**: รอรับคำสั่ง
2. **CALIBRATE**: โหมด Homing หาตำแหน่ง 0
3. **MANUAL**: ควบคุมผ่าน Joystick หรือ Modbus Jog
4. **AUTO**: รันโหมดอัตโนมัติ (P2P / Sequence)
5. **EMER**: โหมดฉุกเฉิน (หยุดทุกอย่าง)

## 🎮 Joystick Mapping
* **ก้านซ้าย (LY)**: ควบคุมมอเตอร์ (ต้องกำ **RT** ค้างไว้ - Deadman Switch)
* **L3 + R3**: สั่ง Emergency Stop
* **LT + X**: เริ่มโหมด Homing
* **ปุ่ม Back**: รีเซ็ต Alarm
* **ปุ่ม Y**: กระบอกลม ขึ้น/ลง
* **ปุ่ม B**: กริปเปอร์ จับ/ปล่อย
* **ปุ่ม X**: เทสไฟ Reset LED
