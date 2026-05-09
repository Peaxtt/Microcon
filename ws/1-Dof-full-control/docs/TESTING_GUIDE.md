# 🛠️ Testing & Handoff

## 💻 Developer Dashboard (Live Expressions)
แอดตัวแปร `dev_dash` ในหน้าต่าง Live Expressions เพื่อมอนิเตอร์และเทสระบบ:
*   **system_mode**: 0 = ปกติ, 1 = เทสผ่านคอม (UI), 2 = เทสผ่านจอย
*   **force_...**: ใช้สั่งงานฮาร์ดแวร์ตรงๆ เมื่ออยู่ในโหมด 1
*   **Joy**: เช็คสถานะจอยและรหัสปุ่มดิบ
*   **Status**: เช็คความเร็วมอเตอร์, Encoder, และกระแส

## 📡 Modbus Map (ID: 21)
*   **0x00**: Heartbeat (STM ส่ง 22881, PC ตอบ 18537)
*   **0x01**: Mode (1=Home, 2=Man, 4=Auto)
*   **0x28**: Encoder Position
*   **0x31**: Emergency Status (1=Active)
