# 🛠️ Hardware Raw Testing Guide (Automated Loop Test)

เอกสารนี้สำหรับเทสฮาร์ดแวร์ **"ทุกชิ้นในตู้ไฟ"** แบบอัตโนมัติ โดยไม่ต้องต่อจอยสติ๊กและไม่ต้องพึ่งพา Modbus UI ครับ ระบบจะวิ่งวนลูปเปิดไฟและรีเลย์ทีละดวง พร้อมกับขยับมอเตอร์นิดหน่อยให้ดู และปรินต์ค่าเซนเซอร์ทั้งหมด (รวมถึง Encoder) ออกทางหน้าจอคอมพิวเตอร์ตลอดเวลา

## 📍 สิ่งที่ต้องเตรียม
1. เสียบสาย USB ระหว่างบอร์ด Nucleo G474 กับคอมพิวเตอร์ (พอร์ต LPUART1)
2. เปิดโปรแกรม Serial Monitor บนคอมพิวเตอร์ (เช่น PuTTY, TeraTerm)
   *   **COM Port:** เลือกพอร์ตของ ST-Link
   *   **Baudrate:** `19200`
   *   **Parity:** `Even`, **Data bits:** `8`, **Stop bits:** `1`

---

## 💻 โค้ดสำหรับเทสฮาร์ดแวร์แบบ Auto-Loop

ให้บอสไปที่ไฟล์ `Core/Src/main.c` 
เลื่อนลงไปในลูป `while (1)` ตรงส่วน **`/* USER CODE BEGIN 3 */`** 

**วิธีทำ:** 
ให้คอมเม้นต์ (`/* ... */`) โค้ดชุดเดิมที่เป็น State Machine และ Modbus ทิ้งไปก่อนชั่วคราว แล้ว **ก๊อปปี้โค้ดด้านล่างนี้ไปแปะแทนที่ข้างใน `if (flag_10ms) { ... }`** ครับ:

```c
    if (flag_10ms) {
      flag_10ms = 0;
      
      // 1. อ่านค่า ADC Current Sensor
      HAL_ADC_Start(&hadc1);
      if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
        current_sensor_val = HAL_ADC_GetValue(&hadc1);
      }
      
      // 2. ปริ้นค่า Input ทั้งหมดออกทาง Serial (LPUART1 @ 19200) เพื่อดูบนคอมพิวเตอร์
      static uint8_t print_cnt = 0;
      if (++print_cnt >= 20) { // ปริ้นทุกๆ 200ms
        print_cnt = 0;
        printf("\033[2J\033[H"); // Clear Screen (ANSI Escape code)
        printf("=== 🤖 1-DOF HARDWARE AUTO-TEST ===\r\n\r\n");
        printf("[ INPUTS ]\r\n");
        printf("ESTOP(PB13) : %d  |  HOME(PB5)  : %d\r\n", HAL_GPIO_ReadPin(ESTOP_GPIO_Port, ESTOP_Pin), HAL_GPIO_ReadPin(HOME_GPIO_Port, HOME_Pin));
        printf("RESET(PB1)  : %d  |  POWER(PB10): %d  |  MODE(PC0) : %d\r\n", HAL_GPIO_ReadPin(RESET_BTN_GPIO_Port, RESET_BTN_Pin), HAL_GPIO_ReadPin(POWER_BTN_GPIO_Port, POWER_BTN_Pin), HAL_GPIO_ReadPin(MODE_BTN_GPIO_Port, MODE_BTN_Pin));
        printf("REED_UP(PA0): %d  |  REED_DN(PA4): %d  |  GRIP(PB0) : %d\r\n", HAL_GPIO_ReadPin(REED_UP_GPIO_Port, REED_UP_Pin), HAL_GPIO_ReadPin(REED_DOWN_GPIO_Port, REED_DOWN_Pin), HAL_GPIO_ReadPin(REED_GRIP_GPIO_Port, REED_GRIP_Pin));
        printf("\r\n[ SENSORS ]\r\n");
        printf("ENCODER (TIM1) : %ld\r\n", (int32_t)__HAL_TIM_GET_COUNTER(&htim1));
        printf("CURRENT (ADC)  : %d\r\n", current_sensor_val);
      }

      // 3. วนลูปเทส Output อัตโนมัติ (เปลี่ยนสเต็ปทุกๆ 1 วินาที)
      static uint16_t auto_timer = 0;
      static uint8_t test_step = 0;
      
      auto_timer++;
      if (auto_timer >= 100) { // 100 ticks = 1000ms = 1 วินาที
        auto_timer = 0;
        test_step++;
        if (test_step > 9) test_step = 0; // วนกลับไปสเต็ป 0
      }

      // ปิดทุกอย่างก่อนในทุกๆ ลูป (แล้วค่อยเปิดเฉพาะตัวที่ถึงคิว)
      HAL_GPIO_WritePin(TOWER_G_GPIO_Port, TOWER_G_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(TOWER_Y_GPIO_Port, TOWER_Y_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(TOWER_R_GPIO_Port, TOWER_R_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(EMER_OUTPUT_GPIO_Port, EMER_OUTPUT_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(PNEUMATIC_GPIO_Port, PNEUMATIC_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GRIPPER_GPIO_Port, GRIPPER_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(RESET_LED_GPIO_Port, RESET_LED_Pin, GPIO_PIN_RESET);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0); // ดับมอเตอร์

      // เปิด Output ตามสเต็ป
      if (print_cnt == 0) printf("\r\n[ OUTPUT TEST STEP: %d ] -> ", test_step);
      
      switch (test_step) {
        case 0: 
            if (print_cnt == 0) printf("ALL OFF\r\n"); 
            break;
        case 1: 
            HAL_GPIO_WritePin(TOWER_G_GPIO_Port, TOWER_G_Pin, GPIO_PIN_SET); 
            if (print_cnt == 0) printf("TOWER GREEN ON\r\n"); 
            break;
        case 2: 
            HAL_GPIO_WritePin(TOWER_Y_GPIO_Port, TOWER_Y_Pin, GPIO_PIN_SET); 
            if (print_cnt == 0) printf("TOWER YELLOW ON\r\n"); 
            break;
        case 3: 
            HAL_GPIO_WritePin(TOWER_R_GPIO_Port, TOWER_R_Pin, GPIO_PIN_SET); 
            if (print_cnt == 0) printf("TOWER RED ON\r\n"); 
            break;
        case 4: 
            HAL_GPIO_WritePin(EMER_OUTPUT_GPIO_Port, EMER_OUTPUT_Pin, GPIO_PIN_SET); 
            if (print_cnt == 0) printf("EMER_OUT (PB6) ON\r\n"); 
            break;
        case 5: 
            HAL_GPIO_WritePin(PNEUMATIC_GPIO_Port, PNEUMATIC_Pin, GPIO_PIN_SET); 
            if (print_cnt == 0) printf("PNEUMATIC ON (UP/DOWN)\r\n"); 
            break;
        case 6: 
            HAL_GPIO_WritePin(GRIPPER_GPIO_Port, GRIPPER_Pin, GPIO_PIN_SET); 
            if (print_cnt == 0) printf("GRIPPER ON (CLOSE/OPEN)\r\n"); 
            break;
        case 7: 
            HAL_GPIO_WritePin(RESET_LED_GPIO_Port, RESET_LED_Pin, GPIO_PIN_SET); 
            if (print_cnt == 0) printf("RESET LED ON\r\n"); 
            break;
        case 8: 
            HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, GPIO_PIN_SET);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)(0.15f * 8499.0f)); // หมุน 15%
            if (print_cnt == 0) printf("MOTOR FORWARD (15%% Speed)\r\n"); 
            break;
        case 9: 
            HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, GPIO_PIN_RESET);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)(0.15f * 8499.0f)); // หมุน 15%
            if (print_cnt == 0) printf("MOTOR BACKWARD (15%% Speed)\r\n"); 
            break;
      }
      
      // Refresh IWDG เผื่อกันระบบรีเซ็ตตัวเอง
      HAL_IWDG_Refresh(&hiwdg);
      
      // ไฟกระพริบบนบอร์ด (Heartbeat ของลูป)
      static uint32_t last_hb = 0;
      if (++last_hb > 50) {
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        last_hb = 0;
      }
    }
```

---

## 🎯 วิธีเทสหน้าตู้

1.  **ดูจอคอมพิวเตอร์:** มันจะเคลียร์หน้าจอและปรินต์ค่าใหม่ทุกๆ 0.2 วินาที 
    *   ลองเอามือหมุนแกนมอเตอร์ดูครับ เลข **`ENCODER (TIM1)`** ต้องวิ่งขึ้นลง
    *   ลองกดปุ่ม E-Stop, Home, Reset ดู เลขสเตตัสข้างบนต้องเปลี่ยน
2.  **ดูตู้ไฟ (Output):** โค้ดจะเปลี่ยนสเต็ปทุกๆ 1 วินาที โดยเปิดไฟทีละดวงไล่ไปเรื่อยๆ:
    *   ไฟเขียว -> เหลือง -> แดง -> EMER_OUT -> กระบอกลม -> กริปเปอร์ -> ไฟปุ่ม Reset
    *   จากนั้นมอเตอร์จะหมุนเดินหน้าเบาๆ (15%) 1 วินาที แล้วถอยหลังเบาๆ 1 วินาที
    *   แล้ววนลูปกลับไปปิดทุกอย่างใหม่

ถ้าตู้ไฟกระพริบไล่สเต็ปตามนี้เป๊ะๆ และตัวเลข Encoder ขึ้นตรง แสดงว่าประกอบตู้ผ่าน 100% ครับ! ลุยเทสได้เลย!