#ifndef PCB_IO_TEST_H
#define PCB_IO_TEST_H

#include "main.h"

/*
 * PCB I/O Validation — ทดสอบ pin assignment ใหม่ทั้งหมดก่อน commit IOC
 *
 * Phase 1  Output blink  — กระพริบ output ทุก pin ทีละอัน (วัดด้วย multimeter)
 * Phase 2  Input pull    — อ่าน input ขณะ float (PULLUP → HIGH, PULLDOWN → LOW)
 * Phase 3  ADC           — อ่าน PA0 (ADC2_IN1) รายงานแรงดัน
 * Phase 4  RP2040        — เช็ค PB9 idle HIGH (RP2040 เปิดอยู่และต่อสาย)
 *
 * วิธีใช้:
 *   เปิด  pcb_io_test(&hlpuart1);  ใน USER CODE BEGIN 2
 *   เปิด terminal 115200 8N1
 *   ดูผลและวัดพิน ตามที่ terminal บอก
 *   ฟังก์ชันค้างตอนจบ — reflash ใหม่เพื่อกลับ firmware ปกติ
 */
void pcb_io_test(UART_HandleTypeDef *huart);

#endif
