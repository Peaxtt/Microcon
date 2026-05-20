#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>
#include <math.h>

// Encoder hardware specs
#define ENCODER_PPR       2048.0f
#define ENCODER_QEI_MODE  4.0f
#define ENCODER_CPR       (ENCODER_PPR * ENCODER_QEI_MODE)   // 8192 counts/rev
#define PI_M2             (2.0f * 3.14159265359f)
#define COUNTS_TO_RAD     (PI_M2 / ENCODER_CPR)

// Windowed velocity: averages delta counts over VEL_WINDOW ticks.
// Reduces quantization noise by √VEL_WINDOW vs single-tick measurement.
//   Single tick:  1 count = 0.767 rad/s  (very noisy at 1 kHz)
//   10-tick window: resolution = 0.077 rad/s  (10× improvement)
// Lag introduced = VEL_WINDOW × dt / 2 = 5 ms (acceptable for 1 kHz control).
#define VEL_WINDOW  10

typedef struct {
    uint16_t last_counter_value;    // Raw 16-bit hardware timer value (previous tick)
    int32_t  count_accum;           // 32-bit accumulated counts — unlimited range

    // Windowed velocity ring buffer
    int32_t  vel_buf[VEL_WINDOW];   // Stores count_accum history
    uint8_t  vel_idx;               // Next write index (0 to VEL_WINDOW-1)
    uint8_t  vel_full;              // 1 once the buffer has been filled once

    float    position_rad;          // Absolute position (rad)
    float    velocity_rad_s;        // Windowed velocity (rad/s) — smoother than single-tick
    float    dt;                    // Control loop time step (s)
} Encoder_t;

void Encoder_Init  (Encoder_t *enc, float dt);
void Encoder_Update(Encoder_t *enc, uint16_t current_timer_value);

#endif /* ENCODER_H */
