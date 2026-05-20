/*
 * ENCODER.c
 *
 *  Created on: May 10, 2026
 *      Author: User
 *
 * --- Wrap-around (ref: sensor_concepts_share.md) ---
 * TIM1 ARR = 65535 (full 16-bit range).
 * delta = (int16_t)(now_cnt - last_cnt)
 * Unsigned 16-bit subtraction wraps at 65536; int16_t cast reinterprets
 * values > 32767 as negative, giving correct signed deltas automatically.
 *
 * --- Windowed velocity ---
 * Single-tick velocity (delta/dt) has 0.767 rad/s quantization noise at 1 kHz.
 * Averaging over VEL_WINDOW ticks reduces this by a factor of VEL_WINDOW,
 * giving a velocity estimate that closely follows smooth trajectory profiles.
 */

#include "encoder.h"
#include <string.h>

void Encoder_Init(Encoder_t *enc, float dt) {
    enc->last_counter_value = 0;
    enc->count_accum        = 0;
    memset(enc->vel_buf, 0, sizeof(enc->vel_buf));
    enc->vel_idx            = 0;
    enc->vel_full           = 0;
    enc->position_rad       = 0.0f;
    enc->velocity_rad_s     = 0.0f;
    enc->dt                 = dt;
}

void Encoder_Update(Encoder_t *enc, uint16_t current_timer_value) {
    // 1. Signed delta — handles 0-65535 hardware wrap automatically
    int16_t delta_counts = (int16_t)(current_timer_value - enc->last_counter_value);
    enc->last_counter_value = current_timer_value;

    // 2. Accumulate in int32 — unlimited multi-turn, no float precision loss
    enc->count_accum += (int32_t)delta_counts;

    // 3. Position — converted at output only
    enc->position_rad = (float)enc->count_accum * COUNTS_TO_RAD;

    // 4. Windowed velocity — average over VEL_WINDOW ticks
    //    Store current accumulated count; oldest entry sits at vel_idx
    //    (the slot about to be overwritten next tick when buffer is full).
    enc->vel_buf[enc->vel_idx] = enc->count_accum;
    enc->vel_idx++;
    if (enc->vel_idx >= VEL_WINDOW) {
        enc->vel_idx = 0;
        enc->vel_full = 1;
    }

    if (enc->vel_full) {
        // vel_buf[vel_idx] is the oldest stored value (window span = VEL_WINDOW ticks)
        int32_t oldest = enc->vel_buf[enc->vel_idx];
        enc->velocity_rad_s = (float)(enc->count_accum - oldest)
                            * COUNTS_TO_RAD / ((float)VEL_WINDOW * enc->dt);
    } else {
        // Buffer not yet full — fall back to single-tick estimate
        enc->velocity_rad_s = (float)delta_counts * COUNTS_TO_RAD / enc->dt;
    }
}
