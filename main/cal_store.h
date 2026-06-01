#pragma once

#include <stdint.h>
#include <stdbool.h>

/*
 * Persistent storage for runtime-calibrated values (NVS, namespace "bb_cal").
 * Call cal_store_init() once before any load/save operation.
 */

typedef struct {
    float kp, ki, kd;
} cal_pid_t;

typedef struct {
    int32_t x_raw_min, x_raw_max;
    int32_t y_raw_min, y_raw_max;
    int8_t  flip_x, flip_y, swap_xy;
} cal_touch_t;

/* Initialize NVS flash — erases partition only if version mismatch or full. */
void cal_store_init(void);

/* Returns true if a valid PID blob was found in NVS. */
bool cal_store_load_pid(cal_pid_t *out);
void cal_store_save_pid(const cal_pid_t *in);

/* Returns true if a valid touch calibration blob was found in NVS. */
bool cal_store_load_touch(cal_touch_t *out);
void cal_store_save_touch(const cal_touch_t *in);
