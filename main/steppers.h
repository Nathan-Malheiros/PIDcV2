#pragma once

#include <stdbool.h>
#include <stdint.h>

/*
 * Software step generator for 3 NEMA 17 motors driven by STEP/DIR drivers
 * (A4988 / DRV8825 / TMC2208, etc.). Replaces AccelStepper + MultiStepper
 * from the Arduino reference.
 *
 * A 10 kHz esp_timer integrates a TRAPEZOIDAL velocity profile per motor
 * (accel-limited, like AccelStepper in the Arduino reference): each axis
 * accelerates up to its max speed and decelerates in time to stop exactly on
 * target. This gives smooth motion and avoids skipped steps from abrupt
 * starts. For higher step rates port this to the RMT or MCPWM peripheral.
 *
 * GPIOs below avoid the touch pins (4,5,8,9), the strapping pins (0,3,45,46)
 * and the USB pins (19,20). VERIFY them against your own wiring before flashing.
 */

#define STEPPER_COUNT     3

#define STEP_PIN_A        15
#define DIR_PIN_A         16
#define STEP_PIN_B        17
#define DIR_PIN_B         18
#define STEP_PIN_C        40
#define DIR_PIN_C         41
#define STEPPER_EN_PIN    10     /* common ENABLE, active LOW on most drivers */

void  steppers_init(void);
void  steppers_enable(bool on);                 /* on = drivers energised */

void  stepper_set_max_speed(int idx, float steps_per_sec);
void  stepper_set_acceleration(int idx, float steps_per_sec2);
void  stepper_move_to(int idx, long abs_steps);
long  stepper_current(int idx);
void  steppers_move_to_all(const long target[STEPPER_COUNT]);

/* Block until every motor has reached its target (used for homing). */
void  steppers_run_to_position_blocking(void);
