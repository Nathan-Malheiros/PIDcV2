#pragma once

#include <stdbool.h>
#include <stdint.h>

/*
 * Software step generator for 3 NEMA 17 motors driven by STEP/DIR drivers
 * (A4988 / DRV8825 / TMC2208, etc.). Replaces AccelStepper + MultiStepper
 * from the Arduino reference.
 *
 * A 10 kHz esp_timer emits at most one step per motor per tick toward each
 * motor's target, at that motor's configured speed. This is a constant-speed
 * coordinated move (like MultiStepper::runSpeedToPosition), which is plenty
 * for a 50 Hz balancing loop. For higher step rates port this to the RMT or
 * MCPWM peripheral.
 *
 * GPIOs below avoid the touch pins (4,5,8,9), the strapping pins (0,3,45,46)
 * and the USB pins (19,20). VERIFY them against your own wiring before flashing.
 */

#define STEPPER_COUNT     3

#define STEP_PIN_A        11
#define DIR_PIN_A         12
#define STEP_PIN_B        13
#define DIR_PIN_B         14
#define STEP_PIN_C        21
#define DIR_PIN_C         47
#define STEPPER_EN_PIN    10     /* common ENABLE, active LOW on most drivers */

void  steppers_init(void);
void  steppers_enable(bool on);                 /* on = drivers energised */

void  stepper_set_max_speed(int idx, float steps_per_sec);
void  stepper_move_to(int idx, long abs_steps);
long  stepper_current(int idx);
void  steppers_move_to_all(const long target[STEPPER_COUNT]);

/* Block until every motor has reached its target (used for homing). */
void  steppers_run_to_position_blocking(void);
