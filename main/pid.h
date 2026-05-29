#pragma once

#include <stdbool.h>

/*
 * Single-axis PID controller (one instance per axis: X and Y).
 *
 * Convention used by the ball balancer:
 *   error = measurement - setpoint     (positive = ball past the target)
 * so a positive output tilts the platform toward the displacement side, which
 * is the stabilising direction for the 3RPS geometry. If the table ever pushes
 * the ball AWAY from the target, flip the sign of the output where it is
 * applied (see control.c, TILT_SIGN).
 */

typedef struct {
    float kp, ki, kd;
    float out_min, out_max;   /* output clamp (also clamps the integrator) */
    float integ;              /* integral accumulator */
    float prev_err;           /* error from the previous update */
    bool  primed;             /* false until the first update seeds prev_err */
    float d_tau;              /* time-constant do filtro do derivativo (s), 0=off */
    float d_lpf;              /* estado do filtro passa-baixa do derivativo */
} pid_t;

void  pid_init(pid_t *p, float kp, float ki, float kd, float out_min, float out_max);
void  pid_reset(pid_t *p);

/*
 * One control step.
 *   meas     : current measurement (mm)
 *   setpoint : target (mm)
 *   dt       : seconds since the previous update
 * Returns the clamped controller output.
 */
float pid_update(pid_t *p, float meas, float setpoint, float dt);
