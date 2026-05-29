#include "pid.h"
#include <math.h>

void pid_init(pid_t *p, float kp, float ki, float kd, float out_min, float out_max)
{
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
    p->out_min = out_min;
    p->out_max = out_max;
    pid_reset(p);
}

void pid_reset(pid_t *p)
{
    p->integ    = 0.0f;
    p->prev_err = 0.0f;
    p->primed   = false;
}

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

float pid_update(pid_t *p, float meas, float setpoint, float dt)
{
    if (dt <= 0.0f) dt = 1e-3f;

    float err = meas - setpoint;

    /* derivative on error; first call has no history so we skip it */
    float deriv = 0.0f;
    if (p->primed) {
        deriv = (err - p->prev_err) / dt;
        if (isnan(deriv) || isinf(deriv)) deriv = 0.0f;
    }
    p->prev_err = err;
    p->primed   = true;

    /* integrate */
    p->integ += err * dt;

    float out = p->kp * err + p->ki * p->integ + p->kd * deriv;

    /* clamp output and anti-windup: pull the integrator back if we saturate */
    if (out > p->out_max) {
        if (p->ki != 0.0f) p->integ -= (out - p->out_max) / p->ki;
        out = p->out_max;
    } else if (out < p->out_min) {
        if (p->ki != 0.0f) p->integ -= (out - p->out_min) / p->ki;
        out = p->out_min;
    }

    return clampf(out, p->out_min, p->out_max);
}
