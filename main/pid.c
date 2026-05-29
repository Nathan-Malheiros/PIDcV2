#include "pid.h"
#include <math.h>

/* Filtro passa-baixa no termo derivativo (time-constant, s). A derivada bruta
 * sobre um sinal de toque ruidoso gera "chutes" que saturam a saida; este
 * filtro de 1a ordem os suaviza. 0.0f desliga. ~0.03 s ≈ 2-3 amostras a 50 Hz.
 * A base do Aaed Musa usa derivada NAO normalizada (err-errPrev), que ja e
 * naturalmente mais branda; aqui usamos derivada real (mm/s), entao o filtro
 * recupera esse comportamento sem perder a normalizacao por dt. */
#define PID_D_TAU   0.03f

void pid_init(pid_t *p, float kp, float ki, float kd, float out_min, float out_max)
{
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
    p->out_min = out_min;
    p->out_max = out_max;
    p->d_tau   = PID_D_TAU;
    pid_reset(p);
}

void pid_reset(pid_t *p)
{
    p->integ    = 0.0f;
    p->prev_err = 0.0f;
    p->primed   = false;
    p->d_lpf    = 0.0f;
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
        /* filtro passa-baixa de 1a ordem no derivativo (anti-ruido) */
        if (p->d_tau > 0.0f) {
            float a = dt / (p->d_tau + dt);
            p->d_lpf += a * (deriv - p->d_lpf);
            deriv = p->d_lpf;
        }
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
