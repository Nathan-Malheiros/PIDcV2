#include "steppers.h"

#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

/* ── Step engine ────────────────────────────────────────────────────────────
 * 10 kHz esp_timer integra um perfil de velocidade TRAPEZOIDAL por motor
 * (estilo AccelStepper, a base do projeto): cada eixo acelera ate max_speed e
 * desacelera a tempo de PARAR exatamente no alvo (v_lim = sqrt(2*a*dist)).
 * Isso da movimento suave e evita perda de passos por partida brusca — o
 * firmware antigo movia a velocidade CONSTANTE, sem rampa.
 */
#define TICK_PERIOD_US   100          /* 10 kHz */
#define TICK_DT          (TICK_PERIOD_US / 1.0e6f)
#define STEP_PULSE_US      3          /* STEP em nivel alto (drivers: ~1-2 us) */
#define DIR_SETUP_US       3          /* tempo DIR->STEP apos trocar de sentido */

#define DEFAULT_MAX_SPEED  1000.0f    /* steps/s */
#define DEFAULT_ACCEL      8000.0f    /* steps/s^2 */

typedef struct {
    int   step_pin;
    int   dir_pin;
    volatile long  current;           /* posicao absoluta atual (steps) */
    volatile long  target;            /* posicao absoluta desejada (steps) */
    float          max_speed;         /* teto de velocidade (steps/s) */
    float          accel;             /* aceleracao (steps/s^2) */
    float          vel;               /* velocidade atual com sinal (steps/s) */
    float          accum;             /* acumulador fracionario de passo */
    int            last_dir;          /* ultimo sentido aplicado ao pino DIR */
} stepper_t;

static stepper_t s_mot[STEPPER_COUNT] = {
    { .step_pin = STEP_PIN_A, .dir_pin = DIR_PIN_A },
    { .step_pin = STEP_PIN_B, .dir_pin = DIR_PIN_B },
    { .step_pin = STEP_PIN_C, .dir_pin = DIR_PIN_C },
};

static esp_timer_handle_t s_timer;

static void out_pin(int pin)
{
    gpio_config_t c = {
        .pin_bit_mask = (1ULL << pin),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&c);
    gpio_set_level(pin, 0);
}

/* Emite um pulso de passo, respeitando o setup-time do pino DIR. */
static inline void emit_step(stepper_t *m, int dir)
{
    if (dir != m->last_dir) {
        gpio_set_level(m->dir_pin, dir > 0 ? 1 : 0);
        esp_rom_delay_us(DIR_SETUP_US);     /* DIR precisa estabilizar antes do STEP */
        m->last_dir = dir;
    }
    gpio_set_level(m->step_pin, 1);
    esp_rom_delay_us(STEP_PULSE_US);
    gpio_set_level(m->step_pin, 0);
}

/* esp_timer callback (10 kHz): integra o perfil de cada motor. */
static void step_engine(void *arg)
{
    for (int i = 0; i < STEPPER_COUNT; i++) {
        stepper_t *m = &s_mot[i];

        long dist = m->target - m->current;

        if (dist == 0) {
            /* chegou — para de vez (evita "dithering" de 1 passo no alvo) */
            m->vel = 0.0f;
            m->accum = 0.0f;
            continue;
        }

        /* velocidade da qual ainda da pra frear a tempo (v^2 = 2*a*d) */
        float v_lim  = sqrtf(2.0f * m->accel * (float)labs(dist));
        float cruise = m->max_speed < v_lim ? m->max_speed : v_lim;
        float v_des  = (dist > 0) ? cruise : -cruise;

        /* rampa de velocidade limitada pela aceleracao */
        float dv = m->accel * TICK_DT;
        if (m->vel < v_des)      m->vel = (m->vel + dv < v_des) ? m->vel + dv : v_des;
        else if (m->vel > v_des) m->vel = (m->vel - dv > v_des) ? m->vel - dv : v_des;

        /* integra a posicao; emite passos quando o acumulador cruza 1 */
        m->accum += m->vel * TICK_DT;
        while (m->accum >= 1.0f) {
            emit_step(m, +1);
            m->current += 1;
            m->accum   -= 1.0f;
            if (m->current == m->target) { m->accum = 0.0f; break; }
        }
        while (m->accum <= -1.0f) {
            emit_step(m, -1);
            m->current -= 1;
            m->accum   += 1.0f;
            if (m->current == m->target) { m->accum = 0.0f; break; }
        }
    }
}

void steppers_init(void)
{
    out_pin(STEPPER_EN_PIN);
    gpio_set_level(STEPPER_EN_PIN, 1);   /* comeca desabilitado (ativo LOW) */

    for (int i = 0; i < STEPPER_COUNT; i++) {
        out_pin(s_mot[i].step_pin);
        out_pin(s_mot[i].dir_pin);
        s_mot[i].current   = 0;
        s_mot[i].target    = 0;
        s_mot[i].max_speed = DEFAULT_MAX_SPEED;
        s_mot[i].accel     = DEFAULT_ACCEL;
        s_mot[i].vel       = 0.0f;
        s_mot[i].accum     = 0.0f;
        s_mot[i].last_dir  = 0;
    }

    const esp_timer_create_args_t targs = {
        .callback = step_engine,
        .name     = "step_engine",
    };
    esp_timer_create(&targs, &s_timer);
    esp_timer_start_periodic(s_timer, TICK_PERIOD_US);
}

void steppers_enable(bool on)
{
    gpio_set_level(STEPPER_EN_PIN, on ? 0 : 1);  /* ativo LOW */
}

void stepper_set_max_speed(int idx, float steps_per_sec)
{
    if (idx < 0 || idx >= STEPPER_COUNT) return;
    s_mot[idx].max_speed = (steps_per_sec > 0.0f) ? steps_per_sec : 0.0f;
}

void stepper_set_acceleration(int idx, float steps_per_sec2)
{
    if (idx < 0 || idx >= STEPPER_COUNT) return;
    s_mot[idx].accel = (steps_per_sec2 > 0.0f) ? steps_per_sec2 : DEFAULT_ACCEL;
}

void stepper_move_to(int idx, long abs_steps)
{
    if (idx < 0 || idx >= STEPPER_COUNT) return;
    s_mot[idx].target = abs_steps;
}

long stepper_current(int idx)
{
    if (idx < 0 || idx >= STEPPER_COUNT) return 0;
    return s_mot[idx].current;
}

void steppers_move_to_all(const long target[STEPPER_COUNT])
{
    for (int i = 0; i < STEPPER_COUNT; i++) {
        s_mot[i].target = target[i];
    }
}

void steppers_run_to_position_blocking(void)
{
    for (;;) {
        bool done = true;
        for (int i = 0; i < STEPPER_COUNT; i++) {
            if (s_mot[i].current != s_mot[i].target) { done = false; break; }
        }
        if (done) return;
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}
