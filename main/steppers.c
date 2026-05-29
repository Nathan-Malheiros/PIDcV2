#include "steppers.h"

#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TICK_PERIOD_US   100          /* 10 kHz step engine */
#define STEP_PULSE_US      3          /* STEP high time (drivers need ~1-2 us) */

typedef struct {
    int   step_pin;
    int   dir_pin;
    volatile long  current;           /* current absolute position (steps) */
    volatile long  target;            /* desired absolute position (steps)  */
    volatile int64_t interval_us;     /* min us between steps (1e6 / speed)  */
    int64_t last_step_us;
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

/* esp_timer callback (TASK dispatch): step every motor that is due. */
static void step_engine(void *arg)
{
    int64_t now = esp_timer_get_time();

    for (int i = 0; i < STEPPER_COUNT; i++) {
        stepper_t *m = &s_mot[i];
        long cur = m->current;
        long tgt = m->target;
        if (cur == tgt || m->interval_us <= 0) continue;
        if ((now - m->last_step_us) < m->interval_us) continue;

        int dir = (tgt > cur) ? 1 : -1;
        gpio_set_level(m->dir_pin, dir > 0 ? 1 : 0);

        gpio_set_level(m->step_pin, 1);
        esp_rom_delay_us(STEP_PULSE_US);
        gpio_set_level(m->step_pin, 0);

        m->current = cur + dir;
        m->last_step_us = now;
    }
}

void steppers_init(void)
{
    out_pin(STEPPER_EN_PIN);
    gpio_set_level(STEPPER_EN_PIN, 1);   /* start disabled (active LOW) */

    for (int i = 0; i < STEPPER_COUNT; i++) {
        out_pin(s_mot[i].step_pin);
        out_pin(s_mot[i].dir_pin);
        s_mot[i].current      = 0;
        s_mot[i].target       = 0;
        s_mot[i].interval_us  = 0;
        s_mot[i].last_step_us = 0;
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
    gpio_set_level(STEPPER_EN_PIN, on ? 0 : 1);  /* active LOW */
}

void stepper_set_max_speed(int idx, float steps_per_sec)
{
    if (idx < 0 || idx >= STEPPER_COUNT) return;
    if (steps_per_sec <= 0.0f) {
        s_mot[idx].interval_us = 0;
    } else {
        s_mot[idx].interval_us = (int64_t)(1e6f / steps_per_sec);
    }
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
