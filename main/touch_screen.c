#include "touch_screen.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_rom_sys.h"
#include "esp_log.h"

static const char *TAG = "touch";

/*
 * ADC1 channel map (ESP32-S3):
 *   GPIO 4  →  ADC1_CHANNEL_3   Y+  sensed when measuring X
 *   GPIO 5  →  ADC1_CHANNEL_4   X-  sensed when measuring Y
 *   GPIO 9  →  X+  output only — ADC inoperante nesta placa
 */
#define ADC_CHAN_YP   ADC_CHANNEL_3
#define ADC_CHAN_XM   ADC_CHANNEL_4

static adc_oneshot_unit_handle_t s_adc;

static const adc_oneshot_chan_cfg_t s_ch_cfg = {
    .atten    = ADC_ATTEN_DB_12,
    .bitwidth = ADC_BITWIDTH_12,
};

static float s_x_filt = -1.0f;
static float s_y_filt = -1.0f;

/* ── Runtime calibration (initialised from compile-time defaults) ─────────── */

static int32_t s_x_raw_min = TOUCH_X_RAW_MIN;
static int32_t s_x_raw_max = TOUCH_X_RAW_MAX;
static int32_t s_y_raw_min = TOUCH_Y_RAW_MIN;
static int32_t s_y_raw_max = TOUCH_Y_RAW_MAX;
static int      s_flip_x   = TOUCH_FLIP_X;
static int      s_flip_y   = TOUCH_FLIP_Y;
static int      s_swap_xy  = TOUCH_SWAP_XY;

void touch_set_cal(int32_t x_raw_min, int32_t x_raw_max,
                   int32_t y_raw_min, int32_t y_raw_max,
                   int flip_x, int flip_y, int swap_xy)
{
    s_x_raw_min = x_raw_min;
    s_x_raw_max = x_raw_max;
    s_y_raw_min = y_raw_min;
    s_y_raw_max = y_raw_max;
    s_flip_x    = flip_x;
    s_flip_y    = flip_y;
    s_swap_xy   = swap_xy;
    /* reset filter so the first sample after recalibration is seed-fresh */
    s_x_filt = -1.0f;
    s_y_filt = -1.0f;
    ESP_LOGI(TAG, "cal updated  X[%ld-%ld]  Y[%ld-%ld]  flip=%d/%d  swap=%d",
             (long)x_raw_min, (long)x_raw_max,
             (long)y_raw_min, (long)y_raw_max,
             flip_x, flip_y, swap_xy);
}

void touch_get_cal(int32_t *x_raw_min, int32_t *x_raw_max,
                   int32_t *y_raw_min, int32_t *y_raw_max,
                   int *flip_x, int *flip_y, int *swap_xy)
{
    *x_raw_min = s_x_raw_min;
    *x_raw_max = s_x_raw_max;
    *y_raw_min = s_y_raw_min;
    *y_raw_max = s_y_raw_max;
    *flip_x    = s_flip_x;
    *flip_y    = s_flip_y;
    *swap_xy   = s_swap_xy;
}

/* ── GPIO helpers ────────────────────────────────────────────────────────── */

static void pin_out(int pin, int level)
{
    gpio_config_t c = {
        .pin_bit_mask = (1ULL << pin),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&c);
    gpio_set_level(pin, level);
}

static void pin_in(int pin)
{
    gpio_config_t c = {
        .pin_bit_mask = (1ULL << pin),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&c);
}

/* ── One-axis read ───────────────────────────────────────────────────────── */

static int32_t read_axis(int drive_hi, int drive_lo, int float_pin,
                         int sense_pin, adc_channel_t sense_ch)
{
    pin_out(drive_hi, 1);
    pin_out(drive_lo, 0);
    pin_in(float_pin);

    pin_out(sense_pin, 0);
    esp_rom_delay_us(TOUCH_DISCHARGE_US);

    adc_oneshot_config_channel(s_adc, sense_ch, &s_ch_cfg);
    esp_rom_delay_us(TOUCH_SETTLE_US);

    int64_t sum = 0;
    for (int i = 0; i < TOUCH_SAMPLES; i++) {
        int raw = 0;
        adc_oneshot_read(s_adc, sense_ch, &raw);
        sum += raw;
    }
    return (int32_t)(sum / TOUCH_SAMPLES);
}

/* ── Public API ──────────────────────────────────────────────────────────── */

void touch_init(void)
{
    adc_oneshot_unit_init_cfg_t unit_cfg = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &s_adc));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, ADC_CHAN_YP, &s_ch_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, ADC_CHAN_XM, &s_ch_cfg));

    pin_in(TOUCH_PIN_XP);
    pin_in(TOUCH_PIN_XM);
    pin_in(TOUCH_PIN_YP);
    pin_in(TOUCH_PIN_YM);

    ESP_LOGI(TAG, "init OK  XP=%d XM=%d YP=%d YM=%d",
             TOUCH_PIN_XP, TOUCH_PIN_XM, TOUCH_PIN_YP, TOUCH_PIN_YM);
    ESP_LOGI(TAG, "cal X[%ld-%ld] Y[%ld-%ld] flip=%d/%d swap=%d detect>%d",
             (long)s_x_raw_min, (long)s_x_raw_max,
             (long)s_y_raw_min, (long)s_y_raw_max,
             s_flip_x, s_flip_y, s_swap_xy,
             TOUCH_DETECT_THRESHOLD);
}

bool touch_read(touch_pos_t *pos)
{
    memset(pos, 0, sizeof(*pos));

    /* X: drive X+ HIGH / X- LOW, float Y-, sense Y+ (GPIO 4) */
    int32_t x_raw = read_axis(TOUCH_PIN_XP, TOUCH_PIN_XM,
                              TOUCH_PIN_YM, TOUCH_PIN_YP, ADC_CHAN_YP);

    /* Y: drive Y+ HIGH / Y- LOW, float X+, sense X- (GPIO 5) */
    int32_t y_raw = read_axis(TOUCH_PIN_YP, TOUCH_PIN_YM,
                              TOUCH_PIN_XP, TOUCH_PIN_XM, ADC_CHAN_XM);

    pin_in(TOUCH_PIN_XP);
    pin_in(TOUCH_PIN_XM);
    pin_in(TOUCH_PIN_YP);
    pin_in(TOUCH_PIN_YM);

    pos->x_raw = x_raw;
    pos->y_raw = y_raw;

    if (x_raw < TOUCH_DETECT_THRESHOLD || y_raw < TOUCH_DETECT_THRESHOLD) {
        s_x_filt   = -1.0f;
        s_y_filt   = -1.0f;
        pos->valid = false;
        return false;
    }

    /* raw → normalised [0,1] using runtime calibration */
    float x_norm = (float)(x_raw - s_x_raw_min) / (float)(s_x_raw_max - s_x_raw_min);
    float y_norm = (float)(y_raw - s_y_raw_min) / (float)(s_y_raw_max - s_y_raw_min);

    if (x_norm < 0.0f) x_norm = 0.0f;
    if (x_norm > 1.0f) x_norm = 1.0f;
    if (y_norm < 0.0f) y_norm = 0.0f;
    if (y_norm > 1.0f) y_norm = 1.0f;

    /* orientation transform using runtime flags */
    if (s_swap_xy) { float t = x_norm; x_norm = y_norm; y_norm = t; }
    if (s_flip_x)  { x_norm = 1.0f - x_norm; }
    if (s_flip_y)  { y_norm = 1.0f - y_norm; }

    float x_mm = x_norm * TOUCH_WIDTH_MM;
    float y_mm = y_norm * TOUCH_HEIGHT_MM;

    if (s_x_filt < 0.0f) {
        s_x_filt = x_mm;
        s_y_filt = y_mm;
    } else {
        s_x_filt = TOUCH_IIR_ALPHA * x_mm + (1.0f - TOUCH_IIR_ALPHA) * s_x_filt;
        s_y_filt = TOUCH_IIR_ALPHA * y_mm + (1.0f - TOUCH_IIR_ALPHA) * s_y_filt;
    }

    pos->x_mm  = s_x_filt;
    pos->y_mm  = s_y_filt;
    pos->valid = true;
    return true;
}
