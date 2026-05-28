#include "touch_screen.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_rom_sys.h"     /* esp_rom_delay_us */
#include "esp_log.h"

static const char *TAG = "touch";

/*
 * ADC1 channel map (ESP32-S3):
 *   GPIO 4  →  ADC1_CHANNEL_3   Y+  sensed when measuring X
 *   GPIO 5  →  ADC1_CHANNEL_4   X-  sensed when measuring Y
 *   GPIO 9  →  X+  output only — ADC inoperante nesta placa
 *
 * The two sense pins (GPIO 4 / GPIO 5) alternate between being driven as a
 * digital output (to excite the panel) and being read as an ADC input. After
 * a pin has been used as a digital GPIO, its analog routing must be
 * re-established before the ADC can read it again — that is why
 * adc_oneshot_config_channel() is called inside read_axis() on every cycle.
 */
#define ADC_CHAN_YP   ADC_CHANNEL_3
#define ADC_CHAN_XM   ADC_CHANNEL_4

static adc_oneshot_unit_handle_t s_adc;

static const adc_oneshot_chan_cfg_t s_ch_cfg = {
    .atten    = ADC_ATTEN_DB_12,     /* full 0–~3.1 V span */
    .bitwidth = ADC_BITWIDTH_12,
};

static float s_x_filt = -1.0f;
static float s_y_filt = -1.0f;

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

/* ── One-axis read ───────────────────────────────────────────────────────────
 *
 * drive_hi / drive_lo : the resistive strip terminals (HIGH / LOW gradient)
 * float_pin           : opposite-layer terminal left high-impedance
 * sense_pin / sense_ch: the layer probe — discharged, then ADC-read
 *
 * Sequence:
 *   1. Establish the voltage gradient across the driven strip.
 *   2. Pull the sense node to 0 (discharge) so a floating "no touch" reads ~0.
 *   3. Reconnect the sense pad to the ADC (restores analog after digital use).
 *   4. Let it settle — with contact, it charges to the divider voltage in µs.
 *   5. Average TOUCH_SAMPLES conversions.
 */
static int32_t read_axis(int drive_hi, int drive_lo, int float_pin,
                         int sense_pin, adc_channel_t sense_ch)
{
    pin_out(drive_hi, 1);
    pin_out(drive_lo, 0);
    pin_in(float_pin);

    /* discharge the sense node to a known 0 */
    pin_out(sense_pin, 0);
    esp_rom_delay_us(TOUCH_DISCHARGE_US);

    /* re-route the sense pad to the ADC (critical after digital GPIO use) */
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
    ESP_LOGI(TAG, "cal X[%d-%d] Y[%d-%d] detect>%d",
             TOUCH_X_RAW_MIN, TOUCH_X_RAW_MAX,
             TOUCH_Y_RAW_MIN, TOUCH_Y_RAW_MAX,
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

    /* park everything high-impedance */
    pin_in(TOUCH_PIN_XP);
    pin_in(TOUCH_PIN_XM);
    pin_in(TOUCH_PIN_YP);
    pin_in(TOUCH_PIN_YM);

    /* raw is ALWAYS reported, even with no touch, for mapping/diagnostics */
    pos->x_raw = x_raw;
    pos->y_raw = y_raw;

    /* Touch detection: both probes must rise out of the discharged baseline. */
    if (x_raw < TOUCH_DETECT_THRESHOLD || y_raw < TOUCH_DETECT_THRESHOLD) {
        s_x_filt = -1.0f;
        s_y_filt = -1.0f;
        pos->valid = false;
        return false;
    }

    /* raw → mm (linear, clamped). No range rejection: during mapping the raw
     * may legitimately exceed the stale calibration window. */
    float x_norm = (float)(x_raw - TOUCH_X_RAW_MIN) /
                   (float)(TOUCH_X_RAW_MAX - TOUCH_X_RAW_MIN);
    float y_norm = (float)(y_raw - TOUCH_Y_RAW_MIN) /
                   (float)(TOUCH_Y_RAW_MAX - TOUCH_Y_RAW_MIN);

    if (x_norm < 0.0f) x_norm = 0.0f;
    if (x_norm > 1.0f) x_norm = 1.0f;
    if (y_norm < 0.0f) y_norm = 0.0f;
    if (y_norm > 1.0f) y_norm = 1.0f;

    /* orientation transform (see touch_screen.h / tools/config.py) */
    if (TOUCH_SWAP_XY) { float t = x_norm; x_norm = y_norm; y_norm = t; }
    if (TOUCH_FLIP_X)  { x_norm = 1.0f - x_norm; }
    if (TOUCH_FLIP_Y)  { y_norm = 1.0f - y_norm; }

    float x_mm = x_norm * TOUCH_WIDTH_MM;
    float y_mm = y_norm * TOUCH_HEIGHT_MM;

    /* IIR low-pass; first sample after lift-off seeds directly */
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
