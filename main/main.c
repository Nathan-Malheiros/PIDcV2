#include <stdio.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "touch_screen.h"
#include "control.h"

/* ── Firmware mode ───────────────────────────────────────────────────────────
 *   BB_MODE_MAPPING : read-only touch mapping (raw + mm telemetry, no motors).
 *   BB_MODE_CONTROL : full balancing loop (PID + 3RPS kinematics + steppers).
 * Override at build time with  -DBB_MODE=BB_MODE_MAPPING  if you need to remap.
 */
#define BB_MODE_MAPPING  0
#define BB_MODE_CONTROL  1
#ifndef BB_MODE
#define BB_MODE  BB_MODE_CONTROL
#endif

/*
 * Serial output protocol (parsed by tools/visualizer.py):
 *
 *   POS,<x_raw>,<y_raw>,<x_mm>,<y_mm>
 *       Valid touch — filtered mm + raw ADC values.
 *
 *   NOTOUCH,<x_raw>,<y_raw>
 *       No contact detected; raw values shown for diagnostics.
 *
 *   CAL_HINT,<obs_x_min>,<obs_x_max>,<obs_y_min>,<obs_y_max>
 *       Observed raw range since boot — emitted every CAL_HINT_MS.
 *       Move the ball to all four corners to capture the full range,
 *       then enter these values in bb_firmware_tool option 8 (Calibracao).
 */

#if BB_MODE == BB_MODE_MAPPING

static const char *TAG = "main";

#define SAMPLE_PERIOD_MS    20     /* 50 Hz */
#define CAL_HINT_MS       5000     /* print calibration hint every 5 s */

/* ── Calibration range observer ─────────────────────────────────────────── */

typedef struct {
    int32_t x_min, x_max;
    int32_t y_min, y_max;
    bool    has_data;
} cal_obs_t;

static void cal_obs_reset(cal_obs_t *c)
{
    c->x_min =  0x7FFFFFFF;
    c->x_max = -0x7FFFFFFF;
    c->y_min =  0x7FFFFFFF;
    c->y_max = -0x7FFFFFFF;
    c->has_data = false;
}

static void cal_obs_update(cal_obs_t *c, int32_t xr, int32_t yr)
{
    if (xr < c->x_min) c->x_min = xr;
    if (xr > c->x_max) c->x_max = xr;
    if (yr < c->y_min) c->y_min = yr;
    if (yr > c->y_max) c->y_max = yr;
    c->has_data = true;
}

/* ── Touch mapping loop (BB_MODE_MAPPING) ──────────────────────────────────── */

static void mapping_loop(void)
{
    ESP_LOGI(TAG, "Ball Balancer — Touch Mapping Mode");
    ESP_LOGI(TAG, "Screen: %.0f x %.0f mm", TOUCH_WIDTH_MM, TOUCH_HEIGHT_MM);
    ESP_LOGI(TAG, "Cal constants:  X[%d-%d]  Y[%d-%d]",
             TOUCH_X_RAW_MIN, TOUCH_X_RAW_MAX,
             TOUCH_Y_RAW_MIN, TOUCH_Y_RAW_MAX);
    ESP_LOGI(TAG, "Output: POS,x_raw,y_raw,x_mm,y_mm  @%d Hz", 1000 / SAMPLE_PERIOD_MS);

    touch_init();

    /* Print a header comment that the visualizer ignores */
    printf("# Ball Balancer touch mapping\n");
    printf("# Format: POS,x_raw,y_raw,x_mm,y_mm\n");
    printf("# Screen: %.0f x %.0f mm\n", TOUCH_WIDTH_MM, TOUCH_HEIGHT_MM);
    printf("# Cal: X[%d-%d] Y[%d-%d]\n",
           TOUCH_X_RAW_MIN, TOUCH_X_RAW_MAX,
           TOUCH_Y_RAW_MIN, TOUCH_Y_RAW_MAX);

    cal_obs_t obs;
    cal_obs_reset(&obs);

    touch_pos_t pos;
    TickType_t  last_wake    = xTaskGetTickCount();
    TickType_t  last_cal_hint = xTaskGetTickCount();

    while (1) {
        bool touched = touch_read(&pos);

        if (touched) {
            /* Primary data line — parsed by visualizer */
            printf("POS,%ld,%ld,%.1f,%.1f\n",
                   (long)pos.x_raw, (long)pos.y_raw,
                   pos.x_mm, pos.y_mm);

            cal_obs_update(&obs, pos.x_raw, pos.y_raw);
        } else {
            /* Diagnostic line when no contact */
            printf("NOTOUCH,%ld,%ld\n",
                   (long)pos.x_raw, (long)pos.y_raw);
        }

        /* Periodic calibration hint */
        TickType_t now = xTaskGetTickCount();
        if ((now - last_cal_hint) >= pdMS_TO_TICKS(CAL_HINT_MS)) {
            last_cal_hint = now;
            if (obs.has_data) {
                printf("CAL_HINT,%ld,%ld,%ld,%ld\n",
                       (long)obs.x_min, (long)obs.x_max,
                       (long)obs.y_min, (long)obs.y_max);
                ESP_LOGI(TAG, "Observed range  X[%ld-%ld]  Y[%ld-%ld]  "
                              "(update via bb_firmware_tool option 8 if needed)",
                         (long)obs.x_min, (long)obs.x_max,
                         (long)obs.y_min, (long)obs.y_max);
            } else {
                printf("CAL_HINT,no_data\n");
                ESP_LOGI(TAG, "No touch detected yet — move ball across the full surface");
            }
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
}

#endif /* BB_MODE == BB_MODE_MAPPING */

/* ── Entry point ─────────────────────────────────────────────────────────── */

void app_main(void)
{
    /* Unbuffered stdout so every printf reaches the serial monitor immediately */
    setvbuf(stdout, NULL, _IONBF, 0);

#if BB_MODE == BB_MODE_CONTROL
    control_run();      /* never returns */
#else
    mapping_loop();     /* never returns */
#endif
}
