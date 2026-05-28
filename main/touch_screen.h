#pragma once

#include <stdint.h>
#include <stdbool.h>

/* ── Physical dimensions (VSDISPLAY VS084TP-A1, 8.4-inch) ───────────────── */
#define TOUCH_WIDTH_MM    187.0f
#define TOUCH_HEIGHT_MM   141.0f

/* ── GPIO assignments (verified physically, FPC connector) ──────────────── */
/*    FPC pin 1 = Y+  →  GPIO 4   ADC1_CH3   (sense for X measurement)      */
/*    FPC pin 2 = X-  →  GPIO 5   ADC1_CH4   (sense for Y measurement)      */
/*    FPC pin 3 = Y-  →  GPIO 8   ADC1_CH7   (driven during Y measurement)  */
/*    FPC pin 4 = X+  →  GPIO 9   output only — ADC inoperante nesta placa  */
#define TOUCH_PIN_XP   9
#define TOUCH_PIN_XM   5
#define TOUCH_PIN_YP   4
#define TOUCH_PIN_YM   8

/* ── ADC calibration boundaries (12-bit raw, 0–4095) ────────────────────── */
/*    Updated via bb_firmware_tool option 8.                                  */
/*    These shift slightly between power cycles — recalibrate when needed.   */
#define TOUCH_X_RAW_MIN  2000
#define TOUCH_X_RAW_MAX  3700
#define TOUCH_Y_RAW_MIN  1700
#define TOUCH_Y_RAW_MAX  2600

/* ── Tuning parameters ───────────────────────────────────────────────────── */

/* ADC samples averaged per axis per read cycle. */
#define TOUCH_SAMPLES        8

/*
 * IIR low-pass filter weight applied to the converted mm value.
 * Range: 0.0 (frozen) – 1.0 (no filter).
 * 0.25 gives ~4-sample settling time — adequate for a 50 Hz PID loop.
 */
#define TOUCH_IIR_ALPHA      0.25f

/*
 * ADC threshold for touch detection.
 * Each sense pin is discharged to ~0 before reading, so with no contact it
 * stays near 0. A real contact bridges the layers and the sense pin charges
 * to the divider voltage (>1700 raw). 300 sits safely in the gap and rejects
 * ADC noise. A touch is only accepted when BOTH axes read above this.
 */
#define TOUCH_DETECT_THRESHOLD  300

/*
 * Microsecond timings for the resistive scan.
 *   DISCHARGE: time the sense node is pulled to 0 before floating to read.
 *   SETTLE:    time for the node to charge through the contact before sampling.
 * Resistive panels settle in well under 100 us; these are conservative.
 */
#define TOUCH_DISCHARGE_US   150
#define TOUCH_SETTLE_US      150

/* ── Data type ───────────────────────────────────────────────────────────── */

typedef struct {
    float   x_mm;    /* Filtered X position  [0 .. TOUCH_WIDTH_MM]  */
    float   y_mm;    /* Filtered Y position  [0 .. TOUCH_HEIGHT_MM] */
    int32_t x_raw;   /* Averaged raw ADC for X axis                 */
    int32_t y_raw;   /* Averaged raw ADC for Y axis                 */
    bool    valid;   /* true = object detected on surface           */
} touch_pos_t;

/* ── Public API ──────────────────────────────────────────────────────────── */

/**
 * Initialise GPIO and ADC peripheral.
 * Call once before any touch_read() call.
 */
void touch_init(void);

/**
 * Perform one full X+Y measurement cycle.
 *
 * Switching sequence:
 *   1. GPIO touch-detection check (no ADC needed).
 *   2. Drive X axis, ADC-sense on Y+ (GPIO 4) → X coordinate.
 *   3. Drive Y axis, ADC-sense on X- (GPIO 5) → Y coordinate.
 *   4. Apply IIR filter and convert raw → mm.
 *
 * @param[out] pos  Populated on every call; pos->valid indicates contact.
 * @return          Same value as pos->valid.
 */
bool touch_read(touch_pos_t *pos);
