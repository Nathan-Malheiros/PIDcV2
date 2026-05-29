#include "control.h"

#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "touch_screen.h"
#include "kinematics.h"
#include "pid.h"
#include "steppers.h"

static const char *TAG = "control";

/* ── Machine geometry (mm) ──────────────────────────────────────────────────
 * PLACEHOLDERS scaled from Aaed Musa's reference build (inches -> mm).
 * MEASURE these four lengths on YOUR table and replace them. The numbers only
 * affect the tilt-normal -> motor-angle mapping; the PID itself works in mm
 * regardless, so the loop is stable for tuning even before they are exact.
 */
#define GEO_D   50.8     /* base radius          */
#define GEO_E   79.4     /* platform radius      */
#define GEO_F   44.45    /* motor arm length     */
#define GEO_G   93.2     /* connecting rod length */
#define GEO_HZ  108.0    /* neutral platform height */

/* ── Stepper scaling ───────────────────────────────────────────────────────
 * NEMA 17 = 1.8 deg/step = 200 full steps/rev. Set MICROSTEPS to match your
 * driver's DIP switches (the reference used 16 -> 3200 steps/rev).
 */
#define MICROSTEPS       16
#define STEPS_PER_REV    (200 * MICROSTEPS)
#define ANG_TO_STEP      (STEPS_PER_REV / 360.0)

/* ── Control loop ──────────────────────────────────────────────────────────*/
#define LOOP_HZ          50
#define LOOP_DT          (1.0f / LOOP_HZ)
#define LOOP_PERIOD_MS   (1000 / LOOP_HZ)

/* Setpoint: centre of the table by default. */
#define SETPOINT_X_MM    (TOUCH_WIDTH_MM  / 2.0f)
#define SETPOINT_Y_MM    (TOUCH_HEIGHT_MM / 2.0f)

/* Tilt-normal clamp — keep small so the platform never hits a mechanical stop.*/
#define TILT_LIMIT       0.25f

/* If the table pushes the ball AWAY from the centre instead of toward it,
 * set this to -1.0f (depends on motor wiring / arm orientation).            */
#define TILT_SIGN        (+1.0f)

/* ── PID gains (mm domain) ──────────────────────────────────────────────────
 * STARTING POINTS — expect to tune. Method: kp until it oscillates around the
 * centre, add kd to damp the oscillation, then a touch of ki to kill steady
 * offset. Tune them live in tools/sim.py (mode=sim) before trusting hardware.
 */
#define KP   8.0e-4f
#define KI   2.0e-5f
#define KD   1.2e-2f

/* Stepper speed (steps/sec) while balancing and while homing. */
#define SPEED_BALANCE    1500.0f
#define SPEED_HOME        800.0f

static machine_t s_machine;
static pid_t     s_pid_x, s_pid_y;
static double    s_ang_orig;     /* leg angle at perfectly level (nx=ny=0) */

/* Convert a tilt normal into the 3 motor step targets and command them. */
static void apply_tilt(double nx, double ny, double thoff[3])
{
    long pos[STEPPER_COUNT];
    for (int i = 0; i < STEPPER_COUNT; i++) {
        double theta = machine_theta(&s_machine, i, GEO_HZ, nx, ny);
        thoff[i] = theta;
        pos[i] = lround((s_ang_orig - theta) * ANG_TO_STEP);
    }
    steppers_move_to_all(pos);
}

void control_run(void)
{
    ESP_LOGI(TAG, "Ball Balancer — Control Mode @ %d Hz", LOOP_HZ);

    machine_init(&s_machine, GEO_D, GEO_E, GEO_F, GEO_G);
    pid_init(&s_pid_x, KP, KI, KD, -TILT_LIMIT, TILT_LIMIT);
    pid_init(&s_pid_y, KP, KI, KD, -TILT_LIMIT, TILT_LIMIT);

    /* Level reference: by symmetry every leg shares the same angle at nx=ny=0. */
    s_ang_orig = machine_theta(&s_machine, LEG_A, GEO_HZ, 0.0, 0.0);
    ESP_LOGI(TAG, "ang_orig = %.3f deg  (steps/rev=%d)", s_ang_orig, STEPS_PER_REV);

    touch_init();
    steppers_init();

    for (int i = 0; i < STEPPER_COUNT; i++) stepper_set_max_speed(i, SPEED_HOME);
    steppers_enable(true);
    vTaskDelay(pdMS_TO_TICKS(500));      /* let the user square the platform */

    /* Home to level and wait until the platform settles there. */
    double thoff[3];
    apply_tilt(0.0, 0.0, thoff);
    steppers_run_to_position_blocking();

    for (int i = 0; i < STEPPER_COUNT; i++) stepper_set_max_speed(i, SPEED_BALANCE);

    printf("# Ball Balancer control mode\n");
    printf("# CTRL,x_mm,y_mm,setx,sety,nx,ny,thA,thB,thC\n");

    bool detected = false;
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        touch_pos_t p;
        bool touched = touch_read(&p);

        double nx = 0.0, ny = 0.0;

        if (touched) {
            detected = true;
            float ox = pid_update(&s_pid_x, p.x_mm, SETPOINT_X_MM, LOOP_DT);
            float oy = pid_update(&s_pid_y, p.y_mm, SETPOINT_Y_MM, LOOP_DT);
            nx = TILT_SIGN * ox;
            ny = TILT_SIGN * oy;
        } else if (detected) {
            /* Ball just left the surface — relax to level and reset state. */
            detected = false;
            pid_reset(&s_pid_x);
            pid_reset(&s_pid_y);
        }

        apply_tilt(nx, ny, thoff);

        /* Telemetry for tools/sim.py and the visualizer. */
        if (touched) {
            printf("POS,%ld,%ld,%.1f,%.1f\n",
                   (long)p.x_raw, (long)p.y_raw, p.x_mm, p.y_mm);
        } else {
            printf("NOTOUCH,%ld,%ld\n", (long)p.x_raw, (long)p.y_raw);
        }
        printf("CTRL,%.1f,%.1f,%.1f,%.1f,%.4f,%.4f,%.2f,%.2f,%.2f\n",
               touched ? p.x_mm : -1.0f, touched ? p.y_mm : -1.0f,
               SETPOINT_X_MM, SETPOINT_Y_MM, nx, ny,
               thoff[0], thoff[1], thoff[2]);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(LOOP_PERIOD_MS));
    }
}
