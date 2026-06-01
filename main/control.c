#include "control.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/uart.h"

#include "touch_screen.h"
#include "kinematics.h"
#include "pid.h"
#include "steppers.h"
#include "cal_store.h"

static const char *TAG = "control";

/* ── Machine geometry (mm) ──────────────────────────────────────────────────
 * PLACEHOLDERS — MEDIR no rig real e substituir.
 */
#define GEO_D   50.8
#define GEO_E   79.4
#define GEO_F   44.45
#define GEO_G   93.2
#define GEO_HZ  108.0

/* ── Stepper scaling ──────────────────────────────────────────────────────── */
#define MICROSTEPS       16
#define STEPS_PER_REV    (200 * MICROSTEPS)
#define ANG_TO_STEP      (STEPS_PER_REV / 360.0)

/* ── Control loop ─────────────────────────────────────────────────────────── */
#define LOOP_HZ          50
#define LOOP_DT          (1.0f / LOOP_HZ)
#define LOOP_PERIOD_MS   (1000 / LOOP_HZ)

#define SETPOINT_X_MM    (TOUCH_WIDTH_MM  / 2.0f)
#define SETPOINT_Y_MM    (TOUCH_HEIGHT_MM / 2.0f)

#define TILT_LIMIT       0.25f

#define TILT_SIGN_X      (+1.0f)
#define TILT_SIGN_Y      (+1.0f)

/* ── PID default gains (mm domain) ───────────────────────────────────────────
 * Ajuste via serial em tempo real; os valores salvos no NVS têm prioridade.
 */
#define KP   8.0e-4f
#define KI   2.0e-5f
#define KD   1.2e-2f

/* ── Stepper speed/accel limits ──────────────────────────────────────────── */
#define SPEED_BALANCE    1500.0f
#define ACCEL_BALANCE   20000.0f
#define SPEED_HOME        800.0f
#define ACCEL_HOME       4000.0f

/* ── UART para comandos seriais ───────────────────────────────────────────── */
#define CMD_UART   UART_NUM_0

/* ── Estado compartilhado ─────────────────────────────────────────────────── */

static machine_t s_machine;
static pid_t     s_pid_x, s_pid_y;
static double    s_ang_orig;
static float     s_sign_x = TILT_SIGN_X;
static float     s_sign_y = TILT_SIGN_Y;

/* Último toque lido pelo loop principal; usado pelo cmd_task para capturar
 * cantos. Campos 32-bit são atômicos em ARM — sem mutex para leitura simples. */
static volatile int32_t s_snap_x_raw   = 0;
static volatile int32_t s_snap_y_raw   = 0;
static volatile bool    s_snap_touched = false;

/* ── Calibração de cantos da mesa ─────────────────────────────────────────
 *
 * Orientação (cabo flat da tela resistiva à DIREITA, olhando de frente):
 *
 *     TL ──────────── TR   ← cabo FPC aqui (direita)
 *     │                │
 *     │     MESA       │
 *     │                │
 *     BL ──────────── BR
 *
 * Convenção de saída após calibração:
 *   x_mm = 0 na borda ESQUERDA, TOUCH_WIDTH_MM na borda DIREITA
 *   y_mm = 0 na borda SUPERIOR,  TOUCH_HEIGHT_MM na borda INFERIOR
 */
#define COR_TL 0
#define COR_TR 1
#define COR_BL 2
#define COR_BR 3

typedef struct { int32_t x, y; bool valid; } corner_t;
static corner_t s_corners[4];

/* ── Impressão de estado ──────────────────────────────────────────────────── */

static void cmd_print_gains(void)
{
    printf("GAINS,%.6g,%.6g,%.6g\n",
           (double)s_pid_x.kp, (double)s_pid_x.ki, (double)s_pid_x.kd);
}

static void cmd_print_signs(void)
{
    printf("SIGNS,%+.0f,%+.0f\n", (double)s_sign_x, (double)s_sign_y);
}

static void cmd_print_cal(void)
{
    int32_t xmin, xmax, ymin, ymax;
    int fx, fy, sw;
    touch_get_cal(&xmin, &xmax, &ymin, &ymax, &fx, &fy, &sw);
    printf("CAL,%ld,%ld,%ld,%ld,%d,%d,%d\n",
           (long)xmin, (long)xmax, (long)ymin, (long)ymax, fx, fy, sw);
}

/* ── Comandos de calibração ───────────────────────────────────────────────── */

static void cmd_set_all_gains(float kp, float ki, float kd)
{
    s_pid_x.kp = kp; s_pid_x.ki = ki; s_pid_x.kd = kd;
    s_pid_y.kp = kp; s_pid_y.ki = ki; s_pid_y.kd = kd;
}

static void cmd_capture_corner(int idx)
{
    if (!s_snap_touched) { printf("ERR,no_touch\n"); return; }
    s_corners[idx].x     = s_snap_x_raw;
    s_corners[idx].y     = s_snap_y_raw;
    s_corners[idx].valid = true;
    const char *names[] = { "TL", "TR", "BL", "BR" };
    printf("CORNER,%s,%ld,%ld\n", names[idx],
           (long)s_corners[idx].x, (long)s_corners[idx].y);
}

static void cmd_cal_apply(void)
{
    if (!s_corners[COR_TL].valid || !s_corners[COR_TR].valid ||
        !s_corners[COR_BL].valid || !s_corners[COR_BR].valid) {
        printf("ERR,missing_corners\n");
        return;
    }

    /* Média de cada lado/aresta */
    int32_t left_x  = (s_corners[COR_TL].x + s_corners[COR_BL].x) / 2;
    int32_t right_x = (s_corners[COR_TR].x + s_corners[COR_BR].x) / 2;
    int32_t top_x   = (s_corners[COR_TL].x + s_corners[COR_TR].x) / 2;
    int32_t bot_x   = (s_corners[COR_BL].x + s_corners[COR_BR].x) / 2;
    int32_t left_y  = (s_corners[COR_TL].y + s_corners[COR_BL].y) / 2;
    int32_t right_y = (s_corners[COR_TR].y + s_corners[COR_BR].y) / 2;
    int32_t top_y   = (s_corners[COR_TL].y + s_corners[COR_TR].y) / 2;
    int32_t bot_y   = (s_corners[COR_BL].y + s_corners[COR_BR].y) / 2;

    /* Auto-detecção de swap_xy: se x_raw varia mais verticalmente do que
     * horizontalmente, o eixo elétrico X está mapeado no eixo físico Y.    */
    int32_t x_var_horiz = abs(right_x - left_x);
    int32_t x_var_vert  = abs(bot_x   - top_x);
    int swap_xy = (x_var_vert > x_var_horiz) ? 1 : 0;

    int32_t x_raw_min, x_raw_max, y_raw_min, y_raw_max;
    int flip_x, flip_y;

    if (!swap_xy) {
        /* x_raw → horizontal, y_raw → vertical */
        x_raw_min = left_x  < right_x ? left_x  : right_x;
        x_raw_max = left_x  > right_x ? left_x  : right_x;
        y_raw_min = top_y   < bot_y   ? top_y   : bot_y;
        y_raw_max = top_y   > bot_y   ? top_y   : bot_y;
        /* flip: se o lado esquerdo tem raw maior, precisamos inverter
         * para que x_mm=0 fique na esquerda.                           */
        flip_x = (left_x > right_x) ? 1 : 0;
        /* flip: se a borda superior tem raw maior, invertemos
         * para que y_mm=0 fique no topo.                               */
        flip_y = (top_y  > bot_y  ) ? 1 : 0;
    } else {
        /* x_raw → vertical, y_raw → horizontal (após swap: new_x=old_y, new_y=old_x) */
        x_raw_min = top_x  < bot_x   ? top_x  : bot_x;
        x_raw_max = top_x  > bot_x   ? top_x  : bot_x;
        y_raw_min = left_y < right_y ? left_y : right_y;
        y_raw_max = left_y > right_y ? left_y : right_y;
        flip_x = (left_y > right_y) ? 1 : 0;
        flip_y = (top_x  > bot_x  ) ? 1 : 0;
    }

    touch_set_cal(x_raw_min, x_raw_max, y_raw_min, y_raw_max, flip_x, flip_y, swap_xy);
    cmd_print_cal();
}

static void cmd_cal_save(void)
{
    int32_t xmin, xmax, ymin, ymax;
    int fx, fy, sw;
    touch_get_cal(&xmin, &xmax, &ymin, &ymax, &fx, &fy, &sw);
    cal_touch_t tc = { xmin, xmax, ymin, ymax, (int8_t)fx, (int8_t)fy, (int8_t)sw };
    cal_store_save_touch(&tc);
    printf("SAVED,touch\n");
    cmd_print_cal();
}

static void cmd_cal_reset(void)
{
    touch_set_cal(TOUCH_X_RAW_MIN, TOUCH_X_RAW_MAX,
                  TOUCH_Y_RAW_MIN, TOUCH_Y_RAW_MAX,
                  TOUCH_FLIP_X, TOUCH_FLIP_Y, TOUCH_SWAP_XY);
    for (int i = 0; i < 4; i++) s_corners[i].valid = false;
    cmd_print_cal();
}

static void cmd_handle_cal(const char *sub)
{
    if      (strcmp(sub, "TL")    == 0) { cmd_capture_corner(COR_TL); }
    else if (strcmp(sub, "TR")    == 0) { cmd_capture_corner(COR_TR); }
    else if (strcmp(sub, "BL")    == 0) { cmd_capture_corner(COR_BL); }
    else if (strcmp(sub, "BR")    == 0) { cmd_capture_corner(COR_BR); }
    else if (strcmp(sub, "APPLY") == 0) { cmd_cal_apply(); }
    else if (strcmp(sub, "SAVE")  == 0) { cmd_cal_save(); }
    else if (strcmp(sub, "SHOW")  == 0) { cmd_print_cal(); }
    else if (strcmp(sub, "RESET") == 0) { cmd_cal_reset(); }
    else { printf("ERR,unknown_cal_cmd\n"); }
}

/* ── Parser de linha serial ───────────────────────────────────────────────── */

static void cmd_handle(char *s)
{
    /* converter para maiúsculas */
    for (char *p = s; *p; ++p) *p = (char)toupper((unsigned char)*p);

    float a, b, c;

    /* SETCAL <xmin> <xmax> <ymin> <ymax> <flip_x> <flip_y> <swap_xy>
     * Comando enviado pelo calibrate.py / pid_tuner.py para aplicar a
     * calibracao calculada no PC diretamente, sem precisar de rebuild.
     * Diferente de "CAL TL/TR..." que captura o toque atual do hardware,
     * SETCAL aceita os valores crus calculados externamente. */
    int ixmin, ixmax, iymin, iymax, ifx, ify, isw;
    if (sscanf(s, "SETCAL %d %d %d %d %d %d %d",
               &ixmin, &ixmax, &iymin, &iymax, &ifx, &ify, &isw) == 7) {
        touch_set_cal(ixmin, ixmax, iymin, iymax, ifx, ify, isw);
        cmd_print_cal();
        return;
    }

    /* PID SAVE — deve vir antes do "PID %f %f %f" para não haver conflito */
    if (strncmp(s, "PID SAVE", 8) == 0) {
        cal_pid_t p = { s_pid_x.kp, s_pid_x.ki, s_pid_x.kd };
        cal_store_save_pid(&p);
        printf("SAVED,pid\n");
        cmd_print_gains();
    }
    /* Ajuste de ganhos PID:
     *   PID <kp> <ki> <kd>   — ajusta os três de uma vez
     *   KP <v> | KI <v> | KD <v> — ajusta um por vez
     *   ?                    — consulta ganhos e sinais
     *   SX <v> | SY <v>      — sinal do controle por eixo (+1/-1)
     */
    else if (sscanf(s, "PID %f %f %f", &a, &b, &c) == 3) {
        cmd_set_all_gains(a, b, c);
        cmd_print_gains();
    } else if (sscanf(s, "KP %f", &a) == 1) {
        s_pid_x.kp = a; s_pid_y.kp = a; cmd_print_gains();
    } else if (sscanf(s, "KI %f", &a) == 1) {
        s_pid_x.ki = a; s_pid_y.ki = a; cmd_print_gains();
    } else if (sscanf(s, "KD %f", &a) == 1) {
        s_pid_x.kd = a; s_pid_y.kd = a; cmd_print_gains();
    } else if (sscanf(s, "SX %f", &a) == 1) {
        s_sign_x = (a < 0.0f) ? -1.0f : +1.0f; cmd_print_signs();
    } else if (sscanf(s, "SY %f", &a) == 1) {
        s_sign_y = (a < 0.0f) ? -1.0f : +1.0f; cmd_print_signs();
    }
    /* Calibração da mesa — CAL <subcomando> */
    else if (strncmp(s, "CAL ", 4) == 0) {
        cmd_handle_cal(s + 4);
    }
    else if (s[0] == '?') {
        cmd_print_gains();
        cmd_print_signs();
        cmd_print_cal();
    }
    /* comando desconhecido: ignora */
}

/* ── Task serial (leitura UART em background) ─────────────────────────────── */

static void cmd_task(void *arg)
{
    uint8_t ch;
    char    line[64];
    int     n = 0;
    for (;;) {
        int r = uart_read_bytes(CMD_UART, &ch, 1, pdMS_TO_TICKS(100));
        if (r != 1) continue;
        if (ch == '\n' || ch == '\r') {
            if (n > 0) { line[n] = '\0'; cmd_handle(line); n = 0; }
        } else if (n < (int)sizeof(line) - 1) {
            line[n++] = (char)ch;
        }
    }
}

/* ── Cinemática → motores ─────────────────────────────────────────────────── */

static void apply_tilt(double nx, double ny, double thoff[3])
{
    long pos[STEPPER_COUNT];
    for (int i = 0; i < STEPPER_COUNT; i++) {
        double theta = machine_theta(&s_machine, i, GEO_HZ, nx, ny);
        if (!isfinite(theta)) theta = s_ang_orig;
        thoff[i] = theta;
        pos[i] = lround((s_ang_orig - theta) * ANG_TO_STEP);
    }
    steppers_move_to_all(pos);
}

/* ── Entry point ──────────────────────────────────────────────────────────── */

void control_run(void)
{
    ESP_LOGI(TAG, "Ball Balancer — Control Mode @ %d Hz", LOOP_HZ);

    /* NVS deve ser inicializado antes de qualquer load/save */
    cal_store_init();

    machine_init(&s_machine, GEO_D, GEO_E, GEO_F, GEO_G);

    /* Carrega ganhos PID do NVS; usa defaults de compilação se não há nada salvo */
    cal_pid_t saved_pid;
    if (cal_store_load_pid(&saved_pid)) {
        pid_init(&s_pid_x, saved_pid.kp, saved_pid.ki, saved_pid.kd,
                 -TILT_LIMIT, TILT_LIMIT);
        pid_init(&s_pid_y, saved_pid.kp, saved_pid.ki, saved_pid.kd,
                 -TILT_LIMIT, TILT_LIMIT);
    } else {
        pid_init(&s_pid_x, KP, KI, KD, -TILT_LIMIT, TILT_LIMIT);
        pid_init(&s_pid_y, KP, KI, KD, -TILT_LIMIT, TILT_LIMIT);
    }

    s_ang_orig = machine_theta(&s_machine, LEG_A, GEO_HZ, 0.0, 0.0);
    ESP_LOGI(TAG, "ang_orig = %.3f deg  (steps/rev=%d)", s_ang_orig, STEPS_PER_REV);

    touch_init();

    /* Carrega calibração do touch do NVS se disponível */
    cal_touch_t saved_touch;
    if (cal_store_load_touch(&saved_touch)) {
        touch_set_cal(saved_touch.x_raw_min, saved_touch.x_raw_max,
                      saved_touch.y_raw_min, saved_touch.y_raw_max,
                      saved_touch.flip_x, saved_touch.flip_y, saved_touch.swap_xy);
    }

    steppers_init();

    for (int i = 0; i < STEPPER_COUNT; i++) {
        stepper_set_max_speed(i, SPEED_HOME);
        stepper_set_acceleration(i, ACCEL_HOME);
    }
    steppers_enable(true);
    vTaskDelay(pdMS_TO_TICKS(500));

    double thoff[3];
    apply_tilt(0.0, 0.0, thoff);
    steppers_run_to_position_blocking();

    for (int i = 0; i < STEPPER_COUNT; i++) {
        stepper_set_max_speed(i, SPEED_BALANCE);
        stepper_set_acceleration(i, ACCEL_BALANCE);
    }

    printf("# Ball Balancer control mode\n");
    printf("# Comandos seriais disponíveis:\n");
    printf("#   PID <kp> <ki> <kd>  KP/KI/KD <v>  SX/SY <v>  PID SAVE\n");
    printf("#   CAL TL|TR|BL|BR     CAL APPLY  CAL SAVE  CAL SHOW  CAL RESET\n");
    printf("#   ? — exibe estado atual\n");

    uart_driver_install(CMD_UART, 256, 0, 0, NULL, 0);
    xTaskCreate(cmd_task, "pid_cmd", 3072, NULL, 5, NULL);

    cmd_print_gains();
    cmd_print_signs();
    cmd_print_cal();

    bool detected = false;
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        touch_pos_t p;
        bool touched = touch_read(&p);

        /* Atualiza snapshot compartilhado com o cmd_task */
        s_snap_touched = touched;
        if (touched) {
            s_snap_x_raw = p.x_raw;
            s_snap_y_raw = p.y_raw;
        }

        double nx = 0.0, ny = 0.0;

        if (touched) {
            detected = true;
            float ox = pid_update(&s_pid_x, p.x_mm, SETPOINT_X_MM, LOOP_DT);
            float oy = pid_update(&s_pid_y, p.y_mm, SETPOINT_Y_MM, LOOP_DT);
            nx = s_sign_x * ox;
            ny = s_sign_y * oy;
        } else if (detected) {
            detected = false;
            pid_reset(&s_pid_x);
            pid_reset(&s_pid_y);
        }

        apply_tilt(nx, ny, thoff);

        /* Telemetria */
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
