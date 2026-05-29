#include "control.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/uart.h"

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

/* Sinal do controle POR EIXO. Se a mesa empurra a bola para LONGE do centro
 * num eixo (eixo "invertido"), inverta o sinal daquele eixo para -1. Depende
 * da fiacao dos motores e da orientacao do touch. AJUSTAVEL EM TEMPO REAL pela
 * serial:  SX <v>  e  SY <v>  (v < 0 vira -1, v >= 0 vira +1). */
#define TILT_SIGN_X      (+1.0f)
#define TILT_SIGN_Y      (+1.0f)

/* ── PID gains (mm domain) ──────────────────────────────────────────────────
 * STARTING POINTS — expect to tune ON HARDWARE. Estes ganhos NAO sao um porte
 * direto da base codes/Ball_Balancing.ino: la o erro esta em unidades CRUAS do
 * touch e a derivada/integral nao sao normalizadas por dt; aqui o erro esta em
 * mm e usamos PID real (integral em mm*s, derivada em mm/s). Logo os numeros
 * tem escalas diferentes — re-tune. Metodo: suba kp ate oscilar em torno do
 * centro, adicione kd para amortecer, depois um toque de ki para matar offset.
 * Ajuste ao vivo com os ganhos editaveis do tools/PIDSimba.py (modo SIM).
 */
#define KP   8.0e-4f
#define KI   2.0e-5f
#define KD   1.2e-2f

/* Stepper limits (steps/sec, steps/sec^2) while balancing and while homing.
 * O motor de passos agora tem perfil trapezoidal (ver steppers.c): o eixo
 * acelera ate SPEED_* e desacelera para parar no alvo. ACCEL controla a
 * suavidade — alto = responde rapido; baixo = mais suave (porem mais atraso).
 * Na base do Aaed Musa a aceleracao era ~ velocidade*30. */
#define SPEED_BALANCE    1500.0f
#define ACCEL_BALANCE   20000.0f
#define SPEED_HOME        800.0f
#define ACCEL_HOME       4000.0f

static machine_t s_machine;
static pid_t     s_pid_x, s_pid_y;
static double    s_ang_orig;     /* leg angle at perfectly level (nx=ny=0) */
static float     s_sign_x = TILT_SIGN_X;   /* sinal do controle X (+1/-1) */
static float     s_sign_y = TILT_SIGN_Y;   /* sinal do controle Y (+1/-1) */

/* ── Ajuste de ganhos PID em TEMPO REAL pela serial (UART0) ──────────────────
 * Para calibrar com a mesa funcionando. Comandos (uma linha, termina em \n),
 * maiusculas/minusculas indiferentes, aceita notacao cientifica (ex: 3e-4):
 *   PID <kp> <ki> <kd>   — ajusta os tres de uma vez (ambos os eixos)
 *   KP <v> | KI <v> | KD <v> — ajusta um ganho
 *   ?                    — consulta os ganhos atuais
 * Apos qualquer comando o firmware ecoa  GAINS,<kp>,<ki>,<kd>  (o PIDSimba le
 * essa linha para sincronizar o painel). Os ganhos sao aplicados aos dois PIDs.
 */
#define CMD_UART   UART_NUM_0

static void cmd_print_gains(void)
{
    printf("GAINS,%.6g,%.6g,%.6g\n",
           (double)s_pid_x.kp, (double)s_pid_x.ki, (double)s_pid_x.kd);
}

static void cmd_print_signs(void)
{
    printf("SIGNS,%+.0f,%+.0f\n", (double)s_sign_x, (double)s_sign_y);
}

static void cmd_set_all(float kp, float ki, float kd)
{
    s_pid_x.kp = kp; s_pid_x.ki = ki; s_pid_x.kd = kd;
    s_pid_y.kp = kp; s_pid_y.ki = ki; s_pid_y.kd = kd;
}

static void cmd_handle(char *s)
{
    for (char *p = s; *p; ++p) *p = (char)toupper((unsigned char)*p);

    float a, b, c;
    if (sscanf(s, "PID %f %f %f", &a, &b, &c) == 3) {
        cmd_set_all(a, b, c);
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
    } else if (s[0] == '?') {
        cmd_print_gains();
        cmd_print_signs();
    }
    /* comando desconhecido: ignora silenciosamente */
}

static void cmd_task(void *arg)
{
    uint8_t ch;
    char    line[80];
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

/* Convert a tilt normal into the 3 motor step targets and command them. */
static void apply_tilt(double nx, double ny, double thoff[3])
{
    long pos[STEPPER_COUNT];
    for (int i = 0; i < STEPPER_COUNT; i++) {
        double theta = machine_theta(&s_machine, i, GEO_HZ, nx, ny);
        /* Inclinacao inalcancavel p/ a geometria -> acos fora de [-1,1] -> NaN.
         * lround(NaN) e indefinido (passo lixo). Cai para o nivel com seguranca.
         * Se isto acontecer muito, MEDIR GEO_D/E/F/G/HZ do rig real. */
        if (!isfinite(theta)) theta = s_ang_orig;
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

    for (int i = 0; i < STEPPER_COUNT; i++) {
        stepper_set_max_speed(i, SPEED_HOME);
        stepper_set_acceleration(i, ACCEL_HOME);
    }
    steppers_enable(true);
    vTaskDelay(pdMS_TO_TICKS(500));      /* let the user square the platform */

    /* Home to level and wait until the platform settles there.
     * NB: nao ha chave de fim de curso — a posicao 0 dos motores e ASSUMIDA
     * como nivelada no boot. Deixe a mesa plana antes de energizar. */
    double thoff[3];
    apply_tilt(0.0, 0.0, thoff);
    steppers_run_to_position_blocking();

    for (int i = 0; i < STEPPER_COUNT; i++) {
        stepper_set_max_speed(i, SPEED_BALANCE);
        stepper_set_acceleration(i, ACCEL_BALANCE);
    }

    printf("# Ball Balancer control mode\n");
    printf("# CTRL,x_mm,y_mm,setx,sety,nx,ny,thA,thB,thC\n");

    /* Leitor de comandos de ganhos pela serial (ajuste em tempo real). */
    uart_driver_install(CMD_UART, 256, 0, 0, NULL, 0);
    xTaskCreate(cmd_task, "pid_cmd", 3072, NULL, 5, NULL);
    cmd_print_gains();      /* anuncia os ganhos iniciais */
    cmd_print_signs();      /* e os sinais de controle por eixo */

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
            nx = s_sign_x * ox;
            ny = s_sign_y * oy;
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
