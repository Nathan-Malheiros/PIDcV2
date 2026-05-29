#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PIDSimba — Ball Balancer 3RPS  ·  Console de Controle & Simulacao
=================================================================
Evolucao do calibrate.py: mesmo dominio (ESP32-S3 + tela resistiva + 3 NEMA 17
em arranjo 3RPS), porem voltado a operacao e demonstracao com qualidade de
software de engenharia.

Destaques
---------
  * Movimento REALISTA da mesa: cada motor e simulado como um stepper de verdade,
    com perfil trapezoidal (limites de velocidade e aceleracao). O tampo e
    RECONSTRUIDO a partir dos angulos reais dos 3 motores (ajuste de plano por
    3 pontos) -> atraso de um motor inclina a mesa de forma assimetrica, como
    no hardware. Nada de saltos.
  * Interface fullscreen, responsiva, tema escuro moderno, anti-aliasing,
    paineis com sombra, render 3D iluminado, osciloscopio ao vivo.
  * Modo SIMULACAO FISICA: bola virtual rolando no plano inclinado sob gravidade
    (rolamento 5/7, atrito, colisao com as bordas). Demonstra o loop PID inteiro
    SEM hardware.
  * Modo REAL: bola vinda da serial (POS/NOTOUCH), suavizada por IIR temporal.
  * Pular calibracao: usa a calibracao salva em config.py (ou recalibra).

Uso
---
    python PIDSimba.py                 # usa PORT/BAUD do config.py
    python PIDSimba.py COM5            # porta explicita
    python PIDSimba.py COM5 115200
    python PIDSimba.py --sim           # inicia direto em simulacao fisica
    python PIDSimba.py --windowed      # janela em vez de fullscreen
    python PIDSimba.py --selftest      # teste headless (sem janela)

Teclas (no dashboard)
---------------------
    WASD / setas  mover alvo            M   alternar REAL <-> SIM
    Q / E         girar camera          O   orbita automatica
    + / -         zoom                  R   re-soltar bola (SIM) / limpar traco
    T / X / Y     orientacao            G   gravar orientacao (config + .h)
    [ / ]         derrubar/centralizar  SPACE  pausar
    P             perturbar bola (SIM)  C   recalibrar
    F11           fullscreen            H   ajuda        ESC  sair
"""

import sys, os, re, math, time, queue, threading, statistics, collections, pathlib, argparse, traceback

try:
    import pygame
    from pygame import gfxdraw
except ImportError:
    print("[ERRO] pygame nao instalado.  pip install -r requirements.txt")
    sys.exit(1)

try:
    import serial
except ImportError:
    serial = None  # serial e opcional no modo --sim

# ── Configuracao (tools/config.py) com fallback ──────────────────────────────
try:
    import config as cfg
except Exception:
    cfg = None

def _cfg(name, default):
    return getattr(cfg, name, default) if cfg else default

DEFAULT_PORT = _cfg('PORT', 'COM8')
DEFAULT_BAUD = int(_cfg('BAUD', 115200))

HEADER_PATH = pathlib.Path(__file__).parent.parent / "main" / "touch_screen.h"
CONFIG_PATH = pathlib.Path(__file__).parent / "config.py"
CRASH_LOG   = pathlib.Path(__file__).parent / "pidsimba_crash.log"

TABLE_W_MM = float(_cfg('SCREEN_W_MM', 187.0))
TABLE_H_MM = float(_cfg('SCREEN_H_MM', 141.0))

# ═════════════════════════════════════════════════════════════════════════════
#  CONTROLE 3RPS + CINEMATICA  (espelha main/control.c / kinematics.c / pid.c)
# ═════════════════════════════════════════════════════════════════════════════

GEO_D, GEO_E, GEO_F, GEO_G = 50.8, 79.4, 44.45, 93.2   # geometria 3RPS (mm)
GEO_HZ      = 108.0                                     # altura neutra (mm)
SQRT3       = math.sqrt(3.0)
TILT_LIMIT  = 0.25                                      # clamp do vetor normal
KP, KI, KD  = 8.0e-4, 2.0e-5, 1.2e-2                    # ganhos PID do FIRMWARE (ref.)

# Ganhos dedicados a SIMULACAO. O modo SIM fecha o laco em software contra um
# modelo de planta com gravidade REAL (g=9.81) — bem mais "quente" que a planta
# implicita dos ganhos do firmware (cujo derivativo satura com v>~21mm/s). Estes
# foram tunados (amortecimento ~critico, sem overshoot, assenta ~2-3 s) por
# varredura no proprio modelo fisico. No modo REAL usamos os ganhos do firmware,
# pois ali o ESP32 e quem controla e o tool apenas visualiza.
SIM_KP, SIM_KI, SIM_KD = 3.0e-4, 1.2e-5, 4.0e-4

LEG_AZ   = [math.radians(90), math.radians(210), math.radians(330)]
LEG_NAME = ["A", "B", "C"]

# ── Dinamica do stepper (NEMA 17) — o que da o "movimento de verdade" ────────
STEP_PER_REV   = 200          # passo cheio
MICROSTEP      = 16           # 1/16
STEPS_PER_DEG  = STEP_PER_REV * MICROSTEP / 360.0
MOTOR_VMAX     = 700.0        # deg/s   (velocidade maxima do eixo)
MOTOR_AMAX     = 4000.0       # deg/s^2 (aceleracao maxima)

# ── Reconstrucao do tampo a partir dos angulos dos motores ───────────────────
PLATE_MOUNT_R  = 72.0                       # raio dos pontos de fixacao (mm)
LIFT_PER_DEG   = math.radians(1.0) * GEO_F  # dz/dtheta do braco (mm/deg)
PLATE_BASE_Z   = 70.0                        # altura visual do centro (mm)

# ── Fisica da bola simulada ──────────────────────────────────────────────────
G_MM        = 9810.0          # gravidade (mm/s^2)
ROLL_FACTOR = 5.0 / 7.0       # esfera solida rolando sem deslizar
ROLL_DAMP   = 1.0             # atrito viscoso (1/s)
WALL_REST   = 0.35            # restituicao na borda
BALL_R_MM   = 7.5             # raio fisico da bola (mm)
SLOPE_SIGN  = +1.0            # sinal que torna o laco estavel (ver _physics)

# Suavizacao no modo REAL (serial)
BALL_TAU      = 0.06
PRESENCE_HOLD = 0.35

# Suavizacao APENAS visual (mata o tremor sem mexer na fisica/controle).
# A mesa se move de verdade (steppers); estes filtros so deixam o DESENHO liso.
TILT_DISP_TAU = 0.11    # filtro de exibicao da inclinacao/motores
BALL_DISP_TAU = 0.05    # filtro de exibicao da bola


def machine_theta(leg, hz, nx, ny, d=GEO_D, e=GEO_E, f=GEO_F, g=GEO_G):
    """Cinematica inversa 3RPS — porte identico a main/kinematics.c."""
    nmag = math.sqrt(nx * nx + ny * ny + 1.0)
    nx /= nmag; ny /= nmag
    nz = 1.0 / nmag
    try:
        if leg == 0:
            y = d + (e / 2.0) * (1.0 - (nx * nx + 3.0 * nz * nz + 3.0 * nz) /
                    (nz + 1.0 - nx * nx +
                     (nx ** 4 - 3.0 * nx * nx * ny * ny) /
                     ((nz + 1.0) * (nz + 1.0 - nx * nx))))
            z = hz + e * ny
            mag = math.sqrt(y * y + z * z)
            ang = math.acos(y / mag) + math.acos((mag * mag + f * f - g * g) / (2.0 * mag * f))
        elif leg == 1:
            x = (SQRT3 / 2.0) * (e * (1.0 - (nx * nx + SQRT3 * nx * ny) / (nz + 1.0)) - d)
            y = x / SQRT3
            z = hz - (e / 2.0) * (SQRT3 * nx + ny)
            mag = math.sqrt(x * x + y * y + z * z)
            ang = math.acos((SQRT3 * x + y) / (-2.0 * mag)) + math.acos((mag * mag + f * f - g * g) / (2.0 * mag * f))
        else:
            x = (SQRT3 / 2.0) * (d - e * (1.0 - (nx * nx - SQRT3 * nx * ny) / (nz + 1.0)))
            y = -x / SQRT3
            z = hz + (e / 2.0) * (SQRT3 * nx - ny)
            mag = math.sqrt(x * x + y * y + z * z)
            ang = math.acos((-SQRT3 * x + y) / (-2.0 * mag)) + math.acos((mag * mag + f * f - g * g) / (2.0 * mag * f))
        return math.degrees(ang)
    except (ValueError, ZeroDivisionError):
        return ANG_ORIG


ANG_ORIG = machine_theta(0, GEO_HZ, 0.0, 0.0)   # angulo de cada perna no nivel


class PID:
    """PID por eixo — espelha main/pid.c (derivada no erro + anti-windup)."""
    def __init__(self, kp, ki, kd, omin, omax):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.omin, self.omax = omin, omax
        self.reset()

    def reset(self):
        self.integ = 0.0; self.prev = 0.0; self.primed = False

    def update(self, meas, setpoint, dt):
        if dt <= 0.0: dt = 1e-3
        err = meas - setpoint
        deriv = (err - self.prev) / dt if self.primed else 0.0
        self.prev = err; self.primed = True
        self.integ += err * dt
        out = self.kp * err + self.ki * self.integ + self.kd * deriv
        if out > self.omax:
            if self.ki: self.integ -= (out - self.omax) / self.ki
            out = self.omax
        elif out < self.omin:
            if self.ki: self.integ -= (out - self.omin) / self.ki
            out = self.omin
        return out


class Stepper:
    """Eixo de um motor de passo com perfil trapezoidal (estilo AccelStepper).

    Aproxima a resposta de um driver real: acelera ate VMAX, e desacelera a
    tempo de parar exatamente no alvo. Sem saltos -> movimento da mesa suave.
    """
    def __init__(self, pos, vmax=MOTOR_VMAX, amax=MOTOR_AMAX):
        self.pos = pos          # angulo atual (deg)
        self.vel = 0.0          # velocidade (deg/s)
        self.vmax, self.amax = vmax, amax

    def update(self, target, dt):
        err = target - self.pos
        # velocidade maxima da qual ainda da pra frear a tempo (v^2 = 2*a*d)
        v_stop = math.sqrt(2.0 * self.amax * abs(err)) if err else 0.0
        v_des = math.copysign(min(self.vmax, v_stop), err)
        # rampa de velocidade limitada pela aceleracao
        dv = v_des - self.vel
        dv_max = self.amax * dt
        if dv >  dv_max: dv =  dv_max
        if dv < -dv_max: dv = -dv_max
        self.vel += dv
        self.pos += self.vel * dt
        # captura no alvo quando muito proximo e quase parado
        if abs(err) < 0.02 and abs(self.vel) < 1.0:
            self.pos = target; self.vel = 0.0
        return self.vel

    @property
    def steps(self):
        return int(round((self.pos - ANG_ORIG) * STEPS_PER_DEG))


def fit_plane(p0, p1, p2):
    """Plano z = a*x + b*y + c por 3 pontos."""
    (x0, y0, z0), (x1, y1, z1), (x2, y2, z2) = p0, p1, p2
    det = (x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0)
    if abs(det) < 1e-9:
        return 0.0, 0.0, (z0 + z1 + z2) / 3.0
    a = ((z1 - z0) * (y2 - y0) - (z2 - z0) * (y1 - y0)) / det
    b = ((x1 - x0) * (z2 - z0) - (x2 - x0) * (z1 - z0)) / det
    c = z0 - a * x0 - b * y0
    return a, b, c


# ═════════════════════════════════════════════════════════════════════════════
#  TEMA / PALETA
# ═════════════════════════════════════════════════════════════════════════════

T = {
    'bg_top':    (10, 12, 20),
    'bg_bot':    (18, 22, 38),
    'panel':     (24, 28, 44),
    'panel_hi':  (32, 38, 58),
    'border':    (54, 62, 92),
    'border_hi': (88, 120, 200),
    'text':      (216, 222, 236),
    'dim':       (120, 132, 160),
    'faint':     (78, 88, 116),
    'accent':    (64, 200, 224),    # ciano — destaque principal
    'accent2':   (120, 150, 255),   # azul
    'good':      (74, 222, 128),
    'warn':      (250, 196, 76),
    'bad':       (248, 96, 96),
    'motor':     (250, 176, 64),    # ambar — motores
    'target':    (90, 220, 130),    # verde — alvo
    'ball':      (244, 72, 72),     # vermelho — bola
    'ball_hi':   (255, 168, 168),
    # 3D
    'floor':     (28, 32, 50),
    'base':      (52, 60, 86),
    'arm':       (250, 176, 64),
    'rod':       (96, 170, 230),
    'plate':     (168, 190, 206),
    'grid3d':    (40, 70, 80),
    'shadow':    (8, 10, 16),
}


def lerp(a, b, t):
    return a + (b - a) * t

def lerp_col(c1, c2, t):
    return (int(lerp(c1[0], c2[0], t)),
            int(lerp(c1[1], c2[1], t)),
            int(lerp(c1[2], c2[2], t)))

def clamp(v, lo, hi):
    if v != v:            # NaN -> valor seguro
        return lo
    return lo if v < lo else hi if v > hi else v


# ═════════════════════════════════════════════════════════════════════════════
#  THREAD SERIAL
# ═════════════════════════════════════════════════════════════════════════════

class SerialReader(threading.Thread):
    def __init__(self, port, baud, q):
        super().__init__(daemon=True)
        self.port, self.baud, self.q = port, baud, q
        self._stop = threading.Event()
        self.ser = None
        self._wlock = threading.Lock()

    def send(self, line):
        """Envia uma linha de comando ao firmware (ex.: 'PID 3e-4 1e-5 2e-3')."""
        ser = self.ser
        if ser is None:
            return False
        try:
            with self._wlock:
                ser.write((line + '\n').encode('ascii'))
            return True
        except Exception:
            return False

    def run(self):
        if serial is None:
            self.q.put(('status', 'pyserial ausente'))
            return
        while not self._stop.is_set():
            try:
                with serial.Serial(self.port, self.baud, timeout=1.0) as ser:
                    self.ser = ser
                    self.q.put(('status', 'conectado'))
                    while not self._stop.is_set():
                        raw = ser.readline()
                        if not raw:
                            continue
                        line = raw.decode('utf-8', errors='replace').strip()
                        parts = line.split(',')
                        if not parts:
                            continue
                        tag = parts[0]
                        if tag == 'POS' and len(parts) >= 3:
                            try:
                                self.q.put(('pos', int(parts[1]), int(parts[2])))
                            except ValueError:
                                pass
                        elif tag == 'NOTOUCH':
                            if len(parts) >= 3:
                                try:
                                    self.q.put(('notouch', int(parts[1]), int(parts[2])))
                                except ValueError:
                                    self.q.put(('notouch', 0, 0))
                            else:
                                self.q.put(('notouch', 0, 0))
                        elif tag == 'GAINS' and len(parts) >= 4:
                            try:
                                vals = tuple(float(v) for v in parts[1:4])
                                if all(math.isfinite(v) for v in vals):
                                    self.q.put(('gains', *vals))
                            except ValueError:
                                pass
                        elif tag == 'CTRL' and len(parts) >= 10:
                            # CTRL,x,y,setx,sety,nx,ny,thA,thB,thC (do firmware)
                            try:
                                vals = tuple(float(v) for v in parts[1:10])
                                if all(math.isfinite(v) for v in vals):
                                    self.q.put(('ctrl', vals))
                            except ValueError:
                                pass
                        elif tag == 'SIGNS' and len(parts) >= 3:
                            try:
                                vals = tuple(float(v) for v in parts[1:3])
                                if all(math.isfinite(v) for v in vals):
                                    self.q.put(('signs', *vals))
                            except ValueError:
                                pass
            except Exception as e:
                self.ser = None
                self.q.put(('status', f'sem conexao'))
                time.sleep(2.0)
            finally:
                self.ser = None

    def stop(self):
        self._stop.set()


# ═════════════════════════════════════════════════════════════════════════════
#  CALIBRACAO (mini fluxo, opcional — pode pular se ja salva)
# ═════════════════════════════════════════════════════════════════════════════

SAMPLES_NEEDED = 80
CORNERS = [
    ("SUPERIOR ESQUERDO", (0, 1)),
    ("SUPERIOR DIREITO",  (1, 1)),
    ("INFERIOR ESQUERDO", (0, 0)),
    ("INFERIOR DIREITO",  (1, 0)),
]


# ═════════════════════════════════════════════════════════════════════════════
#  APLICACAO
# ═════════════════════════════════════════════════════════════════════════════

class App:
    def __init__(self, port=DEFAULT_PORT, baud=DEFAULT_BAUD,
                 fullscreen=True, mode='REAL', win_size=(1280, 760)):
        pygame.init()
        pygame.display.set_caption("PIDSimba — Ball Balancer 3RPS")
        self.fullscreen = fullscreen
        if fullscreen:
            self.screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
        else:
            self.screen = pygame.display.set_mode(win_size, pygame.RESIZABLE)
        self.w, self.h = self.screen.get_size()
        self.clock = pygame.time.Clock()

        self._font_cache = {}
        self._shadow_cache = {}
        self._bg = None
        self._build_bg()

        self.port, self.baud = port, baud
        self.q = queue.Queue()
        self.reader = SerialReader(port, baud, self.q)
        self.reader.start()
        self.conn_txt = 'conectando...'
        self.got_serial = False

        # leitura crua
        self.cur_xr = 0; self.cur_yr = 0
        self.touched = False; self.pos_ts = 0.0

        # orientacao
        self.swap_xy = bool(_cfg('SWAP_XY', False))
        self.flip_x  = bool(_cfg('FLIP_X', False))
        self.flip_y  = bool(_cfg('FLIP_Y', False))

        # calibracao (tenta carregar do config)
        self.cal = self._load_cal_from_cfg()

        # calibracao em andamento
        self.corner_idx = 0
        self.corner_data = [None] * 4
        self.samples = []

        # ── controle / estado dinamico ───────────────────────────────────────
        self.mode = mode          # 'REAL' | 'SIM'  (define os ganhos)
        # conjuntos de ganhos editaveis ao vivo, um por modo (editar SIM nao
        # mexe nos do firmware e vice-versa). Persistem entre trocas de modo.
        self.gain_sets = {'SIM': [SIM_KP, SIM_KI, SIM_KD], 'REAL': [KP, KI, KD]}
        self.sel_gain = 0          # 0=Kp 1=Ki 2=Kd  (qual esta selecionado)
        self.hw_gains = None       # ultimo eco GAINS do firmware (modo REAL)
        # telemetria CTRL do firmware (espelho real dos motores em modo REAL)
        self.fw_ts  = None
        self.fw_set = (TABLE_W_MM / 2.0, TABLE_H_MM / 2.0)
        self.fw_n   = (0.0, 0.0)
        self.fw_th  = (ANG_ORIG, ANG_ORIG, ANG_ORIG)
        self.fw_sign = (1.0, 1.0)  # sinal de controle por eixo no firmware
        self.fw_mirror = False     # True quando desenhando os motores reais
        self.pid_x = self.pid_y = None
        self._apply_gains()
        self.sx = TABLE_W_MM / 2.0          # alvo (mm)
        self.sy = TABLE_H_MM / 2.0
        self.steppers = [Stepper(ANG_ORIG) for _ in range(3)]
        self.plate = (0.0, 0.0, PLATE_BASE_Z)        # plano REAL (fisica)
        self.disp_theta = [ANG_ORIG, ANG_ORIG, ANG_ORIG]   # angulos p/ desenho
        self.disp_plate = (0.0, 0.0, PLATE_BASE_Z)   # plano suavizado (desenho)
        self.disp_ball = None                        # bola suavizada (desenho)
        self.cmd_nx = 0.0; self.cmd_ny = 0.0    # comando do PID (alvo de tilt)

        self.ball_mm = None       # posicao exibida da bola [x,y] mm
        self.ball_vel = [0.0, 0.0]
        self.bal_seen_ts = 0.0

        # camera
        self.cam_az = math.radians(35)
        self.cam_pitch = math.radians(28)
        self.cam_scale = 2.4
        self.orbit = False

        # historico p/ osciloscopio: (t, valor)
        self.hist_err = collections.deque(maxlen=900)
        self.hist_mot = [collections.deque(maxlen=900) for _ in range(3)]
        self.t0 = time.time()

        if mode == 'SIM':
            self._spawn_sim_ball()
        self.paused = False
        self.show_help = False
        self.save_msg = ''; self.save_ts = 0.0
        self.last_t = time.time()
        self.crash_tb = None       # traceback do ultimo erro (mostra na tela)

        # toast inicial
        self._toast("PIDSimba pronto" if self.cal else "Sem calibracao — pressione C")

        # estado: se ja ha calibracao, vai pro dashboard; senao splash
        self.state = 'SPLASH'

    # ── recursos ──────────────────────────────────────────────────────────────

    def font(self, size, bold=False, mono=False):
        key = (size, bold, mono)
        f = self._font_cache.get(key)
        if f is None:
            name = 'consolas' if mono else 'segoeui,arial'
            f = pygame.font.SysFont(name, size, bold=bold)
            self._font_cache[key] = f
        return f

    def _build_bg(self):
        bg = pygame.Surface((self.w, self.h))
        for y in range(self.h):
            t = y / max(1, self.h - 1)
            bg.fill(lerp_col(T['bg_top'], T['bg_bot'], t), (0, y, self.w, 1))
        # brilho radial suave no topo
        glow = pygame.Surface((self.w, self.h), pygame.SRCALPHA)
        cx, cy = int(self.w * 0.5), int(self.h * 0.18)
        maxr = int(self.h * 0.9)
        for i in range(18, 0, -1):
            r = int(maxr * i / 18)
            a = int(5 * (i / 18))
            gfxdraw.filled_circle(glow, cx, cy, r, (40, 90, 140, a))
        bg.blit(glow, (0, 0))
        self._bg = bg

    def _resize(self, w, h):
        self.w, self.h = max(900, w), max(600, h)
        self.screen = pygame.display.set_mode((self.w, self.h), pygame.RESIZABLE)
        self._build_bg()
        self._shadow_cache.clear()

    def _toggle_fullscreen(self):
        self.fullscreen = not self.fullscreen
        if self.fullscreen:
            self.screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
        else:
            self.screen = pygame.display.set_mode((1280, 760), pygame.RESIZABLE)
        self.w, self.h = self.screen.get_size()
        self._build_bg(); self._shadow_cache.clear()

    # ── primitivas de desenho ──────────────────────────────────────────────────

    @staticmethod
    def _fill_poly(surf, pts, col):
        try:                              # int(NaN/inf) estoura -> ignora
            ip = [(int(x), int(y)) for x, y in pts]
        except (ValueError, OverflowError):
            return
        if len(ip) >= 3:
            gfxdraw.filled_polygon(surf, ip, col)
            gfxdraw.aapolygon(surf, ip, col)

    @staticmethod
    def _circle(surf, x, y, r, col):
        try:
            x, y, r = int(x), int(y), int(r)
        except (ValueError, OverflowError):
            return
        if r < 1:
            return
        gfxdraw.filled_circle(surf, x, y, r, col)
        gfxdraw.aacircle(surf, x, y, r, col)

    @staticmethod
    def _ring(surf, x, y, r, col, w=1):
        try:
            x, y, r = int(x), int(y), int(r)
        except (ValueError, OverflowError):
            return
        for k in range(w):
            gfxdraw.aacircle(surf, x, y, r - k, col)

    def _shadow_for(self, w, h, rad=18):
        key = (w, h, rad)
        s = self._shadow_cache.get(key)
        if s is None:
            pad = 26
            small_scale = 0.25
            sw, sh = int((w + pad * 2) * small_scale), int((h + pad * 2) * small_scale)
            tmp = pygame.Surface((sw, sh), pygame.SRCALPHA)
            pygame.draw.rect(tmp, (0, 0, 0, 150),
                             (int(pad * small_scale), int(pad * small_scale),
                              int(w * small_scale), int(h * small_scale)),
                             border_radius=int(rad * small_scale))
            s = pygame.transform.smoothscale(tmp, (w + pad * 2, h + pad * 2))
            self._shadow_cache[key] = s
        return s

    def panel(self, rect, title=None, accent=None):
        sh = self._shadow_for(rect.w, rect.h)
        self.screen.blit(sh, (rect.x - 26, rect.y - 20))
        pygame.draw.rect(self.screen, T['panel'], rect, border_radius=16)
        # leve gradiente no topo
        top = pygame.Surface((rect.w, max(2, rect.h // 2)), pygame.SRCALPHA)
        top.fill((255, 255, 255, 6))
        self.screen.blit(top, (rect.x, rect.y))
        pygame.draw.rect(self.screen, T['border'], rect, 1, border_radius=16)
        if title:
            ac = accent or T['accent']
            pygame.draw.rect(self.screen, ac, (rect.x + 14, rect.y + 16, 4, 16),
                             border_radius=2)
            self.text(title, rect.x + 26, rect.y + 13, self.font(15, bold=True), T['text'])
        return rect.y + (40 if title else 12)

    def text(self, s, x, y, font, col, center=False, right=False):
        surf = font.render(str(s), True, col)
        if center:
            x -= surf.get_width() // 2
        elif right:
            x -= surf.get_width()
        self.screen.blit(surf, (x, y))
        return y + surf.get_height()

    def pill(self, x, y, label, col, filled=False):
        f = self.font(13, bold=True)
        tw = f.size(label)[0]
        w = tw + 26
        h = 24
        rect = pygame.Rect(x, y, w, h)
        if filled:
            pygame.draw.rect(self.screen, col, rect, border_radius=12)
            tcol = (12, 14, 20)
        else:
            pygame.draw.rect(self.screen, T['panel_hi'], rect, border_radius=12)
            pygame.draw.rect(self.screen, col, rect, 1, border_radius=12)
            tcol = col
        self._circle(self.screen, x + 12, y + h // 2, 4, tcol)
        self.text(label, x + 20, y + 4, f, tcol)
        return x + w

    def hbar(self, x, y, w, h, ratio, col, bipolar=False):
        pygame.draw.rect(self.screen, (16, 18, 28), (x, y, w, h), border_radius=h // 2)
        if bipolar:
            cxp = x + w // 2
            pygame.draw.line(self.screen, T['faint'], (cxp, y), (cxp, y + h))
            half = int((w // 2) * clamp(abs(ratio), 0, 1))
            if ratio >= 0:
                pygame.draw.rect(self.screen, col, (cxp, y, half, h), border_radius=h // 2)
            else:
                pygame.draw.rect(self.screen, col, (cxp - half, y, half, h), border_radius=h // 2)
        else:
            fw = int(w * clamp(ratio, 0, 1))
            if fw > 0:
                pygame.draw.rect(self.screen, col, (x, y, fw, h), border_radius=h // 2)
        pygame.draw.rect(self.screen, T['border'], (x, y, w, h), 1, border_radius=h // 2)

    def _toast(self, msg):
        self.save_msg = msg; self.save_ts = time.time()

    # ── fila serial ────────────────────────────────────────────────────────────

    def _drain(self):
        for _ in range(80):
            if self.q.empty():
                break
            item = self.q.get_nowait()
            tag = item[0]
            if tag == 'status':
                self.conn_txt = item[1]
                # ao (re)conectar em modo REAL, pede os ganhos atuais do firmware
                if item[1] == 'conectado' and self.mode == 'REAL':
                    self.reader.send("?")
            elif tag == 'gains':
                # eco do firmware: sincroniza o conjunto REAL e o painel
                self.gain_sets['REAL'] = [item[1], item[2], item[3]]
                if self.mode == 'REAL':
                    self._push_gains()
                self.hw_gains = (item[1], item[2], item[3])
            elif tag == 'ctrl':
                v = item[1]
                self.fw_set = (v[2], v[3])
                self.fw_n   = (v[4], v[5])
                self.fw_th  = (v[6], v[7], v[8])
                self.fw_ts  = time.time()
                self.got_serial = True
            elif tag == 'signs':
                self.fw_sign = (item[1], item[2])
                self._toast(f"HW sinal de controle  X{item[1]:+.0f}  Y{item[2]:+.0f}")
            elif tag == 'pos':
                self.cur_xr, self.cur_yr = item[1], item[2]
                self.touched = True; self.pos_ts = time.time()
                self.got_serial = True
                if self.state == 'CAL_COLLECTING':
                    self.samples.append((item[1], item[2]))
            elif tag == 'notouch':
                self.touched = False
                if len(item) >= 3:
                    self.cur_xr, self.cur_yr = item[1], item[2]
                self.got_serial = True
        if self.touched and (time.time() - self.pos_ts) > 0.5:
            self.touched = False

    # ── orientacao / conversao ──────────────────────────────────────────────────

    def _orient(self, nx, ny):
        if self.swap_xy: nx, ny = ny, nx
        if self.flip_x:  nx = 1.0 - nx
        if self.flip_y:  ny = 1.0 - ny
        return nx, ny

    def _orient_str(self):
        parts = []
        if self.swap_xy: parts.append("SWAP")
        if self.flip_x:  parts.append("FLIP_X")
        if self.flip_y:  parts.append("FLIP_Y")
        return " + ".join(parts) if parts else "padrao"

    def _raw2mm(self, xr, yr):
        if not self.cal:
            return TABLE_W_MM / 2, TABLE_H_MM / 2
        x_min, x_max, y_min, y_max = self.cal
        nx = clamp((xr - x_min) / ((x_max - x_min) or 1), 0.0, 1.0)
        ny = clamp((yr - y_min) / ((y_max - y_min) or 1), 0.0, 1.0)
        nx, ny = self._orient(nx, ny)
        return nx * TABLE_W_MM, ny * TABLE_H_MM

    # ── simulacao fisica da bola ────────────────────────────────────────────────

    def _apply_gains(self):
        """(Re)cria os PIDs a partir do conjunto de ganhos do modo atual."""
        kp, ki, kd = self.gain_sets[self.mode]
        self.pid_x = PID(kp, ki, kd, -TILT_LIMIT, TILT_LIMIT)
        self.pid_y = PID(kp, ki, kd, -TILT_LIMIT, TILT_LIMIT)
        self.gains = (kp, ki, kd)

    def _push_gains(self):
        """Aplica nos PIDs os ganhos editados (sem zerar estado integral)."""
        kp, ki, kd = self.gain_sets[self.mode]
        for p in (self.pid_x, self.pid_y):
            p.kp, p.ki, p.kd = kp, ki, kd
        self.gains = (kp, ki, kd)

    def _adjust_gain(self, factor):
        g = self.gain_sets[self.mode]
        g[self.sel_gain] = max(0.0, g[self.sel_gain] * factor)
        self._push_gains()
        self._send_gains_to_hw()
        self._toast(f"{['Kp','Ki','Kd'][self.sel_gain]} = "
                    f"{g[self.sel_gain]:.2e}  ({self.mode})")

    def _reset_gains(self):
        self.gain_sets[self.mode] = ([SIM_KP, SIM_KI, SIM_KD] if self.mode == 'SIM'
                                     else [KP, KI, KD])
        self._push_gains()
        self._send_gains_to_hw()
        self._toast(f"Ganhos {self.mode} restaurados")

    def _send_gains_to_hw(self):
        """No modo REAL, envia os ganhos ao ESP32 (ajuste em tempo real)."""
        if self.mode != 'REAL':
            return
        kp, ki, kd = self.gain_sets['REAL']
        self.reader.send(f"PID {kp:.6g} {ki:.6g} {kd:.6g}")

    def _spawn_sim_ball(self):
        # solta a bola num canto aleatorio-ish (deterministico) longe do alvo
        self.ball_mm = [TABLE_W_MM * 0.18, TABLE_H_MM * 0.80]
        self.ball_vel = [0.0, 0.0]
        self.bal_seen_ts = time.time()

    def _perturb(self):
        if self.ball_mm:
            self.ball_vel[0] += 380.0
            self.ball_vel[1] -= 280.0

    def _physics(self, dt):
        """Integra a bola rolando no plano atual (slopes a,b)."""
        if self.ball_mm is None:
            self._spawn_sim_ball()
        a, b, _ = self.plate
        # aceleracao do rolamento: a = -k*g*slope ; SLOPE_SIGN ajusta estabilidade
        ax = SLOPE_SIGN * ROLL_FACTOR * G_MM * a
        ay = SLOPE_SIGN * ROLL_FACTOR * G_MM * b
        vx, vy = self.ball_vel
        vx += ax * dt; vy += ay * dt
        d = math.exp(-ROLL_DAMP * dt)
        vx *= d; vy *= d
        x = self.ball_mm[0] + vx * dt
        y = self.ball_mm[1] + vy * dt
        # colisao com as bordas
        if x < BALL_R_MM:
            x = BALL_R_MM; vx = -vx * WALL_REST
        elif x > TABLE_W_MM - BALL_R_MM:
            x = TABLE_W_MM - BALL_R_MM; vx = -vx * WALL_REST
        if y < BALL_R_MM:
            y = BALL_R_MM; vy = -vy * WALL_REST
        elif y > TABLE_H_MM - BALL_R_MM:
            y = TABLE_H_MM - BALL_R_MM; vy = -vy * WALL_REST
        self.ball_mm = [x, y]; self.ball_vel = [vx, vy]
        self.bal_seen_ts = time.time()

    # ── laco de atualizacao (fisica + controle + motores) ──────────────────────

    def update(self, dt):
        dt = clamp(dt, 1e-3, 0.05)

        # 1) posicao da bola conforme o modo
        if self.mode == 'SIM':
            if not self.paused:
                self._physics(dt)
            present = True
        else:  # REAL
            if self.touched:
                tx, ty = self._raw2mm(self.cur_xr, self.cur_yr)
                if self.ball_mm is None:
                    self.ball_mm = [tx, ty]
                else:
                    af = 1.0 - math.exp(-dt / BALL_TAU)
                    self.ball_mm[0] += (tx - self.ball_mm[0]) * af
                    self.ball_mm[1] += (ty - self.ball_mm[1]) * af
                self.bal_seen_ts = time.time()
            present = (self.ball_mm is not None and
                       (time.time() - self.bal_seen_ts) < PRESENCE_HOLD)

        # 2) comando de inclinacao + alvos dos motores
        # Em REAL com telemetria CTRL fresca: ESPELHA o firmware (usa os angulos
        # thA/thB/thC e nx/ny que o ESP32 REALMENTE comandou). Sem CTRL (ou em
        # SIM): roda o controlador local.
        self.fw_mirror = (self.mode == 'REAL' and self.fw_ts is not None and
                          (time.time() - self.fw_ts) < 0.5)
        if self.fw_mirror:
            self.cmd_nx, self.cmd_ny = self.fw_n
            self.sx, self.sy = self.fw_set
            targets = list(self.fw_th)
        else:
            if present and self.ball_mm is not None and not self.paused:
                self.cmd_nx = self.pid_x.update(self.ball_mm[0], self.sx, dt)
                self.cmd_ny = self.pid_y.update(self.ball_mm[1], self.sy, dt)
            else:
                self.pid_x.reset(); self.pid_y.reset()
                self.cmd_nx = self.cmd_ny = 0.0
                if (self.mode == 'REAL' and self.ball_mm is not None and
                        (time.time() - self.bal_seen_ts) > 1.2):
                    self.ball_mm = None
            targets = [machine_theta(i, GEO_HZ, self.cmd_nx, self.cmd_ny)
                       for i in range(3)]

        # 3) steppers seguem os alvos com perfil trapezoidal (movimento suave)
        for i in range(3):
            self.steppers[i].update(targets[i], dt)

        # 4) reconstroi o plano do tampo a partir dos angulos REAIS dos motores
        pts = []
        for i, az in enumerate(LEG_AZ):
            z = PLATE_BASE_Z + LIFT_PER_DEG * (self.steppers[i].pos - ANG_ORIG)
            pts.append((PLATE_MOUNT_R * math.cos(az),
                        PLATE_MOUNT_R * math.sin(az), z))
        self.plate = fit_plane(*pts)

        # 4b) suavizacao VISUAL (so desenho): mata o tremor sem mexer na fisica.
        at = 1.0 - math.exp(-dt / TILT_DISP_TAU)
        dpts = []
        for i, az in enumerate(LEG_AZ):
            self.disp_theta[i] += (self.steppers[i].pos - self.disp_theta[i]) * at
            z = PLATE_BASE_Z + LIFT_PER_DEG * (self.disp_theta[i] - ANG_ORIG)
            dpts.append((PLATE_MOUNT_R * math.cos(az),
                         PLATE_MOUNT_R * math.sin(az), z))
        self.disp_plate = fit_plane(*dpts)
        if self.ball_mm is not None:
            ab = 1.0 - math.exp(-dt / BALL_DISP_TAU)
            if self.disp_ball is None:
                self.disp_ball = list(self.ball_mm)
            else:
                self.disp_ball[0] += (self.ball_mm[0] - self.disp_ball[0]) * ab
                self.disp_ball[1] += (self.ball_mm[1] - self.disp_ball[1]) * ab
        else:
            self.disp_ball = None

        # 5) historico p/ osciloscopio
        t = time.time() - self.t0
        if self.ball_mm is not None:
            err = math.hypot(self.ball_mm[0] - self.sx, self.ball_mm[1] - self.sy)
            self.hist_err.append((t, err))
        for i in range(3):
            self.hist_mot[i].append((t, self.steppers[i].pos - ANG_ORIG))

        # orbita automatica da camera
        if self.orbit and not self.paused:
            self.cam_az += 0.35 * dt

    def plate_z(self, X, Y):
        # usa o plano SUAVIZADO (desenho). A fisica usa self.plate diretamente.
        a, b, c = self.disp_plate
        return a * X + b * Y + c

    # ── projecao isometrica ─────────────────────────────────────────────────────

    def _iso(self, x, y, z, cx, cy):
        ca, sa = math.cos(self.cam_az), math.sin(self.cam_az)
        xr = x * ca - y * sa
        yr = x * sa + y * ca
        sp = math.sin(self.cam_pitch)
        return (cx + xr * self.cam_scale,
                cy + (yr * sp - z) * self.cam_scale)

    # ═══════════════════════════════════════════════════════════════════════════
    #  RENDER: cena 3D
    # ═══════════════════════════════════════════════════════════════════════════

    def draw_scene(self, rect):
        s = self.screen
        prev_clip = s.get_clip()
        s.set_clip(rect)
        # sanitiza estado de desenho (qualquer NaN/inf vira nivel) — blindagem
        if not all(math.isfinite(t) for t in self.disp_theta):
            self.disp_theta = [ANG_ORIG, ANG_ORIG, ANG_ORIG]
        if not all(math.isfinite(v) for v in self.disp_plate):
            self.disp_plate = (0.0, 0.0, PLATE_BASE_Z)
        if self.disp_ball is not None and not all(math.isfinite(v) for v in self.disp_ball):
            self.disp_ball = None
        cx = rect.centerx
        cy = rect.centery + int(rect.h * 0.14)
        # escala da cena proporcional ao painel (afastada — sem zoom excessivo)
        self.cam_scale = min(rect.w, rect.h) / 235.0 * self.cam_scale_user
        hw, hh = TABLE_W_MM / 2.0, TABLE_H_MM / 2.0

        # piso/base
        base_r = PLATE_MOUNT_R * 1.55
        floor = [self._iso(base_r * math.cos(t), base_r * math.sin(t), 0, cx, cy)
                 for t in [math.radians(a) for a in range(0, 360, 30)]]
        self._fill_poly(s, floor, T['floor'])
        gfxdraw.aapolygon(s, [(int(x), int(y)) for x, y in floor], T['base'])

        # pontos de fixacao / motores no chao
        mount = []
        for az in LEG_AZ:
            mount.append((PLATE_MOUNT_R * math.cos(az), PLATE_MOUNT_R * math.sin(az)))

        # corpos dos motores (caixas) + bracos + bielas
        plate_corner_world = []
        for i, az in enumerate(LEG_AZ):
            mx, my = mount[i]
            base_pt = self._iso(mx, my, 0, cx, cy)
            # corpo do motor (pequeno bloco)
            self._circle(s, base_pt[0], base_pt[1], max(4, self.cam_scale * 6), T['base'])
            # braco do motor segue o angulo real
            delta = math.radians(self.disp_theta[i] - ANG_ORIG)
            rad = (math.cos(az), math.sin(az))
            ax3 = mx - rad[0] * GEO_F * math.cos(delta)
            ay3 = my - rad[1] * GEO_F * math.cos(delta)
            az3 = GEO_F * math.sin(delta) + 12.0
            arm_tip = self._iso(ax3, ay3, az3, cx, cy)
            # ponto de fixacao na plataforma (no plano reconstruido)
            pz = self.plate_z(mx, my)
            plate_pt = self._iso(mx, my, pz, cx, cy)
            plate_corner_world.append((mx, my, pz))
            # desenha braco (ambar) e biela (azul)
            pygame.draw.line(s, T['arm'], base_pt, arm_tip, max(2, int(self.cam_scale * 1.6)))
            pygame.draw.line(s, T['rod'], arm_tip, plate_pt, max(2, int(self.cam_scale)))
            self._circle(s, arm_tip[0], arm_tip[1], max(2, self.cam_scale * 1.4), T['arm'])

        # tampo inclinado, com sombreamento por iluminacao
        a, b, _ = self.disp_plate
        n = (-a, -b, 1.0)
        nmag = math.sqrt(n[0] ** 2 + n[1] ** 2 + n[2] ** 2)
        nz = (n[0] / nmag, n[1] / nmag, n[2] / nmag)
        L = (0.35, -0.45, 0.82)
        intensity = clamp(nz[0] * L[0] + nz[1] * L[1] + nz[2] * L[2], 0.35, 1.0)
        plate_col = lerp_col((40, 60, 70), T['plate'], intensity)

        corners = [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)]
        quad = [self._iso(X, Y, self.plate_z(X, Y), cx, cy) for X, Y in corners]
        # sombra projetada do tampo no chao
        shadow_quad = [self._iso(X, Y, 0.5, cx, cy) for X, Y in corners]
        shsurf = pygame.Surface((rect.w, rect.h), pygame.SRCALPHA)
        self._fill_poly(shsurf, [(x - rect.x, y - rect.y) for x, y in shadow_quad],
                        (0, 0, 0, 70))
        s.blit(shsurf, (rect.x, rect.y))

        self._fill_poly(s, quad, plate_col)
        # borda do tampo (espessura)
        edge_col = lerp_col((20, 30, 36), (70, 110, 120), intensity)
        for k in range(4):
            p1 = quad[k]; p2 = quad[(k + 1) % 4]
            pygame.draw.line(s, edge_col, p1, p2, 2)

        # grade na superficie
        for k in range(1, 8):
            t = k / 8.0
            X = -hw + t * TABLE_W_MM
            pygame.draw.aaline(s, T['grid3d'],
                self._iso(X, -hh, self.plate_z(X, -hh), cx, cy),
                self._iso(X,  hh, self.plate_z(X,  hh), cx, cy))
            Y = -hh + t * TABLE_H_MM
            pygame.draw.aaline(s, T['grid3d'],
                self._iso(-hw, Y, self.plate_z(-hw, Y), cx, cy),
                self._iso( hw, Y, self.plate_z( hw, Y), cx, cy))

        # alvo (reticula verde)
        tX, tY = self.sx - hw, self.sy - hh
        tp = self._iso(tX, tY, self.plate_z(tX, tY), cx, cy)
        self._ring(s, tp[0], tp[1], max(6, self.cam_scale * 5), T['target'], 2)
        pygame.draw.aaline(s, T['target'], (tp[0] - 11, tp[1]), (tp[0] + 11, tp[1]))
        pygame.draw.aaline(s, T['target'], (tp[0], tp[1] - 11), (tp[0], tp[1] + 11))

        # bola + sombra + glow  (posicao suavizada p/ desenho)
        if self.disp_ball is not None:
            bX, bY = self.disp_ball[0] - hw, self.disp_ball[1] - hh
            bZ = self.plate_z(bX, bY)
            sp = self._iso(bX, bY, bZ, cx, cy)
            bp = self._iso(bX, bY, bZ + BALL_R_MM, cx, cy)
            # sombra
            sh = pygame.Surface((rect.w, rect.h), pygame.SRCALPHA)
            self._circle(sh, sp[0] - rect.x, sp[1] - rect.y,
                         max(5, self.cam_scale * 6), (0, 0, 0, 90))
            s.blit(sh, (rect.x, rect.y))
            r = max(6, int(BALL_R_MM * self.cam_scale))
            # glow
            gl = pygame.Surface((rect.w, rect.h), pygame.SRCALPHA)
            for i in range(4, 0, -1):
                self._circle(gl, bp[0] - rect.x, bp[1] - rect.y, r + i * 4,
                             (244, 72, 72, 18))
            s.blit(gl, (rect.x, rect.y))
            self._circle(s, bp[0], bp[1], r, T['ball'])
            self._circle(s, bp[0] - r // 3, bp[1] - r // 3, max(2, r // 3), T['ball_hi'])

        # rotulos dos motores
        f = self.font(13, bold=True, mono=True)
        for i, az in enumerate(LEG_AZ):
            mx, my = mount[i]
            delta = math.radians(self.disp_theta[i] - ANG_ORIG)
            rad = (math.cos(az), math.sin(az))
            ax3 = mx - rad[0] * GEO_F * math.cos(delta)
            ay3 = my - rad[1] * GEO_F * math.cos(delta)
            az3 = GEO_F * math.sin(delta) + 12.0
            at = self._iso(ax3, ay3, az3, cx, cy)
            self.text(LEG_NAME[i], at[0] + 8, at[1] - 8, f, T['motor'])

        s.set_clip(prev_clip)

    cam_scale_user = 1.0

    # ═══════════════════════════════════════════════════════════════════════════
    #  RENDER: dashboard
    # ═══════════════════════════════════════════════════════════════════════════

    def draw_header(self):
        s = self.screen
        hh = max(56, int(self.h * 0.075))
        bar = pygame.Rect(0, 0, self.w, hh)
        grad = pygame.Surface((self.w, hh), pygame.SRCALPHA)
        grad.fill((255, 255, 255, 6))
        s.blit(grad, (0, 0))
        pygame.draw.line(s, T['border'], (0, hh), (self.w, hh), 1)

        # logo / titulo
        cy = hh // 2
        self._ring(s, 30, cy, 11, T['accent'], 2)
        self._circle(s, 30, cy, 4, T['accent'])
        self.text("PIDSimba", 50, cy - 18, self.font(22, bold=True), T['text'])
        self.text("Ball Balancer 3RPS · console de controle", 52, cy + 6,
                  self.font(12), T['dim'])

        # pills de status (direita)
        x = self.w - 20
        # modo
        mcol = T['accent'] if self.mode == 'SIM' else T['good']
        mlbl = "SIMULACAO" if self.mode == 'SIM' else "HARDWARE"
        f = self.font(13, bold=True)
        w_mode = f.size(mlbl)[0] + 26
        x -= w_mode; self.pill(x, cy - 12, mlbl, mcol, filled=True)
        x -= 10
        # conexao
        if self.mode == 'REAL':
            ccol = T['good'] if self.conn_txt == 'conectado' else T['warn']
            clbl = self.port
            w_c = f.size(clbl)[0] + 26
            x -= w_c; self.pill(x, cy - 12, clbl, ccol)
            x -= 10
        # fps
        fps = int(self.clock.get_fps())
        flbl = f"{fps} FPS"
        w_f = f.size(flbl)[0] + 26
        x -= w_f; self.pill(x, cy - 12, flbl, T['dim'])
        if self.paused:
            x -= 10
            wlbl = "PAUSA"; w_p = f.size(wlbl)[0] + 26
            x -= w_p; self.pill(x, cy - 12, wlbl, T['warn'], filled=True)
        return hh

    def draw_status_panel(self, rect):
        y = self.panel(rect, "ESTADO", T['accent'])
        x = rect.x + 16
        fmd = self.font(15); fsm = self.font(13); fnum = self.font(22, bold=True, mono=True)

        present = self.ball_mm is not None
        if present:
            err = math.hypot(self.ball_mm[0] - self.sx, self.ball_mm[1] - self.sy)
            ecol = T['good'] if err < 8 else T['warn'] if err < 30 else T['bad']
            self.text("ERRO", x, y, fsm, T['dim'])
            self.text(f"{err:5.1f} mm", x + 90, y - 6, fnum, ecol)
            y += 30
            self.hbar(x, y, rect.w - 32, 8, 1.0 - clamp(err / 90, 0, 1),
                      ecol)
            y += 22
            self.text(f"bola   X {self.ball_mm[0]:6.1f}   Y {self.ball_mm[1]:6.1f} mm",
                      x, y, fsm, T['text']); y += 20
            if self.mode == 'SIM':
                sp = math.hypot(*self.ball_vel)
                self.text(f"vel    {sp:6.0f} mm/s", x, y, fsm, T['dim']); y += 20
        else:
            self.text("aguardando bola...", x, y, fmd, T['warn']); y += 28
        y += 6
        self.text(f"alvo   X {self.sx:6.1f}   Y {self.sy:6.1f} mm",
                  x, y, fsm, T['target']); y += 20
        self.text(f"tilt   nx {self.cmd_nx:+.3f}  ny {self.cmd_ny:+.3f}",
                  x, y, fsm, T['accent2']); y += 24
        if self.mode == 'REAL':
            sx, sy = self.fw_sign
            self.text(f"sinal ctrl  X {sx:+.0f}  Y {sy:+.0f}   (N/B inverte)",
                      x, y, fsm, T['accent2']); y += 18
        self.text(f"orient {self._orient_str()}", x, y, fsm, T['dim']); y += 18
        if self.cal:
            xmn, xmx, ymn, ymx = self.cal
            self.text(f"cal    X {xmn}-{xmx}  Y {ymn}-{ymx}", x, y, fsm, T['faint'])
        else:
            self.text("sem calibracao  (C)", x, y, fsm, T['warn'])

    def draw_motor_panel(self, rect):
        suffix = "espelho HW" if self.fw_mirror else ("SIM" if self.mode == 'SIM'
                                                      else "re-sim local")
        y = self.panel(rect, f"MOTORES  ·  3x NEMA 17  ·  {suffix}", T['motor'])
        x = rect.x + 16
        fsm = self.font(13); fmono = self.font(13, mono=True)
        bw = rect.w - 110
        max_dev = 14.0  # deg para escala da barra
        for i in range(3):
            st = self.steppers[i]
            dev = st.pos - ANG_ORIG
            self.text(LEG_NAME[i], x, y, self.font(15, bold=True), T['motor'])
            self.hbar(x + 22, y + 4, bw, 10, dev / max_dev, T['motor'], bipolar=True)
            self.text(f"{dev:+5.1f}", x + 28 + bw, y + 1, fmono, T['text'])
            y += 18
            self.text(f"   {st.steps:+5d} passos   {st.vel:+6.0f} deg/s",
                      x, y, fmono, T['dim'])
            y += 22

    def draw_scope(self, rect):
        y0 = self.panel(rect, "TELEMETRIA  ·  erro & motores", T['accent2'])
        s = self.screen
        plot = pygame.Rect(rect.x + 16, y0 + 4, rect.w - 32, rect.bottom - y0 - 16)
        pygame.draw.rect(s, (14, 16, 26), plot, border_radius=8)
        pygame.draw.rect(s, T['border'], plot, 1, border_radius=8)
        # linhas de grade horizontais
        for k in range(1, 4):
            yy = plot.y + plot.h * k // 4
            pygame.draw.line(s, (28, 32, 48), (plot.x, yy), (plot.right, yy))

        WINDOW = 10.0
        now = time.time() - self.t0
        tmin = now - WINDOW

        def draw_trace(hist, vmin, vmax, col, w=2):
            pts = [(t, v) for (t, v) in hist if t >= tmin]
            if len(pts) < 2:
                return
            span = (vmax - vmin) or 1
            scr = []
            for t, v in pts:
                px = plot.x + (t - tmin) / WINDOW * plot.w
                py = plot.bottom - (clamp(v, vmin, vmax) - vmin) / span * plot.h
                scr.append((px, py))
            if len(scr) >= 2:
                pygame.draw.aalines(s, col, False, scr)

        # erro (0..90mm) em ciano
        draw_trace(self.hist_err, 0, 90, T['accent'])
        # motores (deg, bipolar) em ambar/verde/azul
        mcols = [T['motor'], T['good'], T['rod']]
        for i in range(3):
            draw_trace(self.hist_mot[i], -14, 14, mcols[i])

        # legenda
        fx = plot.x + 8; fy = plot.y + 6; fsm = self.font(12, bold=True)
        self.text("erro", fx, fy, fsm, T['accent']); fx += 50
        for i in range(3):
            self.text(f"M{LEG_NAME[i]}", fx, fy, fsm, mcols[i]); fx += 38

    def draw_footer(self):
        s = self.screen
        fh = 30
        y = self.h - fh
        pygame.draw.line(s, T['border'], (0, y), (self.w, y), 1)
        keys = ("WASD alvo   ·   1/2/3+setas PID   ·   N/B inverte sinal X/Y   ·   "
                "M REAL/SIM   ·   QE girar   ·   V topo   ·   +/- zoom   ·   "
                "T/X/Y orient   ·   G gravar   ·   C calibrar   ·   H ajuda   ·   ESC sair")
        self.text(keys, self.w // 2, y + 7, self.font(13), T['dim'], center=True)

        if self.save_msg and (time.time() - self.save_ts) < 3.2:
            col = T['good'] if ('salv' in self.save_msg.lower() or
                                'pronto' in self.save_msg.lower()) else T['warn']
            f = self.font(15, bold=True)
            tw = f.size(self.save_msg)[0]
            bx = self.w // 2 - tw // 2 - 14
            rect = pygame.Rect(bx, y - 44, tw + 28, 32)
            pygame.draw.rect(s, T['panel_hi'], rect, border_radius=10)
            pygame.draw.rect(s, col, rect, 1, border_radius=10)
            self.text(self.save_msg, self.w // 2, y - 38, f, col, center=True)

    def draw_dashboard(self):
        self.screen.blit(self._bg, (0, 0))
        hh = self.draw_header()
        fh = 30
        M = 16
        top = hh + M
        bottom = self.h - fh - M
        right_w = max(300, int(self.w * 0.27))
        scope_h = max(150, int((bottom - top) * 0.30))

        scene_rect = pygame.Rect(M, top, self.w - right_w - 3 * M, bottom - top - scope_h - M)
        scope_rect = pygame.Rect(M, scene_rect.bottom + M, scene_rect.w, scope_h)
        # painel cena
        self.panel(scene_rect, "VISTA 3D  ·  mesa 3RPS", T['accent'])
        self.draw_scene(scene_rect.inflate(-8, -44).move(0, 18))

        # coluna direita: ESTADO · PID · MOTORES
        col_x = scene_rect.right + M
        avail = bottom - top
        st_h  = int(avail * 0.36)
        pid_h = int(avail * 0.26)
        status_rect = pygame.Rect(col_x, top, right_w, st_h)
        pid_rect    = pygame.Rect(col_x, status_rect.bottom + M, right_w, pid_h)
        motor_rect  = pygame.Rect(col_x, pid_rect.bottom + M, right_w,
                                  bottom - pid_rect.bottom - M)
        self.draw_status_panel(status_rect)
        self.draw_pid_panel(pid_rect)
        self.draw_motor_panel(motor_rect)

        self.draw_scope(scope_rect)
        self.draw_footer()
        if self.show_help:
            self.draw_help_overlay()

    def draw_pid_panel(self, rect):
        gset = "SIM" if self.mode == 'SIM' else "firmware"
        y = self.panel(rect, f"CONTROLE PID  ·  {gset}", T['accent2'])
        x = rect.x + 18
        names = ['Kp', 'Ki', 'Kd']
        vals = self.gain_sets[self.mode]
        frow = self.font(17, bold=True); fval = self.font(17, mono=True)
        rowh = 32
        for i in range(3):
            ry = y + i * rowh
            sel = (i == self.sel_gain)
            if sel:
                hl = pygame.Rect(rect.x + 8, ry - 4, rect.w - 16, rowh - 4)
                pygame.draw.rect(self.screen, T['panel_hi'], hl, border_radius=8)
                pygame.draw.rect(self.screen, T['accent2'], hl, 1, border_radius=8)
                self._circle(self.screen, rect.x + 16, ry + 9, 3, T['accent'])
            col = T['accent'] if sel else T['text']
            self.text(names[i], x + 8, ry, frow, col)
            self.text(f"{vals[i]:.3e}", x + 64, ry, fval, col)
        y += 3 * rowh + 6
        self.text("1/2/3 escolher  ·  setas CIMA/BAIXO ajustar  ·  0 reset",
                  x - 2, y, self.font(12), T['dim']); y += 17
        if self.mode == 'REAL':
            if self.conn_txt == 'conectado':
                tip = "enviando ao ESP32 em tempo real"; tc = T['good']
                if self.hw_gains:
                    tip = (f"HW: {self.hw_gains[0]:.2e}/{self.hw_gains[1]:.2e}"
                           f"/{self.hw_gains[2]:.2e}")
            else:
                tip = "sem conexao — ganhos nao enviados"; tc = T['warn']
            self.text(tip, x - 2, y, self.font(12), tc)
        else:
            self.text("SIM: ganhos locais (nao vao ao hardware)",
                      x - 2, y, self.font(12), T['faint'])

    def draw_help_overlay(self):
        s = self.screen
        ov = pygame.Surface((self.w, self.h), pygame.SRCALPHA)
        ov.fill((6, 8, 14, 210))
        s.blit(ov, (0, 0))
        bw, bh = min(720, self.w - 80), min(560, self.h - 80)
        rect = pygame.Rect((self.w - bw) // 2, (self.h - bh) // 2, bw, bh)
        self.panel(rect, "AJUDA  ·  PIDSimba", T['accent'])
        x = rect.x + 30; y = rect.y + 56
        lines = [
            ("Movimento", "Cada motor e um stepper com perfil trapezoidal (vel/acel"),
            ("",          "limitadas). O tampo e reconstruido dos angulos reais ->"),
            ("",          "movimento suave e fisicamente coerente."),
            ("", ""),
            ("M", "alterna entre HARDWARE (serial) e SIMULACAO fisica"),
            ("WASD", "move o alvo (setpoint) na mesa"),
            ("1 / 2 / 3", "escolhe o ganho Kp / Ki / Kd"),
            ("setas C/B", "ajusta o ganho selecionado (x1.12)  ·  0 = reset"),
            ("N / B", "inverte o sinal de controle do eixo X / Y (modo REAL)"),
            ("P", "aplica um empurrao na bola (so no modo SIM)"),
            ("R", "re-solta a bola (SIM) / limpa historico"),
            ("Q / E", "gira a camera   ·   + / -  zoom   ·   O  orbita auto"),
            ("V", "alterna vista de TOPO <-> isometrica"),
            ("[ / ]", "perturba / centraliza o alvo"),
            ("T / X / Y", "swap / espelha eixos (orientacao)"),
            ("G", "grava orientacao em config.py e touch_screen.h"),
            ("C", "(re)calibrar a tela resistiva"),
            ("SPACE", "pausa   ·   F11  fullscreen   ·   ESC  sair"),
        ]
        for k, d in lines:
            if k:
                self.text(k, x, y, self.font(14, bold=True, mono=True), T['accent'])
            self.text(d, x + 160, y, self.font(14), T['text'])
            y += 26
        self.text("Pressione H para fechar", rect.centerx, rect.bottom - 36,
                  self.font(14, bold=True), T['warn'], center=True)

    # ═══════════════════════════════════════════════════════════════════════════
    #  RENDER: splash + calibracao
    # ═══════════════════════════════════════════════════════════════════════════

    def draw_splash(self):
        self.screen.blit(self._bg, (0, 0))
        cxp = self.w // 2
        y = int(self.h * 0.16)
        self._ring(self.screen, cxp, y - 6, 30, T['accent'], 3)
        self._circle(self.screen, cxp, y - 6, 8, T['accent'])
        y += 40
        y = self.text("PIDSimba", cxp, y, self.font(54, bold=True), T['text'], center=True) + 4
        y = self.text("Ball Balancer 3RPS · console de controle & simulacao",
                      cxp, y, self.font(18), T['dim'], center=True) + 40

        # status conexao
        ccol = T['good'] if self.conn_txt == 'conectado' else T['warn']
        self.text(f"porta {self.port} @ {self.baud}  —  {self.conn_txt}",
                  cxp, y, self.font(15), ccol, center=True); y += 26
        raw_col = T['good'] if self.touched else T['faint']
        self.text(f"X {self.cur_xr:5d}   Y {self.cur_yr:5d}   "
                  f"{'TOCANDO' if self.touched else 'sem toque'}",
                  cxp, y, self.font(15, mono=True), raw_col, center=True); y += 46

        # opcoes
        if self.cal:
            xmn, xmx, ymn, ymx = self.cal
            self.text(f"Calibracao encontrada:  X {xmn}-{xmx}   Y {ymn}-{ymx}",
                      cxp, y, self.font(15), T['accent'], center=True); y += 40
            self._splash_button(cxp, y, "ENTER  —  iniciar (usar calibracao salva)", T['good']); y += 50
            self._splash_button(cxp, y, "C  —  recalibrar a tela", T['accent2']); y += 50
            self._splash_button(cxp, y, "M  —  iniciar em SIMULACAO (sem hardware)", T['accent']); y += 50
        else:
            self.text("Nenhuma calibracao salva em config.py",
                      cxp, y, self.font(15), T['warn'], center=True); y += 40
            self._splash_button(cxp, y, "C  —  calibrar a tela resistiva", T['accent2']); y += 50
            self._splash_button(cxp, y, "M  —  iniciar em SIMULACAO (sem hardware)", T['accent']); y += 50

    def _splash_button(self, cx, y, label, col):
        f = self.font(17, bold=True)
        tw = f.size(label)[0]
        rect = pygame.Rect(cx - tw // 2 - 24, y, tw + 48, 40)
        pulse = (math.sin(time.time() * 3) + 1) / 2
        pygame.draw.rect(self.screen, T['panel'], rect, border_radius=12)
        bc = lerp_col(T['border'], col, pulse)
        pygame.draw.rect(self.screen, bc, rect, 2, border_radius=12)
        self.text(label, cx, y + 9, f, col, center=True)

    def draw_calibration(self):
        self.screen.blit(self._bg, (0, 0))
        cxp = self.w // 2
        idx = self.corner_idx
        done = [i for i in range(4) if self.corner_data[i] is not None]

        if self.state == 'CAL_READY':
            title = f"CALIBRACAO  —  passo {idx + 1} / 4"
        elif self.state == 'CAL_COLLECTING':
            title = f"COLETANDO  —  {CORNERS[idx][0]}"
        else:
            title = "CALIBRACAO"
        self.text(title, cxp, int(self.h * 0.08), self.font(28, bold=True),
                  T['accent'], center=True)

        # mini-mesa
        mw = min(int(self.w * 0.42), 560)
        mh = int(mw * TABLE_H_MM / TABLE_W_MM)
        mrect = pygame.Rect(cxp - mw // 2, int(self.h * 0.20), mw, mh)
        pygame.draw.rect(self.screen, (235, 238, 230), mrect, border_radius=8)
        pygame.draw.rect(self.screen, T['border'], mrect, 2, border_radius=8)
        for i in range(1, 9):
            xx = mrect.x + i * mrect.w // 9
            pygame.draw.line(self.screen, (205, 208, 198), (xx, mrect.y), (xx, mrect.bottom))
        for i in range(1, 7):
            yy = mrect.y + i * mrect.h // 7
            pygame.draw.line(self.screen, (205, 208, 198), (mrect.x, yy), (mrect.right, yy))
        for i, (name, (rx, ry)) in enumerate(CORNERS):
            px = mrect.x + 14 if rx == 0 else mrect.right - 14
            py = mrect.y + 14 if ry == 1 else mrect.bottom - 14
            if i in done:
                self._circle(self.screen, px, py, 11, T['good'])
            elif i == idx and self.state in ('CAL_READY', 'CAL_COLLECTING'):
                self._circle(self.screen, px, py, 13, T['warn'])
                if int(time.time() * 3) % 2 == 0:
                    self._ring(self.screen, px, py, 19, T['warn'], 2)
            else:
                self._circle(self.screen, px, py, 7, T['faint'])

        y = mrect.bottom + 30
        if self.state == 'CAL_READY':
            self.text(f"Posicione a bola no canto: {CORNERS[idx][0]}",
                      cxp, y, self.font(18, bold=True), T['text'], center=True); y += 34
            col = T['good'] if self.touched else T['warn']
            self.text(f"X {self.cur_xr:5d}   Y {self.cur_yr:5d}   "
                      f"{'TOCANDO' if self.touched else 'sem toque'}",
                      cxp, y, self.font(16, mono=True), col, center=True); y += 40
            self.text("ESPACO  coletar      ESC  voltar",
                      cxp, y, self.font(16, bold=True), T['accent'], center=True)
        elif self.state == 'CAL_COLLECTING':
            n = len(self.samples)
            self.text("MANTENHA A BOLA PARADA", cxp, y, self.font(18, bold=True),
                      T['warn'], center=True); y += 34
            bw = int(self.w * 0.4)
            self.hbar(cxp - bw // 2, y, bw, 18, n / SAMPLES_NEEDED, T['good']); y += 28
            self.text(f"{n} / {SAMPLES_NEEDED}", cxp, y, self.font(16, mono=True),
                      T['text'], center=True)
        elif self.state == 'CAL_RESULTS':
            xmn, xmx, ymn, ymx = self.cal
            self.text("Calibracao concluida", cxp, y, self.font(20, bold=True),
                      T['good'], center=True); y += 36
            for lbl in (f"X_MIN {xmn}   X_MAX {xmx}", f"Y_MIN {ymn}   Y_MAX {ymx}"):
                self.text(lbl, cxp, y, self.font(16, mono=True), T['accent'], center=True); y += 26
            y += 16
            self.text("S  salvar (config.py + touch_screen.h)      ENTER  iniciar",
                      cxp, y, self.font(16, bold=True), T['accent2'], center=True)

        if self.save_msg and (time.time() - self.save_ts) < 3.2:
            col = T['good'] if 'salv' in self.save_msg.lower() else T['warn']
            self.text(self.save_msg, cxp, self.h - 50, self.font(15, bold=True),
                      col, center=True)

    # ═══════════════════════════════════════════════════════════════════════════
    #  CALIBRACAO: logica + persistencia
    # ═══════════════════════════════════════════════════════════════════════════

    def _load_cal_from_cfg(self):
        try:
            xmn = int(_cfg('X_RAW_MIN', 0)); xmx = int(_cfg('X_RAW_MAX', 0))
            ymn = int(_cfg('Y_RAW_MIN', 0)); ymx = int(_cfg('Y_RAW_MAX', 0))
            if xmx > xmn and ymx > ymn:
                return (xmn, xmx, ymn, ymx)
        except Exception:
            pass
        return None

    def _finish_corner(self):
        xs = [p[0] for p in self.samples]; ys = [p[1] for p in self.samples]
        self.corner_data[self.corner_idx] = (int(statistics.median(xs)),
                                             int(statistics.median(ys)))
        self.corner_idx += 1
        if self.corner_idx >= 4:
            ax = [d[0] for d in self.corner_data]; ay = [d[1] for d in self.corner_data]
            xmn, xmx = min(ax), max(ax); ymn, ymx = min(ay), max(ay)
            if xmn >= xmx: xmx = xmn + 1
            if ymn >= ymx: ymx = ymn + 1
            self.cal = (xmn, xmx, ymn, ymx)
            self.state = 'CAL_RESULTS'
        else:
            self.state = 'CAL_READY'

    @staticmethod
    def _sub_file(path, replacements):
        """replacements: list of (regex, repl_str)."""
        try:
            if not path.exists():
                return False
            txt = path.read_text(encoding='utf-8')
            for pat, rep in replacements:
                txt = re.sub(pat, rep, txt)
            path.write_text(txt, encoding='utf-8')
            return True
        except Exception as e:
            print(f"[ERRO] {path.name}: {e}")
            return False

    def _save_calibration(self):
        xmn, xmx, ymn, ymx = self.cal
        ok_h = self._sub_file(HEADER_PATH, [
            (r'(#define\s+TOUCH_X_RAW_MIN\s+)\d+', lambda m: m.group(1) + str(xmn)),
            (r'(#define\s+TOUCH_X_RAW_MAX\s+)\d+', lambda m: m.group(1) + str(xmx)),
            (r'(#define\s+TOUCH_Y_RAW_MIN\s+)\d+', lambda m: m.group(1) + str(ymn)),
            (r'(#define\s+TOUCH_Y_RAW_MAX\s+)\d+', lambda m: m.group(1) + str(ymx)),
        ])
        ok_c = self._sub_file(CONFIG_PATH, [
            (r'(?m)^(X_RAW_MIN\s*=\s*)\d+', lambda m: m.group(1) + str(xmn)),
            (r'(?m)^(X_RAW_MAX\s*=\s*)\d+', lambda m: m.group(1) + str(xmx)),
            (r'(?m)^(Y_RAW_MIN\s*=\s*)\d+', lambda m: m.group(1) + str(ymn)),
            (r'(?m)^(Y_RAW_MAX\s*=\s*)\d+', lambda m: m.group(1) + str(ymx)),
        ])
        if ok_c and ok_h:
            self._toast("Calibracao salva: config.py + touch_screen.h")
        elif ok_c:
            self._toast("Salvo em config.py (touch_screen.h ausente)")
        else:
            self._toast("Erro ao salvar calibracao")

    def _persist_orientation(self):
        ok_c = self._sub_file(CONFIG_PATH, [
            (r'(?m)^(SWAP_XY\s*=\s*)(?:True|False)', lambda m: m.group(1) + str(self.swap_xy)),
            (r'(?m)^(FLIP_X\s*=\s*)(?:True|False)',  lambda m: m.group(1) + str(self.flip_x)),
            (r'(?m)^(FLIP_Y\s*=\s*)(?:True|False)',  lambda m: m.group(1) + str(self.flip_y)),
        ])
        ok_h = self._sub_file(HEADER_PATH, [
            (r'(#define\s+TOUCH_SWAP_XY\s+)[01]', lambda m: m.group(1) + ('1' if self.swap_xy else '0')),
            (r'(#define\s+TOUCH_FLIP_X\s+)[01]',  lambda m: m.group(1) + ('1' if self.flip_x else '0')),
            (r'(#define\s+TOUCH_FLIP_Y\s+)[01]',  lambda m: m.group(1) + ('1' if self.flip_y else '0')),
        ])
        if ok_c and ok_h:
            self._toast("Orientacao salva: config.py + touch_screen.h")
        elif ok_c:
            self._toast("Orientacao salva em config.py")
        else:
            self._toast("Erro ao salvar orientacao")

    # ═══════════════════════════════════════════════════════════════════════════
    #  EVENTOS
    # ═══════════════════════════════════════════════════════════════════════════

    def _start_calibration(self):
        self.corner_idx = 0
        self.corner_data = [None] * 4
        self.samples = []
        self.state = 'CAL_READY'

    def handle_key(self, k):
        if k == pygame.K_ESCAPE:
            if self.state in ('CAL_READY', 'CAL_COLLECTING', 'CAL_RESULTS') and self.cal:
                self.state = 'RUN'
            elif self.state in ('CAL_READY', 'CAL_COLLECTING', 'CAL_RESULTS'):
                self.state = 'SPLASH'
            else:
                return False
            return True
        if k == pygame.K_F11:
            self._toggle_fullscreen(); return True

        if self.state == 'SPLASH':
            if k == pygame.K_RETURN and self.cal:
                self.state = 'RUN'
            elif k == pygame.K_c:
                self._start_calibration()
            elif k == pygame.K_m:
                self.mode = 'SIM'; self._apply_gains()
                self._spawn_sim_ball(); self.state = 'RUN'
                self._toast("Modo SIMULACAO")
        elif self.state == 'CAL_READY':
            if k == pygame.K_SPACE:
                self.samples = []; self.state = 'CAL_COLLECTING'
        elif self.state == 'CAL_RESULTS':
            if k == pygame.K_s:
                self._save_calibration()
            elif k == pygame.K_RETURN:
                self.state = 'RUN'
        elif self.state == 'RUN':
            self._handle_run_key(k)
        return True

    def _handle_run_key(self, k):
        step = 8.0
        # WASD = mover alvo
        if k == pygame.K_a:
            self.sx = clamp(self.sx - step, 0, TABLE_W_MM)
        elif k == pygame.K_d:
            self.sx = clamp(self.sx + step, 0, TABLE_W_MM)
        elif k == pygame.K_s:
            self.sy = clamp(self.sy - step, 0, TABLE_H_MM)
        elif k == pygame.K_w:
            self.sy = clamp(self.sy + step, 0, TABLE_H_MM)
        # ajuste de ganhos PID
        elif k == pygame.K_UP:
            self._adjust_gain(1.12)
        elif k == pygame.K_DOWN:
            self._adjust_gain(1.0 / 1.12)
        elif k == pygame.K_LEFT:
            self.sel_gain = (self.sel_gain - 1) % 3
        elif k == pygame.K_RIGHT:
            self.sel_gain = (self.sel_gain + 1) % 3
        elif k == pygame.K_1:
            self.sel_gain = 0
        elif k == pygame.K_2:
            self.sel_gain = 1
        elif k == pygame.K_3:
            self.sel_gain = 2
        elif k == pygame.K_0:
            self._reset_gains()
        elif k == pygame.K_LEFTBRACKET:
            self._perturb()
        elif k == pygame.K_RIGHTBRACKET:
            self.sx, self.sy = TABLE_W_MM / 2, TABLE_H_MM / 2
        elif k == pygame.K_m:
            if self.mode == 'REAL':
                self.mode = 'SIM'; self._apply_gains()
                self._spawn_sim_ball(); self._toast("Modo SIMULACAO")
            else:
                self.mode = 'REAL'; self._apply_gains()
                self.ball_mm = None; self._toast("Modo HARDWARE")
                self.reader.send("?")     # sincroniza ganhos com o firmware
        elif k == pygame.K_p:
            self._perturb()
        elif k == pygame.K_r:
            if self.mode == 'SIM':
                self._spawn_sim_ball()
            self.hist_err.clear()
            for h in self.hist_mot: h.clear()
        elif k == pygame.K_q:
            self.cam_az -= math.radians(8)
        elif k == pygame.K_e:
            self.cam_az += math.radians(8)
        elif k == pygame.K_n:
            new = -self.fw_sign[0]
            if self.reader.send(f"SX {new:+.0f}"):
                self._toast(f"inverter sinal X -> {new:+.0f} (enviado)")
            else:
                self._toast("sem conexao — sinal X nao enviado")
        elif k == pygame.K_b:
            new = -self.fw_sign[1]
            if self.reader.send(f"SY {new:+.0f}"):
                self._toast(f"inverter sinal Y -> {new:+.0f} (enviado)")
            else:
                self._toast("sem conexao — sinal Y nao enviado")
        elif k == pygame.K_o:
            self.orbit = not self.orbit
        elif k == pygame.K_v:
            if self.cam_pitch < math.radians(80):       # ir para topo
                self._saved_cam = (self.cam_az, self.cam_pitch)
                self.cam_az = 0.0                        # mesa alinhada aos eixos
                self.cam_pitch = math.radians(90)
                self.orbit = False
                self._toast("Vista de topo")
            else:                                        # voltar p/ isometrica
                self.cam_az, self.cam_pitch = getattr(
                    self, '_saved_cam', (math.radians(35), math.radians(28)))
                self._toast("Vista isometrica")
        elif k in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):
            self.cam_scale_user = min(2.5, self.cam_scale_user * 1.12)
        elif k in (pygame.K_MINUS, pygame.K_KP_MINUS):
            self.cam_scale_user = max(0.5, self.cam_scale_user / 1.12)
        elif k == pygame.K_t:
            self.swap_xy = not self.swap_xy
        elif k == pygame.K_x:
            self.flip_x = not self.flip_x
        elif k == pygame.K_y:
            self.flip_y = not self.flip_y
        elif k == pygame.K_g:
            self._persist_orientation()
        elif k == pygame.K_c:
            self._start_calibration()
        elif k == pygame.K_h:
            self.show_help = not self.show_help
        elif k == pygame.K_SPACE:
            self.paused = not self.paused

    # ═══════════════════════════════════════════════════════════════════════════
    #  LOOP PRINCIPAL
    # ═══════════════════════════════════════════════════════════════════════════

    def run(self):
        running = True
        crash_logged = False
        try:
            while running:
                try:
                    self._drain()
                    for ev in pygame.event.get():
                        if ev.type == pygame.QUIT:
                            running = False
                        elif ev.type == pygame.VIDEORESIZE and not self.fullscreen:
                            self._resize(ev.w, ev.h)
                        elif ev.type == pygame.MOUSEWHEEL and self.state == 'RUN':
                            f = 1.1 if ev.y > 0 else 1.0 / 1.1
                            self.cam_scale_user = clamp(self.cam_scale_user * f, 0.5, 2.5)
                        elif ev.type == pygame.KEYDOWN:
                            if self.crash_tb and ev.key != pygame.K_ESCAPE:
                                self.crash_tb = None      # qualquer tecla limpa o erro
                            elif not self.handle_key(ev.key):
                                running = False

                    now = time.time()
                    dt = now - self.last_t
                    self.last_t = now

                    if self.crash_tb:
                        self._draw_crash(self.crash_tb)
                    elif self.state == 'RUN':
                        self.update(dt)
                        self.draw_dashboard()
                    elif self.state == 'SPLASH':
                        self.draw_splash()
                    else:
                        if self.state == 'CAL_COLLECTING' and len(self.samples) >= SAMPLES_NEEDED:
                            self._finish_corner()
                        self.draw_calibration()
                except Exception:
                    tb = traceback.format_exc()
                    self.crash_tb = tb
                    print(tb, file=sys.stderr)
                    if not crash_logged:
                        try:
                            CRASH_LOG.write_text(tb, encoding='utf-8')
                        except Exception:
                            pass
                        crash_logged = True

                pygame.display.flip()
                self.clock.tick(60)
        except KeyboardInterrupt:
            pass
        finally:
            self.reader.stop()
            pygame.quit()

    def _draw_crash(self, tb):
        s = self.screen
        s.fill((24, 10, 12))
        self.text("PIDSimba travou — erro salvo em pidsimba_crash.log",
                  24, 18, self.font(20, bold=True), (255, 120, 120))
        self.text("Mande esse arquivo (tools/pidsimba_crash.log) ou tire um print. "
                  "Tecla = continua · ESC = sai",
                  24, 48, self.font(14), (210, 210, 220))
        y = 84
        f = self.font(14, mono=True)
        for line in tb.rstrip().split('\n')[-30:]:
            self.text(line[:170], 24, y, f, (245, 205, 205))
            y += 17


# ═════════════════════════════════════════════════════════════════════════════
#  SELF-TEST (headless) — valida ausencia de crash + convergencia do laco SIM
# ═════════════════════════════════════════════════════════════════════════════

def selftest():
    os.environ['SDL_VIDEODRIVER'] = 'dummy'
    os.environ['SDL_AUDIODRIVER'] = 'dummy'
    app = App(port='COMnone', baud=115200, fullscreen=False, mode='SIM',
              win_size=(1280, 760))
    if not app.cal:
        app.cal = (2000, 3700, 1700, 2600)
    app.state = 'RUN'
    app.sx, app.sy = TABLE_W_MM / 2, TABLE_H_MM / 2
    app._spawn_sim_ball()
    err0 = math.hypot(app.ball_mm[0] - app.sx, app.ball_mm[1] - app.sy)
    dt = 1.0 / 60.0
    # roda 12 s de simulacao + desenho
    frames = int(12.0 / dt)
    for f in range(frames):
        app.update(dt)
        app.draw_dashboard()
        pygame.display.flip()
    errf = math.hypot(app.ball_mm[0] - app.sx, app.ball_mm[1] - app.sy)
    app.reader.stop(); pygame.quit()
    print(f"[selftest] frames={frames}  erro inicial={err0:.1f}mm  final={errf:.1f}mm")
    print(f"[selftest] motores deg: " +
          ", ".join(f"{LEG_NAME[i]}={app.steppers[i].pos - ANG_ORIG:+.2f}" for i in range(3)))
    if errf < err0 * 0.5:
        print("[selftest] OK — laco convergindo (erro caiu > 50%)")
        return 0
    else:
        print("[selftest] ATENCAO — laco nao convergiu; revisar SLOPE_SIGN/ganhos")
        return 1


def main():
    ap = argparse.ArgumentParser(description="PIDSimba — Ball Balancer 3RPS")
    ap.add_argument('port', nargs='?', default=DEFAULT_PORT)
    ap.add_argument('baud', nargs='?', type=int, default=DEFAULT_BAUD)
    ap.add_argument('--sim', action='store_true', help='inicia em simulacao fisica')
    ap.add_argument('--windowed', action='store_true', help='janela em vez de fullscreen')
    ap.add_argument('--selftest', action='store_true', help='teste headless e sai')
    args = ap.parse_args()

    if args.selftest:
        sys.exit(selftest())

    app = App(port=args.port, baud=args.baud,
              fullscreen=not args.windowed,
              mode='SIM' if args.sim else 'REAL')
    if args.sim:
        app.state = 'RUN'
    app.run()


if __name__ == '__main__':
    main()
