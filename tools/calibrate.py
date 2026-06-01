#!/usr/bin/env python3
"""
Ball Balancer — Calibracao e Visualizacao ao Vivo
==================================================
Uso:   python calibrate.py COM5
       python calibrate.py COM5 115200

Fases:
  1. Calibracao guiada: coleta 80 amostras em cada um dos 4 cantos
  2. Resultados: exibe X_MIN/MAX, Y_MIN/MAX com opcao de salvar em touch_screen.h
  3. Visualizacao: plano cartesiano com eixos em raw (sem conversao para mm)
"""

import sys, os, re, threading, queue, time, statistics, collections, pathlib, math

try:
    import serial
except ImportError:
    print("[ERRO] pyserial nao instalado.  pip install -r requirements.txt")
    sys.exit(1)

try:
    import pygame
except ImportError:
    print("[ERRO] pygame nao instalado.  pip install -r requirements.txt")
    sys.exit(1)

# ── Configuracao (tools/config.py) — com fallback caso o arquivo falte ───────
# Garante que config.py seja encontrado independente de onde o script é rodado
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    import config as cfg
except Exception:
    cfg = None

def _cfg(name, default):
    return getattr(cfg, name, default) if cfg else default

DEFAULT_PORT = _cfg('PORT', 'COM8')
DEFAULT_BAUD = _cfg('BAUD', 115200)

# ── Constantes ───────────────────────────────────────────────────────────────

WIN_W, WIN_H      = 1060, 660
SAMPLES_NEEDED    = 80
FPS               = 60
TRAIL_LEN         = 150
TABLE_W_MM        = float(_cfg('SCREEN_W_MM', 187.0))
TABLE_H_MM        = float(_cfg('SCREEN_H_MM', 141.0))

HEADER_PATH = pathlib.Path(__file__).parent.parent / "main" / "touch_screen.h"
CONFIG_PATH = pathlib.Path(__file__).parent / "config.py"

# ── Paleta ───────────────────────────────────────────────────────────────────

C = {
    'bg':          (14,  14,  20),
    'panel':       (22,  22,  32),
    'border':      (48,  48,  68),
    'table_bg':    (242, 242, 228),
    'table_edge':  (55,  55,  55),
    'grid':        (195, 195, 180),
    'highlight':   (255, 200,  40),
    'ball':        (210,  38,  38),
    'corner_ok':   (55,  200,  90),
    'trail_head':  (100, 160, 255),
    'trail_tail':  (18,  38,  78),
    'text':        (208, 208, 214),
    'dim':         (88,  88,  100),
    'heading':     (128, 184, 255),
    'good':        (55,  210,  95),
    'warn':        (218, 178,  58),
    'bad':         (218,  55,  55),
    'hint':        (168, 128, 220),
    'axis':        (110, 175, 110),
    'save_ok':     (80,  230, 120),
    # cena isometrica (estado BALANCE)
    'iso_plate':   (210, 205, 180),
    'iso_plate2':  (150, 146, 120),
    'iso_edge':    (60,  60,  70),
    'iso_grid':    (120, 118, 100),
    'iso_leg':     (90,  160, 220),
    'iso_arm':     (230, 170,  60),
    'iso_base':    (70,  70,  85),
    'iso_target':  (90,  200, 120),
    'iso_ball':    (220, 60,  60),
    'iso_ballhi':  (255, 150, 150),
}

# ── Cantos de calibracao (ordem de coleta) ───────────────────────────────────
# (nome_exibido, descricao, (rx, ry))
#   rx=0 → esquerdo, rx=1 → direito
#   ry=1 → superior,  ry=0 → inferior
CORNERS = [
    ("SUPERIOR ESQUERDO",  "Coloque a bola no\ncanto superior esquerdo",  (0, 1)),
    ("SUPERIOR DIREITO",   "Coloque a bola no\ncanto superior direito",   (1, 1)),
    ("INFERIOR ESQUERDO",  "Coloque a bola no\ncanto inferior esquerdo",  (0, 0)),
    ("INFERIOR DIREITO",   "Coloque a bola no\ncanto inferior direito",   (1, 0)),
]

# ── Controle 3RPS + simulacao dos motores (espelha main/control.c) ───────────
# A bola e REAL (vem da serial); os 3 motores NEMA 17 sao SIMULADOS reagindo a
# ela. Mantenha estes valores em sincronia com main/control.c.

GEO_D, GEO_E, GEO_F, GEO_G = 50.8, 79.4, 44.45, 93.2   # geometria 3RPS (mm)
GEO_HZ      = 108.0                                     # altura neutra (mm)
SQRT3       = math.sqrt(3.0)
TILT_LIMIT  = 0.25                                      # clamp do vetor normal
KP, KI, KD  = 8.0e-4, 2.0e-5, 1.2e-2                    # ganhos PID (dominio mm)

# Suavizacao da visualizacao (constantes de tempo, segundos) — matam o flicker.
BALL_TAU      = 0.06    # filtro da posicao da bola
TILT_TAU      = 0.12    # filtro da inclinacao/motores
PRESENCE_HOLD = 0.35    # segura a bola como "presente" apos um dropout curto

LEG_AZ   = [math.radians(90), math.radians(210), math.radians(330)]
LEG_NAME = ["A", "B", "C"]
PLATE_BASE_Z = 70.0     # altura visual do centro da mesa (mm)


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


def plate_z(X, Y, nx, ny, base_z):
    """Altura do plano inclinado da mesa no ponto local (X,Y)."""
    nmag = math.sqrt(nx * nx + ny * ny + 1.0)
    nz = 1.0 / nmag
    return base_z - (nx * X + ny * Y) / nz


# ── Thread leitora serial ────────────────────────────────────────────────────

class SerialReader(threading.Thread):
    def __init__(self, port, baud, q):
        super().__init__(daemon=True)
        self.port, self.baud, self.q = port, baud, q
        self._stop   = threading.Event()
        self._wq     = queue.Queue()   # fila de escrita (main thread → serial)

    def write(self, line: str):
        """Envia um comando para o firmware (thread-safe)."""
        self._wq.put(line.rstrip('\n') + '\n')

    def run(self):
        while not self._stop.is_set():
            try:
                with serial.Serial(self.port, self.baud, timeout=1.0) as ser:
                    self.q.put(('status', 'conectado'))
                    while not self._stop.is_set():
                        # Drena fila de escrita antes de ler
                        while not self._wq.empty():
                            try:
                                ser.write(self._wq.get_nowait().encode())
                            except queue.Empty:
                                break
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
                        elif tag == 'NOTOUCH' and len(parts) >= 3:
                            try:
                                self.q.put(('notouch', int(parts[1]), int(parts[2])))
                            except ValueError:
                                self.q.put(('notouch', 0, 0))
                        elif tag == 'NOTOUCH':
                            self.q.put(('notouch', 0, 0))
                        elif tag == 'SAVED':
                            self.q.put(('saved', ','.join(parts[1:])))
                        elif tag == 'CAL' and len(parts) >= 5:
                            self.q.put(('cal_confirmed', line))
            except serial.SerialException as e:
                self.q.put(('status', f'erro: {e}'))
                time.sleep(2.0)

    def stop(self):
        self._stop.set()

# ── Aplicacao ────────────────────────────────────────────────────────────────

class App:
    def __init__(self, port, baud):
        pygame.init()
        self.screen = pygame.display.set_mode((WIN_W, WIN_H))
        pygame.display.set_caption(f"Ball Balancer — Calibracao  [{port}]")
        self.clock = pygame.time.Clock()

        self.f_xl = pygame.font.SysFont('consolas', 26, bold=True)
        self.f_lg = pygame.font.SysFont('consolas', 20, bold=True)
        self.f_md = pygame.font.SysFont('consolas', 17)
        self.f_sm = pygame.font.SysFont('consolas', 14)

        self.port, self.baud = port, baud
        self.q = queue.Queue()
        self.reader = SerialReader(port, baud, self.q)
        self.reader.start()

        # Dados de posicao
        self.cur_xr    = 0
        self.cur_yr    = 0
        self.touched   = False
        self.pos_ts    = 0.0          # timestamp da ultima leitura valida
        self.conn_txt  = 'conectando...'

        # Calibracao
        self.corner_idx  = 0                       # canto atual 0–3
        self.corner_data = [None] * 4              # (xmed, ymed) por canto
        self.samples     = []                      # amostras do canto atual
        self.cal         = None                    # (x_min, x_max, y_min, y_max)
        self.save_msg    = ''
        self.save_ts     = 0.0

        # Orientacao (carregada do config, ajustavel ao vivo no LIVE)
        self.swap_xy = bool(_cfg('SWAP_XY', False))
        self.flip_x  = bool(_cfg('FLIP_X', False))
        self.flip_y  = bool(_cfg('FLIP_Y', False))

        # Live
        self.trail = collections.deque(maxlen=TRAIL_LEN)

        # ── Estado BALANCE (simulacao dos motores) ───────────────────────────
        self.pid_x = PID(KP, KI, KD, -TILT_LIMIT, TILT_LIMIT)
        self.pid_y = PID(KP, KI, KD, -TILT_LIMIT, TILT_LIMIT)
        self.sx    = TABLE_W_MM / 2.0     # alvo (setpoint) em mm
        self.sy    = TABLE_H_MM / 2.0
        self.ball_mm     = None           # posicao suavizada da bola [x,y] mm
        self.bal_seen_ts = 0.0            # ultimo instante com bola
        self.disp_nx = 0.0                # inclinacao exibida (suavizada)
        self.disp_ny = 0.0
        self.disp_theta = [ANG_ORIG, ANG_ORIG, ANG_ORIG]
        self.cam_az    = math.radians(35)
        self.cam_pitch = math.radians(30)
        self.cam_scale = 2.2
        self.bal_t     = None             # timestamp p/ dt do laco de balance

        # Maquina de estados
        # SPLASH → CAL_READY → CAL_COLLECTING → CAL_DONE → CAL_READY → ...
        # → CAL_RESULTS → LIVE
        self.state = 'SPLASH'

    # ── Fila serial ──────────────────────────────────────────────────────────

    def _drain(self):
        for _ in range(40):
            if self.q.empty():
                break
            item = self.q.get_nowait()
            tag  = item[0]
            if tag == 'status':
                self.conn_txt = item[1]
            elif tag == 'pos':
                self.cur_xr  = item[1]
                self.cur_yr  = item[2]
                self.touched = True
                self.pos_ts  = time.time()
                if self.state == 'CAL_COLLECTING':
                    self.samples.append((item[1], item[2]))
                elif self.state == 'LIVE':
                    self.trail.append((item[1], item[2], time.time()))
            elif tag == 'notouch':
                self.touched = False
                if len(item) >= 3:
                    self.cur_xr = item[1]
                    self.cur_yr = item[2]
            elif tag == 'saved':
                what = item[1] if len(item) > 1 else ''
                if 'touch' in what:
                    self.save_msg = "✓ Calibração salva no NVS do ESP32!"
                    self.save_ts  = time.time()
            elif tag == 'cal_confirmed':
                pass   # confirmação de echo — já mostrado no save_msg

        # Timeout de toque
        if self.touched and (time.time() - self.pos_ts) > 0.5:
            self.touched = False

    # ── Helpers de desenho ───────────────────────────────────────────────────

    def _t(self, text, color, x, y, font=None, cx=False):
        """Render texto (multi-linha por \n). Retorna y apos o bloco."""
        font = font or self.f_md
        for line in str(text).split('\n'):
            s = font.render(line, True, color)
            bx = x - s.get_width() // 2 if cx else x
            self.screen.blit(s, (bx, y))
            y += s.get_height() + 2
        return y

    def _bar(self, x, y, w, h, ratio, fill_col):
        pygame.draw.rect(self.screen, (35, 35, 48), (x, y, w, h), border_radius=4)
        fw = int(w * min(max(ratio, 0), 1))
        if fw > 0:
            pygame.draw.rect(self.screen, fill_col, (x, y, fw, h), border_radius=4)
        pygame.draw.rect(self.screen, C['border'], (x, y, w, h), 1, border_radius=4)

    def _corner_px(self, rect, rx, ry):
        """Pixel position of corner (rx,ry) inside rect."""
        margin = 6
        px = rect.x + margin           if rx == 0 else rect.x + rect.w - margin
        py = rect.y + margin           if ry == 1 else rect.y + rect.h - margin
        return px, py

    def _draw_mini_table(self, rect, active_corner, done_list):
        """Mesa em miniatura com indicadores de canto."""
        pygame.draw.rect(self.screen, C['table_bg'],  rect)
        pygame.draw.rect(self.screen, C['table_edge'], rect, 2)
        # Grade
        for i in range(1, 9):
            x = rect.x + i * rect.w // 9
            pygame.draw.line(self.screen, C['grid'], (x, rect.y), (x, rect.y+rect.h))
        for i in range(1, 7):
            y = rect.y + i * rect.h // 7
            pygame.draw.line(self.screen, C['grid'], (rect.x, y), (rect.x+rect.w, y))
        # Cruz central
        cx, cy = rect.x + rect.w//2, rect.y + rect.h//2
        pygame.draw.line(self.screen, C['dim'], (cx-10, cy), (cx+10, cy))
        pygame.draw.line(self.screen, C['dim'], (cx, cy-10), (cx, cy+10))
        # Cantos
        for i, (name, _, (rx, ry)) in enumerate(CORNERS):
            px, py = self._corner_px(rect, rx, ry)
            if i in done_list:
                pygame.draw.circle(self.screen, C['corner_ok'], (px, py), 9)
                pygame.draw.circle(self.screen, (30, 150, 55), (px, py), 9, 2)
            elif i == active_corner:
                pygame.draw.circle(self.screen, C['highlight'], (px, py), 11)
                if int(time.time() * 3) % 2 == 0:
                    pygame.draw.circle(self.screen, C['highlight'], (px, py), 15, 2)
            else:
                pygame.draw.circle(self.screen, C['border'], (px, py), 6)

    # ── SPLASH ───────────────────────────────────────────────────────────────

    def _splash(self):
        s = self.screen
        s.fill(C['bg'])

        y = 60
        y = self._t("BALL BALANCER", C['heading'], WIN_W//2, y, self.f_xl, cx=True) + 6
        y = self._t("Calibracao da Tela Resistiva", C['text'], WIN_W//2, y, self.f_lg, cx=True) + 36

        linhas = [
            ("Passo 1 —",  "Coleta de 80 amostras mediadas em cada um dos 4 cantos"),
            ("Passo 2 —",  "Calculo de X_MIN, X_MAX, Y_MIN, Y_MAX"),
            ("Passo 3 —",  "Opcao de salvar direto em main/touch_screen.h"),
            ("Passo 4 —",  "Visualizacao ao vivo no plano cartesiano (eixos em raw)"),
        ]
        for lbl, desc in linhas:
            x0 = WIN_W//2 - 340
            s2 = self.f_md.render(lbl, True, C['hint'])
            s3 = self.f_md.render(desc, True, C['text'])
            self.screen.blit(s2, (x0, y))
            self.screen.blit(s3, (x0 + s2.get_width() + 8, y))
            y += s2.get_height() + 6

        y += 20
        col = C['good'] if self.conn_txt == 'conectado' else C['warn']
        y = self._t(f"Porta: {self.port}  @{self.baud}  —  {self.conn_txt}",
                    col, WIN_W//2, y, self.f_sm, cx=True) + 10

        # Valores ao vivo — confirma que a serial esta funcionando
        raw_col = C['good'] if self.touched else C['dim']
        raw_lbl = "TOCANDO" if self.touched else "sem toque"
        y = self._t(f"X raw = {self.cur_xr:5d}    Y raw = {self.cur_yr:5d}    {raw_lbl}",
                    raw_col, WIN_W//2, y, self.f_md, cx=True) + 18

        pulse = int(time.time() * 2) % 2 == 0
        msg_col = C['highlight'] if pulse else C['warn']
        self._t("Pressione  ENTER  para comecar a calibracao", msg_col, WIN_W//2, y, self.f_lg, cx=True)
        y += 30

        cfg_ok = (hasattr(cfg, 'X_RAW_MIN') and hasattr(cfg, 'X_RAW_MAX') and
                  hasattr(cfg, 'Y_RAW_MIN') and hasattr(cfg, 'Y_RAW_MAX'))
        if cfg_ok:
            self._t("Pressione  L  para ir direto ao LIVE (calibracao de config.py)",
                    C['hint'], WIN_W//2, y, self.f_sm, cx=True)

    # ── CAL_READY ────────────────────────────────────────────────────────────

    def _cal_ready(self):
        s = self.screen
        s.fill(C['bg'])

        idx  = self.corner_idx
        name = CORNERS[idx][0]
        done = [i for i in range(4) if self.corner_data[i] is not None]

        self._t(f"CALIBRACAO — PASSO {idx+1} / 4", C['heading'],
                WIN_W//2, 18, self.f_lg, cx=True)

        # Mesa
        tbl = pygame.Rect(55, 75, 380, 295)
        self._draw_mini_table(tbl, idx, done)

        # Legenda de cantos coletados
        ly = tbl.y + tbl.h + 14
        for i, (cname, _, _) in enumerate(CORNERS):
            if self.corner_data[i]:
                xm, ym = self.corner_data[i]
                self._t(f"  {cname}: X={xm}  Y={ym}",
                        C['corner_ok'], tbl.x, ly, self.f_sm)
            else:
                self._t(f"  {cname}: —", C['dim'], tbl.x, ly, self.f_sm)
            ly += 18

        # Painel direito
        rx, ry = 490, 80
        ry = self._t(CORNERS[idx][1], C['text'], rx, ry, self.f_lg) + 18

        col = C['good'] if self.touched else C['warn']
        if self.touched:
            ry = self._t(f"X raw = {self.cur_xr:5d}", col, rx, ry)
            ry = self._t(f"Y raw = {self.cur_yr:5d}", col, rx, ry)
        else:
            ry = self._t("X raw =   ---", col, rx, ry)
            ry = self._t("Y raw =   ---", col, rx, ry)

        ry += 6
        status = "TOCANDO  ✓" if self.touched else "SEM TOQUE  (sem bola)"
        ry = self._t(status, col, rx, ry, self.f_sm) + 24

        self._t("Posicione a bola no canto indicado.", C['dim'], rx, ry, self.f_sm)
        ry += 20
        pulse = int(time.time() * 2) % 2 == 0
        self._t("Pressione  ESPACO  para coletar",
                C['highlight'] if pulse else C['warn'], rx, ry, self.f_md)

    # ── CAL_COLLECTING ───────────────────────────────────────────────────────

    def _cal_collecting(self):
        s = self.screen
        s.fill(C['bg'])

        idx  = self.corner_idx
        name = CORNERS[idx][0]
        done = [i for i in range(4) if self.corner_data[i] is not None]

        self._t(f"COLETANDO — {name}", C['heading'], WIN_W//2, 18, self.f_lg, cx=True)

        tbl = pygame.Rect(55, 75, 380, 295)
        self._draw_mini_table(tbl, idx, done)

        n     = len(self.samples)
        ratio = n / SAMPLES_NEEDED

        rx, ry = 490, 80

        ry = self._t("MANTENHA A BOLA PARADA!", C['warn'], rx, ry, self.f_lg) + 12
        ry = self._t(f"Amostras coletadas: {n} / {SAMPLES_NEEDED}", C['text'], rx, ry) + 8
        self._bar(rx, ry, 440, 22, ratio, C['good'])
        ry += 34

        col = C['good'] if self.touched else C['bad']
        if self.touched:
            ry = self._t(f"X raw = {self.cur_xr:5d}", col, rx, ry)
            ry = self._t(f"Y raw = {self.cur_yr:5d}", col, rx, ry)
        else:
            ry = self._t("X raw =   ---  (bola saiu!)", C['bad'], rx, ry)
            ry = self._t("Y raw =   ---", C['bad'], rx, ry)

        # Estatisticas em tempo real
        if len(self.samples) >= 6:
            xs = [p[0] for p in self.samples]
            ys = [p[1] for p in self.samples]
            ry += 16
            ry = self._t("Estatisticas parciais:", C['dim'], rx, ry, self.f_sm)
            ry = self._t(f"  X  mediana={statistics.median(xs):.0f}  desvio={statistics.stdev(xs):.1f}",
                         C['dim'], rx, ry, self.f_sm)
            ry = self._t(f"  Y  mediana={statistics.median(ys):.0f}  desvio={statistics.stdev(ys):.1f}",
                         C['dim'], rx, ry, self.f_sm)

    # ── CAL_DONE (canto concluido) ────────────────────────────────────────────

    def _cal_done(self):
        s = self.screen
        s.fill(C['bg'])

        prev_idx  = self.corner_idx - 1
        prev_name = CORNERS[prev_idx][0]
        done = [i for i in range(4) if self.corner_data[i] is not None]

        self._t(f"CANTO COLETADO — {prev_name}",
                C['good'], WIN_W//2, 18, self.f_lg, cx=True)

        tbl = pygame.Rect(55, 75, 380, 295)
        active = self.corner_idx if self.corner_idx < 4 else None
        self._draw_mini_table(tbl, active, done)

        xm, ym = self.corner_data[prev_idx]

        rx, ry = 490, 80
        ry = self._t(f"Mediana coletada:", C['dim'], rx, ry, self.f_sm) + 4
        ry = self._t(f"X raw = {xm}", C['good'], rx, ry) + 2
        ry = self._t(f"Y raw = {ym}", C['good'], rx, ry) + 20

        if self.corner_idx < 4:
            ry = self._t(f"Proximo canto:", C['dim'], rx, ry, self.f_sm) + 4
            ry = self._t(CORNERS[self.corner_idx][0], C['text'], rx, ry) + 16
            pulse = int(time.time() * 2) % 2 == 0
            self._t("Pressione  ESPACO  para continuar",
                    C['highlight'] if pulse else C['warn'], rx, ry)
        else:
            ry = self._t("Todos os 4 cantos coletados!", C['good'], rx, ry, self.f_lg) + 20
            pulse = int(time.time() * 2) % 2 == 0
            self._t("Pressione  ESPACO  para ver resultado",
                    C['highlight'] if pulse else C['warn'], rx, ry)

    # ── CAL_RESULTS ──────────────────────────────────────────────────────────

    def _cal_results(self):
        s = self.screen
        s.fill(C['bg'])

        self._t("RESULTADO DA CALIBRACAO", C['heading'],
                WIN_W//2, 18, self.f_xl, cx=True)

        x_min, x_max, y_min, y_max = self.cal

        done = [0, 1, 2, 3]
        tbl = pygame.Rect(55, 80, 300, 230)
        self._draw_mini_table(tbl, None, done)

        # Tabela de cantos
        ty = tbl.y + tbl.h + 14
        self._t("Cantos coletados (mediana raw):", C['dim'], tbl.x, ty, self.f_sm)
        ty += 18
        hdrs = [("CANTO", 220), ("X raw", 70), ("Y raw", 70)]
        tx   = tbl.x
        for hdr, w in hdrs:
            self._t(hdr, C['dim'], tx, ty, self.f_sm)
            tx += w
        ty += 16
        for i, (name, _, _) in enumerate(CORNERS):
            tx = tbl.x
            xm, ym = self.corner_data[i]
            for val, w in [(name, 220), (str(xm), 70), (str(ym), 70)]:
                self._t(val, C['text'], tx, ty, self.f_sm)
                tx += w
            ty += 16

        # Valores calculados
        rx, ry = 400, 88
        ry = self._t("Valores calculados:", C['dim'], rx, ry, self.f_sm) + 6
        ry = self._t(f"#define TOUCH_X_RAW_MIN  {x_min}", C['good'], rx, ry) + 2
        ry = self._t(f"#define TOUCH_X_RAW_MAX  {x_max}", C['good'], rx, ry) + 2
        ry = self._t(f"#define TOUCH_Y_RAW_MIN  {y_min}", C['good'], rx, ry) + 2
        ry = self._t(f"#define TOUCH_Y_RAW_MAX  {y_max}", C['good'], rx, ry) + 2
        ry += 8
        ry = self._t(f"Span X: {x_max - x_min} raw  ({(x_max-x_min)/4095*100:.1f}% de fundo de escala)",
                     C['dim'], rx, ry, self.f_sm) + 2
        ry = self._t(f"Span Y: {y_max - y_min} raw  ({(y_max-y_min)/4095*100:.1f}% de fundo de escala)",
                     C['dim'], rx, ry, self.f_sm) + 24

        # Salvar
        exists = HEADER_PATH.exists()
        if exists:
            self._t(f"Arquivo: {HEADER_PATH}", C['dim'], rx, ry, self.f_sm)
            ry += 20
            self._t("Pressione  S  para salvar em touch_screen.h", C['hint'], rx, ry)
            ry += 26
        else:
            self._t(f"touch_screen.h nao encontrado em {HEADER_PATH}",
                    C['warn'], rx, ry, self.f_sm)
            ry += 24

        self._t("Pressione  ENTER  para visualizacao ao vivo",
                C['heading'], rx, ry, self.f_lg)
        ry += 30
        self._t("Pressione  N  para salvar no NVS do ESP32 (modo CONTROL)",
                C['hint'], rx, ry, self.f_sm)
        ry += 18
        self._t("  Sem rebuild! Valores carregados automaticamente no proximo boot.",
                C['dim'], rx, ry, self.f_sm)

        # Toast de salvamento
        if self.save_msg and (time.time() - self.save_ts) < 3.0:
            col = C['save_ok'] if 'salvo' in self.save_msg.lower() else C['bad']
            surf = self.f_lg.render(self.save_msg, True, col)
            self.screen.blit(surf, (WIN_W//2 - surf.get_width()//2, WIN_H - 52))

    # ── LIVE ────────────────────────────────────────────────────────────────

    @staticmethod
    def _nice_step(span, divs):
        raw = span / divs
        mag = 10 ** math.floor(math.log10(max(raw, 1)))
        for f in (1, 2, 5, 10):
            if mag * f >= raw:
                return int(mag * f)
        return int(raw) + 1

    def _orient(self, nx, ny):
        """Aplica swap/flip a coordenadas normalizadas (0..1)."""
        if self.swap_xy:
            nx, ny = ny, nx
        if self.flip_x:
            nx = 1.0 - nx
        if self.flip_y:
            ny = 1.0 - ny
        return nx, ny

    def _orient_str(self):
        parts = []
        if self.swap_xy: parts.append("SWAP")
        if self.flip_x:  parts.append("FLIP_X")
        if self.flip_y:  parts.append("FLIP_Y")
        return " + ".join(parts) if parts else "padrao"

    def _persist_orientation(self):
        ok_cfg = self._write_orientation(
            CONFIG_PATH,
            [('SWAP_XY', self.swap_xy), ('FLIP_X', self.flip_x), ('FLIP_Y', self.flip_y)],
            r'(?m)^(%s\s*=\s*)(?:True|False)', lambda v: str(bool(v)))
        ok_hdr = self._write_orientation(
            HEADER_PATH,
            [('TOUCH_SWAP_XY', self.swap_xy), ('TOUCH_FLIP_X', self.flip_x), ('TOUCH_FLIP_Y', self.flip_y)],
            r'(#define\s+%s\s+)[01]', lambda v: '1' if v else '0')
        if ok_cfg and ok_hdr:
            self.save_msg = "Orientacao salva: config.py + touch_screen.h (rebuild p/ aplicar no firmware)"
        elif ok_cfg:
            self.save_msg = "Salvo em config.py (touch_screen.h falhou)"
        else:
            self.save_msg = "Erro ao salvar orientacao"
        self.save_ts = time.time()

    @staticmethod
    def _write_orientation(path, pairs, pattern_tmpl, fmt):
        try:
            if not path.exists():
                return False
            txt = path.read_text(encoding='utf-8')
            for name, val in pairs:
                txt = re.sub(pattern_tmpl % name,
                             lambda m, v=val: m.group(1) + fmt(v),
                             txt)
            path.write_text(txt, encoding='utf-8')
            return True
        except Exception as e:
            print(f"[ERRO] gravando {path.name}: {e}")
            return False

    def _live(self):
        s = self.screen
        s.fill(C['bg'])

        x_min, x_max, y_min, y_max = self.cal
        x_span = (x_max - x_min) or 1
        y_span = (y_max - y_min) or 1

        # Titulo
        self._t("VISUALIZACAO AO VIVO  —  tela fisica em mm (aspecto real)",
                C['heading'], 30, 14, self.f_md)

        # Area disponivel
        PAD_L, PAD_R, PAD_T, PAD_B = 70, 20, 50, 72
        PANEL_W = 300
        avail_w = WIN_W - PANEL_W - PAD_L - PAD_R
        avail_h = WIN_H - PAD_T - PAD_B

        # Retangulo com aspecto correto (W:H) e escala uniforme — sem distorcer
        scale = min(avail_w / TABLE_W_MM, avail_h / TABLE_H_MM)
        rw = int(TABLE_W_MM * scale)
        rh = int(TABLE_H_MM * scale)
        ox = PAD_L + (avail_w - rw) // 2
        oy = PAD_T + (avail_h - rh) // 2
        plot_rect = pygame.Rect(ox, oy, rw, rh)

        def mm2p(xmm, ymm):
            px = ox + int(xmm / TABLE_W_MM * rw)
            py = oy + rh - int(ymm / TABLE_H_MM * rh)   # Y para cima
            return px, py

        def raw2mm(xr, yr):
            nx = max(0.0, min(1.0, (xr - x_min) / x_span))
            ny = max(0.0, min(1.0, (yr - y_min) / y_span))
            nx, ny = self._orient(nx, ny)
            return nx * TABLE_W_MM, ny * TABLE_H_MM

        pygame.draw.rect(s, C['table_bg'], plot_rect)

        # Grade a cada 20 mm
        gx = 20
        v = gx
        while v < TABLE_W_MM:
            px = ox + int(v / TABLE_W_MM * rw)
            pygame.draw.line(s, C['grid'], (px, oy), (px, oy + rh))
            v += gx
        v = gx
        while v < TABLE_H_MM:
            py = oy + rh - int(v / TABLE_H_MM * rh)
            pygame.draw.line(s, C['grid'], (ox, py), (ox + rw, py))
            v += gx

        # Cruz central
        cx, cy = mm2p(TABLE_W_MM / 2, TABLE_H_MM / 2)
        pygame.draw.line(s, C['dim'], (cx - 12, cy), (cx + 12, cy))
        pygame.draw.line(s, C['dim'], (cx, cy - 12), (cx, cy + 12))

        # Eixos + borda
        pygame.draw.rect(s, C['table_edge'], plot_rect, 2)
        pygame.draw.line(s, C['axis'], (ox, oy + rh), (ox + rw, oy + rh), 2)
        pygame.draw.line(s, C['axis'], (ox, oy), (ox, oy + rh), 2)

        # Ticks em mm
        for v in range(0, int(TABLE_W_MM) + 1, 40):
            px = ox + int(v / TABLE_W_MM * rw)
            lbl = self.f_sm.render(str(v), True, C['dim'])
            s.blit(lbl, (px - lbl.get_width() // 2, oy + rh + 6))
        for v in range(0, int(TABLE_H_MM) + 1, 40):
            py = oy + rh - int(v / TABLE_H_MM * rh)
            lbl = self.f_sm.render(str(v), True, C['dim'])
            s.blit(lbl, (ox - lbl.get_width() - 6, py - lbl.get_height() // 2))

        # Labels de eixo
        xl = self.f_sm.render(f"X (mm)  0 - {TABLE_W_MM:.0f}", True, C['axis'])
        s.blit(xl, (ox + rw // 2 - xl.get_width() // 2, oy + rh + 28))
        yl = self.f_sm.render(f"Y (mm)  0 - {TABLE_H_MM:.0f}", True, C['axis'])
        yl_rot = pygame.transform.rotate(yl, 90)
        s.blit(yl_rot, (ox - yl_rot.get_width() - 30,
                        oy + rh // 2 - yl_rot.get_height() // 2))

        # Marcadores dos cantos calibrados (com orientacao aplicada)
        for _, _, (rx_rel, ry_rel) in CORNERS:
            cxr = x_min if rx_rel == 0 else x_max
            cyr = y_min if ry_rel == 0 else y_max
            p = mm2p(*raw2mm(cxr, cyr))
            pygame.draw.circle(s, C['corner_ok'], p, 5)
            pygame.draw.circle(s, (20, 120, 50), p, 5, 2)

        # Trilha
        now = time.time()
        for xr, yr, ts in list(self.trail):
            fade = max(0.0, 1.0 - (now - ts) / 5.0)
            if fade <= 0:
                continue
            r = int(C['trail_tail'][0] + (C['trail_head'][0] - C['trail_tail'][0]) * fade)
            g = int(C['trail_tail'][1] + (C['trail_head'][1] - C['trail_tail'][1]) * fade)
            b = int(C['trail_tail'][2] + (C['trail_head'][2] - C['trail_tail'][2]) * fade)
            p = mm2p(*raw2mm(xr, yr))
            if plot_rect.collidepoint(p):
                pygame.draw.circle(s, (r, g, b), p, max(2, int(5 * fade)))

        # Posicao atual
        cur_mm = None
        if self.touched:
            cur_mm = raw2mm(self.cur_xr, self.cur_yr)
            p = mm2p(*cur_mm)
            # Clamp: garante visibilidade mesmo quando mm == limite exato
            px = max(plot_rect.x, min(p[0], plot_rect.right  - 1))
            py = max(plot_rect.y, min(p[1], plot_rect.bottom - 1))
            pygame.draw.circle(s, C['ball'], (px, py), 12)
            pygame.draw.circle(s, (255, 90, 90), (px, py), 12, 2)

        # ── Painel lateral ───────────────────────────────────────────────────
        rpx = WIN_W - PANEL_W + 14
        rpy = 46

        def rp(text, color, y_off, fnt=None):
            fnt = fnt or self.f_md
            for line in str(text).split('\n'):
                surf = fnt.render(line, True, color)
                s.blit(surf, (rpx, rpy + y_off))
                y_off += surf.get_height() + 2
            return y_off

        y = 0
        y = rp("POSICAO", C['heading'], y, self.f_lg) + 6
        col = C['good'] if self.touched else C['warn']
        y = rp("TOCANDO" if self.touched else "SEM TOQUE", col, y) + 4
        y = rp(f"X raw = {self.cur_xr:5d}", col, y)
        y = rp(f"Y raw = {self.cur_yr:5d}", col, y) + 6
        if cur_mm:
            y = rp(f"X = {cur_mm[0]:6.1f} mm", C['text'], y, self.f_sm)
            y = rp(f"Y = {cur_mm[1]:6.1f} mm", C['text'], y, self.f_sm)
        else:
            y = rp("X =   --  mm", C['dim'], y, self.f_sm)
            y = rp("Y =   --  mm", C['dim'], y, self.f_sm)

        y += 14
        y = rp("ORIENTACAO", C['heading'], y, self.f_sm) + 4
        y = rp(self._orient_str(), C['hint'], y) + 2
        y = rp(f"  SWAP_XY = {self.swap_xy}", C['dim'], y, self.f_sm)
        y = rp(f"  FLIP_X  = {self.flip_x}", C['dim'], y, self.f_sm)
        y = rp(f"  FLIP_Y  = {self.flip_y}", C['dim'], y, self.f_sm)

        y += 14
        y = rp("CALIBRACAO", C['heading'], y, self.f_sm) + 4
        y = rp(f"X: {x_min} - {x_max}", C['hint'], y, self.f_sm)
        y = rp(f"Y: {y_min} - {y_max}", C['hint'], y, self.f_sm)

        y += 14
        conn_col = C['good'] if self.conn_txt == 'conectado' else C['warn']
        y = rp(f"{self.port} @{self.baud}", C['dim'], y, self.f_sm)

        y += 14
        y = rp("── TECLAS ──", C['dim'], y, self.f_sm)
        y = rp("T = trocar eixos (swap)", C['text'], y, self.f_sm)
        y = rp("X = espelhar X", C['text'], y, self.f_sm)
        y = rp("Y = espelhar Y", C['text'], y, self.f_sm)
        y = rp("G = gravar orientacao", C['save_ok'], y, self.f_sm)
        y = rp("R = limpar trilha", C['dim'], y, self.f_sm)
        y = rp("B = simular motores 3RPS", C['heading'], y, self.f_sm)
        y = rp("ESC = sair", C['dim'], y, self.f_sm)

        # Toast de salvamento
        if self.save_msg and (time.time() - self.save_ts) < 3.5:
            col2 = C['save_ok'] if 'salv' in self.save_msg.lower() else C['bad']
            surf = self.f_sm.render(self.save_msg, True, col2)
            s.blit(surf, (ox, WIN_H - 26))

    # ── BALANCE — simulacao isometrica dos motores ───────────────────────────

    def _raw2mm(self, xr, yr):
        """raw ADC -> mm, aplicando calibracao + orientacao (swap/flip)."""
        x_min, x_max, y_min, y_max = self.cal
        nx = max(0.0, min(1.0, (xr - x_min) / ((x_max - x_min) or 1)))
        ny = max(0.0, min(1.0, (yr - y_min) / ((y_max - y_min) or 1)))
        nx, ny = self._orient(nx, ny)
        return nx * TABLE_W_MM, ny * TABLE_H_MM

    def _iso(self, x, y, z, cx, cy):
        """Projecao isometrica (vista diagonal de cima). x,y,z em mm centrados."""
        ca, sa = math.cos(self.cam_az), math.sin(self.cam_az)
        xr = x * ca - y * sa
        yr = x * sa + y * ca
        sp = math.sin(self.cam_pitch)
        return (int(cx + xr * self.cam_scale),
                int(cy + (yr * sp - z) * self.cam_scale))

    def _balance_update(self):
        """Passo de controle + suavizacao (independente do FPS)."""
        now = time.time()
        dt = 0.0 if self.bal_t is None else now - self.bal_t
        self.bal_t = now
        dt = min(max(dt, 1e-3), 0.05)

        # 1) posicao da bola: alvo real -> suavizado (IIR temporal)
        if self.touched:
            tx, ty = self._raw2mm(self.cur_xr, self.cur_yr)
            if self.ball_mm is None:
                self.ball_mm = [tx, ty]
            else:
                a = 1.0 - math.exp(-dt / BALL_TAU)
                self.ball_mm[0] += (tx - self.ball_mm[0]) * a
                self.ball_mm[1] += (ty - self.ball_mm[1]) * a
            self.bal_seen_ts = now

        present = (self.ball_mm is not None and
                   (now - self.bal_seen_ts) < PRESENCE_HOLD)

        # 2) controlador (sobre a bola suavizada) -> inclinacao alvo
        if present:
            tnx = self.pid_x.update(self.ball_mm[0], self.sx, dt)
            tny = self.pid_y.update(self.ball_mm[1], self.sy, dt)
        else:
            self.pid_x.reset(); self.pid_y.reset()
            tnx = tny = 0.0
            if self.ball_mm is not None and (now - self.bal_seen_ts) > 1.2:
                self.ball_mm = None     # some de vez apos sair

        # 3) inclinacao/motores exibidos: suavizados -> sem salto/flicker
        a2 = 1.0 - math.exp(-dt / TILT_TAU)
        self.disp_nx += (tnx - self.disp_nx) * a2
        self.disp_ny += (tny - self.disp_ny) * a2
        self.disp_theta = [machine_theta(i, GEO_HZ, self.disp_nx, self.disp_ny)
                           for i in range(3)]
        return present

    def _balance(self):
        present = self._balance_update()
        s = self.screen
        s.fill(C['bg'])

        self._t("SIMULACAO DOS MOTORES  —  bola REAL, 3x NEMA 17 simulados",
                C['heading'], 30, 14, self.f_md)

        nx, ny = self.disp_nx, self.disp_ny
        hw, hh = TABLE_W_MM / 2.0, TABLE_H_MM / 2.0
        PANEL_W = 300
        cx = (WIN_W - PANEL_W) // 2
        cy = WIN_H // 2 + 70

        # --- base + bielas + braços dos motores ---
        base_pts, plat_pts = [], []
        for az in LEG_AZ:
            base_pts.append((GEO_D * 2.2 * math.cos(az),
                             GEO_D * 2.2 * math.sin(az), 0.0))
            px, py = hw * math.cos(az), hh * math.sin(az)
            plat_pts.append((px, py, plate_z(px, py, nx, ny, PLATE_BASE_Z)))

        for i in range(3):
            pygame.draw.line(s, C['iso_base'],
                             self._iso(*base_pts[i], cx, cy),
                             self._iso(*base_pts[(i + 1) % 3], cx, cy), 2)

        arm_tips = []
        for i, az in enumerate(LEG_AZ):
            bx3, by3, _ = base_pts[i]
            delta = math.radians(self.disp_theta[i] - ANG_ORIG)
            rad = (math.cos(az), math.sin(az))
            ax3 = bx3 - rad[0] * GEO_F * math.cos(delta)
            ay3 = by3 - rad[1] * GEO_F * math.cos(delta)
            az3 = GEO_F * math.sin(delta) + 10.0
            bj = self._iso(bx3, by3, 0.0, cx, cy)
            at = self._iso(ax3, ay3, az3, cx, cy)
            pj = self._iso(*plat_pts[i], cx, cy)
            arm_tips.append((at, pj, az))
            pygame.draw.circle(s, C['iso_base'], bj, 5)
            pygame.draw.line(s, C['iso_arm'], bj, at, 4)   # braço do motor
            pygame.draw.line(s, C['iso_leg'], at, pj, 2)   # biela

        # --- tampo inclinado ---
        quad = [self._iso(X, Y, plate_z(X, Y, nx, ny, PLATE_BASE_Z), cx, cy)
                for (X, Y) in [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)]]
        pygame.draw.polygon(s, C['iso_plate'], quad)
        pygame.draw.polygon(s, C['iso_edge'], quad, 2)

        # grade na superficie
        for k in range(1, 8):
            t = k / 8.0
            X = -hw + t * TABLE_W_MM
            pygame.draw.line(s, C['iso_grid'],
                self._iso(X, -hh, plate_z(X, -hh, nx, ny, PLATE_BASE_Z), cx, cy),
                self._iso(X,  hh, plate_z(X,  hh, nx, ny, PLATE_BASE_Z), cx, cy), 1)
            Y = -hh + t * TABLE_H_MM
            pygame.draw.line(s, C['iso_grid'],
                self._iso(-hw, Y, plate_z(-hw, Y, nx, ny, PLATE_BASE_Z), cx, cy),
                self._iso( hw, Y, plate_z( hw, Y, nx, ny, PLATE_BASE_Z), cx, cy), 1)

        # alvo
        tX, tY = self.sx - hw, self.sy - hh
        tp = self._iso(tX, tY, plate_z(tX, tY, nx, ny, PLATE_BASE_Z), cx, cy)
        pygame.draw.circle(s, C['iso_target'], tp, 7, 2)
        pygame.draw.line(s, C['iso_target'], (tp[0] - 9, tp[1]), (tp[0] + 9, tp[1]), 1)
        pygame.draw.line(s, C['iso_target'], (tp[0], tp[1] - 9), (tp[0], tp[1] + 9), 1)

        # bola (suavizada)
        if self.ball_mm is not None:
            bX, bY = self.ball_mm[0] - hw, self.ball_mm[1] - hh
            bZ = plate_z(bX, bY, nx, ny, PLATE_BASE_Z)
            sp = self._iso(bX, bY, bZ, cx, cy)
            bp = self._iso(bX, bY, bZ + 6.0, cx, cy)
            pygame.draw.circle(s, C['iso_plate2'], sp, 7)      # sombra
            r = max(7, int(6.0 * self.cam_scale))
            pygame.draw.circle(s, C['iso_ball'], bp, r)
            pygame.draw.circle(s, C['iso_ballhi'], (bp[0] - r // 3, bp[1] - r // 3),
                               max(2, r // 3))

        # rotulos dos motores (por cima de tudo)
        for i, (at, pj, az) in enumerate(arm_tips):
            pygame.draw.circle(s, C['iso_arm'], at, 3)
            lbl = self.f_sm.render(f"{LEG_NAME[i]} {self.disp_theta[i]:.1f}°",
                                   True, C['iso_arm'])
            s.blit(lbl, (at[0] + 6, at[1] - 6))

        # ── painel lateral ───────────────────────────────────────────────────
        self._balance_panel(present, PANEL_W)

    def _balance_panel(self, present, panel_w):
        s = self.screen
        rpx = WIN_W - panel_w + 14
        rpy = 46

        def rp(text, color, y_off, fnt=None):
            fnt = fnt or self.f_md
            for line in str(text).split('\n'):
                surf = fnt.render(line, True, color)
                s.blit(surf, (rpx, rpy + y_off))
                y_off += surf.get_height() + 2
            return y_off

        y = 0
        y = rp("MOTORES 3RPS", C['heading'], y, self.f_lg) + 6
        col = C['good'] if present else C['warn']
        y = rp("BOLA DETECTADA" if present else "SEM BOLA (nivelando)", col, y) + 6

        if self.ball_mm is not None:
            y = rp(f"Bola X = {self.ball_mm[0]:6.1f} mm", C['text'], y, self.f_sm)
            y = rp(f"Bola Y = {self.ball_mm[1]:6.1f} mm", C['text'], y, self.f_sm)
            err = math.hypot(self.ball_mm[0] - self.sx, self.ball_mm[1] - self.sy)
            y = rp(f"Erro   = {err:6.1f} mm", C['text'], y, self.f_sm) + 6
        else:
            y = rp("Bola: --", C['dim'], y, self.f_sm) + 6

        y = rp(f"Alvo X = {self.sx:6.1f} mm", C['iso_target'], y, self.f_sm)
        y = rp(f"Alvo Y = {self.sy:6.1f} mm", C['iso_target'], y, self.f_sm) + 8

        y = rp("INCLINACAO", C['heading'], y, self.f_sm) + 4
        y = rp(f"nx = {self.disp_nx:+.3f}", C['text'], y, self.f_sm)
        y = rp(f"ny = {self.disp_ny:+.3f}", C['text'], y, self.f_sm) + 8

        y = rp("ANGULOS (passos)", C['heading'], y, self.f_sm) + 4
        for i in range(3):
            y = rp(f"  {LEG_NAME[i]} = {self.disp_theta[i]:7.2f}°",
                   C['iso_arm'], y, self.f_sm)
        y += 10

        y = rp("ORIENTACAO", C['heading'], y, self.f_sm) + 4
        y = rp(self._orient_str(), C['hint'], y, self.f_sm) + 8

        y = rp("── TECLAS ──", C['dim'], y, self.f_sm)
        y = rp("WASD = mover alvo", C['text'], y, self.f_sm)
        y = rp("T/X/Y = orientacao", C['text'], y, self.f_sm)
        y = rp("G = gravar orientacao", C['save_ok'], y, self.f_sm)
        y = rp("Q/E = girar  +/- = zoom", C['text'], y, self.f_sm)
        y = rp("B/ENTER = voltar ao mapa", C['text'], y, self.f_sm)
        y = rp("ESC = sair", C['dim'], y, self.f_sm)

        if self.save_msg and (time.time() - self.save_ts) < 3.5:
            col2 = C['save_ok'] if 'salv' in self.save_msg.lower() else C['bad']
            surf = self.f_sm.render(self.save_msg, True, col2)
            s.blit(surf, (30, WIN_H - 26))

    # ── Logica de calibracao ─────────────────────────────────────────────────

    def _finish_corner(self):
        xs = [p[0] for p in self.samples]
        ys = [p[1] for p in self.samples]
        xm = int(statistics.median(xs))
        ym = int(statistics.median(ys))
        self.corner_data[self.corner_idx] = (xm, ym)
        self.corner_idx += 1
        self.state = 'CAL_DONE'

    def _compute_cal(self):
        all_x = [d[0] for d in self.corner_data if d]
        all_y = [d[1] for d in self.corner_data if d]
        x_min, x_max = min(all_x), max(all_x)
        y_min, y_max = min(all_y), max(all_y)
        if x_min >= x_max: x_max = x_min + 1
        if y_min >= y_max: y_max = y_min + 1
        return (x_min, x_max, y_min, y_max)

    def _push_to_nvs(self):
        """Envia a calibracao diretamente para o NVS do ESP32 via SETCAL + CAL SAVE.
        Requer que o firmware esteja em modo CONTROL (BB_MODE_CONTROL) com
        o cmd_task ativo. Nao requer rebuild nem reflash."""
        if not self.cal:
            self.save_msg = "Erro: calibracao nao calculada ainda."
            self.save_ts  = time.time()
            return

        x_min, x_max, y_min, y_max = self.cal

        # Calcula flip_x / flip_y a partir dos cantos coletados
        # corner_data: [TL(0), TR(1), BL(2), BR(3)]
        left_x  = (self.corner_data[0][0] + self.corner_data[2][0]) // 2
        right_x = (self.corner_data[1][0] + self.corner_data[3][0]) // 2
        top_y   = (self.corner_data[0][1] + self.corner_data[1][1]) // 2
        bot_y   = (self.corner_data[2][1] + self.corner_data[3][1]) // 2
        flip_x  = 1 if left_x > right_x else 0
        flip_y  = 1 if top_y  > bot_y   else 0
        swap_xy = 1 if self.swap_xy else 0

        cmd = f"SETCAL {x_min} {x_max} {y_min} {y_max} {flip_x} {flip_y} {swap_xy}"
        self.reader.write(cmd)
        time.sleep(0.15)
        self.reader.write("CAL SAVE")

        self.save_msg = "Enviado ao ESP32 — aguardando confirmacao NVS..."
        self.save_ts  = time.time()

    def _save_header(self):
        x_min, x_max, y_min, y_max = self.cal
        try:
            text = HEADER_PATH.read_text(encoding='utf-8')
            for define, value in [
                ('TOUCH_X_RAW_MIN', x_min),
                ('TOUCH_X_RAW_MAX', x_max),
                ('TOUCH_Y_RAW_MIN', y_min),
                ('TOUCH_Y_RAW_MAX', y_max),
            ]:
                text = re.sub(
                    r'(#define\s+' + define + r'\s+)\d+',
                    lambda m, v=value: m.group(1) + str(v),
                    text
                )
            HEADER_PATH.write_text(text, encoding='utf-8')
            return True
        except Exception as e:
            print(f"[ERRO] {e}")
            return False

    # ── Atalho: pula wizard usando calibracao de config.py ───────────────────

    def _load_cal_from_config(self):
        """Carrega calibracao de config.py e vai direto para o estado LIVE."""
        if not (hasattr(cfg, 'X_RAW_MIN') and hasattr(cfg, 'X_RAW_MAX') and
                hasattr(cfg, 'Y_RAW_MIN') and hasattr(cfg, 'Y_RAW_MAX')):
            return
        x_min = int(_cfg('X_RAW_MIN', 499))
        x_max = int(_cfg('X_RAW_MAX', 3081))
        y_min = int(_cfg('Y_RAW_MIN', 431))
        y_max = int(_cfg('Y_RAW_MAX', 3797))
        if x_min >= x_max: x_max = x_min + 1
        if y_min >= y_max: y_max = y_min + 1
        self.cal = (x_min, x_max, y_min, y_max)
        self.state = 'LIVE'

    # ── Loop principal ────────────────────────────────────────────────────────

    def run(self):
        running = True
        try:
            while running:
                self._drain()

                for ev in pygame.event.get():
                    if ev.type == pygame.QUIT:
                        running = False

                    elif ev.type == pygame.KEYDOWN:
                        k = ev.key

                        if k == pygame.K_ESCAPE:
                            running = False

                        elif self.state == 'SPLASH':
                            if k == pygame.K_RETURN:
                                self.state = 'CAL_READY'
                            elif k == pygame.K_l:
                                self._load_cal_from_config()



                        elif self.state == 'CAL_READY':
                            if k == pygame.K_SPACE:
                                self.samples = []
                                self.state = 'CAL_COLLECTING'

                        elif self.state == 'CAL_DONE':
                            if k == pygame.K_SPACE:
                                if self.corner_idx < 4:
                                    self.state = 'CAL_READY'
                                else:
                                    self.cal = self._compute_cal()
                                    self.state = 'CAL_RESULTS'

                        elif self.state == 'CAL_RESULTS':
                            if k == pygame.K_s and HEADER_PATH.exists():
                                ok = self._save_header()
                                self.save_msg = "Salvo em touch_screen.h  (OK)" if ok else "Erro ao salvar!"
                                self.save_ts  = time.time()
                            elif k == pygame.K_n:
                                self._push_to_nvs()
                            elif k == pygame.K_RETURN:
                                self.state = 'LIVE'

                        elif self.state == 'LIVE':
                            if k == pygame.K_r:
                                self.trail.clear()
                            elif k == pygame.K_t:
                                self.swap_xy = not self.swap_xy
                                self.trail.clear()
                            elif k == pygame.K_x:
                                self.flip_x = not self.flip_x
                                self.trail.clear()
                            elif k == pygame.K_y:
                                self.flip_y = not self.flip_y
                                self.trail.clear()
                            elif k == pygame.K_g:
                                self._persist_orientation()
                            elif k == pygame.K_b:
                                self.bal_t = None       # zera dt ao entrar
                                self.state = 'BALANCE'

                        elif self.state == 'BALANCE':
                            if k in (pygame.K_b, pygame.K_RETURN):
                                self.state = 'LIVE'
                            elif k == pygame.K_t:
                                self.swap_xy = not self.swap_xy
                            elif k == pygame.K_x:
                                self.flip_x = not self.flip_x
                            elif k == pygame.K_y:
                                self.flip_y = not self.flip_y
                            elif k == pygame.K_g:
                                self._persist_orientation()
                            elif k == pygame.K_q:
                                self.cam_az -= math.radians(8)
                            elif k == pygame.K_e:
                                self.cam_az += math.radians(8)
                            elif k in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):
                                self.cam_scale = min(5.0, self.cam_scale * 1.1)
                            elif k in (pygame.K_MINUS, pygame.K_KP_MINUS):
                                self.cam_scale = max(1.0, self.cam_scale / 1.1)
                            elif k == pygame.K_a:
                                self.sx = max(0.0, self.sx - 8.0)
                            elif k == pygame.K_d:
                                self.sx = min(TABLE_W_MM, self.sx + 8.0)
                            elif k == pygame.K_s:
                                self.sy = max(0.0, self.sy - 8.0)
                            elif k == pygame.K_w:
                                self.sy = min(TABLE_H_MM, self.sy + 8.0)

                # Coleta concluida
                if self.state == 'CAL_COLLECTING' and len(self.samples) >= SAMPLES_NEEDED:
                    self._finish_corner()

                # Desenho
                draw = {
                    'SPLASH':          self._splash,
                    'CAL_READY':       self._cal_ready,
                    'CAL_COLLECTING':  self._cal_collecting,
                    'CAL_DONE':        self._cal_done,
                    'CAL_RESULTS':     self._cal_results,
                    'LIVE':            self._live,
                    'BALANCE':         self._balance,
                }
                draw.get(self.state, lambda: None)()

                pygame.display.flip()
                self.clock.tick(FPS)

        except KeyboardInterrupt:
            pass
        finally:
            self.reader.stop()
            pygame.quit()


def main():
    port = sys.argv[1] if len(sys.argv) >= 2 else DEFAULT_PORT
    baud = int(sys.argv[2]) if len(sys.argv) >= 3 else DEFAULT_BAUD
    App(port, baud).run()


if __name__ == '__main__':
    main()
