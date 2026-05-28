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

# ── Porta padrão — altere aqui se necessário ─────────────────────────────────
DEFAULT_PORT = "COM8"
DEFAULT_BAUD = 115200

# ── Constantes ───────────────────────────────────────────────────────────────

WIN_W, WIN_H      = 1060, 660
SAMPLES_NEEDED    = 80
FPS               = 60
TRAIL_LEN         = 150
TABLE_W_MM        = 187.0
TABLE_H_MM        = 141.0

HEADER_PATH = pathlib.Path(__file__).parent.parent / "main" / "touch_screen.h"

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

# ── Thread leitora serial ────────────────────────────────────────────────────

class SerialReader(threading.Thread):
    def __init__(self, port, baud, q):
        super().__init__(daemon=True)
        self.port, self.baud, self.q = port, baud, q
        self._stop = threading.Event()

    def run(self):
        while not self._stop.is_set():
            try:
                with serial.Serial(self.port, self.baud, timeout=1.0) as ser:
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
                        elif tag == 'NOTOUCH' and len(parts) >= 3:
                            try:
                                self.q.put(('notouch', int(parts[1]), int(parts[2])))
                            except ValueError:
                                self.q.put(('notouch', 0, 0))
                        elif tag == 'NOTOUCH':
                            self.q.put(('notouch', 0, 0))
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

        # Live
        self.trail = collections.deque(maxlen=TRAIL_LEN)

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
                # NOTOUCH tambem carrega valores raw reais do ADC — exibe-os
                if len(item) >= 3:
                    self.cur_xr = item[1]
                    self.cur_yr = item[2]

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
        self._t("Pressione  ENTER  para comecar", msg_col, WIN_W//2, y, self.f_lg, cx=True)

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

    def _live(self):
        s = self.screen
        s.fill(C['bg'])

        x_min, x_max, y_min, y_max = self.cal
        x_span = x_max - x_min or 1
        y_span = y_max - y_min or 1

        # Titulo
        self._t("VISUALIZACAO AO VIVO  —  eixos em raw", C['heading'],
                WIN_W//2 - 130, 14, self.f_md)

        # Geometria do plot
        PAD_L, PAD_R, PAD_T, PAD_B = 88, 20, 48, 62
        PANEL_W = 265
        pw = WIN_W - PANEL_W - PAD_L - PAD_R
        ph = WIN_H - PAD_T - PAD_B
        ox, oy = PAD_L, PAD_T

        def r2p(xr, yr):
            fx = max(0.0, min(1.0, (xr - x_min) / x_span))
            fy = max(0.0, min(1.0, (yr - y_min) / y_span))
            return (ox + int(fx * pw), oy + ph - int(fy * ph))

        plot_rect = pygame.Rect(ox, oy, pw, ph)
        pygame.draw.rect(s, C['table_bg'], plot_rect)

        # Grade X
        x_step = self._nice_step(x_span, 8)
        xt = (x_min // x_step + 1) * x_step
        while xt <= x_max:
            px_v = ox + int((xt - x_min) / x_span * pw)
            pygame.draw.line(s, C['grid'], (px_v, oy), (px_v, oy + ph))
            lbl = self.f_sm.render(str(xt), True, C['dim'])
            s.blit(lbl, (px_v - lbl.get_width()//2, oy + ph + 6))
            xt += x_step

        # Grade Y
        y_step = self._nice_step(y_span, 6)
        yt = (y_min // y_step + 1) * y_step
        while yt <= y_max:
            py_v = oy + ph - int((yt - y_min) / y_span * ph)
            pygame.draw.line(s, C['grid'], (ox, py_v), (ox + pw, py_v))
            lbl = self.f_sm.render(str(yt), True, C['dim'])
            s.blit(lbl, (ox - lbl.get_width() - 5, py_v - lbl.get_height()//2))
            yt += y_step

        # Eixos
        pygame.draw.line(s, C['axis'], (ox, oy+ph), (ox+pw, oy+ph), 2)
        pygame.draw.line(s, C['axis'], (ox, oy),    (ox, oy+ph),    2)
        pygame.draw.rect(s, C['table_edge'], plot_rect, 2)

        # Labels de eixo
        xl = self.f_sm.render("X  (raw)", True, C['axis'])
        s.blit(xl, (ox + pw//2 - xl.get_width()//2, oy + ph + 42))
        yl = self.f_sm.render("Y  (raw)", True, C['axis'])
        yr_rot = pygame.transform.rotate(yl, 90)
        s.blit(yr_rot, (ox - yr_rot.get_width() - 48, oy + ph//2 - yr_rot.get_height()//2))

        # Marcadores dos cantos calibrados
        for i, (_, _, (rx_rel, ry_rel)) in enumerate(CORNERS):
            cxr = x_min if rx_rel == 0 else x_max
            cyr = y_min if ry_rel == 0 else y_max
            cpx, cpy = r2p(cxr, cyr)
            pygame.draw.circle(s, C['corner_ok'], (cpx, cpy), 5)
            pygame.draw.circle(s, (20, 120, 50), (cpx, cpy), 5, 2)

        # Trilha
        now = time.time()
        trail = list(self.trail)
        for xr, yr, ts in trail:
            age  = now - ts
            fade = max(0.0, 1.0 - age / 5.0)
            if fade <= 0:
                continue
            r = int(C['trail_tail'][0] + (C['trail_head'][0] - C['trail_tail'][0]) * fade)
            g = int(C['trail_tail'][1] + (C['trail_head'][1] - C['trail_tail'][1]) * fade)
            b = int(C['trail_tail'][2] + (C['trail_head'][2] - C['trail_tail'][2]) * fade)
            ppx, ppy = r2p(xr, yr)
            if plot_rect.collidepoint(ppx, ppy):
                pygame.draw.circle(s, (r, g, b), (ppx, ppy), max(2, int(5 * fade)))

        # Posicao atual
        if self.touched:
            ppx, ppy = r2p(self.cur_xr, self.cur_yr)
            if plot_rect.collidepoint(ppx, ppy):
                pygame.draw.circle(s, C['ball'], (ppx, ppy), 12)
                pygame.draw.circle(s, (255, 90, 90), (ppx, ppy), 12, 2)

        # ── Painel lateral ───────────────────────────────────────────────────
        rpx = WIN_W - PANEL_W + 10
        rpy = 46
        lh  = 22

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
        y = rp(f"Y raw = {self.cur_yr:5d}", col, y) + 8

        if self.touched:
            xmm = (self.cur_xr - x_min) / x_span * TABLE_W_MM
            ymm = (self.cur_yr - y_min) / y_span * TABLE_H_MM
            xmm = max(0.0, min(TABLE_W_MM, xmm))
            ymm = max(0.0, min(TABLE_H_MM, ymm))
            y = rp(f"X = {xmm:6.1f} mm", C['dim'], y, self.f_sm)
            y = rp(f"Y = {ymm:6.1f} mm", C['dim'], y, self.f_sm)
        else:
            y = rp("X =   —  mm", C['dim'], y, self.f_sm)
            y = rp("Y =   —  mm", C['dim'], y, self.f_sm)

        y += 16
        y = rp("CALIBRACAO", C['heading'], y, self.f_sm) + 4
        y = rp(f"X: {x_min} – {x_max}", C['hint'], y, self.f_sm)
        y = rp(f"   span {x_max-x_min} raw", C['dim'], y, self.f_sm)
        y = rp(f"Y: {y_min} – {y_max}", C['hint'], y, self.f_sm)
        y = rp(f"   span {y_max-y_min} raw", C['dim'], y, self.f_sm)

        y += 16
        conn_col = C['good'] if self.conn_txt == 'conectado' else C['warn']
        y = rp(f"{self.port} @{self.baud}", C['dim'], y, self.f_sm)
        y = rp(self.conn_txt, conn_col, y, self.f_sm)

        y += 16
        y = rp("R = limpar trilha", C['dim'], y, self.f_sm)
        y = rp("ESC = sair", C['dim'], y, self.f_sm)

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
                            elif k == pygame.K_RETURN:
                                self.state = 'LIVE'

                        elif self.state == 'LIVE':
                            if k == pygame.K_r:
                                self.trail.clear()

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
