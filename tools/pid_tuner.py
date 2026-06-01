#!/usr/bin/env python3
"""
Ball Balancer — Ajuste em Tempo Real do PID
============================================
Uso:   python pid_tuner.py COM5
       python pid_tuner.py COM5 115200

Teclas (ajuste de ganhos):
  Q / A     — KP sobe / desce
  W / S     — KI sobe / desce
  E / D     — KD sobe / desce
  Tab       — muda passo:  1% → 5% → 10% → 25%
  Espaco    — salva no NVS do ESP32  (PID SAVE)
  R         — reseta integrador (envia  PID <kp> <ki> <kd>  novamente)
  ESC / F4  — sair
"""

import sys, os, threading, queue, time, collections, math

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
TABLE_W_MM   = float(_cfg('SCREEN_W_MM', 187.0))
TABLE_H_MM   = float(_cfg('SCREEN_H_MM', 141.0))

WIN_W, WIN_H  = 1100, 640
FPS           = 60
GRAPH_LEN     = 300          # pontos no histórico de erro
STEPS         = [0.01, 0.05, 0.10, 0.25]   # passos relativos disponíveis

# Paleta
C = {
    'bg':       (14, 14, 20),
    'panel':    (22, 22, 32),
    'border':   (48, 48, 68),
    'text':     (208, 208, 214),
    'dim':      (88,  88, 100),
    'heading':  (128, 184, 255),
    'good':     (55,  210,  95),
    'warn':     (218, 178,  58),
    'bad':      (218,  55,  55),
    'ball':     (210,  38,  38),
    'ballhi':   (255, 140, 140),
    'target':   ( 60, 200, 120),
    'grid':     ( 40,  44,  60),
    'table':    (242, 242, 228),
    'kp_bar':   ( 80, 160, 255),
    'ki_bar':   (255, 180,  60),
    'kd_bar':   ( 80, 220, 140),
    'err_line': (218,  80,  80),
    'save_ok':  ( 80, 230, 120),
    'hint':     (168, 128, 220),
}


# ── Thread leitora / escritora serial ─────────────────────────────────────────

class SerialIO(threading.Thread):
    def __init__(self, port, baud, rx_q):
        super().__init__(daemon=True)
        self.port, self.baud, self.rx_q = port, baud, rx_q
        self._tx_q  = queue.Queue()
        self._stop  = threading.Event()
        # Diagnóstico (leitura thread-safe o suficiente p/ debug)
        self.last_line = ''      # última linha crua recebida
        self.line_count = 0      # total de linhas recebidas
        self.pos_count = 0       # total de linhas POS válidas

    def write(self, line: str):
        self._tx_q.put(line.rstrip('\n') + '\n')

    def run(self):
        while not self._stop.is_set():
            try:
                with serial.Serial(self.port, self.baud, timeout=0.5) as ser:
                    self.rx_q.put(('status', 'conectado'))
                    # Pede o estado atual dos ganhos ao conectar
                    ser.write(b'?\n')
                    while not self._stop.is_set():
                        while not self._tx_q.empty():
                            try:
                                ser.write(self._tx_q.get_nowait().encode())
                            except queue.Empty:
                                break
                        raw = ser.readline()
                        if not raw:
                            continue
                        line = raw.decode('utf-8', errors='replace').strip()
                        if not line:
                            continue
                        self.last_line = line
                        self.line_count += 1
                        parts = line.split(',')
                        if not parts:
                            continue
                        tag = parts[0]
                        if tag == 'GAINS' and len(parts) >= 4:
                            try:
                                self.rx_q.put(('gains',
                                               float(parts[1]),
                                               float(parts[2]),
                                               float(parts[3])))
                            except ValueError:
                                pass
                        elif tag == 'POS' and len(parts) >= 5:
                            try:
                                self.rx_q.put(('pos',
                                               float(parts[3]),
                                               float(parts[4])))
                                self.pos_count += 1
                            except ValueError:
                                pass
                        elif tag == 'NOTOUCH':
                            self.rx_q.put(('notouch',))
                        elif tag == 'CTRL' and len(parts) >= 7:
                            try:
                                self.rx_q.put(('ctrl',
                                               float(parts[1]),
                                               float(parts[2]),
                                               float(parts[3]),
                                               float(parts[4]),
                                               float(parts[5]),
                                               float(parts[6])))
                            except ValueError:
                                pass
                        elif tag == 'SAVED':
                            self.rx_q.put(('saved', ','.join(parts[1:])))
            except serial.SerialException as e:
                self.rx_q.put(('status', f'erro: {e}'))
                time.sleep(2.0)

    def stop(self):
        self._stop.set()


# ── Aplicação ──────────────────────────────────────────────────────────────────

class App:
    def __init__(self, port, baud):
        pygame.init()
        self.screen = pygame.display.set_mode((WIN_W, WIN_H))
        pygame.display.set_caption(f"Ball Balancer — PID Tuner  [{port}]")
        self.clock = pygame.time.Clock()

        self.f_xl = pygame.font.SysFont('consolas', 28, bold=True)
        self.f_lg = pygame.font.SysFont('consolas', 20, bold=True)
        self.f_md = pygame.font.SysFont('consolas', 16)
        self.f_sm = pygame.font.SysFont('consolas', 13)

        self.port, self.baud = port, baud
        self.rx_q  = queue.Queue()
        self.io    = SerialIO(port, baud, self.rx_q)
        self.io.start()

        # Ganhos (sincronizados com o firmware)
        self.kp       = 8.0e-4
        self.ki       = 2.0e-5
        self.kd       = 1.2e-2
        self.step_idx = 1          # índice em STEPS (default: 5%)
        self.gains_ok = False      # True após primeiro GAINS recebido

        # Posição da bola
        self.ball_mm    = None     # (x, y) mm ou None
        self.ball_ts    = 0.0      # timestamp do último POS recebido
        self.setpt_mm   = (TABLE_W_MM / 2, TABLE_H_MM / 2)
        self.touched     = False
        self.nx, self.ny = 0.0, 0.0

        # Histórico de erro (distância ao setpoint, mm)
        self.err_hist = collections.deque(maxlen=GRAPH_LEN)

        # Feedback de salvamento
        self.save_msg = ''
        self.save_ts  = 0.0

        # Status de conexão
        self.conn_txt = 'conectando...'

    # ── Drenagem da fila serial ────────────────────────────────────────────────

    def _drain(self):
        for _ in range(60):
            if self.rx_q.empty():
                break
            item = self.rx_q.get_nowait()
            tag  = item[0]
            if tag == 'status':
                self.conn_txt = item[1]
            elif tag == 'gains':
                self.kp, self.ki, self.kd = item[1], item[2], item[3]
                self.gains_ok = True
            elif tag == 'pos':
                self.ball_mm = (item[1], item[2])
                self.touched = True
                self.ball_ts = time.time()
                err = math.hypot(item[1] - self.setpt_mm[0],
                                 item[2] - self.setpt_mm[1])
                self.err_hist.append((time.time(), err))
            elif tag == 'notouch':
                self.touched = False
                self.err_hist.append((time.time(), float('nan')))
            elif tag == 'ctrl':
                # CTRL,x_mm,y_mm,setx,sety,nx,ny,...
                self.nx, self.ny = item[5], item[6]
            elif tag == 'saved':
                if 'pid' in item[1]:
                    self.save_msg = '✓ Ganhos salvos no NVS do ESP32!'
                    self.save_ts  = time.time()

        # Limpa ball_mm só depois de 0.5 s sem toque (mesmo comportamento do calibrate.py)
        if self.ball_mm is not None and (time.time() - self.ball_ts) > 0.5:
            self.ball_mm = None

    # ── Comandos serial ────────────────────────────────────────────────────────

    def _send_gains(self):
        self.io.write(f"PID {self.kp:.6g} {self.ki:.6g} {self.kd:.6g}")

    def _adjust(self, which: str, sign: int):
        step = STEPS[self.step_idx]
        if which == 'kp':
            self.kp = max(0.0, self.kp * (1.0 + sign * step))
        elif which == 'ki':
            self.ki = max(0.0, self.ki * (1.0 + sign * step))
        elif which == 'kd':
            self.kd = max(0.0, self.kd * (1.0 + sign * step))
        self._send_gains()

    # ── Helpers de desenho ─────────────────────────────────────────────────────

    def _txt(self, text, color, x, y, font=None, cx=False):
        font = font or self.f_md
        for line in str(text).split('\n'):
            s = font.render(line, True, color)
            bx = x - s.get_width() // 2 if cx else x
            self.screen.blit(s, (bx, y))
            y += s.get_height() + 2
        return y

    def _hbar(self, x, y, w, h, frac, col):
        pygame.draw.rect(self.screen, C['grid'], (x, y, w, h), border_radius=3)
        fw = max(0, int(w * min(max(frac, 0.0), 1.0)))
        if fw:
            pygame.draw.rect(self.screen, col, (x, y, fw, h), border_radius=3)
        pygame.draw.rect(self.screen, C['border'], (x, y, w, h), 1, border_radius=3)

    # ── Painel esquerdo: posição da bola ──────────────────────────────────────

    def _draw_table(self, rect):
        pygame.draw.rect(self.screen, C['table'], rect)
        # Grade
        step_px = int(20 / TABLE_W_MM * rect.w)
        for x in range(rect.x + step_px, rect.x + rect.w, step_px):
            pygame.draw.line(self.screen, C['grid'], (x, rect.y), (x, rect.y + rect.h))
        step_py = int(20 / TABLE_H_MM * rect.h)
        for y in range(rect.y + step_py, rect.y + rect.h, step_py):
            pygame.draw.line(self.screen, C['grid'], (rect.x, y), (rect.x + rect.w, y))
        pygame.draw.rect(self.screen, C['border'], rect, 2)

        def mm2px(xmm, ymm):
            px = rect.x + int(xmm / TABLE_W_MM * rect.w)
            py = rect.y + int(ymm / TABLE_H_MM * rect.h)
            return px, py

        # Setpoint
        sp = mm2px(*self.setpt_mm)
        pygame.draw.circle(self.screen, C['target'], sp, 7, 2)
        pygame.draw.line(self.screen, C['target'], (sp[0]-10, sp[1]), (sp[0]+10, sp[1]), 1)
        pygame.draw.line(self.screen, C['target'], (sp[0], sp[1]-10), (sp[0], sp[1]+10), 1)

        # Bola
        if self.ball_mm:
            bp = mm2px(*self.ball_mm)
            # Linha do erro
            pygame.draw.line(self.screen, C['warn'], sp, bp, 1)
            # Clamp para garantir que a bola fique dentro do rect
            bpx = max(rect.x, min(bp[0], rect.right  - 1))
            bpy = max(rect.y, min(bp[1], rect.bottom - 1))
            pygame.draw.circle(self.screen, C['ball'], (bpx, bpy), 10)
            pygame.draw.circle(self.screen, C['ballhi'],
                               (bpx - 3, bpy - 3), 3)

    def _draw_error_graph(self, rect):
        pygame.draw.rect(self.screen, C['panel'], rect)
        pygame.draw.rect(self.screen, C['border'], rect, 1)
        self._txt("Erro (mm)", C['dim'], rect.x + 4, rect.y + 2, self.f_sm)

        if len(self.err_hist) < 2:
            return

        vals = [v for _, v in self.err_hist if not math.isnan(v)]
        if not vals:
            return
        y_max = max(max(vals) * 1.1, 10.0)

        now   = time.time()
        t_min = min(t for t, _ in self.err_hist)
        t_span = max(now - t_min, 1.0)

        pts = []
        for t, v in self.err_hist:
            if math.isnan(v):
                continue
            px = rect.x + int((t - t_min) / t_span * rect.w)
            py = rect.y + rect.h - int(v / y_max * (rect.h - 4)) - 2
            pts.append((px, py))

        if len(pts) >= 2:
            pygame.draw.lines(self.screen, C['err_line'], False, pts, 2)

        # Linha de referência y=0
        y0 = rect.y + rect.h - 2
        pygame.draw.line(self.screen, C['dim'],
                         (rect.x, y0), (rect.x + rect.w, y0), 1)

        # Valor atual
        _, last_v = self.err_hist[-1]
        if not math.isnan(last_v):
            self._txt(f"{last_v:.1f} mm", C['err_line'],
                      rect.x + rect.w - 72, rect.y + 2, self.f_sm)

    # ── Painel direito: ganhos ─────────────────────────────────────────────────

    def _draw_gains_panel(self, x, y):
        W = WIN_W - x - 10

        y = self._txt("PID TUNER", C['heading'], x, y, self.f_xl) + 6
        conn_col = C['good'] if self.conn_txt == 'conectado' else C['warn']
        y = self._txt(f"{self.port}  {self.conn_txt}", conn_col, x, y, self.f_sm) + 12

        step_pct = int(STEPS[self.step_idx] * 100)
        y = self._txt(f"Passo: {step_pct:3d}%  [Tab = mudar]",
                      C['hint'], x, y, self.f_sm) + 14

        # ── KP ──
        y = self._txt("KP", C['kp_bar'], x, y, self.f_lg)
        y = self._txt(f"  {self.kp:.4e}", C['text'], x, y, self.f_md)
        self._hbar(x, y, W, 14, self.kp / (self.kp * 10 + 1e-12), C['kp_bar'])
        y += 20
        y = self._txt("  [Q] sobe    [A] desce", C['dim'], x, y, self.f_sm) + 14

        # ── KI ──
        y = self._txt("KI", C['ki_bar'], x, y, self.f_lg)
        y = self._txt(f"  {self.ki:.4e}", C['text'], x, y, self.f_md)
        self._hbar(x, y, W, 14, self.ki / (self.ki * 10 + 1e-12), C['ki_bar'])
        y += 20
        y = self._txt("  [W] sobe    [S] desce", C['dim'], x, y, self.f_sm) + 14

        # ── KD ──
        y = self._txt("KD", C['kd_bar'], x, y, self.f_lg)
        y = self._txt(f"  {self.kd:.4e}", C['text'], x, y, self.f_md)
        self._hbar(x, y, W, 14, self.kd / (self.kd * 10 + 1e-12), C['kd_bar'])
        y += 20
        y = self._txt("  [E] sobe    [D] desce", C['dim'], x, y, self.f_sm) + 18

        # Inclinação atual
        y = self._txt(f"nx = {self.nx:+.4f}   ny = {self.ny:+.4f}",
                      C['dim'], x, y, self.f_sm) + 14

        # Bola
        if self.ball_mm:
            err = math.hypot(self.ball_mm[0] - self.setpt_mm[0],
                             self.ball_mm[1] - self.setpt_mm[1])
            y = self._txt(f"Bola:  X={self.ball_mm[0]:.1f}  Y={self.ball_mm[1]:.1f}",
                          C['good'], x, y, self.f_sm)
            y = self._txt(f"Erro:  {err:.1f} mm", C['warn'], x, y, self.f_sm) + 10
        else:
            y = self._txt("Bola: sem toque", C['dim'], x, y, self.f_sm) + 10

        # Salvar
        pygame.draw.rect(self.screen, (30, 80, 40),
                         (x, y, W, 30), border_radius=4)
        pygame.draw.rect(self.screen, C['save_ok'],
                         (x, y, W, 30), 1, border_radius=4)
        self._txt("[ESPACO] = SALVAR NO NVS", C['save_ok'],
                  x + W // 2, y + 8, self.f_sm, cx=True)
        y += 38

        # Toast
        if self.save_msg and (time.time() - self.save_ts) < 4.0:
            self._txt(self.save_msg, C['save_ok'], x, y, self.f_sm)

    # ── Render principal ───────────────────────────────────────────────────────

    def _draw(self):
        s = self.screen
        s.fill(C['bg'])

        PANEL_X = 760
        PAD = 12

        # Título
        self._txt("Ball Balancer — Ajuste PID em Tempo Real",
                  C['heading'], PAD, PAD, self.f_lg)

        # Mesa (posição da bola)
        table_rect = pygame.Rect(PAD, 42, PANEL_X - PAD * 2, 380)
        self._draw_table(table_rect)

        # Label mesa
        self._txt(f"Posicao da bola  (setpoint = centro)",
                  C['dim'], PAD, table_rect.y + table_rect.h + 4, self.f_sm)

        # Gráfico de erro
        graph_rect = pygame.Rect(PAD, table_rect.y + table_rect.h + 22,
                                 PANEL_X - PAD * 2, WIN_H - (table_rect.y + table_rect.h + 22) - PAD)
        self._draw_error_graph(graph_rect)

        # Divisor
        pygame.draw.line(s, C['border'], (PANEL_X, 0), (PANEL_X, WIN_H), 1)

        # Painel de ganhos
        self._draw_gains_panel(PANEL_X + PAD, 30)

        # ── HUD de diagnóstico (canto inferior esquerdo) ──────────────────────
        dbg_col = C['good'] if self.io.pos_count > 0 else C['bad']
        bm = f"({self.ball_mm[0]:.1f},{self.ball_mm[1]:.1f})" if self.ball_mm else "None"
        self._txt(
            f"DBG  linhas={self.io.line_count}  POS={self.io.pos_count}  ball_mm={bm}",
            dbg_col, PAD, WIN_H - 30, self.f_sm)
        self._txt(
            f"     ult: {self.io.last_line[:70]}",
            C['dim'], PAD, WIN_H - 16, self.f_sm)

    # ── Loop principal ─────────────────────────────────────────────────────────

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
                        if k in (pygame.K_ESCAPE, pygame.K_F4):
                            running = False
                        elif k == pygame.K_TAB:
                            self.step_idx = (self.step_idx + 1) % len(STEPS)
                        elif k == pygame.K_SPACE:
                            self.io.write("PID SAVE")
                            self.save_msg = "Enviando PID SAVE..."
                            self.save_ts  = time.time()
                        elif k == pygame.K_r:
                            self._send_gains()
                        # Ajuste de KP
                        elif k == pygame.K_q:
                            self._adjust('kp', +1)
                        elif k == pygame.K_a:
                            self._adjust('kp', -1)
                        # Ajuste de KI
                        elif k == pygame.K_w:
                            self._adjust('ki', +1)
                        elif k == pygame.K_s:
                            self._adjust('ki', -1)
                        # Ajuste de KD
                        elif k == pygame.K_e:
                            self._adjust('kd', +1)
                        elif k == pygame.K_d:
                            self._adjust('kd', -1)

                self._draw()
                pygame.display.flip()
                self.clock.tick(FPS)
        except KeyboardInterrupt:
            pass
        finally:
            self.io.stop()
            pygame.quit()


def main():
    port = sys.argv[1] if len(sys.argv) >= 2 else DEFAULT_PORT
    baud = int(sys.argv[2]) if len(sys.argv) >= 3 else DEFAULT_BAUD
    App(port, baud).run()


if __name__ == '__main__':
    main()
