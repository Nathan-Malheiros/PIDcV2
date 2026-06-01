#!/usr/bin/env python3
"""
Ball Balancer Studio — Central de Controle Unificada
=====================================================
Um unico aplicativo com tudo: build, flash, monitor serial, visualizacao da
bola, calibracao, ajuste de PID e simulacao 3D dos motores 3RPS.

Inicie pelo  bb_studio.bat  (carrega o ambiente ESP-IDF antes de rodar).
Ou direto:   python tools/bb_studio.py
A porta serial vem de tools/config.py (PORT) — edite la, sem argumentos.

Paginas (clique na barra lateral ou use as teclas 1-7):
  1 Dashboard      — visao geral, status, acoes rapidas
  2 Build & Flash  — compila e grava o firmware, console ao vivo
  3 Monitor        — terminal serial bruto + envio de comandos
  4 Live Ball      — posicao da bola no plano em mm (tempo real)
  5 Calibrar       — assistente de 4 cantos da tela resistiva
  6 PID Tuner      — ajusta KP/KI/KD ao vivo + grafico de erro
  7 Simulacao 3RPS — visao isometrica dos 3 motores reagindo a bola
"""

import sys, os, re, time, math, queue, threading, subprocess, collections, statistics, pathlib


def _ensure_deps():
    """Instala pygame/pyserial automaticamente se faltarem — assim basta
    rodar 'python bb_studio.py', sem precisar instalar nada antes."""
    need = []
    try:
        import serial  # noqa: F401
        from serial.tools import list_ports  # noqa: F401
    except ImportError:
        need.append('pyserial')
    try:
        import pygame  # noqa: F401
    except ImportError:
        need.append('pygame')
    if not need:
        return
    print("=" * 56)
    print(" Ball Balancer Studio — primeira execucao")
    print(" Instalando dependencias:", ", ".join(need))
    print(" (isso so acontece uma vez, aguarde...)")
    print("=" * 56)
    for pkg in need:
        ok = False
        for extra in ([], ['--user']):
            try:
                subprocess.check_call([sys.executable, '-m', 'pip', 'install', *extra, pkg])
                ok = True
                break
            except Exception:
                continue
        if not ok:
            print(f"\n[ERRO] Nao consegui instalar '{pkg}' automaticamente.")
            print(f"       Rode manualmente:  {sys.executable} -m pip install {pkg}")
            sys.exit(1)


_ensure_deps()

import serial
from serial.tools import list_ports
import pygame

# ── config.py (encontrado independente do diretorio de execucao) ─────────────
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    import config as cfg
except Exception:
    cfg = None

def _cfg(name, default):
    return getattr(cfg, name, default) if cfg else default

# ── Caminhos ─────────────────────────────────────────────────────────────────
TOOL_DIR    = pathlib.Path(__file__).resolve().parent
FW_ROOT     = TOOL_DIR.parent
BUILD_DIR   = FW_ROOT / "build"
HEADER_PATH = FW_ROOT / "main" / "touch_screen.h"
CONFIG_PATH = TOOL_DIR / "config.py"
BIN_PATH    = BUILD_DIR / "ball_balancer.bin"

# ── Parametros ─────────────────────────────────────────────────────────────
PORT        = _cfg('PORT', 'COM8')
BAUD        = _cfg('BAUD', 115200)
TABLE_W_MM  = float(_cfg('SCREEN_W_MM', 187.0))
TABLE_H_MM  = float(_cfg('SCREEN_H_MM', 141.0))


def autodetect_port(default):
    """Acha a porta do ESP32 sozinho — prioriza adaptadores USB-serial conhecidos
    (CP210x, CH340, FTDI...). Sem isso, usa a primeira COM disponivel."""
    try:
        ports = list(list_ports.comports())
    except Exception:
        return default
    pref = ('CP210', 'CH340', 'CH910', 'FTDI', 'SILICON LABS', 'USB-SERIAL',
            'USB SERIAL', 'WCH', 'ESP32', 'JTAG')
    for p in ports:
        hay = ' '.join(str(x) for x in (p.description, p.hwid,
                       getattr(p, 'manufacturer', ''))).upper()
        if any(k in hay for k in pref):
            return p.device
    for p in ports:
        if p.device:
            return p.device
    return default

WIN_W, WIN_H = 1300, 800
SB_W         = 218          # largura da barra lateral
HEADER_H     = 70
FPS          = 60
TRAIL_LEN    = 160
SAMPLES_NEEDED = 80
STEPS        = [0.01, 0.05, 0.10, 0.25]   # passos de ajuste do PID

# ── Geometria / cinematica 3RPS (espelha main/control.c) ─────────────────────
GEO_D, GEO_E, GEO_F, GEO_G = 50.8, 79.4, 44.45, 93.2
GEO_HZ      = 108.0
SQRT3       = math.sqrt(3.0)
TILT_LIMIT  = 0.25
KP, KI, KD  = 8.0e-4, 2.0e-5, 1.2e-2
BALL_TAU, TILT_TAU, PRESENCE_HOLD = 0.06, 0.12, 0.40
LEG_AZ   = [math.radians(90), math.radians(210), math.radians(330)]
LEG_NAME = ["A", "B", "C"]
PLATE_BASE_Z = 70.0


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


ANG_ORIG = machine_theta(0, GEO_HZ, 0.0, 0.0)


def plate_z(X, Y, nx, ny, base_z):
    nmag = math.sqrt(nx * nx + ny * ny + 1.0)
    nz = 1.0 / nmag
    return base_z - (nx * X + ny * Y) / nz


class PID:
    """PID por eixo — espelha main/pid.c."""
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


# ── Tema visual ──────────────────────────────────────────────────────────────
C = {
    'bg':        (15, 16, 22),
    'sidebar':   (21, 23, 31),
    'panel':     (27, 29, 38),
    'panel_hi':  (34, 37, 48),
    'border':    (44, 48, 62),
    'text':      (216, 220, 228),
    'dim':       (120, 126, 142),
    'faint':     (74, 80, 96),
    'accent':    (88, 150, 255),
    'accent_dk': (40, 70, 130),
    'good':      (66, 214, 130),
    'warn':      (240, 190, 70),
    'bad':       (240, 86, 86),
    'mag':       (190, 130, 240),
    'cyan':      (90, 210, 230),
    'table_bg':  (238, 238, 226),
    'grid':      (198, 198, 182),
    'ball':      (224, 56, 56),
    'ball_hi':   (255, 120, 120),
    'target':    (66, 200, 120),
    'trail_h':   (96, 165, 255),
    'trail_t':   (24, 44, 90),
    'iso_plate': (212, 207, 182),
    'iso_edge':  (60, 62, 74),
    'iso_grid':  (122, 120, 102),
    'iso_leg':   (92, 162, 222),
    'iso_arm':   (232, 172, 62),
    'iso_base':  (74, 76, 92),
    'iso_ball':  (224, 64, 64),
    'iso_ballhi':(255, 150, 150),
    'kp':        (90, 162, 255),
    'ki':        (240, 178, 64),
    'kd':        (90, 220, 142),
}

NAV = [
    ('dash',    'Dashboard',      '◈'),
    ('build',   'Build & Flash',  '⚙'),
    ('monitor', 'Serial Monitor', '▤'),
    ('live',    'Live Ball',      '◉'),
    ('cal',     'Calibrar',       '⊹'),
    ('pid',     'PID Tuner',      'P'),
    ('sim',     'Simulacao 3RPS', '⛭'),
]


# ── Gerenciador serial (uma unica conexao compartilhada) ─────────────────────

class SerialManager(threading.Thread):
    def __init__(self, port, baud):
        super().__init__(daemon=True)
        self.port, self.baud = port, baud
        self._tx = queue.Queue()
        self._stop = threading.Event()
        self._paused = threading.Event()
        self._lock = threading.Lock()
        self.port_open = False        # True somente enquanto a serial esta aberta

        self.connected = False
        self.status = 'conectando...'
        self.lines = collections.deque(maxlen=600)   # monitor bruto
        self.last_line = ''
        self.line_count = 0
        self.pos_count = 0

        # Estado mais recente
        self.x_raw = 0
        self.y_raw = 0
        self.touched = False
        self.touch_ts = 0.0
        self.kp = KP; self.ki = KI; self.kd = KD
        self.gains_ok = False
        self.sign_x = 1.0; self.sign_y = 1.0
        self.fw_cal = None            # (xmin,xmax,ymin,ymax,fx,fy,sw) do firmware
        self.saved_msg = ''
        self.saved_ts = 0.0

    # API
    def write(self, line):
        self._tx.put(line.rstrip('\n') + '\n')

    def pause(self):
        self._paused.set()

    def resume(self):
        self._paused.clear()

    def wait_closed(self, timeout=4.0):
        """Bloqueia ate a porta serial estar realmente fechada (ou timeout).
        Usado antes do flash para garantir que o esptool consiga abrir a COM."""
        t0 = time.time()
        while self.port_open and (time.time() - t0) < timeout:
            time.sleep(0.05)
        return not self.port_open

    def set_port(self, port):
        """Troca a porta e forca reconexao imediata."""
        self.port = port
        self.pause()
        self.wait_closed(2.0)
        time.sleep(0.2)
        self.resume()

    def stop(self):
        self._stop.set()

    def present(self):
        return self.touched or (time.time() - self.touch_ts) < PRESENCE_HOLD

    def _push_line(self, line):
        with self._lock:
            self.lines.append(line)
            self.last_line = line
            self.line_count += 1

    def run(self):
        while not self._stop.is_set():
            if self._paused.is_set():
                self.connected = False
                self.status = 'pausado (flash em andamento)'
                time.sleep(0.2)
                continue
            try:
                with serial.Serial(self.port, self.baud, timeout=0.3) as ser:
                    self.port_open = True
                    self.connected = True
                    self.status = 'conectado'
                    # Coloca DTR/RTS em estado "run" e da um pulso de reset limpo,
                    # senao o circuito de auto-reset do ESP32 segura o chip em
                    # reset/bootloader e nenhuma telemetria chega ao app.
                    try:
                        ser.dtr = False      # IO0 = alto (nao entra em bootloader)
                        ser.rts = True       # EN = baixo (reset)
                        time.sleep(0.05)
                        ser.rts = False      # EN = alto -> boota o app
                        time.sleep(0.05)
                    except Exception:
                        pass
                    ser.reset_input_buffer()
                    ser.write(b'?\n')
                    while not self._stop.is_set() and not self._paused.is_set():
                        while not self._tx.empty():
                            try:
                                ser.write(self._tx.get_nowait().encode())
                            except queue.Empty:
                                break
                        raw = ser.readline()
                        if not raw:
                            continue
                        line = raw.decode('utf-8', errors='replace').strip()
                        if not line:
                            continue
                        self._push_line(line)
                        self._parse(line)
            except serial.SerialException as e:
                self.connected = False
                self.status = f'sem conexao: {e}'
                time.sleep(1.5)
            finally:
                self.port_open = False
                self.connected = False

    def _parse(self, line):
        parts = line.split(',')
        tag = parts[0]
        try:
            if tag == 'POS' and len(parts) >= 5:
                self.x_raw = int(float(parts[1]))
                self.y_raw = int(float(parts[2]))
                self.touched = True
                self.touch_ts = time.time()
                self.pos_count += 1
            elif tag == 'NOTOUCH':
                self.touched = False
                if len(parts) >= 3:
                    self.x_raw = int(float(parts[1]))
                    self.y_raw = int(float(parts[2]))
            elif tag == 'GAINS' and len(parts) >= 4:
                self.kp = float(parts[1]); self.ki = float(parts[2]); self.kd = float(parts[3])
                self.gains_ok = True
            elif tag == 'SIGNS' and len(parts) >= 3:
                self.sign_x = float(parts[1]); self.sign_y = float(parts[2])
            elif tag == 'CAL' and len(parts) >= 8:
                self.fw_cal = tuple(int(p) for p in parts[1:8])
            elif tag == 'SAVED':
                self.saved_msg = 'Salvo no NVS: ' + ','.join(parts[1:])
                self.saved_ts = time.time()
        except ValueError:
            pass


# ── Executor do idf.py (build/flash) com saida em tempo real ─────────────────

class IdfRunner:
    def __init__(self, fw_root):
        self.fw_root = str(fw_root)
        self.lines = collections.deque(maxlen=4000)
        self.running = False
        self.action = ''
        self.exit_code = None
        self._proc = None
        self._lock = threading.Lock()
        self.profile = self._locate_profile()

    @staticmethod
    def _locate_profile():
        """A GUI roda no python do sistema (tem pygame); o ESP-IDF vive em outro
        venv. Em vez de misturar os dois, cada build/flash abre um PowerShell que
        carrega o perfil ESP-IDF e chama a funcao 'idf.py' — exatamente como o
        bb_firmware_tool. Aqui localizamos esse perfil .ps1."""
        import glob
        cands = []
        env_p = os.environ.get('IDF_PS_PROFILE')
        if env_p:
            cands.append(env_p)
        for base in (r'C:\Espressif\tools', r'C:\Espressif',
                     os.path.expanduser(r'~\.espressif\tools'),
                     os.path.expanduser(r'~\.espressif')):
            cands += sorted(glob.glob(os.path.join(base, 'Microsoft*.PowerShell_profile.ps1')),
                            reverse=True)
        for c in cands:
            if c and os.path.exists(c):
                return c
        return None

    def idf_available(self):
        return self.profile is not None

    def append(self, text):
        with self._lock:
            self.lines.append(text)

    def clear(self):
        with self._lock:
            self.lines.clear()

    def run(self, args, label, on_done=None):
        if self.running:
            return False
        self.action = label
        self.running = True
        self.exit_code = None
        threading.Thread(target=self._worker, args=(args, on_done), daemon=True).start()
        return True

    def cancel(self):
        if self._proc and self.running:
            try:
                self._proc.terminate()
            except Exception:
                pass

    def _worker(self, args, on_done):
        self.append(f"$ idf.py {args}")
        self.append(f"  (cwd: {self.fw_root})")
        if not self.profile:
            self.append("[ERRO] Perfil ESP-IDF nao encontrado.")
            self.append("       Procurei em C:\\Espressif\\tools\\Microsoft*.PowerShell_profile.ps1")
            self.append("       Defina a variavel IDF_PS_PROFILE com o caminho do .ps1,")
            self.append("       ou ajuste o caminho em _locate_profile().")
            self.exit_code = -1
            self.append(""); self.append("== FALHOU (exit -1) ==")
            self.running = False; self._proc = None
            if on_done: on_done(-1)
            return
        # Abre um PowerShell, carrega o perfil ESP-IDF (silencioso) e chama idf.py.
        ps = f". '{self.profile}' *> $null ; idf.py {args}"
        cmd = ["powershell", "-NoProfile", "-ExecutionPolicy", "Bypass", "-Command", ps]
        try:
            self._proc = subprocess.Popen(
                cmd, cwd=self.fw_root,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, bufsize=1, errors='replace',
            )
            for line in self._proc.stdout:
                self.append(line.rstrip('\n'))
            self._proc.wait()
            self.exit_code = self._proc.returncode
        except FileNotFoundError as e:
            self.append(f"[ERRO] nao foi possivel executar: {e}")
            self.exit_code = -1
        except Exception as e:
            self.append(f"[ERRO] {e}")
            self.exit_code = -1
        finally:
            ec = self.exit_code
            self.append("")
            self.append(f"== {'OK' if ec == 0 else 'FALHOU'} (exit {ec}) ==")
            self.running = False
            self._proc = None
            if on_done:
                on_done(ec)


# ── Aplicacao principal ──────────────────────────────────────────────────────

class Studio:
    def __init__(self):
        pygame.init()
        pygame.key.set_repeat(350, 40)
        self.screen = pygame.display.set_mode((WIN_W, WIN_H))
        pygame.display.set_caption("Ball Balancer Studio")
        self.clock = pygame.time.Clock()

        # Fontes
        self.f_h1 = pygame.font.SysFont('segoeui', 30, bold=True)
        self.f_h2 = pygame.font.SysFont('segoeui', 20, bold=True)
        self.f_ui = pygame.font.SysFont('segoeui', 16)
        self.f_uib = pygame.font.SysFont('segoeui', 16, bold=True)
        self.f_sm = pygame.font.SysFont('segoeui', 13)
        self.f_mono = pygame.font.SysFont('consolas', 14)
        self.f_mono_sm = pygame.font.SysFont('consolas', 12)
        self.f_icon = pygame.font.SysFont('segoeui', 18)

        # Serial + IDF
        self.ser = SerialManager(PORT, BAUD)
        self.ser.start()
        self.idf = IdfRunner(FW_ROOT)

        # Navegacao / interacao
        self.page = 'dash'
        self.click = None        # posicao do clique nao consumido
        self.scroll = 0          # rolagem do console/monitor
        self.mouse = (0, 0)

        # Calibracao (carregada de config.py — Live Ball funciona de imediato)
        self.cal = (int(_cfg('X_RAW_MIN', 499)), int(_cfg('X_RAW_MAX', 3081)),
                    int(_cfg('Y_RAW_MIN', 431)), int(_cfg('Y_RAW_MAX', 3797)))
        self.swap_xy = bool(_cfg('SWAP_XY', False))
        self.flip_x  = bool(_cfg('FLIP_X', True))
        self.flip_y  = bool(_cfg('FLIP_Y', True))

        # Live ball
        self.trail = collections.deque(maxlen=TRAIL_LEN)

        # Assistente de calibracao
        self.cal_state = 'idle'    # idle|collecting|done
        self.cal_idx = 0
        self.cal_corners = [None] * 4    # (xmed, ymed)
        self.cal_samples = []

        # PID tuner
        self.kp = self.ser.kp; self.ki = self.ser.ki; self.kd = self.ser.kd
        self.step_idx = 1
        self.setpt_mm = (TABLE_W_MM / 2, TABLE_H_MM / 2)
        self.err_hist = collections.deque(maxlen=320)

        # Simulacao 3RPS
        self.pid_x = PID(KP, KI, KD, -TILT_LIMIT, TILT_LIMIT)
        self.pid_y = PID(KP, KI, KD, -TILT_LIMIT, TILT_LIMIT)
        self.sim_sx = TABLE_W_MM / 2.0
        self.sim_sy = TABLE_H_MM / 2.0
        self.ball_mm = None
        self.bal_seen_ts = 0.0
        self.disp_nx = 0.0; self.disp_ny = 0.0
        self.disp_theta = [ANG_ORIG] * 3
        self.cam_az = math.radians(35)
        self.cam_pitch = math.radians(30)
        self.cam_scale = 2.2
        self.bal_t = None

        # Monitor
        self.cmd_text = ''
        self.cmd_active = False

        # Toast
        self.toast = ''
        self.toast_ts = 0.0

    # ── helpers de desenho ──────────────────────────────────────────────────
    def panel(self, rect, color=None, radius=10, border=True):
        pygame.draw.rect(self.screen, color or C['panel'], rect, border_radius=radius)
        if border:
            pygame.draw.rect(self.screen, C['border'], rect, 1, border_radius=radius)

    def text(self, s, color, x, y, font=None, cx=False, right=False):
        font = font or self.f_ui
        for ln in str(s).split('\n'):
            surf = font.render(ln, True, color)
            bx = x
            if cx: bx = x - surf.get_width() // 2
            elif right: bx = x - surf.get_width()
            self.screen.blit(surf, (bx, y))
            y += surf.get_height() + 2
        return y

    def button(self, rect, label, color=C['accent'], text_col=(255, 255, 255),
               font=None, enabled=True):
        font = font or self.f_uib
        r = pygame.Rect(rect)
        hover = r.collidepoint(self.mouse)
        if not enabled:
            bg = C['panel_hi']; tc = C['faint']
        elif hover:
            bg = color; tc = text_col
        else:
            bg = tuple(int(c * 0.78) for c in color); tc = text_col
        pygame.draw.rect(self.screen, bg, r, border_radius=8)
        pygame.draw.rect(self.screen, C['border'], r, 1, border_radius=8)
        surf = font.render(label, True, tc)
        self.screen.blit(surf, (r.centerx - surf.get_width() // 2,
                                r.centery - surf.get_height() // 2))
        clicked = enabled and self.click and r.collidepoint(self.click)
        if clicked:
            self.click = None
        return bool(clicked)

    def chip(self, x, y, label, color):
        w = self.f_sm.size(label)[0] + 26
        r = pygame.Rect(x, y, w, 22)
        pygame.draw.rect(self.screen, C['panel_hi'], r, border_radius=11)
        pygame.draw.circle(self.screen, color, (x + 13, y + 11), 4)
        self.text(label, C['text'], x + 22, y + 4, self.f_sm)
        return r.right

    def set_toast(self, msg):
        self.toast = msg
        self.toast_ts = time.time()

    # ── conversoes ──────────────────────────────────────────────────────────
    def raw2mm(self, xr, yr):
        x_min, x_max, y_min, y_max = self.cal
        nx = max(0.0, min(1.0, (xr - x_min) / ((x_max - x_min) or 1)))
        ny = max(0.0, min(1.0, (yr - y_min) / ((y_max - y_min) or 1)))
        if self.swap_xy: nx, ny = ny, nx
        if self.flip_x:  nx = 1.0 - nx
        if self.flip_y:  ny = 1.0 - ny
        return nx * TABLE_W_MM, ny * TABLE_H_MM

    def orient_str(self):
        p = []
        if self.swap_xy: p.append("SWAP")
        if self.flip_x:  p.append("FLIP_X")
        if self.flip_y:  p.append("FLIP_Y")
        return " + ".join(p) if p else "padrao"

    # ── persistencia de orientacao (config.py + header) ──────────────────────
    def save_orientation(self):
        ok1 = self._patch(CONFIG_PATH,
                          [('SWAP_XY', self.swap_xy), ('FLIP_X', self.flip_x), ('FLIP_Y', self.flip_y)],
                          r'(?m)^(%s\s*=\s*)(?:True|False)', lambda v: str(bool(v)))
        ok2 = self._patch(HEADER_PATH,
                          [('TOUCH_SWAP_XY', self.swap_xy), ('TOUCH_FLIP_X', self.flip_x), ('TOUCH_FLIP_Y', self.flip_y)],
                          r'(#define\s+%s\s+)[01]', lambda v: '1' if v else '0')
        self.set_toast("Orientacao salva (rebuild para aplicar no firmware)" if (ok1 and ok2)
                       else "Falha ao salvar orientacao")

    @staticmethod
    def _patch(path, pairs, tmpl, fmt):
        try:
            if not path.exists():
                return False
            txt = path.read_text(encoding='utf-8')
            for name, val in pairs:
                txt = re.sub(tmpl % name, lambda m, v=val: m.group(1) + fmt(v), txt)
            path.write_text(txt, encoding='utf-8')
            return True
        except Exception:
            return False

    def save_cal_header(self):
        x_min, x_max, y_min, y_max = self.cal
        ok = self._patch(HEADER_PATH,
                         [('TOUCH_X_RAW_MIN', x_min), ('TOUCH_X_RAW_MAX', x_max),
                          ('TOUCH_Y_RAW_MIN', y_min), ('TOUCH_Y_RAW_MAX', y_max)],
                         r'(#define\s+%s\s+)\d+', lambda v: str(v))
        self.set_toast("Calibracao salva em touch_screen.h" if ok else "Falha ao salvar header")

    def push_cal_nvs(self):
        x_min, x_max, y_min, y_max = self.cal
        fx = 1 if self.flip_x else 0
        fy = 1 if self.flip_y else 0
        sw = 1 if self.swap_xy else 0
        self.ser.write(f"SETCAL {x_min} {x_max} {y_min} {y_max} {fx} {fy} {sw}")
        time.sleep(0.15)
        self.ser.write("CAL SAVE")
        self.set_toast("Calibracao enviada ao ESP32 (NVS)")

    # ── acoes de build/flash ─────────────────────────────────────────────────
    def do_build(self):
        self.idf.clear()
        self.idf.run("build", "Build")

    def do_fullclean(self):
        self.idf.clear()
        self.idf.run("fullclean", "Full Clean",
                     on_done=lambda ec: self.idf.run("set-target esp32s3", "Set Target",
                              on_done=lambda e2: self.idf.run("build", "Build")) if ec == 0 else None)

    def _flash_after_releasing_port(self):
        """Libera a COM do SerialManager, espera ela fechar de fato e so entao
        roda o flash (esptool precisa de acesso exclusivo a porta)."""
        self.idf.append("-- liberando porta serial para o esptool...")
        self.ser.pause()
        closed = self.ser.wait_closed(4.0)
        time.sleep(0.4)   # folga extra para o driver CP210x soltar a porta
        if not closed:
            self.idf.append("[AVISO] a porta pode ainda estar ocupada.")
        self.idf.run(f"-p {PORT} flash", "Flash",
                     on_done=lambda ec: self.ser.resume())

    def do_flash(self):
        self.idf.clear()
        # Em thread separada para nao travar a UI durante o wait_closed.
        threading.Thread(target=self._flash_after_releasing_port, daemon=True).start()

    def cycle_port(self):
        """Passa para a proxima porta COM disponivel e reconecta."""
        global PORT
        try:
            devs = [p.device for p in list_ports.comports()]
        except Exception:
            devs = []
        if not devs:
            self.set_toast("Nenhuma porta COM encontrada")
            return
        PORT = devs[(devs.index(PORT) + 1) % len(devs)] if PORT in devs else devs[0]
        threading.Thread(target=self.ser.set_port, args=(PORT,), daemon=True).start()
        self.set_toast(f"Conectando em {PORT}")

    def do_build_flash(self):
        self.idf.clear()
        def after_build(ec):
            if ec == 0:
                self._flash_after_releasing_port()
        self.idf.run("build", "Build", on_done=after_build)

    # ════════════════════════════════════════════════════════════════════════
    #  PAGINAS
    # ════════════════════════════════════════════════════════════════════════

    def content_rect(self):
        return pygame.Rect(SB_W + 20, HEADER_H + 16, WIN_W - SB_W - 40, WIN_H - HEADER_H - 36)

    # ── Dashboard ─────────────────────────────────────────────────────────────
    def page_dash(self, area):
        s = self.screen
        col_w = (area.w - 20) // 2

        # Card: Conexao
        c1 = pygame.Rect(area.x, area.y, col_w, 168)
        self.panel(c1)
        self.text("CONEXAO", C['dim'], c1.x + 18, c1.y + 14, self.f_sm)
        col = C['good'] if self.ser.connected else C['bad']
        self.text(self.ser.status, col, c1.x + 18, c1.y + 34, self.f_h2)
        self.text(f"Porta {PORT}  @  {BAUD} baud", C['text'], c1.x + 18, c1.y + 70, self.f_ui)
        if self.button((c1.right - 132, c1.y + 64, 116, 28), "Trocar porta", C['accent_dk'], C['text'], self.f_sm):
            self.cycle_port()
        self.text(f"Linhas recebidas: {self.ser.line_count}    POS: {self.ser.pos_count}",
                  C['dim'], c1.x + 18, c1.y + 96, self.f_sm)
        tcol = C['good'] if self.ser.present() else C['faint']
        self.text(f"Toque: {'BOLA NA MESA' if self.ser.present() else 'sem contato'}",
                  tcol, c1.x + 18, c1.y + 118, self.f_ui)
        self.text(f"raw = ({self.ser.x_raw}, {self.ser.y_raw})", C['dim'], c1.x + 18, c1.y + 140, self.f_mono_sm)

        # Card: Firmware
        c2 = pygame.Rect(area.x + col_w + 20, area.y, col_w, 168)
        self.panel(c2)
        self.text("FIRMWARE", C['dim'], c2.x + 18, c2.y + 14, self.f_sm)
        if BIN_PATH.exists():
            kb = BIN_PATH.stat().st_size // 1024
            mt = time.strftime('%d/%m %H:%M', time.localtime(BIN_PATH.stat().st_mtime))
            self.text("compilado", C['good'], c2.x + 18, c2.y + 34, self.f_h2)
            self.text(f"{kb} KB  —  {mt}", C['text'], c2.x + 18, c2.y + 70, self.f_ui)
        else:
            self.text("sem binario", C['warn'], c2.x + 18, c2.y + 34, self.f_h2)
            self.text("rode Build na pagina Build & Flash", C['dim'], c2.x + 18, c2.y + 70, self.f_sm)
        self.text(f"PID  kp={self.ser.kp:.2e}  ki={self.ser.ki:.2e}  kd={self.ser.kd:.2e}",
                  C['dim'], c2.x + 18, c2.y + 100, self.f_mono_sm)
        self.text(f"Calibracao  X[{self.cal[0]}-{self.cal[1]}]  Y[{self.cal[2]}-{self.cal[3]}]",
                  C['dim'], c2.x + 18, c2.y + 122, self.f_mono_sm)

        # Card: Acoes rapidas
        c3 = pygame.Rect(area.x, area.y + 188, area.w, 120)
        self.panel(c3)
        self.text("ACOES RAPIDAS", C['dim'], c3.x + 18, c3.y + 14, self.f_sm)
        bw, bh, gap = 200, 46, 16
        bx, by = c3.x + 18, c3.y + 44
        if self.button((bx, by, bw, bh), "Build + Flash", C['accent'], enabled=not self.idf.running):
            self.do_build_flash()
        if self.button((bx + (bw + gap), by, bw, bh), "Abrir Monitor", C['accent_dk']):
            self.page = 'monitor'
        if self.button((bx + 2 * (bw + gap), by, bw, bh), "Live Ball", C['accent_dk']):
            self.page = 'live'
        if self.button((bx + 3 * (bw + gap), by, bw, bh), "PID Tuner", C['accent_dk']):
            self.page = 'pid'

        # Card: mini mesa ao vivo
        c4 = pygame.Rect(area.x, area.y + 328, area.w, area.bottom - (area.y + 328))
        self.panel(c4)
        self.text("POSICAO AO VIVO", C['dim'], c4.x + 18, c4.y + 14, self.f_sm)
        inner = pygame.Rect(c4.x + 18, c4.y + 40, c4.w - 36, c4.h - 58)
        self._draw_table_mm(inner, show_ticks=False)

    # ── Build & Flash ───────────────────────────────────────────────────────
    def page_build(self, area):
        # Barra de botoes
        bar = pygame.Rect(area.x, area.y, area.w, 56)
        self.panel(bar)
        bw, bh = 150, 38
        bx = bar.x + 14; by = bar.y + 9
        run = self.idf.running
        if self.button((bx, by, bw, bh), "Build", C['accent'], enabled=not run): self.do_build()
        if self.button((bx + 1*(bw+10), by, bw, bh), "Flash", C['accent'], enabled=not run): self.do_flash()
        if self.button((bx + 2*(bw+10), by, bw, bh), "Build + Flash", C['accent'], enabled=not run): self.do_build_flash()
        if self.button((bx + 3*(bw+10), by, bw, bh), "Full Clean", C['warn'], (20,20,20), enabled=not run): self.do_fullclean()
        if run:
            if self.button((bar.right - 130, by, 116, bh), "Cancelar", C['bad']): self.idf.cancel()
            self.text(f"{self.idf.action}...", C['warn'], bar.right - 280, by + 10, self.f_uib, right=True)
        elif not self.idf.idf_available():
            self.text("ESP-IDF nao carregado — abra pelo bb_studio.bat",
                      C['bad'], bar.right - 14, by + 10, self.f_sm, right=True)
        else:
            self.text("ESP-IDF pronto", C['good'], bar.right - 14, by + 10, self.f_sm, right=True)

        # Console
        con = pygame.Rect(area.x, area.y + 68, area.w, area.bottom - (area.y + 68))
        self.panel(con, C['bg'])
        self._draw_console(con, self.idf.lines)

    def _draw_console(self, rect, lines):
        pad = 12
        lh = self.f_mono_sm.get_height() + 2
        visible = (rect.h - 2 * pad) // lh
        snapshot = list(lines)
        total = len(snapshot)
        max_off = max(0, total - visible)
        off = max(0, min(self.scroll, max_off))
        if off == 0:
            self.scroll = 0
        start = max(0, total - visible - off)
        shown = snapshot[start:start + visible]
        y = rect.y + pad
        clip = self.screen.get_clip()
        self.screen.set_clip(rect)
        for ln in shown:
            color = C['text']
            low = ln.lower()
            if ln.startswith('$'): color = C['accent']
            elif ln.startswith('=='): color = C['good'] if 'OK' in ln else C['bad']
            elif 'error' in low or '[erro]' in low or 'falhou' in low or 'failed' in low: color = C['bad']
            elif 'warning' in low or 'warn' in low: color = C['warn']
            surf = self.f_mono_sm.render(ln[:200], True, color)
            self.screen.blit(surf, (rect.x + pad, y))
            y += lh
        self.screen.set_clip(clip)
        if off > 0:
            self.text(f"▲ {off} linhas abaixo  (fim do log)", C['faint'],
                      rect.right - 14, rect.bottom - 20, self.f_sm, right=True)

    # ── Serial Monitor ─────────────────────────────────────────────────────
    def page_monitor(self, area):
        con = pygame.Rect(area.x, area.y, area.w, area.h - 52)
        self.panel(con, C['bg'])
        with self.ser._lock:
            snap = list(self.ser.lines)
        self._draw_console_lines(con, snap)

        # Linha de comando
        inp = pygame.Rect(area.x, area.bottom - 42, area.w - 110, 36)
        self.panel(inp, C['panel_hi'] if self.cmd_active else C['panel'])
        prompt = "› " + self.cmd_text + ("_" if (self.cmd_active and int(time.time()*2) % 2 == 0) else "")
        self.text(prompt if self.cmd_text or self.cmd_active else "› clique e digite um comando (ex: PID 8e-4 2e-5 1.2e-2)",
                  C['text'] if self.cmd_text else C['faint'], inp.x + 12, inp.y + 8, self.f_mono)
        if self.button((area.right - 98, area.bottom - 42, 98, 36), "Enviar", C['accent']):
            self._send_cmd()

    def _draw_console_lines(self, rect, lines):
        pad = 12
        lh = self.f_mono_sm.get_height() + 1
        visible = (rect.h - 2 * pad) // lh
        total = len(lines)
        max_off = max(0, total - visible)
        off = max(0, min(self.scroll, max_off))
        start = max(0, total - visible - off)
        shown = lines[start:start + visible]
        y = rect.y + pad
        clip = self.screen.get_clip()
        self.screen.set_clip(rect)
        for ln in shown:
            color = C['dim']
            if ln.startswith('POS'): color = C['good']
            elif ln.startswith('NOTOUCH'): color = C['faint']
            elif ln.startswith('CTRL'): color = C['cyan']
            elif ln.startswith(('GAINS', 'SIGNS', 'CAL', 'CORNER')): color = C['mag']
            elif ln.startswith('SAVED'): color = C['warn']
            elif ln.startswith('#'): color = C['accent']
            surf = self.f_mono_sm.render(ln[:200], True, color)
            self.screen.blit(surf, (rect.x + pad, y))
            y += lh
        self.screen.set_clip(clip)

    def _send_cmd(self):
        if self.cmd_text.strip():
            self.ser.write(self.cmd_text.strip())
            self.set_toast(f"Enviado: {self.cmd_text.strip()}")
            self.cmd_text = ''

    # ── tabela em mm (compartilhada: dashboard, live) ─────────────────────────
    def _draw_table_mm(self, area, show_ticks=True):
        s = self.screen
        scale = min(area.w / TABLE_W_MM, area.h / TABLE_H_MM)
        rw = int(TABLE_W_MM * scale); rh = int(TABLE_H_MM * scale)
        ox = area.x + (area.w - rw) // 2
        oy = area.y + (area.h - rh) // 2
        rect = pygame.Rect(ox, oy, rw, rh)
        pygame.draw.rect(s, C['table_bg'], rect)

        def mm2p(xmm, ymm):
            return ox + int(xmm / TABLE_W_MM * rw), oy + rh - int(ymm / TABLE_H_MM * rh)

        v = 20
        while v < TABLE_W_MM:
            px = ox + int(v / TABLE_W_MM * rw)
            pygame.draw.line(s, C['grid'], (px, oy), (px, oy + rh)); v += 20
        v = 20
        while v < TABLE_H_MM:
            py = oy + rh - int(v / TABLE_H_MM * rh)
            pygame.draw.line(s, C['grid'], (ox, py), (ox + rw, py)); v += 20

        cx, cy = mm2p(TABLE_W_MM / 2, TABLE_H_MM / 2)
        pygame.draw.line(s, C['dim'], (cx - 11, cy), (cx + 11, cy))
        pygame.draw.line(s, C['dim'], (cx, cy - 11), (cx, cy + 11))
        pygame.draw.rect(s, C['iso_edge'], rect, 2)

        if show_ticks:
            for vv in range(0, int(TABLE_W_MM) + 1, 40):
                px = ox + int(vv / TABLE_W_MM * rw)
                self.text(str(vv), C['dim'], px, oy + rh + 4, self.f_sm, cx=True)
            for vv in range(0, int(TABLE_H_MM) + 1, 40):
                py = oy + rh - int(vv / TABLE_H_MM * rh)
                self.text(str(vv), C['dim'], ox - 8, py - 7, self.f_sm, right=True)

        # trilha
        now = time.time()
        for xr, yr, ts in list(self.trail):
            fade = max(0.0, 1.0 - (now - ts) / 5.0)
            if fade <= 0: continue
            cc = tuple(int(C['trail_t'][i] + (C['trail_h'][i] - C['trail_t'][i]) * fade) for i in range(3))
            p = mm2p(*self.raw2mm(xr, yr))
            if rect.collidepoint(p):
                pygame.draw.circle(s, cc, p, max(2, int(5 * fade)))

        # bola
        if self.ser.present():
            mm = self.raw2mm(self.ser.x_raw, self.ser.y_raw)
            p = mm2p(*mm)
            px = max(rect.x + 2, min(p[0], rect.right - 2))
            py = max(rect.y + 2, min(p[1], rect.bottom - 2))
            pygame.draw.circle(s, C['ball'], (px, py), 12)
            pygame.draw.circle(s, C['ball_hi'], (px - 4, py - 4), 4)
            return mm
        return None

    # ── Live Ball ─────────────────────────────────────────────────────────────
    def page_live(self, area):
        if self.ser.present():
            self.trail.append((self.ser.x_raw, self.ser.y_raw, time.time()))
        panel_w = 280
        tbl = pygame.Rect(area.x, area.y, area.w - panel_w - 16, area.h)
        self.panel(tbl, C['panel'])
        mm = self._draw_table_mm(tbl.inflate(-30, -30), show_ticks=True)

        side = pygame.Rect(area.right - panel_w, area.y, panel_w, area.h)
        self.panel(side)
        x = side.x + 16; y = side.y + 16
        y = self.text("POSICAO", C['accent'], x, y, self.f_h2) + 4
        col = C['good'] if self.ser.present() else C['warn']
        y = self.text("BOLA NA MESA" if self.ser.present() else "SEM TOQUE", col, x, y, self.f_uib) + 4
        y = self.text(f"raw  X={self.ser.x_raw}  Y={self.ser.y_raw}", C['dim'], x, y, self.f_mono_sm)
        if mm:
            y = self.text(f"mm   X={mm[0]:.1f}  Y={mm[1]:.1f}", C['text'], x, y, self.f_mono_sm) + 14
        else:
            y = self.text("mm   --", C['faint'], x, y, self.f_mono_sm) + 14

        y = self.text("ORIENTACAO", C['accent'], x, y, self.f_h2) + 4
        y = self.text(self.orient_str(), C['mag'], x, y, self.f_ui) + 8
        bw = (panel_w - 40) // 3
        if self.button((x, y, bw, 34), "SWAP", C['accent_dk'] if not self.swap_xy else C['accent']):
            self.swap_xy = not self.swap_xy; self.trail.clear()
        if self.button((x + bw + 6, y, bw, 34), "FLIP X", C['accent_dk'] if not self.flip_x else C['accent']):
            self.flip_x = not self.flip_x; self.trail.clear()
        if self.button((x + 2*(bw+6), y, bw, 34), "FLIP Y", C['accent_dk'] if not self.flip_y else C['accent']):
            self.flip_y = not self.flip_y; self.trail.clear()
        y += 46
        if self.button((x, y, panel_w - 32, 36), "Gravar orientacao (config + header)", C['good'], (15, 30, 20)):
            self.save_orientation()
        y += 44
        if self.button((x, y, panel_w - 32, 32), "Limpar trilha", C['panel_hi'], C['text']):
            self.trail.clear()
        y += 44
        self.text("Teclas: T swap · X/Y flip · G gravar · R limpar", C['faint'], x, y, self.f_sm)

    # ── Calibrar ────────────────────────────────────────────────────────────
    CORNERS = [("SUP. ESQUERDO", (0, 1)), ("SUP. DIREITO", (1, 1)),
               ("INF. ESQUERDO", (0, 0)), ("INF. DIREITO", (1, 0))]

    def page_cal(self, area):
        panel_w = 320
        # mini mesa com indicadores de canto
        tbl = pygame.Rect(area.x, area.y, area.w - panel_w - 16, area.h)
        self.panel(tbl)
        self._draw_cal_table(tbl.inflate(-40, -40))

        side = pygame.Rect(area.right - panel_w, area.y, panel_w, area.h)
        self.panel(side)
        x = side.x + 16; y = side.y + 16

        done = sum(1 for c in self.cal_corners if c)
        y = self.text("ASSISTENTE DE CALIBRACAO", C['accent'], x, y, self.f_h2) + 4
        y = self.text(f"Cantos coletados: {done} / 4", C['text'], x, y, self.f_ui) + 10

        present = self.ser.present()
        if self.cal_state == 'collecting':
            n = len(self.cal_samples)
            y = self.text(f"COLETANDO: {self.CORNERS[self.cal_idx][0]}", C['warn'], x, y, self.f_uib) + 2
            y = self.text("mantenha a bola parada!", C['dim'], x, y, self.f_sm) + 6
            bar = pygame.Rect(x, y, panel_w - 32, 18)
            pygame.draw.rect(self.screen, C['panel_hi'], bar, border_radius=5)
            fw = int(bar.w * n / SAMPLES_NEEDED)
            pygame.draw.rect(self.screen, C['good'], (bar.x, bar.y, fw, bar.h), border_radius=5)
            y += 26
            y = self.text(f"{n} / {SAMPLES_NEEDED}", C['text'], x, y, self.f_ui) + 10
            if not present:
                y = self.text("bola saiu — recoloque", C['bad'], x, y, self.f_sm)
            if present:
                self.cal_samples.append((self.ser.x_raw, self.ser.y_raw))
            if n >= SAMPLES_NEEDED:
                self._finish_corner()
        else:
            nxt = self.cal_idx if self.cal_idx < 4 else 3
            y = self.text(f"Proximo canto:", C['dim'], x, y, self.f_sm)
            y = self.text(self.CORNERS[nxt][0], C['text'], x, y, self.f_uib) + 6
            tcol = C['good'] if present else C['warn']
            y = self.text("BOLA OK" if present else "coloque a bola no canto", tcol, x, y, self.f_ui) + 2
            y = self.text(f"raw ({self.ser.x_raw}, {self.ser.y_raw})", C['dim'], x, y, self.f_mono_sm) + 10
            if self.cal_idx < 4:
                if self.button((x, y, panel_w - 32, 40), f"Coletar canto {self.cal_idx+1}",
                               C['accent'], enabled=present):
                    self._start_collect()
                y += 50
            else:
                self._recompute_cal()
                y = self.text("Calibracao calculada:", C['good'], x, y, self.f_ui) + 4
                y = self.text(f"X [{self.cal[0]} - {self.cal[1]}]", C['text'], x, y, self.f_mono_sm)
                y = self.text(f"Y [{self.cal[2]} - {self.cal[3]}]", C['text'], x, y, self.f_mono_sm) + 10
                if self.button((x, y, panel_w - 32, 36), "Salvar no header (.h)", C['accent']):
                    self.save_cal_header()
                y += 44
                if self.button((x, y, panel_w - 32, 36), "Enviar ao ESP32 (NVS)", C['good'], (15, 30, 20)):
                    self.push_cal_nvs()
                y += 44
                if self.button((x, y, panel_w - 32, 32), "Ver no Live Ball", C['accent_dk']):
                    self.page = 'live'
                y += 40

        if self.button((x, side.bottom - 48, panel_w - 32, 34), "Reiniciar assistente", C['panel_hi'], C['text']):
            self._reset_cal()

    def _draw_cal_table(self, area):
        s = self.screen
        scale = min(area.w / TABLE_W_MM, area.h / TABLE_H_MM)
        rw = int(TABLE_W_MM * scale); rh = int(TABLE_H_MM * scale)
        ox = area.x + (area.w - rw) // 2; oy = area.y + (area.h - rh) // 2
        rect = pygame.Rect(ox, oy, rw, rh)
        pygame.draw.rect(s, C['table_bg'], rect)
        for i in range(1, 9):
            px = ox + i * rw // 9
            pygame.draw.line(s, C['grid'], (px, oy), (px, oy + rh))
        for i in range(1, 7):
            py = oy + i * rh // 7
            pygame.draw.line(s, C['grid'], (ox, py), (ox + rw, py))
        pygame.draw.rect(s, C['iso_edge'], rect, 2)
        m = 14
        for i, (name, (rx, ry)) in enumerate(self.CORNERS):
            px = ox + m if rx == 0 else ox + rw - m
            py = oy + m if ry == 1 else oy + rh - m
            if self.cal_corners[i]:
                pygame.draw.circle(s, C['good'], (px, py), 11)
            elif i == self.cal_idx and self.cal_state != 'done':
                pygame.draw.circle(s, C['warn'], (px, py), 13)
                if int(time.time() * 3) % 2 == 0:
                    pygame.draw.circle(s, C['warn'], (px, py), 18, 2)
            else:
                pygame.draw.circle(s, C['faint'], (px, py), 8)
        # bola atual
        if self.ser.present():
            nx = max(0.0, min(1.0, (self.ser.x_raw - self.cal[0]) / ((self.cal[1]-self.cal[0]) or 1)))
            ny = max(0.0, min(1.0, (self.ser.y_raw - self.cal[2]) / ((self.cal[3]-self.cal[2]) or 1)))
            p = (ox + int(nx * rw), oy + rh - int(ny * rh))
            px = max(rect.x+2, min(p[0], rect.right-2)); py = max(rect.y+2, min(p[1], rect.bottom-2))
            pygame.draw.circle(s, C['ball'], (px, py), 9)

    def _start_collect(self):
        self.cal_samples = []
        self.cal_state = 'collecting'

    def _finish_corner(self):
        xs = [p[0] for p in self.cal_samples]; ys = [p[1] for p in self.cal_samples]
        self.cal_corners[self.cal_idx] = (int(statistics.median(xs)), int(statistics.median(ys)))
        self.cal_idx += 1
        self.cal_state = 'done' if self.cal_idx >= 4 else 'idle'

    def _recompute_cal(self):
        xs = [c[0] for c in self.cal_corners if c]; ys = [c[1] for c in self.cal_corners if c]
        if not xs: return
        x_min, x_max = min(xs), max(xs); y_min, y_max = min(ys), max(ys)
        if x_min >= x_max: x_max = x_min + 1
        if y_min >= y_max: y_max = y_min + 1
        # flip a partir dos cantos: TL=0 TR=1 BL=2 BR=3
        left_x  = (self.cal_corners[0][0] + self.cal_corners[2][0]) // 2
        right_x = (self.cal_corners[1][0] + self.cal_corners[3][0]) // 2
        top_y   = (self.cal_corners[0][1] + self.cal_corners[1][1]) // 2
        bot_y   = (self.cal_corners[2][1] + self.cal_corners[3][1]) // 2
        self.flip_x = left_x > right_x
        self.flip_y = top_y > bot_y
        self.cal = (x_min, x_max, y_min, y_max)

    def _reset_cal(self):
        self.cal_state = 'idle'; self.cal_idx = 0
        self.cal_corners = [None] * 4; self.cal_samples = []

    # ── PID Tuner ──────────────────────────────────────────────────────────
    def page_pid(self, area):
        panel_w = 330
        left = pygame.Rect(area.x, area.y, area.w - panel_w - 16, area.h)
        # mesa
        tbl = pygame.Rect(left.x, left.y, left.w, int(left.h * 0.6))
        self.panel(tbl)
        self._draw_pid_table(tbl.inflate(-30, -30))
        # grafico de erro
        gr = pygame.Rect(left.x, tbl.bottom + 12, left.w, left.bottom - tbl.bottom - 12)
        self.panel(gr, C['bg'])
        self._draw_error_graph(gr)

        side = pygame.Rect(area.right - panel_w, area.y, panel_w, area.h)
        self.panel(side)
        x = side.x + 16; y = side.y + 14
        y = self.text("PID TUNER", C['accent'], x, y, self.f_h2) + 2
        sync = "sincronizado" if self.ser.gains_ok else "aguardando GAINS..."
        y = self.text(sync, C['good'] if self.ser.gains_ok else C['warn'], x, y, self.f_sm) + 8
        step_pct = int(STEPS[self.step_idx] * 100)
        y = self.text(f"Passo de ajuste: {step_pct}%   [Tab]", C['mag'], x, y, self.f_sm) + 12

        for label, val, ckey, keyhint in [
            ("KP", self.kp, 'kp', "Q / A"),
            ("KI", self.ki, 'ki', "W / S"),
            ("KD", self.kd, 'kd', "E / D"),
        ]:
            y = self.text(label, C[ckey], x, y, self.f_uib)
            self.text(f"{val:.4e}", C['text'], x + 50, y - 22, self.f_mono)
            bar = pygame.Rect(x, y, panel_w - 32, 12)
            pygame.draw.rect(self.screen, C['panel_hi'], bar, border_radius=4)
            frac = val / (val * 10 + 1e-12)
            pygame.draw.rect(self.screen, C[ckey], (bar.x, bar.y, int(bar.w * frac), 12), border_radius=4)
            y += 18
            if self.button((x, y, 60, 28), "-", C['panel_hi'], C['text']):
                self._adjust(ckey, -1)
            if self.button((x + 66, y, 60, 28), "+", C[ckey], (15, 20, 30)):
                self._adjust(ckey, +1)
            self.text(f"[{keyhint}]", C['faint'], x + 140, y + 5, self.f_sm)
            y += 38

        # bola / erro
        if self.ser.present():
            mm = self.raw2mm(self.ser.x_raw, self.ser.y_raw)
            err = math.hypot(mm[0] - self.setpt_mm[0], mm[1] - self.setpt_mm[1])
            y = self.text(f"Bola  X={mm[0]:.1f}  Y={mm[1]:.1f}", C['good'], x, y, self.f_sm)
            y = self.text(f"Erro  {err:.1f} mm", C['warn'], x, y, self.f_uib) + 10
        else:
            y = self.text("Bola: sem toque", C['faint'], x, y, self.f_sm) + 10

        if self.button((x, y, panel_w - 32, 38), "SALVAR NO NVS (PID SAVE)", C['good'], (15, 30, 20)):
            self.ser.write("PID SAVE"); self.set_toast("PID SAVE enviado")
        y += 46
        if self.button((x, y, panel_w - 32, 30), "Reenviar ganhos (reset integr.)", C['panel_hi'], C['text']):
            self._send_gains()
        self.text("Teclas: Q/A KP · W/S KI · E/D KD · Tab passo · Espaco salvar",
                  C['faint'], x, side.bottom - 26, self.f_sm)

    def _draw_pid_table(self, area):
        s = self.screen
        scale = min(area.w / TABLE_W_MM, area.h / TABLE_H_MM)
        rw = int(TABLE_W_MM * scale); rh = int(TABLE_H_MM * scale)
        ox = area.x + (area.w - rw) // 2; oy = area.y + (area.h - rh) // 2
        rect = pygame.Rect(ox, oy, rw, rh)
        pygame.draw.rect(s, C['table_bg'], rect)
        v = 20
        while v < TABLE_W_MM:
            px = ox + int(v / TABLE_W_MM * rw); pygame.draw.line(s, C['grid'], (px, oy), (px, oy+rh)); v += 20
        v = 20
        while v < TABLE_H_MM:
            py = oy + rh - int(v / TABLE_H_MM * rh); pygame.draw.line(s, C['grid'], (ox, py), (ox+rw, py)); v += 20
        pygame.draw.rect(s, C['iso_edge'], rect, 2)

        def mm2p(xmm, ymm):
            return ox + int(xmm / TABLE_W_MM * rw), oy + rh - int(ymm / TABLE_H_MM * rh)

        sp = mm2p(*self.setpt_mm)
        pygame.draw.circle(s, C['target'], sp, 8, 2)
        pygame.draw.line(s, C['target'], (sp[0]-11, sp[1]), (sp[0]+11, sp[1]))
        pygame.draw.line(s, C['target'], (sp[0], sp[1]-11), (sp[0], sp[1]+11))

        if self.ser.present():
            mm = self.raw2mm(self.ser.x_raw, self.ser.y_raw)
            bp = mm2p(*mm)
            pygame.draw.line(s, C['warn'], sp, bp, 1)
            bx = max(rect.x+2, min(bp[0], rect.right-2)); by = max(rect.y+2, min(bp[1], rect.bottom-2))
            pygame.draw.circle(s, C['ball'], (bx, by), 11)
            pygame.draw.circle(s, C['ball_hi'], (bx-3, by-3), 3)
            err = math.hypot(mm[0]-self.setpt_mm[0], mm[1]-self.setpt_mm[1])
            self.err_hist.append((time.time(), err))
        else:
            self.err_hist.append((time.time(), float('nan')))

    def _draw_error_graph(self, rect):
        self.text("Erro ao alvo (mm)", C['dim'], rect.x + 10, rect.y + 6, self.f_sm)
        pts_data = list(self.err_hist)
        vals = [v for _, v in pts_data if not math.isnan(v)]
        if len(pts_data) < 2 or not vals:
            return
        y_max = max(max(vals) * 1.1, 10.0)
        t0 = pts_data[0][0]; tspan = max(time.time() - t0, 1.0)
        pad = 28
        pts = []
        for t, v in pts_data:
            if math.isnan(v): continue
            px = rect.x + pad + int((t - t0) / tspan * (rect.w - pad - 10))
            py = rect.bottom - 18 - int(v / y_max * (rect.h - 40))
            pts.append((px, py))
        pygame.draw.line(self.screen, C['faint'], (rect.x + pad, rect.bottom - 18),
                         (rect.right - 10, rect.bottom - 18))
        if len(pts) >= 2:
            pygame.draw.lines(self.screen, C['ball'], False, pts, 2)
        _, last = pts_data[-1]
        if not math.isnan(last):
            self.text(f"{last:.1f} mm", C['ball'], rect.right - 12, rect.y + 6, self.f_sm, right=True)

    def _adjust(self, which, sign):
        step = STEPS[self.step_idx]
        cur = getattr(self, which)
        setattr(self, which, max(0.0, cur * (1.0 + sign * step)))
        self._send_gains()

    def _send_gains(self):
        self.ser.write(f"PID {self.kp:.6g} {self.ki:.6g} {self.kd:.6g}")

    # ── Simulacao 3RPS ────────────────────────────────────────────────────────
    def _sim_update(self):
        now = time.time()
        dt = 0.0 if self.bal_t is None else now - self.bal_t
        self.bal_t = now
        dt = min(max(dt, 1e-3), 0.05)
        if self.ser.present():
            tx, ty = self.raw2mm(self.ser.x_raw, self.ser.y_raw)
            if self.ball_mm is None:
                self.ball_mm = [tx, ty]
            else:
                a = 1.0 - math.exp(-dt / BALL_TAU)
                self.ball_mm[0] += (tx - self.ball_mm[0]) * a
                self.ball_mm[1] += (ty - self.ball_mm[1]) * a
            self.bal_seen_ts = now
        present = self.ball_mm is not None and (now - self.bal_seen_ts) < PRESENCE_HOLD
        if present:
            tnx = self.pid_x.update(self.ball_mm[0], self.sim_sx, dt)
            tny = self.pid_y.update(self.ball_mm[1], self.sim_sy, dt)
        else:
            self.pid_x.reset(); self.pid_y.reset()
            tnx = tny = 0.0
            if self.ball_mm is not None and (now - self.bal_seen_ts) > 1.2:
                self.ball_mm = None
        a2 = 1.0 - math.exp(-dt / TILT_TAU)
        self.disp_nx += (tnx - self.disp_nx) * a2
        self.disp_ny += (tny - self.disp_ny) * a2
        self.disp_theta = [machine_theta(i, GEO_HZ, self.disp_nx, self.disp_ny) for i in range(3)]
        return present

    def _iso(self, x, y, z, cx, cy):
        ca, sa = math.cos(self.cam_az), math.sin(self.cam_az)
        xr = x * ca - y * sa; yr = x * sa + y * ca
        sp = math.sin(self.cam_pitch)
        return int(cx + xr * self.cam_scale), int(cy + (yr * sp - z) * self.cam_scale)

    def page_sim(self, area):
        present = self._sim_update()
        s = self.screen
        panel_w = 300
        view = pygame.Rect(area.x, area.y, area.w - panel_w - 16, area.h)
        self.panel(view, C['bg'])

        nx, ny = self.disp_nx, self.disp_ny
        hw, hh = TABLE_W_MM / 2.0, TABLE_H_MM / 2.0
        cx = view.centerx; cy = view.centery + 60

        base_pts, plat_pts = [], []
        for az in LEG_AZ:
            base_pts.append((GEO_D * 2.2 * math.cos(az), GEO_D * 2.2 * math.sin(az), 0.0))
            px, py = hw * math.cos(az), hh * math.sin(az)
            plat_pts.append((px, py, plate_z(px, py, nx, ny, PLATE_BASE_Z)))
        for i in range(3):
            pygame.draw.line(s, C['iso_base'], self._iso(*base_pts[i], cx, cy),
                             self._iso(*base_pts[(i+1) % 3], cx, cy), 2)
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
            arm_tips.append((at, i))
            pygame.draw.circle(s, C['iso_base'], bj, 5)
            pygame.draw.line(s, C['iso_arm'], bj, at, 4)
            pygame.draw.line(s, C['iso_leg'], at, pj, 2)

        quad = [self._iso(X, Y, plate_z(X, Y, nx, ny, PLATE_BASE_Z), cx, cy)
                for (X, Y) in [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)]]
        pygame.draw.polygon(s, C['iso_plate'], quad)
        pygame.draw.polygon(s, C['iso_edge'], quad, 2)
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

        tX, tY = self.sim_sx - hw, self.sim_sy - hh
        tp = self._iso(tX, tY, plate_z(tX, tY, nx, ny, PLATE_BASE_Z), cx, cy)
        pygame.draw.circle(s, C['target'], tp, 7, 2)

        if self.ball_mm is not None:
            bX, bY = self.ball_mm[0] - hw, self.ball_mm[1] - hh
            bZ = plate_z(bX, bY, nx, ny, PLATE_BASE_Z)
            sp = self._iso(bX, bY, bZ, cx, cy)
            bp = self._iso(bX, bY, bZ + 6.0, cx, cy)
            pygame.draw.circle(s, C['iso_base'], sp, 7)
            r = max(7, int(6.0 * self.cam_scale))
            pygame.draw.circle(s, C['iso_ball'], bp, r)
            pygame.draw.circle(s, C['iso_ballhi'], (bp[0] - r//3, bp[1] - r//3), max(2, r//3))

        for at, i in arm_tips:
            pygame.draw.circle(s, C['iso_arm'], at, 3)
            self.text(f"{LEG_NAME[i]} {self.disp_theta[i]:.1f}", C['iso_arm'], at[0] + 6, at[1] - 6, self.f_sm)

        # painel
        side = pygame.Rect(area.right - panel_w, area.y, panel_w, area.h)
        self.panel(side)
        x = side.x + 16; y = side.y + 14
        y = self.text("MOTORES 3RPS", C['accent'], x, y, self.f_h2) + 4
        col = C['good'] if present else C['warn']
        y = self.text("BOLA DETECTADA" if present else "SEM BOLA (nivelando)", col, x, y, self.f_uib) + 8
        if self.ball_mm is not None:
            y = self.text(f"Bola X={self.ball_mm[0]:.1f}  Y={self.ball_mm[1]:.1f}", C['text'], x, y, self.f_sm)
            err = math.hypot(self.ball_mm[0]-self.sim_sx, self.ball_mm[1]-self.sim_sy)
            y = self.text(f"Erro  {err:.1f} mm", C['warn'], x, y, self.f_sm) + 8
        else:
            y = self.text("Bola: --", C['faint'], x, y, self.f_sm) + 8
        y = self.text(f"Alvo  X={self.sim_sx:.0f}  Y={self.sim_sy:.0f}", C['target'], x, y, self.f_sm) + 8
        y = self.text("INCLINACAO", C['accent'], x, y, self.f_uib) + 2
        y = self.text(f"nx={self.disp_nx:+.3f}  ny={self.disp_ny:+.3f}", C['text'], x, y, self.f_sm) + 8
        y = self.text("ANGULOS", C['accent'], x, y, self.f_uib) + 2
        for i in range(3):
            y = self.text(f"  {LEG_NAME[i]} = {self.disp_theta[i]:7.2f} graus", C['iso_arm'], x, y, self.f_mono_sm)
        y += 12
        self.text("WASD mover alvo · Q/E girar · +/- zoom", C['faint'], x, side.bottom - 44, self.f_sm)
        self.text("T/X/Y orientacao · G gravar", C['faint'], x, side.bottom - 26, self.f_sm)

    # ════════════════════════════════════════════════════════════════════════
    #  CHROME (sidebar + header) e loop
    # ════════════════════════════════════════════════════════════════════════

    def draw_sidebar(self):
        s = self.screen
        sb = pygame.Rect(0, 0, SB_W, WIN_H)
        pygame.draw.rect(s, C['sidebar'], sb)
        pygame.draw.line(s, C['border'], (SB_W, 0), (SB_W, WIN_H))

        # logo
        pygame.draw.circle(s, C['accent'], (30, 38), 11)
        pygame.draw.circle(s, C['bg'], (30, 38), 4)
        self.text("BALL BALANCER", C['text'], 50, 24, self.f_uib)
        self.text("STUDIO", C['accent'], 50, 42, self.f_sm)

        y = 86
        for key, label, icon in NAV:
            r = pygame.Rect(10, y, SB_W - 20, 42)
            active = self.page == key
            hover = r.collidepoint(self.mouse)
            if active:
                pygame.draw.rect(s, C['accent_dk'], r, border_radius=8)
                pygame.draw.rect(s, C['accent'], (10, y, 3, 42), border_radius=2)
            elif hover:
                pygame.draw.rect(s, C['panel'], r, border_radius=8)
            tc = C['text'] if (active or hover) else C['dim']
            self.text(icon, C['accent'] if active else C['dim'], 24, y + 10, self.f_icon)
            self.text(label, tc, 52, y + 11, self.f_ui)
            if self.click and r.collidepoint(self.click):
                self.page = key; self.scroll = 0; self.click = None
            y += 48

        # status conexao no rodape
        dot = C['good'] if self.ser.connected else C['bad']
        pygame.draw.circle(s, dot, (26, WIN_H - 36), 6)
        self.text(PORT, C['text'], 42, WIN_H - 44, self.f_sm)
        self.text(self.ser.status[:22], C['dim'], 42, WIN_H - 28, self.f_sm)

    def draw_header(self):
        s = self.screen
        hd = pygame.Rect(SB_W, 0, WIN_W - SB_W, HEADER_H)
        pygame.draw.rect(s, C['bg'], hd)
        pygame.draw.line(s, C['border'], (SB_W, HEADER_H), (WIN_W, HEADER_H))
        title = next(lbl for k, lbl, _ in NAV if k == self.page)
        self.text(title, C['text'], SB_W + 22, 18, self.f_h1)

        # chips de status a direita
        rx = WIN_W - 24
        present = self.ser.present()
        chips = [
            ("BOLA" if present else "sem bola", C['good'] if present else C['faint']),
            ("conectado" if self.ser.connected else "offline", C['good'] if self.ser.connected else C['bad']),
        ]
        if self.idf.running:
            chips.insert(0, (self.idf.action + "...", C['warn']))
        x = rx
        for label, color in chips:
            w = self.f_sm.size(label)[0] + 26
            x -= w + 8
            self.chip(x, 24, label, color)

    def draw_toast(self):
        if self.toast and time.time() - self.toast_ts < 3.0:
            surf = self.f_uib.render(self.toast, True, (255, 255, 255))
            w = surf.get_width() + 32
            r = pygame.Rect((WIN_W + SB_W) // 2 - w // 2, WIN_H - 56, w, 36)
            pygame.draw.rect(self.screen, C['accent_dk'], r, border_radius=18)
            pygame.draw.rect(self.screen, C['accent'], r, 1, border_radius=18)
            self.screen.blit(surf, (r.centerx - surf.get_width() // 2, r.y + 8))
        # toast do firmware (SAVED)
        if self.ser.saved_msg and time.time() - self.ser.saved_ts < 3.0:
            self.text(self.ser.saved_msg, C['good'], SB_W + 22, WIN_H - 24, self.f_sm)

    # ── eventos ──────────────────────────────────────────────────────────────
    def handle_key(self, ev):
        k = ev.key
        # Monitor com input ativo captura o teclado
        if self.page == 'monitor' and self.cmd_active:
            if k == pygame.K_RETURN:
                self._send_cmd()
            elif k == pygame.K_BACKSPACE:
                self.cmd_text = self.cmd_text[:-1]
            elif k == pygame.K_ESCAPE:
                self.cmd_active = False
            elif ev.unicode and ev.unicode.isprintable():
                self.cmd_text += ev.unicode
            return

        if k == pygame.K_ESCAPE:
            self.running = False
            return
        # atalhos globais de navegacao 1-7
        if pygame.K_1 <= k <= pygame.K_7:
            self.page = NAV[k - pygame.K_1][0]; self.scroll = 0
            return

        if self.page == 'live':
            if k == pygame.K_t: self.swap_xy = not self.swap_xy; self.trail.clear()
            elif k == pygame.K_x: self.flip_x = not self.flip_x; self.trail.clear()
            elif k == pygame.K_y: self.flip_y = not self.flip_y; self.trail.clear()
            elif k == pygame.K_g: self.save_orientation()
            elif k == pygame.K_r: self.trail.clear()
        elif self.page == 'pid':
            if k == pygame.K_TAB: self.step_idx = (self.step_idx + 1) % len(STEPS)
            elif k == pygame.K_q: self._adjust('kp', +1)
            elif k == pygame.K_a: self._adjust('kp', -1)
            elif k == pygame.K_w: self._adjust('ki', +1)
            elif k == pygame.K_s: self._adjust('ki', -1)
            elif k == pygame.K_e: self._adjust('kd', +1)
            elif k == pygame.K_d: self._adjust('kd', -1)
            elif k == pygame.K_SPACE: self.ser.write("PID SAVE"); self.set_toast("PID SAVE enviado")
            elif k == pygame.K_r: self._send_gains()
        elif self.page == 'sim':
            if k == pygame.K_a: self.sim_sx = max(0, self.sim_sx - 8)
            elif k == pygame.K_d: self.sim_sx = min(TABLE_W_MM, self.sim_sx + 8)
            elif k == pygame.K_s: self.sim_sy = max(0, self.sim_sy - 8)
            elif k == pygame.K_w: self.sim_sy = min(TABLE_H_MM, self.sim_sy + 8)
            elif k == pygame.K_q: self.cam_az -= math.radians(8)
            elif k == pygame.K_e: self.cam_az += math.radians(8)
            elif k in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):
                self.cam_scale = min(5.0, self.cam_scale * 1.1)
            elif k in (pygame.K_MINUS, pygame.K_KP_MINUS):
                self.cam_scale = max(1.0, self.cam_scale / 1.1)
            elif k == pygame.K_t: self.swap_xy = not self.swap_xy
            elif k == pygame.K_x: self.flip_x = not self.flip_x
            elif k == pygame.K_y: self.flip_y = not self.flip_y
            elif k == pygame.K_g: self.save_orientation()

    def run(self):
        self.running = True
        pages = {
            'dash': self.page_dash, 'build': self.page_build, 'monitor': self.page_monitor,
            'live': self.page_live, 'cal': self.page_cal, 'pid': self.page_pid, 'sim': self.page_sim,
        }
        try:
            while self.running:
                self.mouse = pygame.mouse.get_pos()
                for ev in pygame.event.get():
                    if ev.type == pygame.QUIT:
                        self.running = False
                    elif ev.type == pygame.MOUSEBUTTONDOWN:
                        if ev.button == 1:
                            self.click = ev.pos
                            # ativa input do monitor ao clicar na faixa inferior
                            if self.page == 'monitor':
                                area = self.content_rect()
                                inp = pygame.Rect(area.x, area.bottom - 42, area.w - 110, 36)
                                self.cmd_active = inp.collidepoint(ev.pos)
                        elif ev.button == 4:
                            self.scroll += 3
                        elif ev.button == 5:
                            self.scroll = max(0, self.scroll - 3)
                    elif ev.type == pygame.KEYDOWN:
                        self.handle_key(ev)

                # sincroniza ganhos do firmware quando chegam pela 1a vez
                if self.ser.gains_ok and self.page != 'pid':
                    self.kp, self.ki, self.kd = self.ser.kp, self.ser.ki, self.ser.kd

                self.screen.fill(C['bg'])
                pages[self.page](self.content_rect())
                self.draw_sidebar()
                self.draw_header()
                self.draw_toast()
                self.click = None     # consome cliques nao usados
                pygame.display.flip()
                self.clock.tick(FPS)
        except KeyboardInterrupt:
            pass
        finally:
            self.ser.stop()
            pygame.quit()


def main():
    global PORT, BAUD
    if len(sys.argv) >= 2:
        PORT = sys.argv[1]          # porta forcada pela linha de comando
    else:
        PORT = autodetect_port(PORT)  # senao, descobre sozinho
    if len(sys.argv) >= 3:
        BAUD = int(sys.argv[2])
    print("=" * 56)
    print(" Ball Balancer Studio")
    print(f"   Porta : {PORT}  @ {BAUD} baud")
    print("   Dica  : feche outros monitores seriais nessa porta.")
    print("   (para forcar outra porta:  python bb_studio.py COM5)")
    print("=" * 56)
    Studio().run()


if __name__ == '__main__':
    main()
