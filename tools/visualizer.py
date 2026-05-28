"""
Ball Balancer — Touch Position Visualizer
==========================================
Usage: python visualizer.py <COMx> [baud]
  ex:  python visualizer.py COM5
  ex:  python visualizer.py COM5 115200

Parses serial output from the ESP32-S3 firmware:
  POS,x_raw,y_raw,x_mm,y_mm   — valid touch position
  NOTOUCH,x_raw,y_raw          — no contact
  CAL_HINT,xmin,xmax,ymin,ymax — calibration range hint from firmware
"""

import sys
import threading
import queue
import time
import collections

try:
    import serial
except ImportError:
    print("[ERROR] pyserial not installed. Run: pip install -r requirements.txt")
    sys.exit(1)

try:
    import pygame
except ImportError:
    print("[ERROR] pygame not installed. Run: pip install -r requirements.txt")
    sys.exit(1)

# ── Screen / table constants ─────────────────────────────────────────────────

TABLE_W_MM  = 187.0
TABLE_H_MM  = 141.0

# Window layout
WIN_W       = 960
WIN_H       = 640

TABLE_MARGIN_X = 40
TABLE_MARGIN_Y = 80
TABLE_PIXEL_W  = 600           # pixels for TABLE_W_MM
TABLE_PIXEL_H  = int(TABLE_PIXEL_W * TABLE_H_MM / TABLE_W_MM)  # ~453 px
SCALE_PX_PER_MM = TABLE_PIXEL_W / TABLE_W_MM   # px/mm

TABLE_ORIGIN_X = TABLE_MARGIN_X
TABLE_ORIGIN_Y = TABLE_MARGIN_Y

STATS_X = TABLE_ORIGIN_X + TABLE_PIXEL_W + 30
STATS_Y = TABLE_MARGIN_Y

TRAIL_LEN = 80  # last N positions drawn as fading dots

# ── Colors ───────────────────────────────────────────────────────────────────

C_BG         = (18,  18,  24)
C_TABLE_BG   = (245, 245, 230)
C_TABLE_EDGE = (80,  80,  80)
C_GRID       = (200, 200, 185)
C_CENTER     = (160, 160, 160)
C_BALL       = (220,  40,  40)
C_TRAIL_HEAD = (100, 150, 220)
C_TRAIL_TAIL = (30,  50,  80)
C_TEXT       = (210, 210, 210)
C_GOOD       = (80,  200, 100)
C_WARN       = (220, 180,  60)
C_BAD        = (220,  60,  60)
C_HEADING    = (130, 180, 255)
C_DIM        = (80,  80,  90)
C_HINT       = (170, 130, 220)


def mm_to_px(x_mm, y_mm):
    px = int(TABLE_ORIGIN_X + x_mm * SCALE_PX_PER_MM)
    py = int(TABLE_ORIGIN_Y + TABLE_PIXEL_H - y_mm * SCALE_PX_PER_MM)
    return px, py


# ── Serial reader thread ──────────────────────────────────────────────────────

class SerialReader(threading.Thread):
    def __init__(self, port, baud, data_q, status_q):
        super().__init__(daemon=True)
        self.port     = port
        self.baud     = baud
        self.data_q   = data_q
        self.status_q = status_q
        self._stop    = threading.Event()

    def run(self):
        ser = None
        while not self._stop.is_set():
            try:
                ser = serial.Serial(self.port, self.baud, timeout=1.0)
                self.status_q.put(("connected", self.port, self.baud))
                while not self._stop.is_set():
                    line = ser.readline().decode("utf-8", errors="replace").strip()
                    if line:
                        self.data_q.put(line)
            except serial.SerialException as e:
                self.status_q.put(("error", str(e)))
                time.sleep(2.0)
            except Exception as e:
                self.status_q.put(("error", str(e)))
                time.sleep(2.0)
            finally:
                if ser and ser.is_open:
                    try:
                        ser.close()
                    except Exception:
                        pass
                ser = None

    def stop(self):
        self._stop.set()


# ── Data parser ───────────────────────────────────────────────────────────────

class TouchData:
    def __init__(self):
        self.x_mm     = 0.0
        self.y_mm     = 0.0
        self.x_raw    = 0
        self.y_raw    = 0
        self.valid    = False
        self.trail    = collections.deque(maxlen=TRAIL_LEN)
        # Observed calibration range
        self.obs_x_min = None
        self.obs_x_max = None
        self.obs_y_min = None
        self.obs_y_max = None
        # Hint sent by firmware
        self.cal_hint  = None
        self.last_ts   = time.time()

    def update_from_line(self, line: str):
        if line.startswith("#"):
            return
        parts = line.split(",")
        tag = parts[0].strip()

        if tag == "POS" and len(parts) >= 5:
            try:
                xr    = int(parts[1])
                yr    = int(parts[2])
                xm    = float(parts[3])
                ym    = float(parts[4])
                self.x_raw  = xr
                self.y_raw  = yr
                self.x_mm   = xm
                self.y_mm   = ym
                self.valid  = True
                self.trail.append((xm, ym, time.time()))
                self.last_ts = time.time()
                # Track observed range
                if self.obs_x_min is None or xr < self.obs_x_min:
                    self.obs_x_min = xr
                if self.obs_x_max is None or xr > self.obs_x_max:
                    self.obs_x_max = xr
                if self.obs_y_min is None or yr < self.obs_y_min:
                    self.obs_y_min = yr
                if self.obs_y_max is None or yr > self.obs_y_max:
                    self.obs_y_max = yr
            except (ValueError, IndexError):
                pass

        elif tag == "NOTOUCH":
            self.valid = False
            if len(parts) >= 3:
                try:
                    self.x_raw = int(parts[1])
                    self.y_raw = int(parts[2])
                except ValueError:
                    pass

        elif tag == "CAL_HINT":
            if len(parts) >= 5 and parts[1].strip() != "no_data":
                try:
                    self.cal_hint = (
                        int(parts[1]), int(parts[2]),
                        int(parts[3]), int(parts[4])
                    )
                except ValueError:
                    pass


# ── Drawing helpers ───────────────────────────────────────────────────────────

def draw_table(surf, font_sm, td: TouchData):
    # Table background
    table_rect = pygame.Rect(TABLE_ORIGIN_X, TABLE_ORIGIN_Y,
                             TABLE_PIXEL_W,  TABLE_PIXEL_H)
    pygame.draw.rect(surf, C_TABLE_BG, table_rect)
    pygame.draw.rect(surf, C_TABLE_EDGE, table_rect, 2)

    # Grid every 10 mm
    step_px = int(10 * SCALE_PX_PER_MM)
    x = TABLE_ORIGIN_X + step_px
    while x < TABLE_ORIGIN_X + TABLE_PIXEL_W:
        pygame.draw.line(surf, C_GRID, (x, TABLE_ORIGIN_Y),
                         (x, TABLE_ORIGIN_Y + TABLE_PIXEL_H), 1)
        x += step_px
    y = TABLE_ORIGIN_Y + step_px
    while y < TABLE_ORIGIN_Y + TABLE_PIXEL_H:
        pygame.draw.line(surf, C_GRID, (TABLE_ORIGIN_X, y),
                         (TABLE_ORIGIN_X + TABLE_PIXEL_W, y), 1)
        y += step_px

    # Center crosshair
    cx, cy = mm_to_px(TABLE_W_MM / 2, TABLE_H_MM / 2)
    pygame.draw.line(surf, C_CENTER, (cx - 12, cy), (cx + 12, cy), 1)
    pygame.draw.line(surf, C_CENTER, (cx, cy - 12), (cx, cy + 12), 1)

    # Corner labels
    offset = 6
    for lbl, xm, ym in [("(0,0)",        0,          0),
                          (f"({TABLE_W_MM:.0f},0)", TABLE_W_MM, 0),
                          (f"(0,{TABLE_H_MM:.0f})", 0,          TABLE_H_MM),
                          (f"({TABLE_W_MM:.0f},{TABLE_H_MM:.0f})",
                           TABLE_W_MM, TABLE_H_MM)]:
        px, py = mm_to_px(xm, ym)
        ts = font_sm.render(lbl, True, C_DIM)
        ax = px - ts.get_width() - offset if xm > 0 else px + offset
        ay = py - ts.get_height() - offset if ym > 0 else py + offset
        surf.blit(ts, (ax, ay))

    # Trail
    now = time.time()
    trail_list = list(td.trail)
    for i, (xm, ym, ts) in enumerate(trail_list):
        age  = now - ts
        fade = max(0.0, 1.0 - age / 3.0)
        if fade <= 0.0:
            continue
        r = int(C_TRAIL_TAIL[0] + (C_TRAIL_HEAD[0] - C_TRAIL_TAIL[0]) * fade)
        g = int(C_TRAIL_TAIL[1] + (C_TRAIL_HEAD[1] - C_TRAIL_TAIL[1]) * fade)
        b = int(C_TRAIL_TAIL[2] + (C_TRAIL_HEAD[2] - C_TRAIL_TAIL[2]) * fade)
        radius = max(2, int(5 * fade))
        px, py = mm_to_px(xm, ym)
        if table_rect.collidepoint(px, py):
            pygame.draw.circle(surf, (r, g, b), (px, py), radius)

    # Current position
    if td.valid:
        px, py = mm_to_px(td.x_mm, td.y_mm)
        if table_rect.collidepoint(px, py):
            pygame.draw.circle(surf, C_BALL, (px, py), 10)
            pygame.draw.circle(surf, (255, 100, 100), (px, py), 10, 2)


def draw_stats(surf, font_lg, font_md, font_sm, td: TouchData,
               conn_status: str, port: str, baud: int):
    x0, y0 = STATS_X, STATS_Y
    line_h = 28

    def text(txt, color, y_off, fnt=None):
        fnt = fnt or font_md
        ts = fnt.render(txt, True, color)
        surf.blit(ts, (x0, y0 + y_off))
        return y_off + line_h

    # Title
    ts = font_lg.render("Touch Mapping", True, C_HEADING)
    surf.blit(ts, (x0, y0 - 40))

    # Connection
    conn_color = C_GOOD if conn_status == "connected" else C_BAD
    y = text(f"Port: {port}  @{baud}", conn_color, 0, font_sm)
    status_txt = "CONNECTED" if conn_status == "connected" else conn_status.upper()
    y = text(status_txt, conn_color, y, font_sm)
    y += 10

    # Position
    touch_txt  = "TOUCHING" if td.valid else "NO TOUCH"
    touch_col  = C_GOOD    if td.valid else C_WARN
    y = text(touch_txt, touch_col, y)
    y += 4

    if td.valid:
        y = text(f"X = {td.x_mm:6.1f} mm", C_TEXT, y)
        y = text(f"Y = {td.y_mm:6.1f} mm", C_TEXT, y)
        y += 6
        y = text(f"X raw = {td.x_raw}", C_DIM, y, font_sm)
        y = text(f"Y raw = {td.y_raw}", C_DIM, y, font_sm)
    else:
        y = text(f"X raw = {td.x_raw}", C_DIM, y, font_sm)
        y = text(f"Y raw = {td.y_raw}", C_DIM, y, font_sm)

    y += 18

    # Observed calibration range
    y = text("── Observed range ──", C_HINT, y, font_sm)
    if td.obs_x_min is not None:
        y = text(f"X raw: {td.obs_x_min} – {td.obs_x_max}", C_HINT, y, font_sm)
        y = text(f"Y raw: {td.obs_y_min} – {td.obs_y_max}", C_HINT, y, font_sm)
    else:
        y = text("(no data yet)", C_DIM, y, font_sm)

    if td.cal_hint:
        y += 6
        y = text("── Firmware hint ──", C_HINT, y, font_sm)
        y = text(f"X: {td.cal_hint[0]} – {td.cal_hint[1]}", C_HINT, y, font_sm)
        y = text(f"Y: {td.cal_hint[2]} – {td.cal_hint[3]}", C_HINT, y, font_sm)

    y += 18
    y = text("── How to calibrate ──", C_DIM, y, font_sm)
    y = text("Move ball to all 4 corners", C_DIM, y, font_sm)
    y = text("Wait for CAL_HINT to update", C_DIM, y, font_sm)
    y = text("Enter values in tool option 8", C_DIM, y, font_sm)


def draw_axis_labels(surf, font_sm):
    # X label at bottom of table
    lbl = font_sm.render("X  (mm)", True, C_DIM)
    surf.blit(lbl, (TABLE_ORIGIN_X + TABLE_PIXEL_W // 2 - lbl.get_width() // 2,
                    TABLE_ORIGIN_Y + TABLE_PIXEL_H + 8))
    # Y label on left side (rotated)
    lbl = font_sm.render("Y  (mm)", True, C_DIM)
    rot = pygame.transform.rotate(lbl, 90)
    surf.blit(rot, (TABLE_ORIGIN_X - rot.get_width() - 8,
                    TABLE_ORIGIN_Y + TABLE_PIXEL_H // 2 - rot.get_height() // 2))


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    port = sys.argv[1]
    baud = int(sys.argv[2]) if len(sys.argv) >= 3 else 115200

    pygame.init()
    screen = pygame.display.set_mode((WIN_W, WIN_H))
    pygame.display.set_caption(f"Ball Balancer — Touch Map  [{port}]")
    clock = pygame.time.Clock()

    font_lg = pygame.font.SysFont("consolas", 20, bold=True)
    font_md = pygame.font.SysFont("consolas", 17)
    font_sm = pygame.font.SysFont("consolas", 14)

    data_q   = queue.Queue()
    status_q = queue.Queue()

    reader = SerialReader(port, baud, data_q, status_q)
    reader.start()

    td             = TouchData()
    conn_status    = "connecting"
    conn_port      = port
    conn_baud      = baud

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_r:
                    td.obs_x_min = td.obs_x_max = None
                    td.obs_y_min = td.obs_y_max = None
                    td.trail.clear()
                    td.cal_hint = None

        # Drain status queue
        while not status_q.empty():
            msg = status_q.get_nowait()
            conn_status = msg[0]
            if msg[0] == "connected":
                conn_port = msg[1]
                conn_baud = msg[2]

        # Drain data queue (process up to 20 lines per frame)
        for _ in range(20):
            if data_q.empty():
                break
            line = data_q.get_nowait()
            td.update_from_line(line)

        # If last valid touch was > 200 ms ago, mark invalid for display
        if td.valid and (time.time() - td.last_ts) > 0.2:
            td.valid = False

        screen.fill(C_BG)

        # Title bar
        title = font_lg.render(
            "Ball Balancer — Touch Position Mapping", True, C_HEADING)
        screen.blit(title, (WIN_W // 2 - title.get_width() // 2, 18))

        draw_table(screen, font_sm, td)
        draw_axis_labels(screen, font_sm)
        draw_stats(screen, font_lg, font_md, font_sm, td,
                   conn_status, conn_port, conn_baud)

        # Footer hint
        hint = font_sm.render(
            "R = reset observed range    ESC = quit", True, C_DIM)
        screen.blit(hint, (WIN_W // 2 - hint.get_width() // 2, WIN_H - 22))

        pygame.display.flip()
        clock.tick(60)

    reader.stop()
    pygame.quit()


if __name__ == "__main__":
    main()
