#!/usr/bin/env python3
"""
Ball Balancer — Configuracao central das ferramentas Python
============================================================
Edite este arquivo para mudar porta, dimensoes e orientacao da tela.
As ferramentas (calibrate.py, diag.py, visualizer.py) leem tudo daqui.

A orientacao tambem pode ser ajustada AO VIVO no calibrate.py (tela LIVE)
com as teclas  X / Y / T , e gravada de volta aqui (e no firmware) com  G .
"""

# ── Conexao serial ───────────────────────────────────────────────────────────
PORT = "COM8"
BAUD = 115200

# ── Dimensoes fisicas da tela (mm) ───────────────────────────────────────────
# Orientacao padrao: deitada, comprimento para a direita.
SCREEN_W_MM = 187.0   # comprimento (eixo X, esquerda -> direita)
SCREEN_H_MM = 141.0   # altura      (eixo Y, baixo -> cima)

# ── Orientacao / mapeamento dos eixos ────────────────────────────────────────
# Ajuste ate a bola na tela fisica bater com o ponto na tela do programa.
# Os tres juntos cobrem as 8 orientacoes possiveis (4 rotacoes x espelho).
SWAP_XY = True   # troca os eixos X <-> Y   (rotaciona 90 graus)
FLIP_X  = True   # espelha horizontalmente
FLIP_Y  = False   # espelha verticalmente

# ── Deteccao ─────────────────────────────────────────────────────────────────
DETECT_THRESHOLD = 300   # mesmo valor de TOUCH_DETECT_THRESHOLD no firmware

# ── Calibracao (preenchido pelo calibrate.py / espelho do touch_screen.h) ─────
# Estes sao apenas referencia; o firmware usa os #define de touch_screen.h.
X_RAW_MIN = 2000
X_RAW_MAX = 3700
Y_RAW_MIN = 1700
Y_RAW_MAX = 2600
