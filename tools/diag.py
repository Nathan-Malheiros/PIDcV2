#!/usr/bin/env python3
"""
Ball Balancer — Diagnostico de leitura crua
============================================
Uso:  python diag.py            (usa COM8)
      python diag.py COM5

Mostra X raw e Y raw ao vivo, marca se cada eixo passou do threshold de
deteccao (300) e guarda o MAXIMO ja visto em cada eixo. Role a bola por
toda a tela e observe se AMBOS os eixos reagem.

Ctrl+C para sair (mostra o resumo).
"""

import sys, time

try:
    import serial
except ImportError:
    print("[ERRO] pyserial nao instalado.  pip install -r requirements.txt")
    sys.exit(1)

DEFAULT_PORT = "COM8"
BAUD         = 115200
THRESHOLD    = 300     # igual ao TOUCH_DETECT_THRESHOLD do firmware


def main():
    port = sys.argv[1] if len(sys.argv) >= 2 else DEFAULT_PORT
    print(f"Abrindo {port} @ {BAUD} ...")
    print("Role a bola por toda a tela. Ctrl+C para encerrar.\n")

    x_max = y_max = 0
    x_min = y_min = 99999
    pos_count = 0
    total = 0

    try:
        ser = serial.Serial(port, BAUD, timeout=1.0)
    except serial.SerialException as e:
        print(f"[ERRO] Nao consegui abrir {port}: {e}")
        print("Feche o calibrate.py / monitor serial antes (a porta so aceita 1 programa).")
        sys.exit(1)

    try:
        while True:
            line = ser.readline().decode("utf-8", errors="replace").strip()
            if not line:
                continue
            parts = line.split(",")
            tag = parts[0]
            if tag in ("POS", "NOTOUCH") and len(parts) >= 3:
                try:
                    xr = int(parts[1]); yr = int(parts[2])
                except ValueError:
                    continue
                total += 1
                x_max = max(x_max, xr); y_max = max(y_max, yr)
                x_min = min(x_min, xr); y_min = min(y_min, yr)
                xflag = "OK " if xr > THRESHOLD else "-- "
                yflag = "OK " if yr > THRESHOLD else "-- "
                detected = xr > THRESHOLD and yr > THRESHOLD
                if detected:
                    pos_count += 1
                mark = ">> BOLA <<" if detected else ""
                print(f"X={xr:5d} [{xflag}]   Y={yr:5d} [{yflag}]   {mark}")
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        print("\n──────────── RESUMO ────────────")
        print(f"Amostras           : {total}")
        print(f"Detectadas (POS)   : {pos_count}")
        print(f"X raw  min/max     : {x_min} / {x_max}")
        print(f"Y raw  min/max     : {y_min} / {y_max}")
        print(f"Threshold deteccao : {THRESHOLD}")
        print("\nDiagnostico:")
        if x_max <= THRESHOLD:
            print("  -> Eixo X NUNCA passou do threshold. X+ (GPIO9) / X- (GPIO5) suspeitos.")
        if y_max <= THRESHOLD:
            print("  -> Eixo Y NUNCA passou do threshold. Y+ (GPIO4) / Y- (GPIO8) suspeitos.")
        if x_max > THRESHOLD and y_max > THRESHOLD and pos_count == 0:
            print("  -> Cada eixo subiu, mas nunca ao mesmo tempo. Contato intermitente da bola.")
        if pos_count > 0:
            print("  -> Deteccao funcionando! Use o calibrate.py normalmente.")


if __name__ == "__main__":
    main()
