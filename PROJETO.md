# Ball Balancer — Descrição do Projeto

## O que é

Uma mesa de equilíbrio de bola controlada automaticamente.
Uma bola é colocada sobre uma plataforma plana. O sistema detecta a posição da bola
em tempo real e inclina a plataforma para manter a bola em um ponto alvo.

---

## Hardware

| Componente | Detalhe |
|---|---|
| Microcontrolador | ESP32-S3 DevKitC-1 |
| Sensor de posição | Tela resistiva 4 fios — VSDISPLAY VS084TP-A1 (8.4 pol) |
| Atuadores | 3 motores de passo NEMA 17 + drivers STEP/DIR (plataforma 3RPS) |
| Dimensões da tela | 187 mm × 141 mm |

### Ligação da tela resistiva (confirmada fisicamente)

| Fio FPC | Sinal | GPIO |
|---|---|---|
| PIN 1 | Y+ | 4 |
| PIN 2 | X- | 5 |
| PIN 3 | Y- | 8 |
| PIN 4 | X+ | 9 |

### Faixas ADC observadas (12 bits, 0–4095)

| Eixo | RAW mínimo | RAW máximo |
|---|---|---|
| X | ~2000 | ~3700 |
| Y | ~1700 | ~2600 |

Esses valores variam a cada reset do hardware — a calibração não pode ser
totalmente estática.

---

## O que o sistema precisa fazer

### 1. Detectar posição da bola
- A tela resistiva é o sensor de posição
- Deve retornar coordenadas X (0 a 187 mm) e Y (0 a 141 mm)
- Mover a bola somente em X não deve alterar Y, e vice-versa
- A leitura deve ser estável o suficiente para alimentar um PID

### 2. Controle PID
- Um PID independente por eixo (X e Y)
- Entrada: posição atual da bola (mm)
- Setpoint: posição alvo (mm) — centro da mesa por padrão
- Saída: sinal de inclinação para os servos

### 3. Acionar os motores de passo
- 3 motores NEMA 17 em arranjo 3RPS inclinam a plataforma
- A saída do PID (vetor normal de inclinação nx, ny) é convertida pela cinemática
  inversa no ângulo de cada perna, e daí em passos (STEP/DIR)

---

---

## Arquitetura do firmware

Dois modos selecionados em `main/main.c` pela macro `BB_MODE`
(padrão `BB_MODE_CONTROL`; use `idf.py build -DBB_MODE=BB_MODE_MAPPING` para o
modo de mapeamento antigo).

| Módulo | Responsabilidade |
|---|---|
| `touch_screen.c/.h` | Leitura da tela resistiva → posição filtrada em mm |
| `kinematics.c/.h` | Cinemática inversa 3RPS (`machine_theta`) |
| `pid.c/.h` | PID por eixo (derivada no erro + anti-windup) |
| `steppers.c/.h` | Gerador de passo p/ 3 NEMA 17 (STEP/DIR, `esp_timer` 10 kHz) |
| `control.c/.h` | Laço 50 Hz: `touch_read → PID(X,Y) → nx,ny → machine_theta → passos` |

### Pinos dos drivers de passo (verificar contra a fiação real)

| Sinal | GPIO |
|---|---|
| Motor A — STEP / DIR | 11 / 12 |
| Motor B — STEP / DIR | 13 / 14 |
| Motor C — STEP / DIR | 21 / 47 |
| ENABLE (comum, ativo LOW) | 10 |

### Parâmetros a calibrar no rig real (em `control.c`)

- `GEO_D/E/F/G/HZ` — comprimentos do manipulador 3RPS (hoje placeholders em mm)
- `MICROSTEPS` — microstepping dos drivers (padrão 16)
- `TILT_SIGN` — trocar para `-1.0f` se a mesa empurrar a bola para longe do alvo
- `KP/KI/KD` — ganhos do PID (ponto de partida: 8e-4 / 2e-5 / 1.2e-2)

### Protocolo serial

- `POS,x_raw,y_raw,x_mm,y_mm` / `NOTOUCH,x_raw,y_raw` (50 Hz)
- `CTRL,x,y,setx,sety,nx,ny,thA,thB,thC` — intenção do controlador (modo controle)

---

## Ferramentas (pasta `tools/`)

- `visualizer.py` — mapa 2D da posição da bola
- `calibrate.py` — calibração guiada, visualização ao vivo (LIVE) com ajuste de
  orientação (T/X/Y, gravar com G) **e** simulação dos motores:
  - na tela LIVE, tecla **`B`** abre o estado **BALANCE** — uma **vista
    isométrica** da mesa 3RPS onde a bola é **real** (lida da serial) e os **3
    motores NEMA 17 são simulados** reagindo a ela, fazendo o papel dos motores
    ainda não montados. O controlador (PID + cinemática) roda dentro da
    ferramenta, espelhando `kinematics.c`/`pid.c`.
  - movimento suave (filtro na posição da bola, *debounce* na detecção e
    interpolação da inclinação) — sem saltos bruscos nem flicker.

---

## O que já funciona

- Ligação física da tela resistiva confirmada por teste de continuidade
- Leitura raw do ADC funcionando em toda a área da tela (4 cantos + centro)
- GPIO9 não funciona como entrada ADC nesta placa — usado apenas como saída
- Faixas RAW aproximadas identificadas manualmente
- Cinemática inversa 3RPS, PID duplo, driver dos 3 NEMA 17 e laço de controle
  esboçados e compilando (modo `BB_MODE_CONTROL`)
- Simulador isométrico em malha fechada (`sim.py`) — estável em teste

## O que ainda precisa ser feito

- Calibração confiável: mapear raw → mm de forma estável entre resets
- Medir a geometria real (`GEO_*`) e definir `MICROSTEPS` / `TILT_SIGN`
- Tunar os ganhos do PID (primeiro no `sim`, depois no hardware)
- Validar a montagem dos drivers e a homing da plataforma no rig físico
