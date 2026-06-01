# Ball Balancer — Documentação do Projeto

Mesa equilibrista controlada por PID duplo (X e Y) com cinemática inversa 3RPS.
A bola é detectada por uma tela resistiva; três motores NEMA 17 inclinam a plataforma
para mantê-la no centro.

---

## Hardware

| Componente | Detalhe |
|---|---|
| Microcontrolador | ESP32-S3 DevKitC-1 |
| Sensor de posição | Tela resistiva 4 fios — VSDISPLAY VS084TP-A1 (8,4 pol) |
| Dimensões da tela | 187 mm × 141 mm (largura × altura) |
| Atuadores | 3 motores de passo NEMA 17 + drivers STEP/DIR (arranjo 3RPS) |
| Drivers de passo | A4988 / DRV8825 / TMC2208 (interface STEP/DIR idêntica) |

### Pinos — Tela resistiva

Cabo flat com o conector voltado para a **direita** (olhando a superfície da mesa).

| Pino FPC | Sinal | GPIO | Canal ADC |
|---|---|---|---|
| 1 | Y+ | 4 | ADC1_CH3 (sense de X) |
| 2 | X- | 5 | ADC1_CH4 (sense de Y) |
| 3 | Y- | 8 | — (saída digital) |
| 4 | X+ | 9 | Não funciona como ADC nesta placa — somente saída |

> **Nota:** GPIO 9 não tem ADC funcional neste módulo. É usado apenas como saída
> digital para polarizar o strip X. Apenas GPIO 4 e GPIO 5 são lidos pelo ADC.

### Pinos — Drivers de passo

| Motor | STEP | DIR |
|---|---|---|
| A | 15 | 16 |
| B | 17 | 18 |
| C | 40 | 41 |
| ENABLE (comum, ativo LOW) | 10 | — |

### Orientação dos motores e cantos da mesa

```
         [cabo FPC aqui]
   TL ──────────────── TR
   │                    │
   │       MESA         │
   │     (superfície)   │
   BL ──────────────── BR

   TL = Superior Esquerdo   TR = Superior Direito
   BL = Inferior Esquerdo   BR = Inferior Direito
```

Convenção de coordenadas após calibração:
- `x_mm = 0` → borda esquerda · `x_mm = 187` → borda direita
- `y_mm = 0` → borda superior · `y_mm = 141` → borda inferior

---

## Arquitetura do firmware

### Modos de operação (`main/main.c`)

| Macro `BB_MODE` | Comportamento |
|---|---|
| `BB_MODE_CONTROL` (padrão) | Laço PID 50 Hz + cinemática + motores |
| `BB_MODE_MAPPING` | Só lê o touch e emite telemetria — sem motores |

Para compilar em modo mapeamento: `idf.py build -DBB_MODE=BB_MODE_MAPPING`

### Módulos do firmware

| Arquivo | Responsabilidade |
|---|---|
| `main.c` | Entry point — seleciona modo e inicializa saída serial sem buffer |
| `touch_screen.c/.h` | Leitura 4 fios, ADC, filtro IIR, calibração **em runtime** |
| `kinematics.c/.h` | Cinemática inversa 3RPS — porte fiel de Aaed Musa |
| `pid.c/.h` | PID por eixo: derivada no erro, filtro LPF, anti-windup back-calculation |
| `steppers.c/.h` | Gerador de passo STEP/DIR com perfil trapezoidal via `esp_timer` 10 kHz |
| `control.c/.h` | Laço de controle 50 Hz + parser de comandos seriais |
| `cal_store.c/.h` | Persistência NVS — salva/carrega ganhos PID e calibração do touch |

### Fluxo do laço de controle (50 Hz)

```
touch_read()  →  PID_X(x_mm)  →  nx
              →  PID_Y(y_mm)  →  ny
                                   ↓
              machine_theta(leg, hz, nx, ny)  × 3 pernas
                                   ↓
              (ang_orig − theta) × ANG_TO_STEP  →  steppers_move_to_all()
```

### Parâmetros de geometria (`control.c`)

Valores atuais são **placeholders** baseados no projeto de referência de Aaed Musa.
**Medir no rig real e substituir.**

| Constante | Valor atual | Significado |
|---|---|---|
| `GEO_D` | 50,8 mm | Raio da base (pivôs inferiores) |
| `GEO_E` | 79,4 mm | Raio da plataforma (juntas esféricas) |
| `GEO_F` | 44,45 mm | Comprimento do braço do motor |
| `GEO_G` | 93,2 mm | Comprimento da biela |
| `GEO_HZ` | 108,0 mm | Altura neutra da plataforma |
| `MICROSTEPS` | 16 | Microstepping dos drivers (DIP switches) |

### Ganhos PID padrão (`control.c`)

O erro está em **mm** e o PID é normalizado por `dt` — os valores têm escalas
diferentes de implementações Arduino que usam unidades brutas do ADC.

| Ganho | Valor padrão | Observação |
|---|---|---|
| `KP` | `8,0 × 10⁻⁴` | Proporcional |
| `KI` | `2,0 × 10⁻⁵` | Integral (anti-windup ativo) |
| `KD` | `1,2 × 10⁻²` | Derivativo (LPF com `d_tau = 0,03 s`) |

Ajuste em tempo real pela serial ou pelo `pid_tuner.py`. Valores salvos no NVS
persistem entre boots.

Método de tuning sugerido:
1. Suba `KP` até a bola oscilar em torno do centro
2. Adicione `KD` para amortecer
3. Adicione um toque de `KI` para eliminar offset residual

### Velocidade e aceleração dos steppers

| Parâmetro | Equilíbrio | Homing |
|---|---|---|
| Velocidade máxima | 1500 steps/s | 800 steps/s |
| Aceleração | 20 000 steps/s² | 4000 steps/s² |

> O perfil é **trapezoidal** (aceleração limitada): `v_lim = √(2·a·d)`.
> Sem rampa → perda de passos. O timer roda a 10 kHz.

---

## Calibração do touch e persistência NVS

### Como funciona a calibração

A conversão `raw → mm` usa variáveis em **runtime** (não `#define`):

```
x_norm = (x_raw − x_raw_min) / (x_raw_max − x_raw_min)
y_norm = (y_raw − y_raw_min) / (y_raw_max − y_raw_min)
→ aplica swap_xy / flip_x / flip_y
→ x_mm = x_norm × 187 mm
```

Os defaults de compilação em `touch_screen.h` (`TOUCH_X_RAW_MIN` etc.) são usados
**somente se** o NVS não tiver uma calibração salva.

### NVS — persistência automática

O namespace NVS é `bb_cal`. Dois blobs são armazenados:

| Chave | Conteúdo |
|---|---|
| `pid` | `float kp, ki, kd` — ganhos PID |
| `touch` | `int32 x_min/x_max/y_min/y_max` + `int8 flip_x/flip_y/swap_xy` |

Na inicialização do firmware (`control_run`):
1. Tenta carregar `pid` do NVS → usa se encontrar, caso contrário usa os `#define`
2. Tenta carregar `touch` do NVS → aplica via `touch_set_cal()` se encontrar

**Os valores salvos persistem para sempre** (até serem sobrescritos ou a flash
ser apagada com `idf.py erase-flash`).

---

## Protocolo serial (UART0, 115200 baud)

### Saída do firmware (50 Hz)

```
POS,<x_raw>,<y_raw>,<x_mm>,<y_mm>         — bola detectada
NOTOUCH,<x_raw>,<y_raw>                    — sem contato
CTRL,<x>,<y>,<setx>,<sety>,<nx>,<ny>,<thA>,<thB>,<thC>
GAINS,<kp>,<ki>,<kd>                       — após qualquer ajuste de ganho
CAL,<xmin>,<xmax>,<ymin>,<ymax>,<fx>,<fy>,<sw>  — após ajuste de calibração
CORNER,<TL|TR|BL|BR>,<x_raw>,<y_raw>       — confirmação de canto capturado
SAVED,<pid|touch>                           — confirmação de NVS salvo
```

### Comandos de entrada (serial ou ferramenta Python)

#### Ganhos PID

| Comando | Efeito |
|---|---|
| `PID <kp> <ki> <kd>` | Ajusta os três ganhos (ambos os eixos) |
| `KP <v>` / `KI <v>` / `KD <v>` | Ajusta um ganho por vez |
| `PID SAVE` | Salva ganhos atuais no NVS |
| `?` | Exibe ganhos, sinais e calibração atual |

#### Sinal de controle por eixo

| Comando | Efeito |
|---|---|
| `SX <v>` | Sinal do eixo X (`v < 0` → `-1`, `v ≥ 0` → `+1`) |
| `SY <v>` | Sinal do eixo Y |

Se a mesa empurrar a bola **para longe** do centro num eixo, inverta o sinal daquele eixo.

#### Calibração do touch

| Comando | Efeito |
|---|---|
| `CAL TL` | Captura posição atual como canto Superior Esquerdo |
| `CAL TR` | Captura posição atual como canto Superior Direito |
| `CAL BL` | Captura posição atual como canto Inferior Esquerdo |
| `CAL BR` | Captura posição atual como canto Inferior Direito |
| `CAL APPLY` | Calcula e aplica calibração a partir dos 4 cantos coletados |
| `CAL SAVE` | Salva calibração atual no NVS |
| `CAL SHOW` | Exibe calibração em vigor |
| `CAL RESET` | Restaura os defaults de compilação |
| `SETCAL <xmin> <xmax> <ymin> <ymax> <fx> <fy> <sw>` | Aplica calibração calculada externamente (usado pelo `calibrate.py`) |

> Para `CAL TL/TR/BL/BR`: a bola deve estar fisicamente no canto no momento do envio.
> `SETCAL` não requer isso — recebe os valores calculados pela ferramenta Python.

---

## Ferramentas (`tools/`)

### Lançamento rápido — `bb_firmware_tool.bat`

Menu principal com todas as operações. Requer perfil ESP-IDF carregado.

| Opção | O que faz |
|---|---|
| 1 | Build |
| 2 | Full clean + rebuild |
| 3 | Flash |
| 4 | Monitor serial |
| 5 | Flash + Monitor |
| 6 | Detectar ESP32 na USB |
| 7 | Diagnósticos do projeto |
| **8** | **Calibração da tela (calibrate.py)** |
| 9 | Visualizador Python |
| 10 | Menuconfig |
| 11 | Apagar flash (PERIGO — apaga NVS também) |
| 12 | Definir porta COM |
| **13** | **Ajuste PID em tempo real (pid_tuner.py)** |

### `calibrate.py` — Calibração guiada com GUI Pygame

Coleta 80 amostras com mediana em cada um dos 4 cantos físicos. Uso:
```
python calibrate.py COM5
```

| Tecla / Estado | Ação |
|---|---|
| `ENTER` na splash | Inicia calibração |
| `ESPAÇO` | Inicia/confirma coleta do canto atual |
| `S` em CAL_RESULTS | Salva em `touch_screen.h` (exige rebuild + flash) |
| **`N` em CAL_RESULTS** | **Envia direto ao NVS do ESP32 via `SETCAL` (sem rebuild)** |
| `ENTER` em CAL_RESULTS | Vai para visualização ao vivo (LIVE) |
| `T` / `X` / `Y` em LIVE | Alterna swap / flip_x / flip_y |
| `G` em LIVE | Grava orientação em `config.py` e `touch_screen.h` |
| `B` em LIVE | Abre simulação isométrica dos motores (BALANCE) |
| `ESC` | Sair |

> Requer firmware em **modo CONTROL** para o `N` funcionar (cmd_task ativo).
> Em modo MAPPING, use `S` e depois rebuild.

### `pid_tuner.py` — Ajuste PID em tempo real com GUI Pygame

Mostra a posição da bola ao vivo + histórico de erro. Permite ajustar KP/KI/KD
com teclas e salvar no NVS sem parar o firmware. Uso:
```
python pid_tuner.py COM5
```

| Tecla | Ação |
|---|---|
| `Q` / `A` | KP sobe / desce |
| `W` / `S` | KI sobe / desce |
| `E` / `D` | KD sobe / desce |
| `Tab` | Muda passo de ajuste: 1% → 5% → 10% → 25% |
| `ESPAÇO` | **Salva ganhos no NVS** (`PID SAVE`) |
| `R` | Reenvia os ganhos atuais (reseta integrador) |
| `ESC` | Sair |

> Requer firmware em **modo CONTROL**. Ao conectar, solicita `?` ao firmware
> para sincronizar os ganhos exibidos com os valores em vigor.

### `diag.py` — Diagnóstico de leitura crua (terminal)

Exibe X raw e Y raw ao vivo e guarda o range observado. Útil para verificar
se ambos os eixos reagem antes de calibrar. Uso:
```
python diag.py COM5
```

### `visualizer.py` — Visualizador 2D da posição

Plota a posição da bola em tempo real sobre o mapa da mesa.

### `config.py` — Configuração central das ferramentas

Porta COM, dimensões da mesa e orientação dos eixos (SWAP_XY, FLIP_X, FLIP_Y).
Editado automaticamente pelo `calibrate.py` ao pressionar `G`.

---

## Fluxo de trabalho completo

### Primeira configuração (hardware novo)

```
1. bb_firmware_tool.bat → opção 1 (Build)
2. bb_firmware_tool.bat → opção 5 (Flash + Monitor)
   Verificar: "Ball Balancer — Control Mode @ 50 Hz" no terminal
3. bb_firmware_tool.bat → opção 8 (calibrate.py)
   → Coleta os 4 cantos → pressiona N → "✓ Calibração salva no NVS"
4. bb_firmware_tool.bat → opção 13 (pid_tuner.py)
   → Ajusta KP/KI/KD → pressiona ESPAÇO → "✓ Ganhos salvos no NVS"
```

### Boots subsequentes

O firmware carrega automaticamente a calibração e os ganhos do NVS.
**Não é necessário recalibrar.** Só refaça se alterar a montagem física.

### Quando recalibrar

| Situação | Ação |
|---|---|
| Mudou a posição/orientação da tela | Refazer opção 8 → pressionar N |
| Quer ajustar os ganhos PID | Abrir opção 13, ajustar, ESPAÇO para salvar |
| Mesa empurra bola para longe | Enviar `SX -1` ou `SY -1` pela serial |
| Apagou a flash (`erase-flash`) | Refazer calibração e PID do zero |

---

## Estrutura de arquivos

```
PIDcV4/
├── CMakeLists.txt              Projeto ESP-IDF (target esp32s3)
├── PROJETO.md                  Este arquivo
├── main/
│   ├── CMakeLists.txt          Lista de fontes + dependências IDF
│   ├── main.c                  Entry point, seleção de modo
│   ├── touch_screen.h/.c       Leitura resistiva + calibração runtime
│   ├── kinematics.h/.c         Cinemática inversa 3RPS
│   ├── pid.h/.c                Controlador PID duplo
│   ├── steppers.h/.c           Driver STEP/DIR com perfil trapezoidal
│   ├── control.h/.c            Laço 50 Hz + comandos seriais
│   └── cal_store.h/.c          Persistência NVS (PID + touch)
└── tools/
    ├── bb_firmware_tool.bat    Lançador (carrega ESP-IDF + abre .ps1)
    ├── bb_firmware_tool.ps1    Menu principal (build/flash/ferramentas)
    ├── config.py               Configuração central das ferramentas
    ├── calibrate.py            GUI Pygame — calibração 4 cantos + envio NVS
    ├── pid_tuner.py            GUI Pygame — ajuste PID em tempo real
    ├── visualizer.py           Visualizador 2D da posição da bola
    ├── diag.py                 Diagnóstico de leitura raw (terminal)
    └── requirements.txt        pygame >= 2.0.0, pyserial >= 3.5
```

---

## Pendências / próximos passos

| Item | Status |
|---|---|
| Medir geometria real (`GEO_D/E/F/G/HZ`) | Pendente |
| Confirmar `MICROSTEPS` nos DIP switches dos drivers | Pendente |
| Montar o rig e validar homing (posição 0 = mesa nivelada) | Pendente |
| Tunar KP/KI/KD no hardware com `pid_tuner.py` | Pendente |
| Verificar `SX`/`SY` — se a mesa corrige ou piora | Pendente |
| Medir e ajustar `GEO_HZ` (altura real da plataforma) | Pendente |
