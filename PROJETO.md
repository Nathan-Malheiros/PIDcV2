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
| Atuadores | 3 servos motores (plataforma 3RPS) |
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

### 3. Acionar os servos
- 3 servos em arranjo 3RPS inclinam a plataforma
- A saída do PID define o ângulo de inclinação de cada servo

---

## O que já funciona

- Ligação física da tela resistiva confirmada por teste de continuidade
- Leitura raw do ADC funcionando em toda a área da tela (4 cantos + centro)
- GPIO9 não funciona como entrada ADC nesta placa — usado apenas como saída
- Faixas RAW aproximadas identificadas manualmente

## O que ainda precisa ser feito

- Calibração confiável: mapear raw → mm de forma estável entre resets
- Leitura de posição independente por eixo (X não afeta Y)
- Filtragem adequada para controle PID
- Implementação do PID duplo (X e Y)
- Implementação do controle dos 3 servos
