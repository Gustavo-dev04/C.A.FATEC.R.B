# ADR 005 — MPU6050 no simulador; rumo pelo giroscópio, calibrado no vermelho

**Data:** 02/11/2026 · **Estado:** aceito

## Contexto

A IMU do carro é a **MPU6050** (confirmado pela equipe; o REV03 listava
BNO085). Ela não tem magnetômetro nem fusão pronta: o rumo sai da integração
do giroscópio z. O datasheet avisa: viés de fábrica de até **±20 °/s** e fator
de escala com **±3 %**.

Sem IMU, a odometria só por encoders erra o rumo com raio de roda diferente e
patinação. Dentro do cruzamento não há faixa para a câmera seguir — é onde o
rumo da odometria mais importa.

## Decisão

1. `vista_sim/imu.py::Mpu6050` simula o sensor a 200 Hz (um por passo de
   física): viés sorteado em ±20 °/s, escala em ±3 %, ruído do datasheet
   (0,005 °/s/√Hz com filtro de 98 Hz ≈ 0,05 °/s RMS), viés que anda ~0,05 °/s
   numa prova (estimativa), ADC de 16 bits e saturação em ±250 °/s.
   Acelerômetro com viés, ruído e braço de alavanca (a IMU fica 21,7 cm à
   frente do eixo).
2. `vista_sim/imu.py::WheelGyroOdometry` é a lógica que vai para o carro:
   - **parado** (nenhum tick há 0,3 s): não integra o rumo — carro de tração
     diferencial não gira sem girar as rodas — e promedia o giroscópio para
     estimar o viés. A espera no vermelho (2–9 s) é a janela natural;
   - **andando**: deslocamento pelos encoders, rumo pelo giroscópio menos o
     viés estimado.
3. A telemetria leva a leitura **bruta** (`gyro_z`, `accel`), o viés estimado,
   se já calibrou, e a odometria combinada. A verdade leva o viés real.

## Medido (40 rotas sorteadas, oráculo, carro e IMU sorteados)

| | só encoders | encoders + giro | + escala calibrada |
|---|---|---|---|
| erro de posição no fim (mediana) | 1,84 m | **0,51 m** | **0,19 m** |
| erro de posição no fim (pior) | 9,31 m | 1,35 m | 0,77 m |
| maior erro de rumo na rota (mediana) | 24,5° | 5,9° | 2,2° |
| maior erro de rumo na rota (pior) | 108,7° | 39,3° | 9,3° |

Viés estimado no vermelho: erro < 0,02 °/s (teste
`test_parado_no_vermelho_calibra_o_vies_do_giroscopio`).

## Consequências

- O firmware/Vista deve **calibrar o viés toda largada** — sem isso, até 20°
  de erro por segundo.
- A calibração parada **não** corrige o fator de escala. Ele é o maior erro que
  sobra (±3 % de cada curva). Calibrar no carro: girar no lugar N voltas
  inteiras e comparar com 360°·N. Com isso o pior erro de rumo cai de 39° para
  9°.
- O viés que anda durante a prova (`gyro_bias_drift_dps`) e a posição da IMU
  são estimativas — medir no carro.
- Não muda a imagem nem o movimento do carro: só a telemetria. Custo medido:
  0,09 ms por quadro (7 amostras), ~2 % do `env.step`.
