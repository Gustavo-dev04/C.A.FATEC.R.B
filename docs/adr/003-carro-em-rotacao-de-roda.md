# ADR 003 — O carro do simulador é comandado em rotação de roda

**Data:** 02/11/2026 · **Estado:** aceito

## Contexto

O carro não tem direção: curva pela diferença de rotação entre as duas rodas.
A primeira versão do simulador recebia `(v, ω)` e tratava cada roda como um
filtro de 1ª ordem em velocidade de superfície. Isso esconde o que o ESP32
realmente faz e o que dá errado no carro de verdade:

- o ESP32 recebe **rpm alvo por roda** e fecha um **PID por roda** com o encoder;
- o motor tem **zona morta** — duty pequeno não gira a roda;
- o encoder é **quantizado** — o PID e a odometria só veem ticks inteiros;
- as rodas **não são iguais** (tolerância de raio) — rotação igual não é reta;
- a roda **patina** — os ticks contam rotação, não chão.

## Decisão

1. `env.step(rpm_esquerda, rpm_direita)` é a interface principal.
   `env.step_vw(v, ω)` existe só como atalho e converte antes:
   `ω_E,D = (v ∓ ω·b/2) / r`, com o raio **nominal** (o ESP32 não sabe que uma
   roda é 0,5 % maior).
2. `vista_sim/car.py` simula a cadeia inteira por roda: PID a 100 Hz com
   feedforward e anti-windup → duty → motor (zona morta + 1ª ordem) → rotação
   → encoder (440 ticks/volta) → rpm medido numa janela de 20 ms.
3. Saturação: se uma roda passaria do limite, **as duas** são escaladas juntas —
   preserva a curvatura pedida.
4. A cada episódio (`randomize_car`) sorteia raio de cada roda (±0,5 %) e
   patinação (0–2 %, igual nas duas).
5. **O ponto que segue a faixa é o centro do carro**, 14 cm à frente do eixo,
   não o eixo. Varrendo o contorno do carro por todas as 96 conversões da
   malha:

   | ponto na trilha | conversões em que o carro sai do asfalto |
   |---|---|
   | eixo (0 cm) | 4 — a frente, 34 cm adiante, balança para fora |
   | 10 cm | 2 |
   | **centro (14 cm)** | **0** |
   | 18 cm | 0 |
   | câmera (25 cm) | 12 — a traseira corta a quina |

   O oráculo controla esse ponto por linearização (ponto fora do eixo), e a
   verdade de campo (`e_y`, `e_ψ`) é medida nele. Teste:
   `tests/test_cenario.py::test_carro_cabe_em_todas_as_conversoes_com_o_centro_na_faixa`.

## Consequências

- O Vista tem que comandar rotação de roda — igual ao carro real. A política
  de controle aprendida no simulador vale na pista sem trocar de interface.
- Com uma roda 0,5 % maior e rpm iguais, o carro desvia **29 cm em 5 m**
  (teste `test_raios_diferentes_puxam_para_um_lado`): andar reto é tarefa da
  **câmera**, não de "rpm iguais".
- Patinação e tolerância fazem a odometria divergir da verdade, como no carro.
- Os números de motor (`deadband_duty`, `mechanical_time_constant`, ganhos do
  PID) são **estimados**. Medir no carro (S4) e atualizar `config/car.yaml`.
- **Pergunta em aberto:** o firmware do ESP32 vai receber rpm por roda (como
  aqui) ou `(v, ω)`? Se for `(v, ω)`, a conversão acima vai para o firmware e
  nada muda no simulador.
