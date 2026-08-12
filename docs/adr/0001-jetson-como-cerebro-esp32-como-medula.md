# ADR 0001 — Jetson como cérebro, ESP32 como medula

- **Status:** aceito
- **Data:** 2026-08-12

## Contexto

Precisamos rodar uma CNN de visão a ~20 Hz e, ao mesmo tempo, gerar PWM estável
para servo de direção e ESC. Temos um Jetson Orin Nano (Linux + GPU) e um ESP32
(microcontrolador).

Três arranjos possíveis:

1. **Só Jetson**, gerando PWM pelo GPIO/pinos de hardware.
2. **Só ESP32**, com visão simplificada por processamento clássico.
3. **Os dois**, com papéis separados.

## Decisão

Arranjo 3. Jetson faz percepção e decisão; ESP32 faz atuação, sensores de
tempo crítico e failsafe.

## Justificativa

**Contra o arranjo 1:** Linux não é tempo real. Um atraso de 20 ms no
agendamento do processo Python vira jitter no pulso do servo, e o servo
responde com trepidação. Pior: se o processo Python morrer, **nada** coloca o
motor em neutro — o carro sai andando sozinho até bater. Um failsafe que depende
do software que pode falhar não é failsafe.

**Contra o arranjo 2:** o regulamento do MASTER exige ler placas de trânsito de
150 mm a alguns metros de distância e interpretar setas. Isso é um problema de
CNN, não de threshold de cor. O ESP32 não tem folga computacional para isso, e
descartar a GPU do Jetson seria desperdiçar o principal ativo do time.

**A favor do 3:** cada processador faz o que sabe fazer. O ganho decisivo é que
o failsafe passa a viver em um processador que **não roda nada complexo** — o
loop do ESP32 tem poucas centenas de linhas, é auditável inteiro e continua
funcionando mesmo com o Jetson travado, em pânico de kernel ou rebootando.

## Consequências

**Positivas**
- Failsafe independente do subsistema mais propenso a falhar.
- Limites de velocidade e obstáculo aplicados em código simples e auditável.
- Dá para calibrar direção e testar atuação sem nada de IA carregado.
- O ESP32 fornece os rótulos "aplicados" que o dataset precisa (ADR 0002).

**Negativas**
- Um protocolo a manter e manter em sincronia nos dois lados
  (`docs/03-protocolo-serial.md` é a defesa contra isso).
- Latência extra de ~1–2 ms na serial. Irrelevante frente aos ~50 ms do
  pipeline de visão.
- Dois firmwares/softwares para atualizar e versionar juntos.

## Alternativa mantida em observação

Se a latência USB se mostrar instável no dia (cabo, ruído do motor), o plano B
é migrar para UART pelos pinos GPIO do Jetson, sem o stack USB no meio. O
protocolo não muda — só a camada física.
