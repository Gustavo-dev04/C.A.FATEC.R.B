# Arquitetura do sistema

## Visão geral

Dois computadores com responsabilidades deliberadamente separadas:

```
┌──────────────────────────────────────────────────────────────────┐
│  JETSON ORIN NANO  —  "o cérebro"                                 │
│  Ubuntu + JetPack · Python · CUDA/TensorRT                        │
│                                                                    │
│   Câmera CSI ──▶ [ captura ] ──┬──▶ [ percepção ]                 │
│    (IMX219)       20-30 Hz     │      ├── faixa/rota (CNN)        │
│                                │      ├── placas   (MASTER)       │
│                                │      └── semáforo (largada)      │
│                                │                                   │
│                                └──▶ [ gravador de dataset ]        │
│                                        JPEG + JSONL                │
│                                                                    │
│                        [ planejamento + controle ]                 │
│                                    │                               │
│                             steer, throttle                        │
└────────────────────────────────────┼───────────────────────────────┘
                                     │ USB serial 115200 8N1
                                     │ $CMD  ▼      ▲  $TLM
┌────────────────────────────────────┼───────────────────────────────┐
│  ESP32  —  "a medula"              │                               │
│  Arduino/PlatformIO · tempo real                                   │
│                                                                    │
│   [ parser do protocolo ] ─▶ [ rampas + limites ] ─▶ servo direção │
│              │                                    └─▶ ESC / motor  │
│              ▼                                                     │
│   [ FAILSAFE 250 ms ]         [ ultrassônicos ×3 ] ──▶ telemetria  │
│   [ leitura de bateria ]      [ rádio RC (só treino) ]             │
└──────────────────────────────────────────────────────────────────┘
```

## Por que dividir assim

O Jetson roda Linux, que **não é sistema de tempo real**. Um pico de GC do
Python, um swap ou o próprio scheduler podem atrasar um loop em dezenas de
milissegundos. Isso é irrelevante para uma CNN a 20 Hz e é inaceitável para o
sinal PWM de um servo.

Então:

| Responsabilidade | Onde | Motivo |
|---|---|---|
| Visão, IA, decisão de rota | Jetson | Precisa de GPU; tolera jitter |
| Geração de PWM (servo/ESC) | ESP32 | Precisa de determinismo em µs |
| Failsafe de tração | ESP32 | **Tem que funcionar mesmo se o Jetson travar** |
| Leitura de ultrassônicos | ESP32 | Medição por tempo de eco, sensível a jitter |
| Gravação do dataset | Jetson | É onde estão a câmera e o disco |
| Rampas e limites de atuação | ESP32 | Última linha de defesa contra comando absurdo |

A regra que organiza tudo: **o ESP32 nunca confia no Jetson**. Se o comando
parar de chegar por 250 ms, ele mesmo põe o motor em neutro. Se o Jetson pedir
throttle 1.0, ele aplica o teto configurado. Se o ultrassônico frontal acusar
menos de 250 mm, ele corta a tração independentemente do que a IA queira.

## Fluxo de uma volta (categoria MASTER)

1. **Boot.** O Jetson sobe, carrega o modelo TensorRT, abre a serial, faz
   handshake `$PING/$PONG` com o ESP32 e envia a configuração (`$CFG`).
   LED de status: azul piscando = carregando, azul fixo = pronto.
2. **Armado.** O operador aperta o botão físico. `$ARM,1`. O carro **não se
   move** — só passa a aceitar comando. LED: amarelo.
3. **Semáforo.** O detector procura o semáforo (100 cm à frente da largada).
   Confirmado o verde em 3 quadros seguidos, o controle é liberado. LED: verde.
4. **Loop de condução**, a 20–30 Hz:
   - captura o frame;
   - recorta `roi_driving` → rede de direção → `steer`;
   - recorta `roi_signs` → detector de placas → atualiza a intenção de rota
     (seguir em frente / virar à esquerda / virar à direita);
   - funde com as distâncias dos ultrassônicos (freia/desvia);
   - envia `$CMD` ao ESP32.
5. **Fim da volta.** Detecção do box de chegada → `$CMD` neutro → `$ARM,0`.

Em JUNIOR o passo de placas some: a rota é conhecida e vira uma sequência fixa
de manobras por cruzamento.

## Camadas do código

```
jetson/robocar/
  comms/      protocolo + link serial (fronteira com o ESP32)
  sensors/    câmera (CSI/V4L2/arquivo)
  capture/    esquema do dataset + gravador de sessões
  control/    fontes de comando (RC, gamepad, teclado, modelo)
  util/       relógio monotônico, logging, git
```

Duas regras de dependência, para o código não virar um novelo:

1. `comms` **não importa** nada de `capture`, `sensors` ou `control`. É a
   camada mais baixa e precisa poder ser testada sozinha (e é: os testes do
   protocolo rodam sem hardware e sem dependências externas).
2. `capture` **não conhece** de onde vem o rótulo. Ele recebe um `Record`
   pronto. Isso é o que permite gravar dataset dirigindo por rádio RC, por
   gamepad ou em modo autônomo sem mudar uma linha do gravador.

## Estado atual do projeto

| Bloco | Estado |
|---|---|
| Protocolo serial Jetson↔ESP32 | ✅ especificado e implementado dos dois lados |
| Firmware ESP32 (PWM, failsafe, ultrassom, telemetria) | ✅ implementado |
| Captura de câmera (CSI/V4L2/arquivo) | ✅ implementado |
| Gravação de dataset (JPEG + JSONL) | ✅ implementado |
| Teleoperação (RC / gamepad / teclado) | ✅ implementado |
| Dataset loader + PilotNet + treino | ✅ baseline implementado |
| Detector de placas (MASTER) | ⬜ a fazer — depende de coletar placas |
| Detector de semáforo | ⬜ a fazer |
| Export TensorRT + loop autônomo | ⬜ a fazer |

A ordem não é acidental: **sem dado coletado, nada do que está em branco pode
ser feito de verdade.** Por isso a prioridade #1 é o pipeline de coleta.

## Decisões registradas

- [ADR 0001 — Jetson como cérebro, ESP32 como medula](adr/0001-jetson-como-cerebro-esp32-como-medula.md)
- [ADR 0002 — Rótulos vêm da telemetria, não do comando](adr/0002-rotulos-vem-da-telemetria.md)
