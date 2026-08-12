# Protocolo serial Jetson ↔ ESP32 — v1

Especificação normativa. As duas implementações **precisam** casar com este
documento:

- Python: `jetson/robocar/comms/protocol.py`
- C++: `firmware/esp32/src/protocol.cpp`

Os testes em `jetson/tests/test_protocol.py` verificam o lado Python contra os
exemplos desta página. Mudou o protocolo? Suba a versão, atualize os dois lados
e os testes na **mesma** alteração.

## Camada física

| Parâmetro | Valor |
|---|---|
| Meio | USB CDC (UART0 do ESP32) |
| Baud rate | 115200 |
| Formato | 8N1, sem controle de fluxo |
| Codificação | ASCII, uma mensagem por linha |
| Terminador | `\n` (LF) |

Texto e não binário por escolha consciente: dá para depurar com um monitor
serial comum, sem ferramenta especial. A 115200 baud, um quadro `$TLM` de ~70
bytes leva ~6 ms — folgado para 50 Hz nas duas direções somadas.

## Formato do quadro

```
$<TIPO>,<campo1>,<campo2>,...,<campoN>*<CS>\n
```

- `$` — início. Qualquer lixo antes do `$` é descartado silenciosamente
  (importante: o ESP32 cospe mensagens de boot antes do firmware assumir).
- `<TIPO>` — 3–4 letras maiúsculas.
- Campos separados por vírgula. **Sem espaços.**
- `*` — fim do payload.
- `<CS>` — checksum, **2 dígitos hexadecimais maiúsculos**.
- Tamanho máximo: **128 bytes**, incluindo `\n`.

### Checksum

XOR de todos os bytes **entre** `$` e `*`, exclusivos.

```python
cs = 0
for b in payload.encode("ascii"):
    cs ^= b
f"{cs:02X}"
```

Mesma ideia do NMEA 0183. Pega corrupção de byte e quadro truncado, que é o
que acontece na prática com cabo USB ruim perto de um motor.

### Números

- `steer` e `throttle`: float normalizado em `[-1.0, +1.0]`, **4 casas
  decimais**, ponto como separador (`-0.3125`).
- Distâncias: inteiro em **milímetros**. `0` = sem eco / fora de alcance.
- Tensão: float com **2 casas**, em volts.
- Tempos: inteiro em **milissegundos**.
- `seq`: inteiro `0..65535`, com wrap.

Convenções de sinal, uma vez só e para sempre:

| Grandeza | −1.0 | 0.0 | +1.0 |
|---|---|---|---|
| `steer` | esquerda máxima | reto | **direita máxima** |
| `throttle` | ré máxima | parado | frente máxima |

---

## Mensagens Jetson → ESP32

### `$CMD` — comando de atuação

```
$CMD,<seq>,<steer>,<throttle>,<mode>*CS
```

| Campo | Tipo | Descrição |
|---|---|---|
| `seq` | uint16 | Sequencial, incrementa a cada envio |
| `steer` | float | `[-1, +1]` |
| `throttle` | float | `[-1, +1]` |
| `mode` | uint8 | `0`=IDLE `1`=TELEOP `2`=AUTO |

Enviado a **50 Hz**. É também o batimento cardíaco: parar de enviar dispara o
failsafe. Exemplo:

```
$CMD,142,-0.3125,0.2000,2*65
```

### `$ARM` — armar / desarmar

```
$ARM,<0|1>*CS
```

Desarmado, o ESP32 ignora o throttle de qualquer `$CMD` e mantém o motor em
neutro. A direção continua respondendo (útil para calibrar no cavalete).
**Armar é sempre uma ação humana explícita.**

### `$CFG` — configuração em runtime

```
$CFG,<chave>,<valor>*CS
```

Enviado no handshake, a partir de `config/vehicle.yaml`. Evita recompilar o
firmware a cada calibração.

| Chave | Unidade | Exemplo |
|---|---|---|
| `steer_center_us` | µs | `1500` |
| `steer_min_us` | µs | `1150` |
| `steer_max_us` | µs | `1850` |
| `steer_invert` | 0/1 | `0` |
| `thr_neutral_us` | µs | `1500` |
| `thr_min_us` | µs | `1000` |
| `thr_max_us` | µs | `2000` |
| `thr_limit_fwd` | 0..1 | `0.3000` |
| `thr_limit_rev` | 0..1 | `0.2000` |
| `thr_deadband` | 0..1 | `0.0500` |
| `slew_steer` | 1/s | `6.0000` |
| `slew_thr` | 1/s | `2.0000` |
| `timeout_ms` | ms | `250` |
| `obst_stop_mm` | mm | `250` |
| `obst_slow_mm` | mm | `600` |
| `rc_enable` | 0/1 | `0` |
| `tlm_hz` | Hz | `50` |

> **Limites de segurança só apertam, nunca afrouxam.** O firmware aplica
> `min(valor_recebido, teto_compilado)` em `thr_limit_fwd`, `thr_limit_rev` e
> `timeout_ms`. Um bug no Jetson não consegue liberar o carro para além do que
> o firmware permite.

Resposta: `$ACK,CFG,<chave>*CS` ou `$ERR,<código>,<chave>*CS`.

### `$PING` — teste de enlace

```
$PING,<seq>*CS
```

Resposta imediata: `$PONG,<seq>,<t_ms>*CS`. Usado por `robocar link --check`
para medir latência ida-e-volta.

---

## Mensagens ESP32 → Jetson

### `$TLM` — telemetria (a mensagem mais importante do sistema)

```
$TLM,<seq>,<t_ms>,<steer>,<throttle>,<src>,<d0>,<d1>,<d2>,<vbat>,<flags>*CS
```

| Campo | Tipo | Descrição |
|---|---|---|
| `seq` | uint16 | Eco do último `$CMD` aceito |
| `t_ms` | uint32 | `millis()` do ESP32 no instante da amostra |
| `steer` | float | Esterço **realmente aplicado** (pós-rampa e limites) |
| `throttle` | float | Aceleração **realmente aplicada** |
| `src` | uint8 | Origem: `0`=failsafe `1`=serial `2`=rádio RC |
| `d0,d1,d2` | uint16 | Distâncias em mm (esq., centro, dir.); `0` = inválido |
| `vbat` | float | Tensão da bateria de tração |
| `flags` | uint8 hex | Bitfield (abaixo) |

Emitida a **50 Hz** sem precisar de pedido.

`steer`/`throttle` são o **valor aplicado, não o comandado** — é o que torna a
telemetria a fonte correta de rótulo para o dataset. Ver
[ADR 0002](adr/0002-rotulos-vem-da-telemetria.md).

Exemplo:

```
$TLM,142,84213,-0.3000,0.2000,1,1230,890,1540,7.92,01*61
```

### Bitfield `flags`

| Bit | Máscara | Nome | Significado |
|---|---|---|---|
| 0 | `0x01` | `ARMED` | Tração liberada |
| 1 | `0x02` | `FAILSAFE` | Sem comando há mais de `timeout_ms` |
| 2 | `0x04` | `RC_VALID` | Rádio RC com sinal válido |
| 3 | `0x08` | `OBSTACLE_STOP` | Obstáculo cortou a tração |
| 4 | `0x10` | `OBSTACLE_SLOW` | Obstáculo limitou a tração |
| 5 | `0x20` | `BATT_LOW` | Tensão abaixo de `battery_min_v` |
| 6 | `0x40` | `BATT_CRITICAL` | Tensão crítica — tração cortada |
| 7 | `0x80` | `CFG_PENDING` | Ainda sem `$CFG` — usando padrões |

**O gravador de dataset descarta frames com `FAILSAFE`, `OBSTACLE_STOP` ou
`BATT_CRITICAL` ativos.** Nesses instantes o comando aplicado não reflete a
intenção do piloto e ensinaria a coisa errada à rede.

### `$LOG` — texto livre para depuração

```
$LOG,<nível>,<texto>*CS
```

Nível: `D`,`I`,`W`,`E`. Vírgulas e `*` no texto devem ser removidos pelo
emissor. Nunca envie `$LOG` no caminho crítico a 50 Hz.

### `$ACK` / `$ERR`

```
$ACK,<tipo>,<detalhe>*CS
$ERR,<código>,<detalhe>*CS
```

| Código | Significado |
|---|---|
| `1` | Checksum inválido |
| `2` | Tipo desconhecido |
| `3` | Número errado de campos |
| `4` | Valor fora de faixa |
| `5` | Chave de `$CFG` desconhecida |
| `6` | Quadro grande demais |

---

## Máquina de estados do failsafe (ESP32)

```
              $CMD recebido
    ┌────────────────────────────────┐
    ▼                                │
┌────────┐  sem $CMD por          ┌──┴──────┐
│ ATIVO  │  timeout_ms (250)      │ FAILSAFE│
│        ├───────────────────────▶│         │
│ aplica │                        │ throttle│
│ comando│◀───────────────────────┤ = neutro│
└────────┘   $CMD volta a chegar  └─────────┘
```

Regras invioláveis do failsafe:

1. Ao entrar, `throttle` vai para o neutro **imediatamente**, sem rampa.
2. `steer` **mantém o último valor**. Travar a direção no centro a 2 m/s
   arremessa o carro para fora da pista; manter o esterço deixa ele desacelerar
   na trajetória em que já estava.
3. Sair do failsafe **não** rearma sozinho o throttle: a rampa
   (`slew_thr`) segura a retomada, evitando arranque brusco quando a serial
   se restabelece.
4. O failsafe é **independente do estado de armado**: desarmado também
   significa motor em neutro.

## Handshake de inicialização

```
Jetson                             ESP32
  │                                  │
  │──── $PING,0 ────────────────────▶│
  │◀─── $PONG,0,1234 ────────────────│   enlace ok
  │                                  │
  │──── $CFG,steer_center_us,1500 ──▶│
  │◀─── $ACK,CFG,steer_center_us ────│   (repete p/ cada chave)
  │                                  │
  │──── $ARM,0 ─────────────────────▶│   garante desarmado
  │◀─── $ACK,ARM,0 ──────────────────│
  │                                  │
  │◀─── $TLM,... (50 Hz) ────────────│   flag CFG_PENDING deve estar limpa
  │──── $CMD,... (50 Hz) ───────────▶│
```

Se qualquer `$CFG` não for confirmado, `robocar` **aborta** em vez de seguir com
calibração parcial — meio calibrado é pior que não calibrado, porque parece que
está funcionando.

## Erros comuns na depuração

| Sintoma | Causa provável |
|---|---|
| `$ERR,1` em rajada | Baud errado, ou cabo USB ruim perto do motor |
| Sem `$TLM` | Firmware não subiu, ou porta errada (`ls /dev/ttyUSB*`) |
| `flags` com `CFG_PENDING` fixo | Handshake não terminou; olhe os `$ERR` |
| `src=2` sem querer | `rc_enable=1`; o rádio está sobrepondo a serial |
| Servo trepidando | Alimentação do servo pelo USB do ESP32 — use o BEC |
| `FAILSAFE` intermitente | Loop do Jetson abaixo de 4 Hz, ou USB caindo |
