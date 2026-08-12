# Firmware do ESP32

O ESP32 é a **medula** do carro: recebe comandos do Jetson e os aplica nos
atuadores, lê os sensores de tempo crítico e — o mais importante — **para o
carro sozinho** quando o Jetson falha.

Ver [docs/01-arquitetura.md](../../docs/01-arquitetura.md) e
[ADR 0001](../../docs/adr/0001-jetson-como-cerebro-esp32-como-medula.md).

## Compilar e gravar

```bash
pio run              # compila
pio run -t upload    # grava
pio device monitor   # monitor serial (115200)
```

Se houver mais de um dispositivo USB conectado:

```bash
pio run -t upload --upload-port /dev/robocar-esp32
```

## O que este firmware faz

| Bloco | Onde |
|---|---|
| Codec do protocolo | `src/protocol.cpp` |
| Direção e tração (rampa, limites, PWM) | `src/actuators.cpp` |
| Ultrassônicos por interrupção, em rodízio | `src/ultrasonic.cpp` |
| Failsafe, telemetria, LED, botão | `src/main.cpp` |
| Pinos e tetos de segurança | `include/config.h` |

## O failsafe

É a razão de existir deste processador. Sem `$CMD` por `timeoutMs` (250 ms
por padrão), o firmware coloca a tração em neutro — independentemente do que o
Jetson estivesse pedindo antes.

Três detalhes que não são óbvios:

1. **O throttle vai a neutro sem rampa.** É parada de emergência.
2. **A direção mantém o último valor.** Travar as rodas no centro a 2 m/s joga
   o carro para fora da trajetória; manter o esterço deixa ele desacelerar na
   curva em que já estava.
3. **Sair do failsafe não devolve a velocidade de imediato** — a rampa segura
   a retomada, evitando arranque brusco quando a serial se restabelece.

### Teste obrigatório

Carro no cavalete, rodas no ar, armado, motor girando devagar. **Desconecte o
cabo USB.** O motor tem que parar em menos de 300 ms.

Faça isso toda vez que alterar o firmware. É o teste que separa "o carro sai
andando sozinho até bater" de "o carro para".

## Tetos de segurança

`include/config.h` define limites que o Jetson **não consegue afrouxar**:

```cpp
constexpr float HARD_LIMIT_FORWARD = 0.75f;
constexpr float HARD_LIMIT_REVERSE = 0.35f;
constexpr uint32_t HARD_TIMEOUT_MS = 500;
```

O `$CFG` aplica `min(valor_recebido, teto_compilado)`. Um bug no Python pode
deixar o carro mais lento, nunca mais rápido do que este arquivo permite.

## LED de status

Sem SSH na prova, o LED é a única saída de diagnóstico:

| Padrão | Estado |
|---|---|
| Apagado | Sem alimentação ou travado |
| 1 piscada/s | Aguardando `$CFG` (handshake incompleto) |
| 2 piscadas/s | Pronto, desarmado |
| Aceso fixo | Armado, recebendo comando |
| Piscada rápida | **FAILSAFE** — sem comando do Jetson |
| 3 piscadas/s | Bateria crítica, tração cortada |

## Rádio RC

Habilitado em tempo de compilação (`-DRC_INPUT_ENABLED=1` no `platformio.ini`)
e em runtime (`$CFG,rc_enable,1`). Quando ativo e com sinal válido, o rádio
**tem prioridade** sobre a serial: quem pilota é o humano e o `$CMD` do Jetson
vira só batimento cardíaco.

É assim que se coleta dataset com boa qualidade — entrada analógica, latência
mínima, e o piloto olhando para o carro em vez da tela.

> ⚠️ **Na prova oficial, `rc_enable` tem que ser 0.** O regulamento proíbe
> qualquer controle externo. `config/profiles/race.yaml` garante isso, mas
> confira no checklist do dia.

## Ultrassônicos

Três HC-SR04 disparados **em rodízio**, um por vez. Disparar em paralelo faria
cada sensor escutar o eco do outro e devolver distâncias fantasma — o tipo de
leitura que faz o carro frear do nada no meio da prova.

O eco é medido por **interrupção**, não por `pulseIn()`: aquela função bloqueia
até 25 ms por sensor e derrubaria o loop de controle para ~13 Hz.

`0` significa **sem leitura** (sem eco, fora de alcance), não "obstáculo
colado". Quem consome precisa tratar os dois casos de forma diferente.

> ⚠️ **O pino ECHO é 5 V.** Use divisor resistivo (1 kΩ + 2 kΩ) ou o GPIO do
> ESP32 queima.

## Testes

```bash
make -C test/host
```

Compila `src/protocol.cpp` **no PC**, contra um stub de `Arduino.h`, e confere
que os bytes gerados batem exatamente com os do Python
(`jetson/robocar/comms/protocol.py`).

Divergência entre as duas implementações do protocolo não aparece em
compilação nem em teste de um lado só — aparece como "o carro não responde" na
véspera da competição. Estes testes existem para evitar isso.
