# ADR 0002 — Os rótulos do dataset vêm da telemetria, não do comando

- **Status:** aceito
- **Data:** 2026-08-12

## Contexto

Em aprendizado por imitação (behavior cloning), cada frame precisa de um rótulo:
o esterço que o piloto humano aplicou naquele instante. Há duas fontes possíveis
desse número:

- **(A)** o comando que o Jetson *enviou* ao ESP32;
- **(B)** o valor que o ESP32 *efetivamente aplicou* nos atuadores, devolvido
  na telemetria `$TLM`.

Parece a mesma coisa. Não é.

## Decisão

Sempre **(B)**: `capture.labels.source: telemetry`. O gravador nunca usa o
comando enviado como rótulo.

## Justificativa

Comando e valor aplicado divergem em pelo menos cinco situações reais:

1. **Rampa (slew rate).** Pedimos `throttle 0.5`; a rampa entrega 0.18 neste
   instante. A imagem corresponde a um carro a 0.18.
2. **Limite de velocidade.** Pedimos 0.8, o teto do modo de coleta é 0.30.
3. **Corte por obstáculo.** O ultrassônico frontal viu 200 mm e zerou a
   tração — a imagem mostra um obstáculo perto e o carro parando.
4. **Failsafe.** A serial engasgou; o motor foi para neutro. O comando enviado
   dizia 0.4 e o carro estava desacelerando.
5. **Direção por rádio RC.** É o caso mais importante: durante a coleta, quem
   dirige é o piloto pelo rádio, e o Jetson **não envia comando nenhum**. Só
   existe rótulo porque o ESP32 informa o que o rádio mandou (`src=2`).

Treinar com (A) ensina a rede uma associação falsa: "esta imagem corresponde a
acelerar forte", quando na cena o carro estava freando. O erro é sutil, não
aparece na curva de perda (a rede aprende a função errada com perda baixa) e só
se manifesta como comportamento estranho na pista. É caríssimo de diagnosticar
depois.

## Consequências

**Positivas**
- Um único caminho de rótulo serve para coleta por rádio RC, por gamepad e em
  modo autônomo — o gravador não precisa saber quem está dirigindo.
- O rótulo carrega implicitamente os efeitos de rampa e limites, que fazem
  parte da dinâmica real do carro.
- As `flags` vêm de brinde e permitem descartar automaticamente os frames em
  failsafe, corte por obstáculo ou bateria crítica.

**Negativas**
- O rótulo tem a idade da telemetria. O gravador rejeita amostras com mais de
  `max_telemetry_age_ms` (60 ms) e conta essas rejeições — se o número subir,
  há problema de enlace, e é melhor descobrir durante a coleta do que no
  treino.
- Cria dependência dura da telemetria: ESP32 mudo = sessão sem rótulo. Por
  isso `robocar record` **aborta na largada** se não houver `$TLM`, em vez de
  gravar horas de imagem inútil.

## Implementação

- `jetson/robocar/capture/recorder.py` — casa frame com a telemetria mais
  recente e aplica o teste de idade.
- `jetson/robocar/capture/schema.py` — campos `steer`/`throttle` documentados
  como valores **aplicados**; `cmd_steer`/`cmd_throttle` são gravados à parte,
  só para diagnóstico. **Nunca treine com `cmd_*`.**
