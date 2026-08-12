# Formato do dataset — v1

Definição normativa. A implementação está em
`jetson/robocar/capture/schema.py` e os testes em
`jetson/tests/test_schema.py`.

## Estrutura em disco

```
data/sessions/
└── 20261204_163045_minicidade_fatec_recuperacao/
    ├── session.json          metadados (uma vez por sessão)
    ├── records.jsonl         um JSON por linha, um por frame
    └── images/
        ├── 000000.jpg
        ├── 000001.jpg
        └── ...
```

Nome do diretório: `AAAAMMDD_HHMMSS_<pista>_<tag>`. O prefixo temporal faz
ordem alfabética coincidir com ordem cronológica, então `ls` e `sorted()` já
fazem a coisa certa.

### Por que este formato

| Escolha | Alternativa | Razão |
|---|---|---|
| JSONL | CSV | Campos aninhados (distâncias) sem gambiarra; *append-only*, então queda de energia custa uma linha, não o arquivo |
| JPEG solto | vídeo H.264 | Acesso aleatório barato no treino; um frame corrompido não invalida a sessão. Custa ~15% mais disco |
| Um diretório por sessão | banco de dados | `rsync`, `cp` e `rm` resolvem tudo; nada a instalar |

## `records.jsonl` — um registro por frame

```json
{"index":42,"image":"images/000042.jpg","t_host_ns":123456789012,"steer":-0.3125,"throttle":0.28,"t_mcu_ms":84213,"telemetry_age_ms":12.4,"source":2,"flags":5,"distances_mm":[1230,890,1540],"vbat":7.82}
```

| Campo | Tipo | Obrigatório | Descrição |
|---|---|:---:|---|
| `index` | int | ✅ | Sequencial na sessão, a partir de 0 |
| `image` | str | ✅ | Caminho relativo à raiz da sessão |
| `t_host_ns` | int | ✅ | Relógio **monotônico** do Jetson na captura |
| `steer` | float | ✅ | **[-1,+1] aplicado.** −1 = esquerda, +1 = direita |
| `throttle` | float | ✅ | **[-1,+1] aplicado.** Negativo = ré |
| `t_mcu_ms` | int | | `millis()` do ESP32 na telemetria usada |
| `telemetry_age_ms` | float | | Idade do rótulo. Alto = enlace ruim |
| `source` | int | | 0=failsafe, 1=serial, 2=rádio RC |
| `flags` | int | | Bitfield de `$TLM` (ver docs/03) |
| `distances_mm` | int[] | | Distâncias em mm; **0 = leitura inválida** |
| `vbat` | float | | Tensão da bateria de tração |
| `cmd_steer` | float | | O que o Jetson **pediu**. Só diagnóstico |
| `cmd_throttle` | float | | O que o Jetson **pediu**. Só diagnóstico |

> ### ⚠️ Treine com `steer`/`throttle`, nunca com `cmd_steer`/`cmd_throttle`
>
> `steer` é o que o ESP32 **aplicou** nos atuadores; `cmd_*` é o que o Jetson
> **pediu**. Eles divergem sempre que há rampa, limite de velocidade, corte
> por obstáculo, failsafe — ou quando quem está pilotando é o rádio RC, caso
> em que `cmd_*` nem existe.
>
> Treinar com `cmd_*` associa a imagem a uma ação que o carro não executou.
> O erro não aparece na curva de perda e só se manifesta como comportamento
> inexplicável na pista. Ver [ADR 0002](adr/0002-rotulos-vem-da-telemetria.md).

### Relógio monotônico

`t_host_ns` vem de `time.monotonic_ns()`, não do relógio de parede. O Jetson
frequentemente não tem RTC com bateria: ele acorda em 1970 e dá um salto de 56
anos quando o NTP sincroniza. Um salto desses no meio de uma sessão embaralharia
a ordem temporal. Para saber *quando* a sessão foi gravada, use
`session.json → started_at`.

## `session.json` — metadados

```json
{
  "session_id": "20261204_163045_minicidade_fatec_recuperacao",
  "schema_version": 1,
  "started_at": "2026-12-04T16:30:45-03:00",
  "ended_at": "2026-12-04T16:42:11-03:00",
  "duration_s": 686.2,
  "frame_count": 11430,

  "track": "minicidade_fatec",
  "driver": "gustavo",
  "lighting": "fluorescente",
  "direction": "horario",
  "notes": "correcoes da borda direita para o centro",

  "vehicle": "cafatecrb-01",
  "category": "master",
  "git_commit": "a1b2c3d",
  "hostname": "jetson-cafatecrb",
  "robocar_version": "0.1.0",

  "camera": { "...": "snapshot de config/camera.yaml" },
  "capture": { "...": "snapshot de config/capture.yaml" },
  "vehicle_config": { "...": "steering, throttle, safety, sensores" },

  "dropped_frames": 3,
  "skipped_stopped": 812,
  "skipped_stale": 15,
  "skipped_unsafe": 0,
  "discarded": false,
  "discard_reason": ""
}
```

O bloco de configuração é um **snapshot**, não uma referência. É ele que
permite responder, dois meses depois, "esta sessão foi gravada antes ou depois
de recalibrarmos a direção?". Sem isso, sessões viram dados órfãos.

`git_commit` recebe o sufixo `-sujo` quando havia alterações não commitadas.

## Contadores de qualidade

Os campos `skipped_*` explicam a diferença entre frames capturados e gravados:

| Campo | Motivo do descarte |
|---|---|
| `dropped_frames` | Fila de escrita cheia (disco lento) |
| `skipped_stopped` | Carro parado (`throttle` abaixo do mínimo) |
| `skipped_stale` | Telemetria mais velha que `max_telemetry_age_ms` |
| `skipped_unsafe` | Flags de failsafe, obstáculo ou bateria crítica |

`yield_ratio` = gravados ÷ considerados. Abaixo de 0,5 significa que algo está
errado — veja qual contador subiu.

## Compatibilidade e versionamento

`schema_version` está em `session.json`. As regras:

- **Adicionar campo opcional** não sobe a versão. Leitores antigos ignoram
  campos desconhecidos (há teste para isso).
- **Remover ou mudar o significado** de um campo sobe a versão e exige um
  conversor em `ml/tools/`.
- Um leitor **nunca** deve falhar por campo desconhecido.

## Dataset de placas de trânsito (MASTER)

Formato separado, porque o problema é classificação/detecção, não regressão:

```
data/datasets/placas/
├── classes.txt
├── images/
│   ├── 000001.jpg
│   └── ...
└── labels/
    ├── 000001.txt        formato YOLO: <classe> <cx> <cy> <w> <h> (normalizado)
    └── ...
```

### Classes candidatas

O regulamento diz apenas "placas de trânsito (setas), no padrão visual
nacional". A lista abaixo é um **ponto de partida** com as placas de sentido do
padrão CONTRAN e **precisa ser confirmada** com o vídeo oficial
(<https://youtu.be/YAg2p1kShZA>) e, se possível, por e-mail à organização —
uma classe faltante significa uma curva errada na prova.

```
0  vire_a_esquerda
1  vire_a_direita
2  siga_em_frente
3  siga_em_frente_ou_a_esquerda
4  siga_em_frente_ou_a_direita
5  sentido_obrigatorio
6  proibido_virar_a_esquerda
7  proibido_virar_a_direita
8  parada_obrigatoria
```

Este dataset ainda **não foi coletado** — é a próxima tarefa depois que a
coleta de condução estiver rodando. Ver `docs/04-coleta-de-dados.md`.
