# data/ — datasets coletados

**O conteúdo deste diretório não vai para o Git.** Ver `.gitignore`.

## Por quê

Gravamos em **1640 × 1232** (campo completo da lente de 120°, ~280 kB por
quadro JPEG q90). A 20 Hz:

| Duração de gravação | Frames (20 Hz) | Disco |
|---|---|---|
| 10 min | 12 000 | ~3,4 GB |
| 1 h | 72 000 | ~20 GB |
| 8 h (dia de treino) | 576 000 | ~160 GB |

O Git não lida bem com isso: cada `clone` traria dezenas de GB, e como imagens
JPEG não comprimem nem fazem *diff*, o histórico só cresceria. Git LFS
resolveria parcialmente, mas as cotas gratuitas não chegam perto do volume.

## Onde os dados ficam

```
data/
├── sessions/          sessões brutas (robocar record)
├── datasets/          datasets derivados (ex.: placas anotadas)
└── raw/               vídeos e material solto
```

**Regra do time: toda sessão tem duas cópias, em mídias diferentes.**
Cartão SD corrompido no dia 04/12 apaga o dia inteiro de coleta — e não há
como recoletar.

Sugestão de fluxo:

```bash
# Do Jetson para o HD externo, ao final de cada sessão
rsync -av --progress \
    data/sessions/20261204_163045_minicidade_fatec/ \
    /media/backup/robocar/sessions/20261204_163045_minicidade_fatec/

# Do HD externo para o notebook de treino
rsync -av /media/backup/robocar/sessions/ ~/robocar-data/sessions/
```

`rsync` é preferível a `cp` porque é incremental e retomável: se o cabo cair no
meio, você continua de onde parou em vez de recomeçar.

## Convenção de nomes

`AAAAMMDD_HHMMSS_<pista>_<tag>` — gerado automaticamente por `robocar record`.
O prefixo temporal faz ordem alfabética coincidir com ordem cronológica.

Não renomeie diretórios de sessão à mão: `session.json` guarda o `session_id`
e o descasamento confunde as ferramentas.

## Conferindo o que existe

```bash
robocar dataset stats --histogram   # resumo por sessão + distribuição do esterço
robocar dataset verify              # imagens ausentes, índices duplicados
```

## Sessões ruins

Não apague — marque:

```bash
robocar dataset tag <sessão> --discard "bateu no quarteirao aos 3min"
```

A sessão continua no disco (entender o que deu errado tem valor) mas o loader
de treino a ignora.

## Formato

Ver [docs/05-formato-dataset.md](../docs/05-formato-dataset.md).
