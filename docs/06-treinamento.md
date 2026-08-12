# Treinamento do modelo de direção

## Fluxo

```
sessões coletadas  →  treino (PC com GPU)  →  ONNX  →  TensorRT (no Jetson)
```

Treinar no Jetson funciona mas é lento. O normal é treinar no PC e levar só o
`.onnx` para o carro. **O engine TensorRT tem que ser gerado no próprio
Jetson** — ele é específico da GPU e da versão do TensorRT.

## Instalação (PC de treino)

```bash
cd ml
python -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
```

## Treinando

```bash
cd ml
python -m robocar_ml.train --data ../data/sessions --balance --epochs 40
```

Argumentos que mais importam:

| Flag | Padrão | Quando mexer |
|---|---|---|
| `--balance` | desligado | **Ligue quase sempre.** Ver "distribuição de esterço" |
| `--epochs` | 40 | A parada antecipada corta antes se estagnar |
| `--batch-size` | 64 | Reduza se faltar memória de GPU |
| `--lr` | 1e-3 | Reduza para 3e-4 se a perda oscilar |
| `--dropout` | 0.3 | Suba para 0.5 com dataset pequeno |
| `--predict-throttle` | desligado | Só depois que a direção estiver boa |
| `--horizontal-flip` | desligado | **Leia o aviso abaixo antes de ligar** |

## As três armadilhas deste problema

### 1. Distribuição de esterço

Qualquer percurso é majoritariamente reta. Sem tratamento, mais da metade das
amostras fica com esterço próximo de zero, e a rede descobre que **prever
sempre ~0 minimiza a perda**. A perda de treino fica ótima, a de validação
também, e o carro vai reto para dentro do quarteirão.

Diagnóstico:

```bash
robocar dataset stats --histogram
```

Tratamento, e vale fazer os dois:

- `--balance` limita cada faixa do histograma a `keep_factor × média`;
- coletar mais trechos com curva.

### 2. Divisão treino/validação por sessão

O loader divide **por sessão inteira**, nunca por frame. Frames vizinhos da
mesma volta são quase a mesma imagem: dividir aleatoriamente coloca imagens
quase idênticas nos dois conjuntos, a validação fica linda e não significa
nada.

Consequência prática: **com uma sessão só, não há validação honesta**. O
código avisa quando isso acontece. Colete em pelo menos 4 sessões distintas.

### 3. Espelhamento horizontal — cuidado nesta competição

Em pista de seguidor de linha, espelhar a imagem e inverter o sinal do esterço
dobra o dataset de graça. **Aqui é diferente:**

- O regulamento manda **manter-se na mão direita** da rua;
- As placas ficam **sempre à direita** da rota.

Espelhar inverte as duas coisas e ensina a regra errada. Por isso
`--horizontal-flip` é **desligado por padrão** e emite aviso quando ligado.

A forma correta de equilibrar curvas para os dois lados é **coletar nos dois
sentidos** (horário e anti-horário), como descrito em
`docs/04-coleta-de-dados.md`.

## Augmentação usada

Ativa apenas no conjunto de treino:

| Técnica | Probabilidade | Para quê |
|---|---|---|
| Brilho (0.6–1.4×) | 50% | Iluminação do ginásio ≠ da garagem |
| Sombra retangular | 30% | Vigas do teto, sombra do público |
| Deslocamento horizontal ±8% | 40% | Gera dado de recuperação sintético; o rótulo é corrigido junto |
| Espelhamento | desligado | Ver acima |

O deslocamento horizontal merece nota: além de deslocar a imagem, ele **ajusta
o rótulo proporcionalmente**. Deslocar a imagem para a direita equivale a ver
o carro mais à esquerda do que estava, e o esterço correto passa a ser mais à
direita. Sem essa correção, a augmentação injetaria rótulo errado.

## Interpretando as métricas

```
época  12/40  treino=0.00412  val=0.00521  mae=0.0483  lr=1.00e-03  38s
```

**`mae`** é o número que interessa: erro absoluto médio em unidades de esterço.

| MAE | Leitura |
|---|---|
| > 0.15 | Não segue faixa. Mais dado ou dado melhor |
| 0.08 – 0.15 | Segue reta, erra curva fechada |
| 0.04 – 0.08 | Utilizável. Testar na pista |
| < 0.04 | Bom — desconfie de vazamento entre treino e validação |

MAE baixíssimo com uma sessão só quase sempre significa vazamento, não
qualidade.

**`val` muito abaixo de `treino`** costuma indicar que a augmentação está
dificultando demais o treino (o que é aceitável) — ou que o conjunto de
validação é fácil demais.

**`val` subindo enquanto `treino` cai** é overfitting: suba `--dropout`,
colete mais dado ou reduza as épocas.

## Exportando

```bash
python -m robocar_ml.export --checkpoint ../models/steering/best.pt
```

Depois, **no Jetson**:

```bash
/usr/src/tensorrt/bin/trtexec \
    --onnx=models/steering/best.onnx \
    --saveEngine=models/steering/best.engine \
    --fp16
```

FP16 dá 2–3× de ganho no Orin Nano com perda desprezível para regressão de
esterço.

## Antes de confiar no modelo na pista

1. **Replay offline.** Rode o modelo sobre uma sessão gravada
   (`--backend file`) e compare a saída com o rótulo humano. Discrepância
   grande em trechos específicos aponta onde falta dado.
2. **Cavalete.** Carro no cavalete, modelo rodando, mova a câmera à mão e veja
   se o servo responde no sentido certo. Sinal invertido aqui é o erro mais
   comum e o mais fácil de detectar.
3. **Pista, devagar.** `throttle.limit_auto` em 0.25. Só suba depois de uma
   volta limpa.
4. **Só então** aumente a velocidade, um degrau por vez.

Vale lembrar o critério de desempate do regulamento: **menor número de erros**.
Um carro lento e limpo vence um carro rápido e errático.

## Próximos modelos (ainda não implementados)

| Modelo | Para quê | Quando |
|---|---|---|
| Detector de placas | MASTER: definir a rota | Depois de coletar as placas |
| Detector de semáforo | Largada nos dois modos | Depois de gravar os 3 estados |
| Predição de velocidade | Frear na curva, acelerar na reta | Depois que a direção estiver boa |

Todos dependem de dado que ainda não existe. É por isso que a coleta vem
primeiro.
