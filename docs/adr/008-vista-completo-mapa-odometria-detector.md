# ADR 008 — Vista completo: cruzamento pelo mapa com odometria, detector no laço

**Data:** 09/11/2026 · **Estado:** aceito (decidido com a equipe em 09/11)

## Contexto

Com a rede de faixa do Colab dirigindo **tudo**, 4/10 rotas (o mesmo piloto
com a verdade: 10/10). 5 das 6 falhas: o carro se perde **dentro do
cruzamento**. Diagnóstico (diário, 09/11): lá dentro a rede estima κ ≈ 0
contra 1,3 1/m de verdade e e_ψ ≈ 0° contra −30…−41° — a curva que o carro
deve fazer **não está pintada no chão**, não há o que ver. O DAgger local não
mudou isso (ADR 007 já apontava a saída: curva por giroscópio + odometria).

Além disso, a avaliação ainda tirava da verdade a **manobra** (placa) e o
**semáforo**.

Decisões da equipe (09/11), em resposta às perguntas:

| Tema | Decisão |
|---|---|
| Cruzamento | curva do **mapa oficial**, seguida por **odometria** (encoders + giroscópio); localização pelo grafo |
| Placas | **Vista completo**: detector no laço + `rules.decide`; só sensores do carro |
| Velocidade | completar a **0,5 m/s** primeiro, depois subir; dados em várias velocidades |
| Cruzamento sem placa | **em frente** (política confirmada) |
| Carro | ainda em montagem |
| Perguntas à organização | **não serão enviadas** — as políticas ficam |
| Repositório | continua público |
| `main` | abrir PR do branch de trabalho |

## Decisão

### Pacote `vista/` — a lógica que vai para o carro

Só entra o que o carro tem: imagem, telemetria (encoders, MPU6050,
odometria), mapa oficial e calibração da câmera.

    AGUARDANDO_VERDE ──(verde em 2 leituras seguidas do detector)──▶ FAIXA
    FAIXA ──(mapa: a 15 cm da entrada do cruzamento)──▶ CRUZAMENTO
    CRUZAMENTO ──(30 cm dentro da faixa de saída)──▶ FAIXA (rua seguinte do grafo)

| Módulo | Faz |
|---|---|
| `mapa.py` — `LocalizadorGrafo` | rua atual (aresta) e pose no mapa: **prevista** pela odometria, **corrigida** na faixa pela rede (`e_y`, `e_ψ` amarram de lado e de rumo; distância ao cruzamento, ao longo) |
| `cruzamento.py` — `ManobraOdometria` | monta `fim da faixa de chegada → net.movement → começo da faixa de saída` e segue com a lei do oráculo (ADR 003) usando a pose da odometria |
| `placas.py` — `LeitorPlacas` | placa e semáforo pelo detector; decisão por `rules.decide` |
| `piloto.py` — `VistaPiloto` | a máquina de estados; manda rpm de cada roda |

A rede de faixa e o detector entram como funções: `DetectorOnnx` (YOLO11n
exportado, decodificação + NMS próprios, letterbox igual ao Ultralytics) ou
`DetectorVerdade`; rede ONNX/PyTorch ou `FaixaVerdade`/`FaixaRuidosa` — para
isolar cada parte na avaliação.

### Localização no grafo

O carro larga numa pose conhecida (linha de largada do mapa). A rua só muda
num cruzamento, e quem escolhe a saída é o próprio Vista: a próxima rua é
`opcoes[manobra]`. As saídas possíveis (`net.options`) vão para
`rules.decide`. Não precisa de GPS nem de reconhecer o lugar.

### Placa da MINHA aproximação

A regra 3.5.2 fixa onde cada placa fica; com a pose estimada e a calibração
da câmera, o Vista prevê **em que pixel e com que tamanho** a placa de cada
aproximação apareceria. Cada placa detectada vai para a aproximação mais
próxima do previsto (janela proporcional ao tamanho; tamanho entre 0,6× e
1,7× do previsto — uma placa do cruzamento seguinte, na mesma direção, aparece
menor). Os votos (soma da confiança) se acumulam **para todas as
aproximações, o tempo todo**.

Por que todas, e não só a atual: com a câmera a 15,6 cm e 15° para baixo, a
placa (centro a 47,5 cm) sai pelo topo da imagem a menos de ~0,8 m. Nas ruas
de ~1 m do centro, a placa do cruzamento seguinte quase nunca aparece na
própria aproximação — só de longe, antes do cruzamento anterior. Medido
(geometria, 24 rotas, 233 placas): legível em algum momento 98,7 %; na própria
aproximação só 66,5 %. Ver tabela por pitch em `docs/regulamento.md`.

Decisão: placas com ≥ 1,5 de voto, por ordem; a primeira que `rules.decide`
aceita vale (uma leitura impossível — "virar à esquerda" sem esquerda — é erro
de detecção). Sem placa lida: em frente. **Reserva provisória** (não é regra
do regulamento): placa lida que não decide (proibição que deixa duas saídas)
→ frente → direita → esquerda entre as que ela não proíbe, registrado como
`alternativa=True`.

### Escala do giroscópio calibrada nas curvas

A calibração parada tira o viés, não a escala (±3 % na MPU6050 = 2,7° numa
conversão de 90° → ~5 cm de erro na saída; foi o que tirou o carro da pista
na semente 100000). Cada curva mede a escala de graça: giro contado pelo
giroscópio × giro da pose, amarrada pela rede à faixa antes e depois.
Mínimos quadrados com prior fraco em 1,0, limitada a 1 ± 0,08.

### Ajustes no controle (valem também para o v1)

- **Filtro**: a previsão do rumo usa a curvatura da faixa **onde o carro
  está** (do mapa), não o κ "à frente" da rede — chegando numa conversão o κ
  à frente já vê a curva e enviesava o rumo filtrado em ~2° na entrada do
  cruzamento.
- **Ângulo de aproximação limitado**: a correção lateral `k·e_y` fica ≤
  0,35·v (~20°). 5 cm de erro de uma vez viravam 1,6 rad/s e a frente do
  carro varria para fora do tapete.
- O detector roda a cada 2 quadros (orçamento de 33 ms, diário 09/11),
  inclusive dentro do cruzamento (a placa seguinte pode aparecer ali).
- **Ganho × velocidade ≤ 2,4** (oráculo, PilotoFaixa e curva do mapa): com
  0,1 s de atraso de comando e o ponto controlado a 14 cm do eixo, k = 4
  oscila e diverge a 0,8 m/s — o próprio oráculo saía da pista em 40/40
  rotas. Até 0,6 m/s nada muda (oráculo idêntico a 0,5); a 0,8 dá k = 3 →
  40/40.

### Dados e DAgger

- `tools/gerar_dataset.py` sorteia a velocidade do oráculo por episódio
  (`--velocidades`, padrão 0,35–0,80 m/s; conversão 0,24–0,34).
- No DAgger, quem dirige agora é o **Vista** (`VistaPiloto`, detector
  perfeito para focar na rede de faixa); o oráculo socorre como antes, mas
  assume a **3,5 cm do lado do meio-fio** (6 cm do outro): na via de 80 cm
  sobram 8 cm até a borda. No DAgger o desenho do semáforo não é sorteado
  depois do verde — o Vista larga pelo que vê.
- `--pitch-graus` / `rodar_vista(pitch_graus=…)`: experimento de montagem
  da câmera (a rede precisa ser treinada no mesmo pitch).

## Resultados no simulador (09/11, 0,5 m/s, sementes 100000+)

| Configuração | Rotas completas | Decisões certas |
|---|---|---|
| Teto do Vista (rede de faixa e detector perfeitos) | **11/12** | 118/119 |
| Vista com verdade + ruído da rede (0,8 cm / 2,5 cm), detector perfeito, 20 rotas | **18/20** | 197/199 |
| *v1 com o mesmo ruído e filtro (02/11)* | *14/20* | *(manobra da verdade)* |
| Teto do Vista a **0,8 m/s** (8 rotas) | **7/8** | 82/83 |

Todas as falhas que sobram são placas de rua curta **nunca lidas** (votos
zerados → em frente); nenhuma é de direção — nem com o ruído.

A única falha do teto é uma placa de rua curta que a câmera não vê inteira
(só ~0,3 s, cortada no topo). A rede de faixa de verdade só pode ser medida
no Colab (`notebooks/treino_vista_v2.ipynb`); a rede local de teste (treinada
minutos em CPU) erra 6–9 cm na própria faixa e não diz nada.

## Consequências

- O cruzamento depende de o **mapa bater com a pista**. Ele vem das cotas do
  regulamento; no treino oficial (04/12) é preciso conferir cotas e larguras.
  A curva é curta (~1 m) e começa e termina amarrada à faixa vista pela rede,
  o que limita o estrago de um erro de mapa — mas não o elimina.
- O Vista precisa ser posto na **largada do mapa** (como no regulamento).
- Placa nunca vista → em frente. Com a montagem atual (15°), algumas placas
  de rua curta ficam no limite da visão. **Recomendação, para a equipe
  decidir**: avaliar pitch de 5–7° (85 % das placas visíveis na própria
  aproximação, contra 66 %); exige regerar dados e retreinar.
- Odometria: escala do giroscópio autocalibrada; a distância pelos encoders
  não (patinação até 2 % no carpete) — dentro do cruzamento são ≤ 2 cm.
- Proibição ambígua: reserva provisória, registrada. A decidir pela equipe.
