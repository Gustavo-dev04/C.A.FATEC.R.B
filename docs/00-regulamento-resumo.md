# Regulamento RoboCar Race 2026 — o que vira requisito de engenharia

Resumo do regulamento oficial (versão de 22/05/2026) filtrado para o que
**impacta decisão técnica**. O documento completo está em `docs/regulamento/`.
Em caso de divergência, vale o PDF oficial.

- Site: <https://www.robocarrace.com.br/>
- Contato: robocar.race@gmail.com

---

## 1. Nossa categoria

Temos Jetson Orin Nano + câmera + ESP32. Isso nos coloca em **JUNIOR** ou
**MASTER** — as duas únicas categorias de "minicidade" que aceitam computação
embarcada pesada.

| Categoria | Rota | Tecnologia sugerida | Diferença prática |
|---|---|---|---|
| JUNIOR | **Definida e conhecida** antecipadamente | Câmera, lidar | Dá para decorar o trajeto |
| MASTER | **Definida no dia, por placas de trânsito** | Câmera, lidar | Exige ler placas e decidir em tempo real |

> **Decisão do time:** mirar MASTER, que é superconjunto de JUNIOR. O
> percepção de faixa (lane keeping) é idêntico nas duas; MASTER só adiciona o
> detector de placas. Treinar para MASTER e, se o detector não amadurecer a
> tempo, cair para JUNIOR é possível — o contrário, não.
> Atenção: **não é permitido trocar de categoria durante a competição**
> (item 2), então a inscrição precisa ser decidida antes.

## 2. Datas

| Evento | Data | Horário |
|---|---|---|
| Treinos e testes na pista | 04/12/2026 (sexta) | 14h–22h |
| Torneio | 05/12/2026 (sábado) | a partir das 9h |

Há **inspeção prévia** no primeiro dia. O dia 04/12 é a única oportunidade de
coletar dados na pista real — o plano de coleta (`docs/04-coleta-de-dados.md`)
é dimensionado para essa janela de 8 horas.

## 3. Restrições de construção (Júnior e Master)

| Item | Limite | Nosso alvo |
|---|---|---|
| Comprimento | **máx. 500,0 mm** | 420 mm |
| Largura | **máx. 250,0 mm** | 195 mm |
| Escala | 1:10 | 1:10 |
| Mínimo | não há | — |

- Veículo elétrico, 2 ou 4 rodas tracionadas tocando o asfalto.
- Autonomia de bateria suficiente para a missão.
- Ajustes e pequenos consertos são permitidos entre as tomadas de tempo.
  **Trocar o robô ou substituir componentes embarcados de forma substancial =
  desclassificação.** Reflexo prático: leve peças sobressalentes, mas o carro
  que passar na inspeção é o carro que corre.

## 4. Restrições de software — as que mais nos afetam

> **"O sensoriamento deve ser 100% embarcado — não é permitida comunicação com
> computação ou sensoriamento externo."**

Consequências diretas, já refletidas em `config/profiles/race.yaml`:

1. **Nada de Wi-Fi/SSH durante a volta oficial.** O carro precisa dar boot,
   carregar o modelo, esperar o semáforo e correr sem nenhum host conectado.
   Botão físico + LED de status são obrigatórios, não luxo.
2. **Nada de telemetria para fora.** Log só em disco local.
3. **Nada de teleoperação.** Gamepad e rádio RC existem apenas para a fase de
   coleta de dados (04/12) e para os testes.
4. Toda a inferência roda no Jetson. Sem nuvem, sem notebook auxiliar.

> "Ação humana somente durante as fases de treino e de testes."

## 5. Pista

- Minicidade de **9,0 m × 13,0 m**, quarteirões de **2,0 m × 4,0 m**.
- Piso: **carpete preto/cinza**, com **delimitações da rua em branco**.
- Dentro dos quarteirões pode haver maquetes de prédios (fundo visual variável
  — outro motivo para caprichar na augmentação de dados).
- O veículo deve **permanecer dentro da rua, na mão direita**.
- **Cada invasão na contramão ou saída de pista: penalidade de 10 s.**

> ⚠️ **Ponto a confirmar com a organização:** o texto escreve "penalizada com
> **-10,0 segundos**", enquanto a penalidade do semáforo é "**+20 segundos**".
> Como a classificação é por *menor* tempo, uma penalidade de −10 s seria um
> prêmio. Assumimos **+10 s** (penalidade). Ver seção 9.

Vídeos de referência das equipes de 2025 (úteis para calibrar expectativa de
velocidade e comportamento):
- <https://youtube.com/shorts/tPz0z-NLVMk>
- <https://youtube.com/shorts/KfGcinLBmJc>

## 6. Placas de trânsito (só MASTER)

A rota **não é divulgada** — é revelada no dia pelas placas. Especificação:

| Parâmetro | Valor |
|---|---|
| Diâmetro | **150,0 mm** |
| Altura do chão à borda inferior | **400,0 mm** |
| Posição | **400,0 mm antes** do cruzamento/esquina |
| Lado | **sempre à direita** da rota, tangenciando a calçada |
| Padrão visual | nacional (CONTRAN) |

Isso é ouro para o detector: **sabemos exatamente onde procurar**. Com a câmera
a ~180 mm de altura, uma placa a 2 m tem o centro ~8° acima do horizonte e
sempre no lado direito do quadro — daí o recorte `roi_signs` em
`config/camera.yaml`. Ver a análise de pixels em `docs/02-hardware.md`.

Vídeo oficial de treinamento com as placas:
<https://youtu.be/YAg2p1kShZA> — **assistir antes de fechar a lista de classes**
em `docs/05-formato-dataset.md`.

## 7. Semáforo (Júnior e Master)

- 3 estados: vermelho → amarelo → verde. **Uma cor por vez.**
- Fica a **100 cm do ponto de largada**.
- O robô só pode se mover **no verde**. A cronometragem começa no verde.
- Queimar a largada:
  - completou o percurso → **+20 s** no tempo total;
  - não completou → **−2 m** na distância percorrida.

Requisito de software: detector de semáforo rodando **antes** do controle de
direção, com confirmação em múltiplos quadros (`min_green_frames: 3`) para não
largar por causa de um reflexo. Ver `config/profiles/race.yaml`.

Vídeo oficial do semáforo: <https://www.youtube.com/watch?v=GvytKrULBW0>

## 8. Pontuação, desempate e desclassificação

- Classificação: **maior distância percorrida**; havendo empate (mais de uma
  equipe completando), **menor tempo**.
- Cada veículo tem **3 voltas cronometradas**; vale a **melhor volta** (item 1.1).
- Desempate: (1) maior distância → (2) **menor número de erros** (escapes e
  paradas inesperadas) → (3) **menor tamanho do robô**.
- Decisão do juiz é incontestável.

> **Implicação estratégica:** "menor número de erros" ser critério de desempate
> e cada saída de pista custar 10 s significa que **um carro lento e limpo
> vence um carro rápido e errático**. Os limites de velocidade em
> `config/vehicle.yaml` começam conservadores de propósito
> (`limit_collect: 0.30`, `limit_race: 0.60`) e só sobem com dado de treino
> comprovando estabilidade.

## 9. Pontos a confirmar com a organização

Levar por e-mail (robocar.race@gmail.com) **antes de 04/12**:

1. A penalidade por invasão/saída de pista é **+10 s** (soma ao tempo) ou
   literalmente −10 s? O texto do item 3.4.1 é ambíguo.
2. Júnior/Master têm **quantas tentativas** por volta? As seções de "Missão"
   das categorias Nano/Micro/Mini falam em 2 tentativas, mas Júnior/Master não
   têm seção equivalente — e o item 1.1 fala em 3 voltas cronometradas.
3. A rota do Júnior será divulgada **quando**? Muda o plano de treinamento.
4. **Lista fechada das placas** que podem aparecer no Master (só setas de
   sentido, ou também Pare / Proibido / velocidade?).
5. Há **marcação de faixa central** entre as duas mãos da rua, ou só as
   bordas brancas externas?
6. A largada é sempre do mesmo ponto da minicidade?
7. Iluminação do ginásio: luz natural, artificial ou mista? (define o plano de
   coleta de dados em condições variadas).

## 10. Infraestrutura no local

- Boxes com bancada e tomada **220 VAC 60 Hz sem pino terra** — levar
  adaptador, extensão e, de preferência, um nobreak/filtro para o carregador.
- Time de 1 a 8 membros; no dia da corrida, **4 nos boxes + 2 de apoio**.
- Alimentação e água por conta da equipe.
