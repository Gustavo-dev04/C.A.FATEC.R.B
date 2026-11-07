# C.A.FATEC.R.B — Vista

Carro autônomo em escala 1:10 para a **[RoboCar Race 2026](https://www.robocarrace.com.br/)**,
categoria **MASTER**. FATEC Santo André · Engenharia Mecatrônica.

**Vista** é a IA do carro: lê a câmera, entende a pista (faixa, cruzamento,
semáforo, placas), decide e manda a **rotação de cada roda** para o ESP32 — o
carro não tem direção, curva pela diferença entre as rodas.

> 📌 **Comece pela [visão geral do projeto](docs/VISAO-GERAL.md)**: o que
> existe, os resultados, o problema em aberto e o plano até a prova.

| | |
|---|---|
| 🏁 Treinos na pista oficial | **04/12/2026** — único dia na pista real |
| 🏁 Prova | **05/12/2026** |
| 🚗 Carro | tração diferencial, 400 × 241 mm (REV03) |
| 📷 Câmera | IMX219 fisheye 120°, 1640×1232 @ 30 |
| 🧠 Computação | Jetson Orin Nano + ESP32-S3 (rpm de cada roda) |
| 🧭 IMU | MPU6050 — 6 eixos, sem magnetômetro |

## Estratégia

1. **Agora — simulador.** A pista é padronizada pelo regulamento, então dá para
   treinar antes do carro existir. O **Vista-Sim** é feito só para este carro e
   esta prova.
2. **Quando o carro estiver pronto — dados reais** da pista caseira até a
   competição.

## Estado (07/11/2026)

| Etapa | Estado |
|---|---|
| Mapa oficial lido nas figuras do regulamento | ✅ |
| Simulador: carro em rotação de roda, câmera fisheye, cidade, placas, luz, MPU6050 | ✅ oráculo completa **200/200** rotas sorteadas |
| Controle a partir da percepção + filtro com encoders e giroscópio | ✅ **40/40** rotas com a verdade |
| Pipeline de treino + notebook do Colab | ✅ |
| Detector de placas e semáforo (YOLO11n) | ✅ mAP50 **0,948**; semáforo **0,99** (no simulador) |
| Rede de faixa (ResNet18) | ⚠️ erra ~0,3 cm por imagem, mas dirigindo completa **4/10** rotas — perde-se **dentro do cruzamento** |
| DAgger (a rede dirige, o oráculo socorre) | ✅ implementado; num teste local não resolveu o cruzamento |
| Cruzamento por odometria + localização no mapa | 💬 proposto, aguardando ok |
| Detector no laço (placa detectada → manobra) | ⬜ |
| Firmware do ESP32, integração na Jetson, calibração no carro | ⬜ |
| Dados reais da pista caseira | ⬜ depende do carro |

Detalhes e números em [docs/VISAO-GERAL.md](docs/VISAO-GERAL.md); cada passo,
com problemas e soluções, no [diário de construção](docs/diario-de-construcao.md).

## Vista-Sim

![Rota pela cidade](docs/img/rota-cidade.jpg)

Câmera fisheye virtual (esq.) e mapa visto de cima (dir.) numa rota sorteada:
largada no vermelho, saída no verde, a rota é decidida **pelas placas**
cruzamento a cruzamento, entrando no centro e contornando a ilha. Cada
episódio sorteia rota, placas (as 6 classes, mais distratoras), faixas de
pedestre, maquetes, público e **iluminação com sombras** — inclusive a do
próprio carro.

![Mesmo ponto, seis iluminações](docs/img/iluminacao.jpg)

```bash
pip install -e ".[dev]"
python -m vista_sim.maps.build_master        # (re)gera o mapa oficial
python -m tools.sim_demo --out runs/demo     # rota pela cidade + vídeo
python -m tools.sim_demo --bench             # passos/s
pytest                                        # 89 testes
```

```python
from vista_sim.env import VistaSimEnv

env = VistaSimEnv()
obs, info = env.reset(seed=139)             # sorteia o cenário inteiro
obs, telemetria, verdade, fim, info = env.step(rpm_esq, rpm_dir)
# atalho: env.step_vw(v, omega) converte para rpm de cada roda antes
```

O Vista recebe `obs` (imagem) e `telemetria` (rpm **medido pelo encoder**,
ticks, leitura bruta da **MPU6050** e a odometria: deslocamento pelos encoders,
rumo pelo giroscópio com o viés calibrado na espera do vermelho). `verdade` — pose,
`e_y`/`e_ψ` do centro do carro, próximo cruzamento com placa e manobra, placas
visíveis com caixa e fração visível, semáforo — serve para medir o Vista e
rotular dados, **nunca** como entrada dele. `info["events"]` traz
`placa_respeitada`, `desrespeitou_placa`, `saiu_da_pista`, `rota_completa`…

O carro é simulado **roda por roda**: PID do ESP32 a 100 Hz, zona morta do
motor, encoder quantizado, raios ligeiramente diferentes e patinação — rotação
igual **não** é andar reto. Ver [ADR 003](docs/adr/003-carro-em-rotacao-de-roda.md).

## Treino (Colab)

[`notebooks/treino_vista.ipynb`](notebooks/treino_vista.ipynb) — abra no Colab
com GPU e `Executar tudo`:

1. gera os dados no simulador;
2. ajusta uma ResNet18 (faixa) e um YOLO11n (placas e semáforo) pré-treinados;
3. exporta ONNX;
4. mede a rede **dirigindo no simulador**, com e sem o filtro;
5. faz rodadas de **DAgger**.

`CONTINUAR_DE` retoma de um modelo já salvo no Drive. Ver
[ADR 006](docs/adr/006-treino-com-dados-do-simulador.md) e
[ADR 007](docs/adr/007-dagger-e-filtro.md).

```bash
pip install -e ".[treino]"
python -m tools.gerar_dataset --out dataset --episodios 150          # só CPU
python -m tools.gerar_dataset --out dataset --episodios 40 \
    --semente-inicial 501000 --piloto-onnx faixa.onnx                # DAgger
python -m tools.avaliar_modelo --onnx faixa.onnx --sementes 100000:100010
```

O treino roda no PC ou no Colab — **nunca na Jetson**. Lá só roda o TensorRT.

## Fonte única de verdade

O simulador lê os mesmos arquivos do carro real. Calibrou a câmera → o
simulador muda junto.

| Arquivo | Conteúdo | Origem |
|---|---|---|
| `config/car.yaml` | geometria, motor, encoder, PID, MPU6050, ultrassom, bateria | desenho REV03 + datasheets; ⚠️ motor/PID/encoder estimados |
| `config/camera/intrinsics.yaml` | modelo fisheye | ⚠️ estimado — calibrar |
| `config/camera/extrinsics.yaml` | pose da câmera, pitch | REV03 — medir |
| `maps/robocar_master.yaml` | mapa oficial | gerado do regulamento |

## Onde está cada coisa

| Caminho | O quê |
|---|---|
| `vista_sim/` | simulador **e** a lógica que vai para o carro: `rules.py` (placa → manobra), `controle.py` (filtro + controle), `imu.py` (odometria) |
| `treino/` | rede de faixa, dataset do detector, avaliação dirigindo |
| `tools/` | `sim_demo.py` (vídeo), `gerar_dataset.py` (dados e DAgger), `avaliar_modelo.py` (ONNX dirigindo) |
| `notebooks/` | notebook do Colab |
| `tests/` | 89 testes |
| `docs/` | visão geral, regulamento lido, análise de FOV, ADRs, diário, resumos |

## Documentação

| Documento | Sobre |
|---|---|
| [**Visão geral**](docs/VISAO-GERAL.md) | Estado do projeto, resultados, plano até a prova |
| [Regulamento](docs/regulamento.md) | O que as regras exigem — **inclui o lido nas figuras** — e as perguntas para a organização |
| [Análise de FOV](docs/fov-analise.md) | Por que o semáforo não restringe o pitch |
| [Diário de construção](docs/diario-de-construcao.md) | Todo o caminho, com problemas e números |
| [Resumo de 02/11](docs/resumo-2026-11-02.md) | O dia em que o simulador e o treino nasceram |
| [ADR 001](docs/adr/001-mapa-pelo-regulamento.md) | Mapa pelo regulamento, não pelo `.obj` do organizador |
| [ADR 002](docs/adr/002-camera-fisheye-e-chao-por-grade.md) | Câmera fisheye em NumPy e chão por grade |
| [ADR 003](docs/adr/003-carro-em-rotacao-de-roda.md) | Carro comandado em rotação de roda; o centro do carro segue a faixa |
| [ADR 004](docs/adr/004-cenarios-placas-e-iluminacao.md) | Cenários sorteados: rota, placas pela regra, luz com sombra |
| [ADR 005](docs/adr/005-imu-mpu6050-e-odometria.md) | MPU6050 simulada; rumo pelo giroscópio, calibrado no vermelho |
| [ADR 006](docs/adr/006-treino-com-dados-do-simulador.md) | Treino: redes pré-treinadas ajustadas com dados do simulador |
| [ADR 007](docs/adr/007-dagger-e-filtro.md) | DAgger (a rede dirige, o oráculo socorre) e filtro com encoders + giroscópio |

## O que o regulamento diz e o simulador do organizador não

Lendo as **figuras** do regulamento (a extração de texto do PDF não pega as
cotas):

- **6 classes de placa**, não 3 — as 3 de proibição mudam a lógica de decisão.
- Placa **Ø150 com a base a 400 mm** (o simulador do organizador tem Ø300).
- Semáforo com centros a **530 / 390 / 250 mm** — o que derruba o alerta de
  pitch: com 15° as três lentes cabem.
- Vias norte-sul de **100 cm**, leste-oeste de **80 cm**; linha central
  **branca**.

Detalhes e perguntas para a organização em [docs/regulamento.md](docs/regulamento.md).
