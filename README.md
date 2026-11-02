# C.A.FATEC.R.B — Vista

Carro autônomo em escala 1:10 para a **[RoboCar Race 2026](https://www.robocarrace.com.br/)**,
categoria **MASTER**. FATEC Santo André · Engenharia Mecatrônica.

**Vista** é a IA do carro: lê a câmera, entende a pista (faixa, semáforo,
placas), decide e manda a **rotação de cada roda** para o ESP32 — o carro não
tem direção, curva pela diferença entre as rodas.

| | |
|---|---|
| 🏁 Treinos na pista oficial | **04/12/2026** — único dia na pista real |
| 🏁 Prova | **05/12/2026** |
| 🚗 Carro | tração diferencial, 400 × 241 mm (REV03) |
| 📷 Câmera | IMX219 fisheye 120°, 1640×1232 @ 30 |
| 🧠 Computação | Jetson Orin Nano + ESP32-S3 |

## Estratégia

1. **Agora — simulador.** A pista é padronizada pelo regulamento, então dá para
   treinar antes do carro existir. O **Vista-Sim** é feito só para este carro e
   esta prova.
2. **Quando o carro estiver pronto — dados reais** da pista caseira até a
   competição.

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
pytest
```

```python
from vista_sim.env import VistaSimEnv

env = VistaSimEnv()
obs, info = env.reset(seed=139)             # sorteia o cenário inteiro
obs, telemetria, verdade, fim, info = env.step(rpm_esq, rpm_dir)
# atalho: env.step_vw(v, omega) converte para rpm de cada roda antes
```

O Vista recebe `obs` (imagem) e `telemetria` (rpm **medido pelo encoder**,
ticks e odometria integrada dos ticks, como no ESP32 real). `verdade` — pose,
`e_y`/`e_ψ` do centro do carro, próximo cruzamento com placa e manobra, placas
visíveis com caixa e fração visível, semáforo — serve para medir o Vista e
rotular dados, **nunca** como entrada dele. `info["events"]` traz
`placa_respeitada`, `desrespeitou_placa`, `saiu_da_pista`, `rota_completa`…

O carro é simulado **roda por roda**: PID do ESP32 a 100 Hz, zona morta do
motor, encoder quantizado, raios ligeiramente diferentes e patinação — rotação
igual **não** é andar reto. Ver [ADR 003](docs/adr/003-carro-em-rotacao-de-roda.md).

### Fonte única de verdade

O simulador lê os mesmos arquivos do carro real. Calibrou a câmera → o
simulador muda junto.

| Arquivo | Conteúdo | Origem |
|---|---|---|
| `config/car.yaml` | geometria, motor, encoder, sensores | desenho REV03 |
| `config/camera/intrinsics.yaml` | modelo fisheye | ⚠️ estimado — calibrar |
| `config/camera/extrinsics.yaml` | pose da câmera, pitch | REV03 — medir |
| `maps/robocar_master.yaml` | mapa oficial | gerado do regulamento |

### Estado (fases do CLAUDE.md §5A)

| Fase | Entrega | Estado |
|---|---|---|
| **S1** | car.yaml, mapa vetorial, cinemática, renderizador `fast` | ✅ oráculo completa **200/200** rotas sorteadas pela cidade |
| S1.1 | rotação de roda, malha viária, placas pela regra, luz e sombra | ✅ ~1.000 passos/s só chão; ~200 com a cidade inteira |
| S2 | Vista em loop fechado + cenários de regressão | ⬜ depende do Vista |
| S3 | renderizador `photo` (Blender) + rótulos automáticos | ⬜ |
| S4 | fidelidade sim × real + ultrassom/IMU | ⬜ depende do carro |

## Documentação

| Documento | Sobre |
|---|---|
| [Regulamento](docs/regulamento.md) | O que as regras exigem — **inclui o lido nas figuras** |
| [Análise de FOV](docs/fov-analise.md) | Por que o semáforo não restringe o pitch |
| [Diário de construção](docs/diario-de-construcao.md) | Todo o caminho, com problemas e números |
| [ADR 001](docs/adr/001-mapa-pelo-regulamento.md) | Mapa pelo regulamento, não pelo `.obj` do organizador |
| [ADR 002](docs/adr/002-camera-fisheye-e-chao-por-grade.md) | Câmera fisheye em NumPy e chão por grade |
| [ADR 003](docs/adr/003-carro-em-rotacao-de-roda.md) | Carro comandado em rotação de roda; o centro do carro segue a faixa |
| [ADR 004](docs/adr/004-cenarios-placas-e-iluminacao.md) | Cenários sorteados: rota, placas pela regra, luz com sombra |

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
