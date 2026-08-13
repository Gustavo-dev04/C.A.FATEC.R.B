# C.A.FATEC.R.B

Carro autônomo em escala 1:10 para a **[RoboCar Race 2026](https://www.robocarrace.com.br/)**,
categoria **MASTER** (minicidade com rota definida por placas de trânsito).

**Jetson Orin Nano** como cérebro (visão e decisão) + **ESP32** como medula
(atuação em tempo real e failsafe), com câmera CSI e sensores ultrassônicos.

| | |
|---|---|
| 📅 Treinos na pista | **04/12/2026**, 14h–22h |
| 🏁 Torneio | **05/12/2026**, a partir das 9h |
| 📏 Limites do carro | 500 × 250 mm (estamos com 420 × 195 mm) |
| 🏙️ Pista | Minicidade de 9,0 × 13,0 m, carpete escuro com faixas brancas |
| 📷 Câmera | IMX219 8 MP, lente 120°, foco ajustável — captura em 1640 × 1232 |

---

## Comece por aqui

```bash
git clone <url> && cd C.A.FATEC.R.B

# Ferramentas do Jetson
pip install -r jetson/requirements.txt
pip install -e jetson

# Confere o ambiente: dependências, disco, porta serial, câmera
robocar doctor
```

Firmware do ESP32:

```bash
cd firmware/esp32
pio run -t upload
```

Primeira coleta de dados:

```bash
robocar link --watch                     # ESP32 respondendo?
robocar camera                           # câmera na taxa certa?
robocar record --control rc --track minicidade_fatec --direction horario
```

---

## Documentação

📋 **[Visão Geral do Projeto](docs/VISAO-GERAL.md)** — comece aqui: objetivo
final, o que já existe, o que falta e o cronograma até a competição.

Depois, na ordem — cada documento assume o anterior.

| # | Documento | Sobre |
|---|---|---|
| 00 | [Resumo do regulamento](docs/00-regulamento-resumo.md) | O que as regras exigem do software |
| 01 | [Arquitetura](docs/01-arquitetura.md) | Como Jetson e ESP32 se dividem |
| 02 | [Hardware](docs/02-hardware.md) | BOM, pinos, escolha da lente, montagem |
| 03 | [Protocolo serial](docs/03-protocolo-serial.md) | Contrato Jetson ↔ ESP32 |
| 04 | [**Coleta de dados**](docs/04-coleta-de-dados.md) | **O procedimento — prioridade atual** |
| 05 | [Formato do dataset](docs/05-formato-dataset.md) | Esquema em disco |
| 06 | [Treinamento](docs/06-treinamento.md) | Treinar, avaliar, exportar |
| 07 | [Checklist de competição](docs/07-checklist-competicao.md) | O que fazer nos dias 04 e 05/12 |

Decisões de arquitetura: [ADR 0001](docs/adr/0001-jetson-como-cerebro-esp32-como-medula.md)
e [ADR 0002](docs/adr/0002-rotulos-vem-da-telemetria.md).

---

## Organização do repositório

```
config/            ⚙️  Fonte única de verdade da calibração (YAML)
  vehicle.yaml         dimensões, direção, tração, segurança, sensores
  camera.yaml          resolução, ROI, exposição fixa
  capture.yaml         taxa de gravação, filtros, metadados
  profiles/race.yaml   perfil da prova oficial (desliga tudo que é externo)

jetson/            🧠  Software do Jetson (Python)
  robocar/
    comms/             protocolo + enlace serial com o ESP32
    sensors/           câmera CSI / USB / replay de sessão
    capture/           esquema do dataset, sessões, gravador
    control/           fontes de comando (rádio RC, gamepad, teclado)
    config.py          carga e validação dos YAML
    cli.py             comando `robocar`
  tests/               109 testes, sem hardware

firmware/esp32/    ⚡  Firmware do ESP32 (C++ / PlatformIO)
  include/, src/       protocolo, atuadores, ultrassônicos, failsafe
  test/host/           testes que rodam no PC, sem ESP32

ml/                🤖  Treinamento (PyTorch)
  robocar_ml/
    dataset.py         loader com divisão por sessão e balanceamento
    models.py          PilotNet
    train.py           laço de treino
    export.py          ONNX -> TensorRT

data/              💾  Datasets coletados (fora do Git — ver data/README.md)
docs/              📚  Documentação
tools/             🔧  Scripts de setup e regras udev
```

---

## Comandos

| Comando | O que faz |
|---|---|
| `robocar doctor` | Confere ambiente, disco, serial e câmera |
| `robocar link --watch` | Telemetria do ESP32 em tempo real |
| `robocar camera --snapshot foto.jpg` | Testa a câmera e mede a taxa real |
| `robocar camera --focus` | Medidor de nitidez para ajustar a rosca de foco |
| `robocar camera --virtual` | Vista retificada das placas (câmera virtual) |
| `robocar calib camera` | Calibra intrínsecos e distorção da lente |
| `robocar record --control rc` | **Grava uma sessão de dataset** |
| `robocar dataset stats --histogram` | Resumo + distribuição do esterço |
| `robocar dataset verify` | Confere integridade das sessões |
| `robocar dataset tag <sessão> --discard "motivo"` | Marca sessão ruim |
| `robocar calib steering` | Calibra centro e batentes da direção |

Desenvolvimento:

```bash
make test      # testes Python + testes de host do firmware
make lint      # ruff
make check     # os dois
```

---

## Estado do projeto

| Bloco | Estado |
|---|---|
| Protocolo serial (Python + C++, verificados byte a byte) | ✅ |
| Firmware ESP32: PWM, rampa, limites, failsafe, ultrassônicos, telemetria | ✅ |
| Captura de câmera (CSI / USB / replay) | ✅ |
| Gravação de dataset (JPEG + JSONL, com filtros de qualidade) | ✅ |
| Pilotagem por rádio RC / gamepad / teclado | ✅ |
| Loader, PilotNet, treino e export ONNX | ✅ baseline |
| **Coleta de dados na pista** | ⬜ **próximo passo** |
| Detector de placas (MASTER) | ⬜ depende de coletar as placas |
| Detector de semáforo | ⬜ depende de gravar os 3 estados |
| Loop autônomo + TensorRT | ⬜ |

A ordem não é acidental: **sem dado coletado, nada do que está em branco pode
ser feito de verdade**. Ver [docs/04](docs/04-coleta-de-dados.md).

---

## Regras que viraram código

Três restrições do regulamento estão embutidas no software, não só na
documentação:

1. **"Sensoriamento 100% embarcado, sem comunicação externa."**
   `config/profiles/race.yaml` desliga Wi-Fi, Bluetooth, rádio RC e
   teleoperação. Rodar a prova oficial com outro perfil é risco de
   desclassificação.
2. **"Ação humana somente em treino e teste."** Rádio e gamepad existem só
   para a coleta de dados; `rc_enable` é `0` por padrão em todo lugar.
3. **Limites de 500 × 250 mm.** `robocar doctor` reprova a configuração se as
   dimensões declaradas passarem disso.

---

## Pendências com a organização

Há ambiguidades no regulamento que precisam ser confirmadas por e-mail
(robocar.race@gmail.com) **antes de 04/12** — a lista está em
[docs/00, seção 9](docs/00-regulamento-resumo.md#9-pontos-a-confirmar-com-a-organização).
A mais importante: a penalidade por invasão/saída de pista é **+10 s** ou
literalmente **−10 s**? O texto oficial diz "−10,0 segundos", mas a
classificação é por menor tempo.
