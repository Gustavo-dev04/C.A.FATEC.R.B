# C.A.FATEC.R.B — Visão Geral do Projeto

> **Carro autônomo em escala 1:10 para a RoboCar Race 2026**
> Documento de referência do projeto — o que já existe, aonde queremos chegar
> e o que falta fazer.

| | |
|---|---|
| **Competição** | [RoboCar Race 2026](https://www.robocarrace.com.br/) — Torneio de Veículos Autônomos em Escala |
| **Categoria** | MASTER (minicidade, rota definida no dia por placas de trânsito) |
| **Treinos na pista** | 04/12/2026, 14h–22h |
| **Prova** | 05/12/2026, a partir das 9h |
| **Repositório** | `Gustavo-dev04/C.A.FATEC.R.B` |
| **Última atualização** | 13/08/2026 |
| **Prazo restante** | **16 semanas** até o dia de treino |

---

## 1. Objetivo final

Construir um veículo elétrico autônomo em escala 1:10 capaz de **percorrer
sozinho uma minicidade de 9 × 13 metros**, decidindo o trajeto em tempo real a
partir de **placas de trânsito** que só são reveladas no dia da prova.

O carro precisa, sem nenhuma intervenção humana e sem comunicação com o mundo
externo:

1. **Esperar o semáforo** ficar verde antes de sair (largar antes custa +20 s);
2. **Manter-se na rua, pela mão direita** (cada invasão ou saída custa 10 s);
3. **Ler as placas de trânsito** e decidir a cada cruzamento se segue reto,
   vira à esquerda ou à direita;
4. **Não bater** em obstáculos, quarteirões ou maquetes;
5. **Completar a volta** no menor tempo possível.

Critério de vitória: maior distância percorrida; havendo empate entre quem
completou, o menor tempo. **O primeiro critério de desempate é o menor número
de erros** — o que define a nossa estratégia: um carro consistente vale mais
que um carro rápido.

### Objetivo pedagógico

Além da competição, o projeto é um exercício completo de sistemas embarcados e
visão computacional: arquitetura de tempo real, protocolo de comunicação,
aquisição e curadoria de dados, treinamento de redes neurais e integração
hardware-software. O regulamento é explícito quanto a isso — "o objetivo não é
fabricar um veículo autônomo, mas preparar e qualificar" os participantes.

---

## 2. Arquitetura

Dois processadores, com responsabilidades deliberadamente separadas.

```
┌────────────────────────────────────────────────────────────────┐
│  JETSON ORIN NANO — "o cérebro"                                 │
│  Ubuntu + JetPack · Python · CUDA / TensorRT                    │
│                                                                  │
│   Câmera CSI ──▶ [ captura ] ──┬──▶ [ percepção ]               │
│   IMX219 120°     20–30 Hz     │     ├── faixa/rota  (CNN)      │
│                                │     ├── placas      (MASTER)   │
│                                │     └── semáforo    (largada)  │
│                                │                                 │
│                                └──▶ [ gravador de dataset ]      │
│                                        JPEG + JSONL              │
│                                                                  │
│                      [ planejamento + controle ]                 │
│                                  │                               │
│                           steer, throttle                        │
└──────────────────────────────────┼───────────────────────────────┘
                                   │ USB serial 115200 8N1
                                   │ $CMD ▼      ▲ $TLM
┌──────────────────────────────────┼───────────────────────────────┐
│  ESP32 — "a medula"              │                               │
│  Arduino / PlatformIO · tempo real                               │
│                                                                  │
│   [ parser do protocolo ] ─▶ [ rampas + limites ] ─▶ servo       │
│              │                                    └─▶ ESC/motor  │
│              ▼                                                   │
│   [ FAILSAFE 250 ms ]     [ ultrassônicos ×3 ] ──▶ telemetria    │
│   [ tensão da bateria ]   [ rádio RC (só treino) ]               │
└────────────────────────────────────────────────────────────────┘
```

### Por que dividir assim

O Jetson roda Linux, que **não é sistema de tempo real**. Um pico de garbage
collection ou do escalonador atrasa o loop em dezenas de milissegundos — o que
é irrelevante para uma CNN a 20 Hz e inaceitável para o sinal PWM de um servo.

A regra que organiza tudo: **o ESP32 nunca confia no Jetson.** Se o comando
parar de chegar por 250 ms, ele mesmo coloca o motor em neutro. Se o Jetson
pedir aceleração máxima, ele aplica o teto configurado. Se o ultrassônico
frontal acusar menos de 250 mm, ele corta a tração independentemente do que a
IA queira.

O failsafe vive num processador que roda poucas centenas de linhas de código,
auditáveis por inteiro, e continua funcionando mesmo com o Jetson travado.

---

## 3. Hardware

| Item | Especificação | Estado |
|---|---|---|
| Computador principal | Jetson Orin Nano 8 GB | ✅ definido |
| Câmera | IMX219 8 MP (3280×2464), lente **120°**, foco ajustável, CSI | ✅ definido |
| Microcontrolador | ESP32 DevKitC (WROOM-32) | ✅ definido |
| Chassi | RC 1:10, direção Ackermann | 🔧 a montar |
| Sensores de distância | 3× HC-SR04 (frontal esq., centro, dir.) | 🔧 a montar |
| Baterias | LiPo 2S (tração) + power bank USB-C (Jetson) — **separadas** | 🔧 a montar |
| Dimensões alvo | 420 × 195 mm (limite: **500 × 250 mm**) | ✅ dentro |

### Decisões de hardware já resolvidas

**Captura em 1640 × 1232, não em 1280 × 720.** No IMX219, os modos 1920×1080 e
1280×720 são *recorte* do sensor, não redução: usá-los descartaria parte dos
120° da lente. O modo 1640×1232 é binning 2×2 do array inteiro — campo
completo, 30 fps, e melhor desempenho sob luz fraca.

**A lente de 120° é viável para o MASTER.** Uma placa de 150 mm precisa de
~32 px para ser classificada; em 1640×1232 ela atinge esse tamanho a 2,2 m no
pior caso, o que deixa ~1,6 m de percurso até a esquina para confirmar e virar.
Verificado numericamente e coberto por testes.

**Ultrassônico, não capacitivo.** Sensor capacitivo tem alcance de 1–30 mm; a
1 m/s o carro percorre 30 mm em 30 ms, então quando ele disparasse a batida já
teria acontecido. O código trata "sensor de distância" genericamente, o que
permite migrar para ToF (VL53L1X) sem mexer no protocolo nem no dataset.

---

## 4. O que já está pronto

> ⚠️ **Ressalva importante:** tudo abaixo está implementado e validado **em
> software** — testes automatizados, verificação numérica, simulação. **Nada
> foi ainda testado no carro físico**, porque o carro ainda não está montado.
> O bring-up em hardware é a próxima etapa e vai revelar ajustes.

### 4.1 Protocolo de comunicação Jetson ↔ ESP32

Protocolo textual sobre USB serial (115200 8N1), no estilo NMEA: `$TIPO,campos*CS`
com checksum XOR. Escolha consciente por texto em vez de binário — dá para
depurar com um monitor serial comum.

| Mensagem | Direção | Função |
|---|---|---|
| `$CMD` | Jetson → ESP32 | Comando de esterço e aceleração (50 Hz) |
| `$TLM` | ESP32 → Jetson | Telemetria: valores aplicados, distâncias, bateria, flags (50 Hz) |
| `$CFG` | Jetson → ESP32 | Calibração em runtime (sem recompilar firmware) |
| `$ARM` | Jetson → ESP32 | Arma/desarma a tração |
| `$PING`/`$PONG` | ambos | Teste de enlace e latência |

**Implementado nos dois lados e verificado byte a byte:** os testes de host
compilam o código C++ do firmware no PC e comparam a saída com a do Python.
Uma divergência entre as duas implementações não apareceria em compilação nem
em teste de um lado só — apareceria como "o carro não responde" na véspera da
prova.

📄 Especificação completa: [`docs/03-protocolo-serial.md`](03-protocolo-serial.md)

### 4.2 Firmware do ESP32

- Geração de PWM para servo de direção e ESC, com **rampa** (slew rate)
- **Failsafe de 250 ms** — sem comando, motor em neutro
- **Tetos de segurança em tempo de compilação** que o Jetson só consegue
  apertar, nunca afrouxar
- 3 ultrassônicos lidos **por interrupção, em rodízio** (evita crosstalk e não
  bloqueia o loop)
- Corte de tração por obstáculo, leitura de tensão da bateria
- **LED de status** com padrões distintos — necessário porque na prova o carro
  roda sem SSH
- Entrada de rádio RC opcional (só para coleta de dados)

📄 [`firmware/esp32/README.md`](../firmware/esp32/README.md)

### 4.3 Pipeline de coleta de dados

O bloco mais importante do que já existe — **é ele que destrava todo o resto**.

```bash
robocar record --control rc --track minicidade_fatec --direction horario
```

- Captura de câmera CSI (GStreamer), USB (V4L2) ou replay de sessão gravada
- Gravação em **três threads**: captura → fila limitada → workers de escrita
- Formato: **JPEG + JSONL**, uma sessão por diretório
- **Filtros de qualidade automáticos**: descarta frames com carro parado,
  telemetria velha, failsafe ativo ou bateria crítica
- Metadados completos por sessão: pista, piloto, iluminação, sentido, commit do
  Git, snapshot da configuração
- Contadores de aproveitamento que denunciam problemas durante a coleta

**Decisão de projeto central:** os rótulos vêm da **telemetria** (o que o ESP32
efetivamente aplicou), nunca do comando enviado. Os dois divergem sempre que há
rampa, limite de velocidade, corte por obstáculo ou failsafe — e principalmente
quando quem pilota é o rádio RC, caso em que o Jetson não envia comando nenhum.
Treinar com o comando associaria a imagem a uma ação que o carro não executou,
e o erro não apareceria na curva de perda.

📄 [`docs/04-coleta-de-dados.md`](04-coleta-de-dados.md) · [`docs/05-formato-dataset.md`](05-formato-dataset.md)

### 4.4 Ferramentas de calibração e diagnóstico

| Comando | Função |
|---|---|
| `robocar doctor` | Confere ambiente, disco, porta serial e câmera antes de sair de casa |
| `robocar link --watch` | Telemetria do ESP32 em tempo real |
| `robocar camera --focus` | Medidor de nitidez ao vivo para ajustar a rosca de foco |
| `robocar camera --virtual` | Vista retificada das placas (câmera virtual) |
| `robocar calib steering` | Centro e batentes da direção |
| `robocar calib camera` | Intrínsecos e distorção da lente (tabuleiro de xadrez) |
| `robocar dataset stats --histogram` | Resumo do coletado + distribuição do esterço |
| `robocar dataset verify` | Integridade das sessões |

### 4.5 Câmera virtual — otimização por software da lente de 120°

A placa fica na borda direita do quadro, exatamente onde a distorção de barril
é mais forte: o círculo vira elipse e a seta entorta. A solução implementada
sintetiza, a partir do quadro grande angular, uma **vista retificada e apontada
para a direita** — como se houvesse uma segunda câmera estreita mirando onde as
placas ficam.

O classificador passa a ver a placa sempre no mesmo enquadramento e com
geometria correta, em vez de precisar aprender todas as deformações possíveis.
**Não há ganho de resolução** — retificar não cria pixels; o ganho é de
precisão.

📄 [`docs/02-hardware.md`](02-hardware.md)

### 4.6 Baseline de treinamento

- Loader do dataset com **divisão treino/validação por sessão inteira** (nunca
  por frame — frames vizinhos são quase idênticos e vazariam informação)
- **Balanceamento do histograma de esterço** — sem ele a rede aprende que
  "prever zero" minimiza a perda e o carro anda reto para dentro do quarteirão
- Rede PilotNet (~250 mil parâmetros), augmentação (brilho, sombra,
  deslocamento com correção de rótulo)
- Exportação para ONNX e caminho para TensorRT

📄 [`docs/06-treinamento.md`](06-treinamento.md)

### 4.7 Números do que existe hoje

| Área | Arquivos | Linhas |
|---|---:|---:|
| Software do Jetson (`jetson/robocar`) | 17 | 4 288 |
| Testes automatizados (`jetson/tests`) | 7 | 1 340 |
| Firmware do ESP32 | 8 | 1 086 |
| Treinamento (`ml/robocar_ml`) | 5 | 750 |
| Documentação (`docs/`) | 10 | 1 878 |
| Configuração (`config/`) | 4 | 295 |

**172 verificações automatizadas**: 146 testes Python + 26 do protocolo em C++.
Todos rodam **sem hardware**, o que permite desenvolver e revisar código longe
do carro. Integração contínua no GitHub Actions em 4 jobs.

---

## 5. O que ainda falta

Em ordem de dependência — cada bloco depende do anterior.

### 🔴 Prioridade 1 — Bring-up do hardware

> **Bloqueia literalmente todo o resto.** Nada do que está implementado foi
> testado no carro físico.

- [ ] Montar o chassi 1:10 com servo, ESC e motor
- [ ] Montar Jetson, câmera e ESP32 com **baterias separadas**
- [ ] Fixar a câmera em suporte **rígido** a ~180 mm de altura
- [ ] Divisores resistivos nos pinos ECHO dos ultrassônicos (5 V → 3,3 V)
- [ ] Gravar o firmware e validar `robocar link --watch`
- [ ] **Testar o failsafe** com o carro no cavalete (obrigatório)
- [ ] Calibrar direção (`robocar calib steering`)
- [ ] Focar e travar a lente (`robocar camera --focus`)
- [ ] Calibrar a lente (`robocar calib camera`)
- [ ] Primeira sessão de gravação de verdade

### 🔴 Prioridade 2 — Pista de treino própria

> A pista oficial só estará disponível **um único dia** (04/12). Sem uma
> réplica para treinar antes, chegaremos lá com o modelo zerado.

**Não é preciso replicar a cidade inteira** (9 × 13 m = 117 m²). O essencial é
**um cruzamento em escala correta**:

- [ ] Piso escuro (carpete/lona preta) com **faixas brancas** de fita
- [ ] Um cruzamento com as dimensões do regulamento
- [ ] **Réplicas das placas**: círculos de **150 mm** de diâmetro, borda
      inferior a **400 mm** do chão, em suportes de cano PVC
- [ ] **Semáforo**: 3 LEDs (vermelho/amarelo/verde), a 100 cm da largada
- [ ] Alguns quarteirões improvisados (caixas) para obstáculo visual

### 🟠 Prioridade 3 — Dataset de condução

- [ ] Coletar **2h30 no mínimo**, idealmente 4–6 h, em ≥ 4 dias diferentes
- [ ] Nos dois sentidos (horário e anti-horário)
- [ ] **20+ min de dados de recuperação** — gravando *só a correção* de volta
      ao centro, nunca o erro sendo cometido
- [ ] Iluminações e velocidades variadas
- [ ] Verificar o histograma de esterço a cada 2 sessões

### 🟠 Prioridade 4 — Modelo de direção funcionando

- [ ] Treinar a PilotNet com `--balance`
- [ ] Atingir **MAE < 0,08** em validação
- [ ] Validar por replay offline sobre sessão gravada
- [ ] Teste no cavalete (o servo responde no sentido certo?)
- [ ] Primeira volta autônoma com `throttle.limit_auto = 0.25`
- [ ] Subir a velocidade um degrau por vez

### 🟡 Prioridade 5 — Detector de placas (exclusivo do MASTER)

- [ ] **Confirmar a lista de classes** com o vídeo oficial
      (<https://youtu.be/YAg2p1kShZA>) e, se possível, por e-mail à organização
- [ ] Coletar **200+ imagens por classe**, de 0,5 a 4 m, vários ângulos
      — usando a nossa própria câmera (imagens da internet não têm a distorção
      da nossa lente)
- [ ] Anotar em formato YOLO
- [ ] Treinar detector/classificador leve
- [ ] Integrar com a câmera virtual
- [ ] Lógica de decisão de rota por cruzamento

### 🟡 Prioridade 6 — Detector de semáforo

- [ ] Gravar os 3 estados de várias distâncias, da posição de largada
- [ ] Detector HSV + confirmação em múltiplos quadros (anti-falso-positivo)
- [ ] Integrar à sequência de largada com timeout e sinalização por LED

### 🟢 Prioridade 7 — Loop autônomo completo

- [ ] Exportar o modelo para **TensorRT** (gerar o engine no próprio Jetson)
- [ ] Loop de condução integrando faixa + placas + semáforo + ultrassônicos
- [ ] **Autostart sem SSH**: liga, carrega, arma pelo botão, espera o verde
- [ ] Watchdog de câmera e comportamento em "perdi a pista" (parar > insistir)
- [ ] Teste de resistência: 3 voltas seguidas sem intervenção

### 🟢 Prioridade 8 — Preparação da competição

- [ ] Enviar as [dúvidas do regulamento](00-regulamento-resumo.md#9-pontos-a-confirmar-com-a-organização) à organização
- [ ] Inscrição no site (**não é possível trocar de categoria depois**)
- [ ] Termo de menor de idade, se aplicável (até 3 dias antes)
- [ ] Peças sobressalentes e ferramentas
- [ ] Ensaio completo do [checklist](07-checklist-competicao.md)

---

## 6. Cronograma sugerido — 16 semanas

| Fase | Período | Objetivo | Marco verificável |
|---|---|---|---|
| **1. Bring-up** | Ago (sem. 1–3) | Carro montado e respondendo | `robocar record` grava uma sessão real |
| **2. Pista de treino** | Set (sem. 4–5) | Réplica de um cruzamento | Placas e semáforo em escala montados |
| **3. Coleta + direção** | Set–Out (sem. 6–9) | Modelo de faixa funcionando | **Volta autônoma completa na réplica** |
| **4. Placas + semáforo** | Out–Nov (sem. 10–13) | Percepção do MASTER | Carro vira conforme a placa indica |
| **5. Integração** | Nov (sem. 14–15) | Sistema completo e robusto | 3 voltas seguidas sem tocar no carro |
| **6. Competição** | 04–05/12 | Coleta na pista real, retreino, prova | 🏁 |

O marco da fase 3 — **volta autônoma completa na réplica** — é o mais
importante do cronograma. Se ele não for atingido até meados de outubro, o
plano B é competir na categoria **JUNIOR** (rota conhecida, sem necessidade do
detector de placas). Essa decisão precisa ser tomada **antes da inscrição**,
porque a troca de categoria não é permitida depois.

---

## 7. Riscos e mitigações

| Risco | Impacto | Mitigação |
|---|---|---|
| Carro não montado a tempo | Bloqueia tudo | Prioridade máxima; nada de software novo antes disso |
| Sem pista de treino própria | Modelo chega zerado em 04/12 | Réplica de um cruzamento (fase 2) |
| Detector de placas não amadurece | Sem MASTER | Decidir por JUNIOR **antes da inscrição** |
| Modelo "anda reto" | Sai da pista na primeira curva | `--balance` + coletar mais curva + dados de recuperação |
| Jetson reinicia com o motor | Perde a volta | **Baterias separadas** (já previsto no projeto) |
| Câmera desfoca no transporte | Sessão inteira vira lixo | Travar a rosca + `robocar camera --focus` no checklist |
| Disco cheio no dia de treino | Perde horas de coleta | Planejar 200 GB + backup entre sessões |
| Iluminação do ginásio diferente | Modelo não generaliza | Coletar em condições variadas; exposição fixa |

---

## 8. Restrições do regulamento que viraram código

Três exigências não ficaram só na documentação:

1. **"Sensoriamento 100% embarcado, sem comunicação externa."**
   O perfil `config/profiles/race.yaml` desliga Wi-Fi, Bluetooth, rádio RC e
   teleoperação. Rodar a prova com outro perfil é risco de desclassificação.
2. **"Ação humana somente em treino e teste."**
   Rádio e gamepad existem apenas para a coleta; `rc_enable` é `0` por padrão.
3. **Limite de 500 × 250 mm.**
   A validação de configuração reprova dimensões declaradas acima disso.

**Verificação feita:** o regulamento **não impõe nenhuma restrição** de lente,
campo de visão, resolução ou número de câmeras nas categorias Júnior e Master —
pelo contrário, permite explicitamente "câmeras, GPS, Lidar e qualquer outro
tipo de sensoriamento, desde que embarcado". A única lista fechada de
componentes é da categoria NANO.

---

## 9. Como rodar

```bash
git clone <url> && cd C.A.FATEC.R.B

# Ferramentas do Jetson
pip install -r jetson/requirements.txt
pip install -e jetson
robocar doctor

# Firmware do ESP32
cd firmware/esp32 && pio run -t upload

# Desenvolvimento
make check      # lint + todos os testes
make test       # testes Python + testes de host do firmware
```

---

## 10. Documentação completa

| # | Documento | Sobre |
|---|---|---|
| 00 | [Resumo do regulamento](00-regulamento-resumo.md) | O que as regras exigem do software |
| 01 | [Arquitetura](01-arquitetura.md) | Divisão Jetson / ESP32 |
| 02 | [Hardware](02-hardware.md) | BOM, pinos, óptica, montagem |
| 03 | [Protocolo serial](03-protocolo-serial.md) | Contrato entre os dois processadores |
| 04 | [Coleta de dados](04-coleta-de-dados.md) | Procedimento e plano de coleta |
| 05 | [Formato do dataset](05-formato-dataset.md) | Esquema em disco |
| 06 | [Treinamento](06-treinamento.md) | Treinar, avaliar, exportar |
| 07 | [Checklist de competição](07-checklist-competicao.md) | Dias 04 e 05/12 |

**Decisões de arquitetura registradas (ADR):**
- [0001 — Jetson como cérebro, ESP32 como medula](adr/0001-jetson-como-cerebro-esp32-como-medula.md)
- [0002 — Rótulos vêm da telemetria, não do comando](adr/0002-rotulos-vem-da-telemetria.md)

---

## 11. Resumo em um parágrafo

O projeto tem **toda a infraestrutura de software pronta e testada**: protocolo
de comunicação verificado byte a byte entre Python e C++, firmware com failsafe
independente, pipeline completo de coleta de dados com filtros de qualidade
automáticos, ferramentas de calibração e um baseline de treinamento que já evita
as duas armadilhas clássicas do aprendizado por imitação. **O que falta é
físico**: montar o carro, construir uma réplica de cruzamento para treinar e
coletar dados. Nenhuma linha de código adicional resolve isso — e nenhum dos
modelos que faltam (placas, semáforo) pode ser feito antes de existir dado real
para treiná-los.
