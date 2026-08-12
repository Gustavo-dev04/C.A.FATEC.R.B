# Como trabalhar neste repositório

## Antes de commitar

```bash
make check    # ruff + testes Python + testes de host do firmware
```

Se `make check` não passar, o commit quebra a CI.

## Branches

```
main                      sempre funcional — é o que vai para a competição
feature/<descrição>       nova funcionalidade
fix/<descrição>           correção
```

Nunca commite direto na `main` depois de novembro: perto da competição, ela
precisa ser sempre o código que sabemos que funciona.

## Mensagens de commit

Em português, no imperativo, dizendo **o que muda**:

```
adiciona filtro de telemetria velha no gravador
corrige inversao do sinal de esterco na calibracao
documenta o formato do dataset de placas
```

Se a mudança mexe no protocolo, diga isso no corpo — é a coisa mais fácil de
quebrar sem perceber.

## Regras que valem a pena respeitar

### 1. O protocolo serial tem dois lados

Mexeu em `jetson/robocar/comms/protocol.py`? Mexa também em
`firmware/esp32/src/protocol.cpp`, atualize `docs/03-protocolo-serial.md` e os
testes — **na mesma alteração**.

Os testes de host (`make test-firmware`) compilam o C++ e comparam os bytes
gerados com os do Python. É a rede de proteção contra os dois lados
divergirem, mas ela só funciona se os vetores de teste forem atualizados.

### 2. Calibração mora em `config/`, não no código

Qualquer número que descreva o carro físico (pulso de servo, dimensão, limite
de velocidade) vai para `config/vehicle.yaml`. Constante mágica no meio do
código é como se perde meia hora de bancada procurando "onde está esse 1500".

### 3. O núcleo não ganha dependências

`robocar.comms.protocol`, `robocar.capture.schema` e `robocar.config` usam
**só a biblioteca padrão** (+ PyYAML no config). É isso que permite rodar os
testes na CI sem OpenCV, sem torch e sem hardware, e inspecionar datasets em
qualquer máquina.

OpenCV, pyserial e evdev são importados **dentro das funções** que precisam
deles, nunca no topo do módulo.

### 4. Nada de dado no Git

`data/` e `models/` estão no `.gitignore`. Se precisar compartilhar dataset,
use o HD externo do time (ver `data/README.md`).

### 5. Decisão relevante vira ADR

Escolha de arquitetura que alguém vá questionar em dois meses merece um
arquivo em `docs/adr/`. Formato: contexto, decisão, justificativa,
consequências. Curto — uma página basta.

## Testes

| Comando | O que roda |
|---|---|
| `make test-python` | Testes do pacote do Jetson (sem hardware) |
| `make test-firmware` | Codec do protocolo em C++, compilado no PC |
| `make test` | Os dois |

Ao corrigir um bug, escreva primeiro o teste que falha. Bug de protocolo e de
esquema de dataset são especialmente fáceis de testar e especialmente caros de
descobrir na pista.

## Estilo

- Python: `ruff` decide. Rode `make format`.
- Identificadores em **inglês**, comentários e documentação em **português**.
- Comentário explica **por quê**, não **o quê**. `# incrementa i` não ajuda
  ninguém; `# rampa evita que a roda perca aderência no arranque` ajuda.
