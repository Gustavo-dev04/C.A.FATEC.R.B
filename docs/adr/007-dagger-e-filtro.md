# ADR 007 — DAgger e filtro com odometria + giroscópio

**Data:** 02/11/2026 · **Estado:** aceito (aprovado pela equipe)

## Contexto

Primeiro treino no Colab: a rede de faixa erra ~0,3 cm por imagem na
validação, mas dirigindo sozinha completa **4/10** rotas (o mesmo piloto com a
verdade: 10/10). 5 das 6 falhas: o carro se perde **dentro do cruzamento**.

Dois efeitos, medidos (diário, 02/11 noite):

1. **Mudança de distribuição.** Os dados vêm do oráculo dirigindo bem. Quando a
   rede dirige, erra um pouco e chega a situações que nunca viu — e erra mais.
   Ruído branco do tamanho do erro de validação ainda dá 17/20; a rede dá 4/10.
2. **O controle amplifica ruído.** O centro do carro está a 14 cm do eixo:
   1 cm de `e_y` vira ~0,3 rad/s de giro. Com ruído 2× maior, 8/20 rotas.

## Decisão

### Filtro (`vista_sim/controle.py::FiltroFaixa`) — vai para o carro

Filtro complementar de `e_y`/`e_ψ`. A cada quadro o **próprio carro** prevê o
novo erro pelo seu movimento — `v` dos encoders, `ω` do giroscópio com o viés
calibrado no vermelho (ADR 005) — e a rede só **corrige** a previsão:

    e_ψ ← e_ψ + (ω − κ·v)·dt          e_y ← e_y + (v·sen e_ψ + ω·d·cos e_ψ)·dt
    x   ← x + g·(rede − x)            g = 0,25 na faixa, 0,12 no cruzamento

- Atenua o ruído **sem atrasar** uma curva de verdade (a previsão já gira junto;
  uma média móvel com o mesmo ganho ficaria ~3° atrás — teste).
- Se a rede discordar da previsão por mais de 10 cm, aceita a rede (algo mudou).
- Verdade + ruído 0,8/2,5 cm: 8/20 → **14/20**.

### DAgger (`tools/gerar_dataset.py --piloto-onnx`) — só no simulador

Dataset Aggregation: a **rede dirige** (com o filtro), o oráculo calcula seu
comando em paralelo e **assume** quando a rede passa de 6 cm ou 25°, devolvendo
quando o carro estabiliza (< 2 cm e < 6° por 0,5 s). Toda imagem recebe o
rótulo exato da verdade e a coluna `piloto` (quem dirigia). A rede é
**ajustada** a partir do modelo atual com todos os dados (oráculo + DAgger),
e a rodada se repete.

- Sementes de DAgger em 500.000+; a validação é dividida **por bloco** de
  100.000 sementes, então a validação dos dados do oráculo não muda entre
  rodadas.
- O notebook escolhe a rodada com mais rotas completas → `faixa_final.onnx`.

## Consequências

- O oráculo continua existindo **só no simulador**; no carro roda rede →
  filtro → controle, com encoders e MPU6050.
- Cada rodada custa ~25 min no Colab (gerar com a rede dirigindo é mais lento:
  a rede roda a cada quadro).
- Se o cruzamento continuar sendo o problema, a próxima opção é fazer a curva
  dentro dele por giroscópio + odometria, usando a rede só para entrar e sair.
