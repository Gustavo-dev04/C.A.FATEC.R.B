# Análise de campo de visão — o pitch da câmera

Responde ao "ALERTA GEOMÉTRICO (prioridade 1)" do CLAUDE.md §3.4.

## Resultado

**O semáforo não restringe o pitch.** Com os centros das lentes nas alturas
corretas, a lente vermelha cabe no quadro até pitch ≈ 17°. O alerta original
partia de alturas erradas e concluía o contrário.

## De onde vinha o erro

O CLAUDE.md lia a cota "390" da figura do semáforo como a base da lente de
baixo, e chegava a centros em 725 / 585 / 445 mm. Medindo a figura por pixel,
**390 vai do chão ao centro da lente amarela** — centros em **530 / 390 /
250 mm**. Ver `docs/regulamento.md`.

| | Altura da vermelha | Elevação na largada (1,086 m) | Pitch máximo |
|---|---|---|---|
| CLAUDE.md | 725 mm | 27,7° | ~8° |
| **Regulamento** | **530 mm** | **19,0°** | **~17°** |

## Premissas

- Câmera a 156 mm do chão, 1,086 m do semáforo na largada (CLAUDE.md §3.4).
- Lente fisheye equidistante, 120° na diagonal do sensor 4:3 →
  **≈ 96° H × 72° V**. Meio-FOV vertical = 36°.
- Borda superior do quadro, no centro: `36° − pitch`.

⚠️ **Os intrínsecos ainda são estimados**, não calibrados. A conclusão
qualitativa é robusta (a folga passou de −8° para +2° a +7°), mas os números
finais saem de `cv2.fisheye.calibrate`. Quando a calibração existir, os testes
`tests/test_camera.py::test_lente_vermelha_*` recalculam tudo sozinhos — eles
leem `config/camera/intrinsics.yaml`.

## O trade-off que sobra

Sem o semáforo restringindo, o pitch vira uma escolha entre ver o chão perto
(bom para a faixa) e manter a placa no quadro por mais tempo:

| Pitch | Borda superior | Vermelha cabe? | Chão visível a partir de | Placa (topo 550 mm) sai a |
|---|---|---|---|---|
| 10° | 26,0° | sim | 0,150 m | 0,81 m |
| 12° | 24,0° | sim | 0,140 m | 0,88 m |
| **15°** (desenho) | 21,0° | sim, folga de 2° | 0,126 m | 1,02 m |
| 17° | 19,0° | no limite | 0,118 m | 1,14 m |

A placa fica 400 mm antes da esquina. Perdê-la a ~1 m deixa a decisão já
travada com folga — a leitura acontece entre 1,5 e 2,5 m.

**Recomendação:** manter os **15° do desenho**. A única folga apertada é a da
lente vermelha (2°), e para largar basta detectar o **verde** com segurança
(CLAUDE.md F4) — e o verde fica a 4,95°, sempre no quadro.

Decisão final é do time, depois da calibração. **Todo o dataset deve usar o
mesmo pitch do carro final.**
