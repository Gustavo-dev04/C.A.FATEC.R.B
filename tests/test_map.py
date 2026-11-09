"""Testes do mapa oficial — travam o que foi lido no regulamento.

Se alguém mudar uma cota no gerador, estes testes dizem exatamente qual
número do regulamento deixou de ser respeitado.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from vista_sim.maps.build_master import REGULATION, build
from vista_sim.roads import RoadNetwork
from vista_sim.track import SIGN_CLASSES


@pytest.fixture(scope="module")
def mapa():
    return build()


def test_cidade_tem_9_por_13_metros(mapa):
    """3.4.1: 'O tamanho total da cidade será de 9,0 m × 13,0 m'."""
    b = mapa["bounds"]
    assert b["x_max"] - b["x_min"] == pytest.approx(13.0)
    assert b["y_max"] - b["y_min"] == pytest.approx(9.0)


def test_quarteiroes_tem_2_por_4_metros(mapa):
    """Legenda da Figura 7: quarteirões de 2,0 × 4,0 m."""
    quadras = [np.asarray(o["polygon"]) for o in mapa["obstacles"] if o["kind"] == "quadra"]
    assert len(quadras) == 4
    for q in quadras:
        size = q.max(axis=0) - q.min(axis=0)
        assert sorted(size) == pytest.approx([2.0, 4.0])


def test_anel_externo_tem_100_cm(mapa):
    """Figura 6: cota '100' entre a borda e os quarteirões."""
    quadras = [np.asarray(o["polygon"]) for o in mapa["obstacles"] if o["kind"] == "quadra"]
    xmax = max(q[:, 0].max() for q in quadras)
    ymax = max(q[:, 1].max() for q in quadras)
    assert mapa["bounds"]["x_max"] - xmax == pytest.approx(1.0)
    assert mapa["bounds"]["y_max"] - ymax == pytest.approx(1.0)


def test_vias_norte_sul_100_cm_e_leste_oeste_80_cm(mapa):
    """Figura 6: '100 | 100' ao lado do canteiro VERTICAL; '80' ao lado do HORIZONTAL."""
    obs = {o["kind"]: [] for o in mapa["obstacles"]}
    for o in mapa["obstacles"]:
        obs[o["kind"]].append(np.asarray(o["polygon"]))
    quadra_ne = next(q for q in obs["quadra"] if q[:, 0].min() > 0 and q[:, 1].min() > 0)
    canteiro_vert = next(c for c in obs["canteiro"] if c[:, 1].min() > 0 and abs(c[:, 0].mean()) < 1e-9)
    canteiro_hor = next(c for c in obs["canteiro"] if c[:, 0].min() > 0 and abs(c[:, 1].mean()) < 1e-9)
    via_ns = quadra_ne[:, 0].min() - canteiro_vert[:, 0].max()
    via_lo = quadra_ne[:, 1].min() - canteiro_hor[:, 1].max()
    assert via_ns == pytest.approx(1.00)
    assert via_lo == pytest.approx(0.80)


def test_folga_de_80_cm_ate_a_ilha(mapa):
    """Figura 6: '80 | Fatec | 80' entre as pontas dos canteiros e a ilha."""
    ilha = np.asarray(next(o["polygon"] for o in mapa["obstacles"] if o["kind"] == "ilha"))
    r = np.hypot(ilha[:, 0], ilha[:, 1]).max()
    canteiro_hor = [np.asarray(o["polygon"]) for o in mapa["obstacles"]
                    if o["kind"] == "canteiro" and np.asarray(o["polygon"])[:, 0].min() > 0]
    ponta = min(c[:, 0].min() for c in canteiro_hor)
    assert ponta - r == pytest.approx(0.80, abs=0.01)


# --- placas ---------------------------------------------------------------


def test_sao_seis_classes_de_placa():
    """Figura 8 (detalhe): 3 de obrigação + 3 de proibição.

    O simulador do organizador só tem 3 (direita/esquerda/frente). As de
    proibição mudam a lógica de decisão: 'proibido seguir em frente' num
    cruzamento OBRIGA a virar.
    """
    assert len(SIGN_CLASSES) == 6
    base = {"seguir_em_frente", "virar_esquerda", "virar_direita"}
    assert base <= set(SIGN_CLASSES)
    assert {f"proibido_{c}" for c in base} <= set(SIGN_CLASSES)


def test_placa_tem_150_mm_com_a_base_a_400_mm(mapa):
    """3.5.2: Ø150 e 'altura do chão até o ponto INFERIOR: 400 mm'.

    Logo o CENTRO fica a 475 mm. O .obj do organizador tem Ø300 com centro
    a 550 mm; o CLAUDE.md anotou 'centro a 400' — ambos divergem.
    """
    s = mapa["sign_spec"]
    assert s["diameter"] == pytest.approx(0.150)
    assert s["bottom_height"] == pytest.approx(0.400)
    assert s["bottom_height"] + s["diameter"] / 2 == pytest.approx(0.475)


def test_placa_400_mm_antes_do_cruzamento_a_direita(mapa):
    """3.5.2: '400 mm antes do cruzamento, do lado direito, tangenciando a calçada'.

    A posição é CALCULADA para cada aproximação de cada cruzamento — vale para
    qualquer rota que a organização montar no dia.
    """
    net = RoadNetwork(mapa["roads"])
    spec = mapa["sign_spec"]
    for nid in net.nodes:
        for arr in net.arrivals(nid):
            p, h, w = net.approach_frame(arr)              # entrada do cruzamento
            sg = net.sign_pose(arr, spec["before_crossing"], spec["outward"])
            road = net.roads[arr[0]]
            # 400 mm ANTES, medidos ao longo da rua
            s_in = road.length - net._trim[(road.id, "b")] if arr[1] else net._trim[(road.id, "a")]
            s_sg = s_in - 0.400 if arr[1] else s_in + 0.400
            cx, cy, hh = road.pose_at(s_sg)
            hh = hh if arr[1] else hh + math.pi
            d = np.array([sg["x"] - cx, sg["y"] - cy])
            assert d @ [math.cos(hh), math.sin(hh)] == pytest.approx(0.0, abs=1e-6)
            assert d @ [-math.sin(hh), math.cos(hh)] == pytest.approx(-(w / 2 + spec["outward"]))   # à direita
            assert math.cos(sg["facing_yaw"] - hh) == pytest.approx(-1.0)                          # virada para quem chega


def test_toda_placa_fica_fora_da_rua(mapa):
    """Em qualquer aproximação — inclusive nas vias com dobra em volta da ilha —
    a placa cai na calçada/ilha/canteiro, nunca no asfalto."""
    from vista_sim.track import Track

    tr = Track(mapa)
    for nid in tr.network.nodes:
        for arr in tr.network.arrivals(nid):
            sg = tr.network.sign_pose(arr, mapa["sign_spec"]["before_crossing"], mapa["sign_spec"]["outward"],
                                      lambda x, y: bool(tr.on_carpet_exact(np.array([[x, y]]))[0]))
            assert not tr.on_carpet_exact(np.array([[sg["x"], sg["y"]]]))[0], (nid, arr)


# --- semáforo -------------------------------------------------------------


def test_semaforo_amarelo_a_390_mm_do_chao(mapa):
    """Figura do item 3.6: a cota 390 vai do chão ao CENTRO da lente amarela.

    Medido na figura: espaçamentos de 356,5 e 348,5 px para 140 mm. Logo os
    centros ficam a 530 / 390 / 250 mm — não 725 / 585 / 445 como estava no
    CLAUDE.md, que tratava 390 como a base da lente de baixo.
    """
    tl = mapa["traffic_light"]
    assert tl["yellow_center_height"] == pytest.approx(0.390)
    assert tl["lens_diameter"] == pytest.approx(0.110)
    assert tl["lens_spacing"] == pytest.approx(0.140)
    centros = [tl["yellow_center_height"] + k * tl["lens_spacing"] for k in (1, 0, -1)]
    assert centros == pytest.approx([0.530, 0.390, 0.250])


def test_semaforo_a_100_cm_da_largada(mapa):
    """3.6: 'Estará localizado a 100 cm do ponto de largada'."""
    frente_do_carro = mapa["start"]["x"] - 0.340      # o carro larga para oeste
    assert frente_do_carro - mapa["traffic_light"]["x"] == pytest.approx(REGULATION["tl_distance_from_start"])


def test_sequencia_do_semaforo(mapa):
    assert mapa["traffic_light"]["sequence"] == ["vermelho", "amarelo", "verde"]


# --- faixas ---------------------------------------------------------------


def test_faixa_da_direita_fica_no_meio_da_metade_da_rua(mapa):
    """Mão direita (3.4.1): indo para oeste no anel de cima, a faixa é a EXTERNA."""
    net = RoadNetwork(mapa["roads"])
    oeste = net.lane(("anel_T_ne_T_no", True))
    leste = net.lane(("anel_T_ne_T_no", False))
    assert oeste[:, 1].mean() == pytest.approx(4.25, abs=0.01)
    assert leste[:, 1].mean() == pytest.approx(3.75, abs=0.01)
    assert oeste[-1, 0] < oeste[0, 0]                         # vai para oeste


def test_todo_cruzamento_tem_saida_sem_retorno(mapa):
    net = RoadNetwork(mapa["roads"])
    assert len(net.nodes) == 12                                # 8 T no anel + 4 em volta da ilha
    for nid in net.nodes:
        for arr in net.arrivals(nid):
            ops = net.options(arr)
            assert 2 <= len(ops) <= 3
            assert set(ops) <= {"frente", "esquerda", "direita"}


def test_conversao_emenda_as_duas_faixas(mapa):
    net = RoadNetwork(mapa["roads"])
    for nid in net.nodes:
        for arr in net.arrivals(nid):
            for dep in net.options(arr).values():
                mv = net.movement(arr, dep)
                assert np.allclose(mv[0], net.lane(arr)[-1])
                assert np.allclose(mv[-1], net.lane(dep)[0])


def test_largada_esta_na_faixa_da_direita(mapa):
    st = mapa["start"]
    assert st["yaw"] == pytest.approx(math.pi)       # sai para oeste (Fig. 10/11)
    assert st["y"] == pytest.approx(4.25)            # faixa externa = direita indo para oeste
