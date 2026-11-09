"""Testes das regras de placa, do gerador de cenários e da iluminação."""

from __future__ import annotations

import math
from collections import Counter

import numpy as np
import pytest

from vista_sim.config import repo_root
from vista_sim.lighting import LightField, Occluder, irradiance, shadow_polygon
from vista_sim.route import RoutePath
from vista_sim.rules import SignalError, decide, signs_that_force
from vista_sim.scenario import generate
from vista_sim.track import SIGN_CLASSES, Track


@pytest.fixture(scope="module")
def track():
    return Track.load(repo_root() / "maps" / "robocar_master.yaml")


@pytest.fixture(scope="module")
def cenarios(track):
    return [generate(track.data, track.network, s) for s in range(40)]


# --- regras ----------------------------------------------------------------


def test_obrigacao_manda_na_manobra():
    tudo = {"frente", "esquerda", "direita"}
    assert decide("virar_esquerda", tudo) == "esquerda"
    assert decide("virar_direita", tudo) == "direita"
    assert decide("seguir_em_frente", tudo) == "frente"


def test_proibicao_so_vale_se_sobra_uma_saida():
    assert decide("proibido_seguir_em_frente", {"frente", "esquerda"}) == "esquerda"
    assert decide("proibido_virar_direita", {"direita", "esquerda"}) == "esquerda"
    with pytest.raises(SignalError):
        decide("proibido_virar_esquerda", {"frente", "esquerda", "direita"})   # ambíguo


def test_sem_placa_segue_em_frente():
    assert decide(None, {"frente", "direita"}) == "frente"
    with pytest.raises(SignalError):
        decide(None, {"esquerda", "direita"})          # T sem placa: não chuta


def test_placa_impossivel_e_erro():
    with pytest.raises(SignalError):
        decide("virar_esquerda", {"frente", "direita"})


def test_placas_que_forcam_cada_manobra():
    assert set(signs_that_force("esquerda", {"frente", "esquerda"})) == {"virar_esquerda", "proibido_seguir_em_frente"}
    assert signs_that_force("frente", {"frente", "esquerda", "direita"}) == ["seguir_em_frente"]


# --- cenários --------------------------------------------------------------


def test_toda_placa_da_rota_leva_a_manobra_da_rota(cenarios, track):
    net = track.network
    for sc in cenarios:
        for d in sc.decisions:
            arr = tuple(d["arrival"])
            assert decide(d["sign"], set(net.options(arr))) == d["maneuver"]


def test_rota_e_consistente_e_volta_a_largada(cenarios, track):
    """Uma aproximação usada duas vezes pede sempre a mesma manobra."""
    for sc in cenarios:
        assert sc.route[0] == sc.route[-1]
        seen = {}
        for a, b in zip(sc.route[:-1], sc.route[1:], strict=True):
            assert seen.setdefault(a, b) == b


def test_rota_entra_no_centro_da_cidade(cenarios):
    entram = sum(sc.meta["entra_na_cidade"] for sc in cenarios)
    assert entram >= 0.9 * len(cenarios)


def test_aparecem_as_seis_placas(cenarios):
    vistas = Counter(s["class"] for sc in cenarios for s in sc.signs)
    assert set(vistas) == set(SIGN_CLASSES)
    assert all(any(not s["on_route"] for s in sc.signs) for sc in cenarios[:10])   # distratoras


def test_placas_ficam_na_calcada_e_zebras_na_rua(cenarios, track):
    for sc in cenarios[:10]:
        xy = np.array([[s["x"], s["y"]] for s in sc.signs])
        assert not track.on_carpet(xy).any()
        for poly in sc.crosswalks:
            assert track.on_carpet(poly.mean(axis=0, keepdims=True)).all()


def test_predios_ficam_dentro_dos_quarteiroes(cenarios, track):
    quadras = [np.asarray(o["polygon"]) for o in track.data["obstacles"] if o["kind"] == "quadra"]
    for sc in cenarios[:10]:
        for b in sc.buildings:
            fp = b["footprint"]
            assert any(np.all((fp >= q.min(axis=0)) & (fp <= q.max(axis=0))) for q in quadras)


def test_mesma_semente_mesmo_cenario(track):
    a = generate(track.data, track.network, 123)
    b = generate(track.data, track.network, 123)
    assert a.route == b.route and a.signs == b.signs
    assert a.lighting == b.lighting


def test_caminho_da_rota_e_continuo(cenarios, track):
    sc = cenarios[0]
    rp = RoutePath(track.network, sc.route, sc.start_s, sc.start_s)
    gaps = np.hypot(*np.diff(rp.points, axis=0).T)
    assert gaps.max() < 0.05
    st = track.start
    assert np.hypot(*(rp.points[0] - [st["x"], st["y"]])) < 0.01
    assert np.hypot(*(rp.points[-1] - [st["x"], st["y"]])) < 0.01
    assert len(rp.junctions) == len(sc.decisions)


def test_carpete_rapido_bate_com_o_exato(track):
    rng = np.random.default_rng(0)
    xy = rng.uniform([-7, -5], [7, 5], size=(4000, 2))
    rapido, exato = track.on_carpet(xy), track.on_carpet_exact(xy)
    assert (rapido == exato).mean() > 0.99            # só erra a ~1 cm das bordas


# --- iluminação ------------------------------------------------------------


def test_irradiancia_cai_com_cos3():
    lamp = {"x": 0.0, "y": 0.0, "z": 4.0, "intensity": 1.0}
    assert float(irradiance(np.array(0.0), np.array(0.0), lamp)) == pytest.approx(1.0)
    th = math.atan2(4.0, 4.0)
    assert float(irradiance(np.array(4.0), np.array(0.0), lamp)) == pytest.approx(math.cos(th) ** 3)


def test_sombra_cai_do_lado_oposto_a_lampada():
    box = Occluder(np.array([[0, 0], [0.5, 0], [0.5, 0.5], [0, 0.5]], float), 0.0, 0.5)
    poly = shadow_polygon(box, (-3.0, 0.25, 4.0))      # lâmpada a oeste
    assert poly[:, 0].max() > 0.55                     # a sombra se estende para leste
    assert poly[:, 0].min() >= -1e-6


def test_predio_escurece_o_chao_atras_dele():
    cfg = {"lamps": [{"x": -3.0, "y": 0.0, "z": 4.0, "intensity": 1.0, "bgr": (1, 1, 1), "size": 0.2}],
           "ambient": 0.3, "ambient_bgr": (1, 1, 1), "exposure": 1.0}
    occ = [Occluder(np.array([[-0.25, -0.25], [0.25, -0.25], [0.25, 0.25], [-0.25, 0.25]], float), 0.0, 0.6)]
    bounds = {"x_min": -2, "x_max": 2, "y_min": -2, "y_max": 2}
    lf = LightField(cfg, occ, -2, 2, -2, 2, bounds)
    atras, frente = lf.at(0.4, 0.0).mean(), lf.at(-0.4, 0.0).mean()
    assert atras < 0.75 * frente


def test_sombra_do_proprio_carro_vai_para_frente_com_luz_atras():
    cfg = {"lamps": [{"x": -2.0, "y": 0.0, "z": 3.0, "intensity": 1.0, "bgr": (1, 1, 1), "size": 0.2}],
           "ambient": 0.2, "ambient_bgr": (1, 1, 1), "exposure": 1.0}
    bounds = {"x_min": -3, "x_max": 3, "y_min": -3, "y_max": 3}
    lf = LightField(cfg, [], -3, 3, -3, 3, bounds)
    body = np.array([[0.34, 0.12], [-0.06, 0.12], [-0.06, -0.12], [0.34, -0.12]])
    dark = lf.car_shadow((0.0, 0.0, 0.0), body, 0.17, (-0.6, -1.0), 0.02, (100, 100))
    row = int((0.42 + 0.6) / 0.02)                     # 42 cm à frente do eixo
    col = int((0.0 + 1.0) / 0.02)
    assert dark[row, col] > 0.3                        # sombra à frente do carro
    assert dark[int((-0.5 + 0.6) / 0.02), col] < 0.05  # e não atrás


# --- o carro cabe nas conversões --------------------------------------------


def _varre(track, center_x: float) -> list[tuple[str, str, str]]:
    """Leva o contorno do carro por TODAS as conversões da malha com o ponto
    ``center_x`` (à frente do eixo) em cima do trajeto; o eixo vem atrás como
    um reboque. Devolve as conversões em que o carro sai do asfalto."""
    from vista_sim.config import load_car
    from vista_sim.geometry import resample

    p = load_car()
    hw = p.width / 2
    c4 = np.array([[p.front_overhang, hw], [p.front_overhang, -hw], [-p.rear_overhang, -hw], [-p.rear_overhang, hw]])
    k = (np.arange(8) / 8)[:, None]
    local = np.concatenate([c4[i] + (c4[(i + 1) % 4] - c4[i]) * k for i in range(4)])
    net, bad = track.network, []
    for nid in net.nodes:
        for arr in net.arrivals(nid):
            for m, dep in net.options(arr).items():
                # passo fino: o "reboque" discretizado com passo grosso atrasa demais
                path = resample(np.vstack([net.lane(arr)[-30:], net.movement(arr, dep)[1:], net.lane(dep)[1:30]]),
                                0.005, False)
                t0 = (path[1] - path[0]) / np.linalg.norm(path[1] - path[0])
                axle = path[0] - center_x * t0
                tang = np.gradient(path, axis=0)
                for P, tg in zip(path, tang, strict=True):
                    if center_x > 0:
                        v = P - axle
                        axle = P - center_x * v / np.linalg.norm(v)
                    else:
                        v, axle = tg, P
                    h = math.atan2(v[1], v[0])
                    c, s = math.cos(h), math.sin(h)
                    if not track.on_carpet(local @ np.array([[c, s], [-s, c]]) + axle).all():
                        bad.append((nid, arr[0], m))
                        break
    return bad


def test_carro_cabe_em_todas_as_conversoes_com_o_centro_na_faixa(track):
    """O carro (40 × 24 cm, eixo traseiro) passa em todas as conversões se o
    CENTRO dele segue a trilha (é o que o oráculo controla)."""
    from vista_sim.config import load_car

    assert _varre(track, load_car().center_x) == []


def test_com_o_eixo_na_faixa_a_frente_raspa(track):
    """Por que não controlar o eixo: a frente, 34 cm adiante, balança para fora."""
    assert _varre(track, 0.0) != []
