"""O Vista completo: localização no mapa, cruzamento por odometria, placas."""

from __future__ import annotations

import math

import numpy as np
import pytest

from vista.mapa import LocalizadorGrafo
from vista.placas import DetectorVerdade, LeitorPlacas, decodificar_yolo, letterbox
from vista_sim.config import load_camera, load_car
from vista_sim.controle import Percepcao
from vista_sim.env import SimConfig, VistaSimEnv
from vista_sim.oracle import OraclePilot
from vista_sim.track import Track


@pytest.fixture(scope="module")
def track():
    from vista_sim.config import repo_root

    return Track.load(repo_root() / "maps" / "robocar_master.yaml")


# ---------------- detector ----------------


def test_letterbox_igual_ao_ultralytics():
    img = np.zeros((308, 410, 3), np.uint8)
    out, r, (px, py) = letterbox(img, (416, 416))
    assert out.shape == (416, 416, 3)
    assert r == pytest.approx(416 / 410)
    assert px == 0 and py == 51                      # 313 px de altura útil: 51 em cima, 52 embaixo


def test_decodificar_yolo_desfaz_letterbox_e_faz_nms():
    nomes = {0: "virar_esquerda", 1: "semaforo_verde"}
    r, pad = 2.0, (0.0, 10.0)
    # 3 candidatos: dois quase iguais da classe 0 (NMS deixa 1) e um da classe 1
    cand = np.array([
        [100, 60, 20, 20, 0.9, 0.0],
        [101, 61, 20, 20, 0.8, 0.0],
        [300, 100, 40, 80, 0.0, 0.7],
        [200, 200, 10, 10, 0.1, 0.1],                # abaixo do limiar
    ], np.float32)
    saida = cand.T[None]                              # (1, 4 + nc, N), como o YOLO11 exporta
    d = sorted(decodificar_yolo(saida, r, pad, nomes, conf=0.35), key=lambda x: x.classe)
    assert [x.classe for x in d] == ["semaforo_verde", "virar_esquerda"]
    ve = d[1]
    assert ve.conf == pytest.approx(0.9)
    assert ve.caixa == pytest.approx((45.0, 20.0, 55.0, 30.0))   # ((100−10)/2, (60−10−10)/2, …)


# ---------------- decisão com reserva ----------------


def _leitor(track):
    return LeitorPlacas(track.network, track.data, load_camera())


def test_decidir_sem_placa_vai_em_frente(track):
    lp = _leitor(track)
    assert lp.decidir({"frente", "esquerda", "direita"}) == ("frente", None, False)


def test_decidir_ignora_leitura_impossivel(track):
    lp = _leitor(track)
    lp.votos.update({"virar_esquerda": 5.0, "virar_direita": 2.0})
    # não há esquerda: a leitura mais votada é erro de detecção, vale a próxima
    assert lp.decidir({"frente", "direita"}) == ("direita", "virar_direita", False)


def test_decidir_proibicao_ambigua_usa_reserva_registrada(track):
    lp = _leitor(track)
    lp.votos.update({"proibido_seguir_em_frente": 3.0})
    m, placa, reserva = lp.decidir({"frente", "esquerda", "direita"})
    assert reserva and placa == "proibido_seguir_em_frente" and m == "direita"


def test_placa_pouco_vista_nao_conta(track):
    lp = _leitor(track)
    lp.votos.update({"virar_esquerda": 0.5})           # uma leitura fraca
    assert lp.decidir({"frente", "esquerda"}) == ("frente", None, False)


# ---------------- placa da MINHA aproximação ----------------


def test_leitor_vota_so_na_placa_da_aproximacao(track):
    """Com a pose certa, a placa lida é a da rota — nunca a distratora de outra rua."""
    env = VistaSimEnv(track=track, config=SimConfig(), seed=100000)
    obs, info = env.reset(seed=100000)
    tr = info["truth"]
    pil = OraclePilot(track.network, info["scenario"].signs, env.car_params)
    pil.reset(tr.x, tr.y, tr.yaw)
    lp = LeitorPlacas(track.network, track.data, env.cam_params)
    det = DetectorVerdade()
    lidas: dict[int, str | None] = {}
    j_atual, aprox = None, None
    for _ in range(1500):
        cmd = pil.act(tr.x, tr.y, tr.yaw, env.t, tr.traffic_light == "verde")
        obs, tel, tr, done, _ = env.step(*cmd)
        e = env.route.edge_at(tr.route_s)
        nj = tr.next_junction
        if e is None or nj is None:
            continue
        if aprox != e:
            if j_atual is not None:
                lidas[j_atual] = lp.placa()
            lp.nova_aproximacao(e)
            aprox, j_atual = e, nj["index"]
        det.objetos = tr.objects
        lp.observar(det(), (tr.x, tr.y, tr.yaw))
        if done:
            break
    juncs = env.route.junctions
    assert len(lidas) >= 3
    for i, placa in lidas.items():
        assert placa == juncs[i].sign, (i, placa, juncs[i].sign)


# ---------------- localização no mapa ----------------


def test_localizador_corrige_pose_pela_faixa(track):
    """Pose inicial errada (5 cm, 4°): a percepção da faixa a puxa de volta."""
    car = load_car()
    net = track.network
    st = track.start
    loc = LocalizadorGrafo(net, car)
    loc.reset(st["x"], st["y"], st["yaw"])
    verdade = (loc.x, loc.y, loc.psi)
    pr = loc.projetar()
    e_y0, s0 = pr.e_y, loc.s
    loc.x += 0.05 * math.sin(st["yaw"])
    loc.psi += math.radians(4)
    dist_v = loc.comprimento_faixa - s0
    for _ in range(40):
        loc.corrigir(Percepcao(e_y0, 0.0, 0.0, dist_v, False))
    assert math.hypot(loc.x - verdade[0], loc.y - verdade[1]) < 0.005
    assert abs(math.degrees(math.remainder(loc.psi - verdade[2], 2 * math.pi))) < 0.2


def test_localizador_segue_o_grafo(track):
    car = load_car()
    loc = LocalizadorGrafo(track.network, car)
    st = track.start
    loc.reset(st["x"], st["y"], st["yaw"])
    opc = loc.opcoes()
    assert set(opc) <= {"frente", "esquerda", "direita"} and opc
    m, saida = next(iter(opc.items()))
    loc.entrar_na_rua(saida)
    assert loc.edge == saida


# ---------------- o Vista inteiro ----------------


def test_vista_completo_com_percepcao_perfeita_cumpre_cruzamentos(track):
    """Rede de faixa e detector perfeitos: o Vista larga no verde, lê as placas
    certas e faz os cruzamentos pela curva do mapa com odometria."""
    from treino.avaliacao import FaixaVerdade
    from vista.piloto import VistaPiloto

    env = VistaSimEnv(track=track, config=SimConfig(), seed=100001)
    obs, info = env.reset(seed=100001)
    tr, tel = info["truth"], info["telemetry"]
    faixa, det = FaixaVerdade(), DetectorVerdade()
    vista = VistaPiloto(track, env.car_params, env.cam_params, faixa, det)
    vista.reset()
    for _ in range(30 * 60):
        faixa.tr, det.objetos = tr, tr.objects
        obs, tel, tr, done, inf = env.step(*vista.act(obs, tel))
        if done or len(vista.leituras) >= 4 and vista.estado == "faixa":
            break
    tipos = [e["tipo"] for e in env.events]
    assert "largou_antes_do_verde" not in tipos
    assert inf["reason"] in ("", "rota_completa"), env.events[-3:]
    juncs = env.route.junctions
    for le, j in zip(vista.leituras, juncs, strict=False):
        assert le.manobra == j.maneuver and le.placa == j.sign
    assert len(vista.leituras) >= 3
