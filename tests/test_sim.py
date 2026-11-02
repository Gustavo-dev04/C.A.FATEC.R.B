"""Testes da cinemática, do ambiente e do renderizador."""

from __future__ import annotations

import math

import numpy as np
import pytest

from vista_sim.car import DiffDriveCar
from vista_sim.config import load_car
from vista_sim.env import SimConfig, VistaSimEnv
from vista_sim.geometry import Polyline, rounded_rect
from vista_sim.oracle import OraclePilot


@pytest.fixture(scope="module")
def car_params():
    return load_car()


# --- geometria ------------------------------------------------------------


def test_retangulo_arredondado_e_anti_horario():
    pts = rounded_rect(0, 0, 2, 1, 0.3)
    area = 0.5 * np.sum(pts[:, 0] * np.roll(pts[:, 1], -1) - np.roll(pts[:, 0], -1) * pts[:, 1])
    assert area > 0                                   # anti-horário ⇒ área positiva


def test_erro_lateral_positivo_a_esquerda():
    lane = Polyline(np.array([[0.0, 0.0], [10.0, 0.0]]), closed=False)
    assert lane.project(5.0, 0.2).e_y == pytest.approx(0.2)
    assert lane.project(5.0, -0.1).e_y == pytest.approx(-0.1)
    assert lane.project(5.0, 0.0).s == pytest.approx(5.0)


def test_curvatura_do_anel_tem_o_sinal_certo():
    """Anti-horário = curvas à ESQUERDA = curvatura positiva."""
    lane = Polyline(rounded_rect(0, 0, 3, 2, 0.75, 0.02), closed=True)
    k = lane.project(*lane.point_at(lane.length * 0.0 + 1.5)[:2]).curvature
    ks = [lane.project(*lane.point_at(s)[:2]).curvature for s in np.linspace(0, lane.length, 200)]
    assert max(ks) == pytest.approx(1 / 0.75, rel=0.15)
    assert min(ks) > -0.1
    assert k >= -0.1


# --- carro ----------------------------------------------------------------


def test_velocidade_maxima_bate_com_o_motor(car_params):
    """530 rpm a 12 V → ~486 rpm a 11 V → ~1,65 m/s na roda Ø65 (CLAUDE.md §5A)."""
    assert car_params.wheel_speed_max == pytest.approx(1.65, abs=0.02)


def test_bitola_e_dimensoes_do_desenho(car_params):
    assert car_params.track_width == pytest.approx(0.214)
    assert car_params.wheel_radius == pytest.approx(0.0325)
    assert car_params.length == pytest.approx(0.400)
    assert car_params.width == pytest.approx(0.241)
    assert car_params.length <= 0.500 and car_params.width <= 0.250   # regulamento


def test_anda_reto(car_params):
    car = DiffDriveCar(car_params)
    car.reset(0, 0, 0)
    car.command(0.5, 0.0)
    for _ in range(400):
        car.step(0.005)
    s = car.state
    assert s.y == pytest.approx(0.0, abs=1e-9)
    assert s.v == pytest.approx(0.5, abs=1e-3)
    assert s.x > 0.8


def test_resposta_de_primeira_ordem(car_params):
    """Em t = τ a roda atinge ~63% do degrau (sem bater no limite de aceleração)."""
    car = DiffDriveCar(car_params)
    car.reset(0, 0, 0)
    car.command(0.2, 0.0)
    dt = 0.001
    for _ in range(int(car_params.time_constant / dt)):
        car.step(dt)
    assert car.state.v / 0.2 == pytest.approx(1 - math.exp(-1), abs=0.03)


def test_curva_com_raio_v_sobre_omega(car_params):
    car = DiffDriveCar(car_params)
    car.reset(0, 0, 0)
    v, w = 0.4, 0.8                                   # raio 0,5 m
    car.command(v, w)
    for _ in range(4000):
        car.step(0.0025)
    # Depois do transitório, o carro gira num círculo de raio v/ω. Amostra uma
    # volta COMPLETA (2π/ω ≈ 7,9 s): com um arco só, o centro da caixa
    # envolvente não é o centro do círculo.
    xs, ys = [], []
    for _ in range(int(2 * math.pi / w / 0.0025) + 50):
        car.step(0.0025)
        xs.append(car.state.x)
        ys.append(car.state.y)
    xs, ys = np.array(xs), np.array(ys)
    cx, cy = (xs.max() + xs.min()) / 2, (ys.max() + ys.min()) / 2
    r = np.hypot(xs - cx, ys - cy).mean()
    assert r == pytest.approx(v / w, rel=0.02)


def test_saturacao_preserva_a_curvatura(car_params):
    car = DiffDriveCar(car_params)
    vl, vr = car.wheel_targets(5.0, 5.0)              # pede o impossível
    assert max(abs(vl), abs(vr)) == pytest.approx(car_params.wheel_speed_max)
    k_pedida = 5.0 / 5.0
    v, w = (vl + vr) / 2, (vr - vl) / car_params.track_width
    assert w / v == pytest.approx(k_pedida)


def test_encoder_conta_por_metro(car_params):
    car = DiffDriveCar(car_params)
    car.reset(0, 0, 0)
    car.command(0.5, 0.0)
    for _ in range(2000):
        car.step(0.002)
    por_metro = car_params.counts_per_rev / (2 * math.pi * car_params.wheel_radius)
    assert car.state.ticks_left == pytest.approx(car.state.x * por_metro, abs=2)


# --- ambiente -------------------------------------------------------------


@pytest.fixture(scope="module")
def env():
    return VistaSimEnv(config=SimConfig(latency_s=0.10), seed=0)


def test_reset_devolve_imagem_e_verdade(env):
    obs, info = env.reset(seed=1)
    assert obs.shape == (env.cam_params.height, env.cam_params.width, 3)
    assert info["truth"].on_carpet
    assert info["truth"].traffic_light == "vermelho"
    assert abs(info["truth"].e_y) < 1e-6


def test_semaforo_segue_vermelho_amarelo_verde(env):
    env.reset(seed=2)
    vistos = []
    for _ in range(int(12 * env.cfg.camera_hz)):
        _, _, tr, _, _ = env.step(0.0, 0.0)
        if not vistos or vistos[-1] != tr.traffic_light:
            vistos.append(tr.traffic_light)
    assert vistos == ["vermelho", "amarelo", "verde"]


def test_comando_chega_com_atraso(env):
    env.reset(seed=3)
    _, _, tr, _, _ = env.step(0.5, 0.0)               # 33 ms < 100 ms de atraso
    assert tr.v == pytest.approx(0.0, abs=1e-9)
    for _ in range(10):
        _, _, tr, _, _ = env.step(0.5, 0.0)
    assert tr.v > 0.1


def test_odometria_vem_dos_encoders_e_nao_da_verdade(env):
    env.reset(seed=4)
    for _ in range(60):
        _, tel, tr, _, _ = env.step(0.4, 0.0)
    erro = math.hypot(tel.x - tr.x, tel.y - tr.y)
    assert erro < 0.01                                # reto: odometria quase exata
    assert (tel.x, tel.y) != (tr.x, tr.y)             # mas é OUTRA medida


def test_sair_da_pista_encerra_o_episodio(env):
    env.reset(seed=5)
    done, reason = False, ""
    for _ in range(300):
        _, _, _, done, info = env.step(0.5, -1.5)     # vira para fora (direita)
        if done:
            reason = info["reason"]
            break
    assert done and reason == "saiu_da_pista"


def test_oraculo_faz_meia_volta_sem_sair(env):
    env.reset(seed=6)
    pilot = OraclePilot(env.lane.line)
    _, _, tr, _, _ = env.step(0.0, 0.0)
    s0, andou = None, 0.0
    for _ in range(int(60 * env.cfg.camera_hz)):
        go = tr.traffic_light == "verde"
        v, w = pilot.act(tr.x, tr.y, tr.yaw, env.t, go)
        _, _, tr, done, _ = env.step(v, w)
        assert not done
        if go:
            if s0 is not None:
                ds = tr.s - s0
                andou += ds + (env.lane.line.length if ds < -10 else 0.0)
            s0 = tr.s
        if andou > env.lane.line.length / 2:
            break
    assert andou > env.lane.line.length / 2


def test_placa_a_frente_aparece_rotulada(env):
    """Na largada, a placa do cruzamento à frente tem que vir rotulada."""
    obs, info = env.reset(seed=7)
    placas = [o for o in info["truth"].objects if o["kind"] == "placa" and o["slot"] == "anel_topo_oeste"]
    assert placas, "a placa a ~2 m da largada deveria estar visível"
    assert placas[0]["bbox_px"][0] >= 10
    assert 1.5 < placas[0]["distance_m"] < 2.5


def test_render_e_deterministico(env):
    a, _ = env.reset(seed=8)
    b, _ = env.reset(seed=8)
    assert np.array_equal(a, b)
