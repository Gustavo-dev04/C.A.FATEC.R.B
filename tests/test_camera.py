"""Testes da câmera fisheye — o modelo óptico e as contas de campo de visão."""

from __future__ import annotations

import math

import numpy as np
import pytest

from vista_sim.camera import FisheyeCamera
from vista_sim.config import load_camera


@pytest.fixture(scope="module")
def params():
    return load_camera()


@pytest.fixture(scope="module")
def cam(params):
    return FisheyeCamera(params)


def test_eixo_optico_cai_no_centro_da_imagem(cam):
    px, ok = cam.project_cam(np.array([[0.0, 0.0, 1.0]]))
    assert px[0] == pytest.approx([cam.p.cx, cam.p.cy])
    assert ok[0]


def test_raio_de_pixel_e_projecao_sao_inversos(cam):
    """pixel → raio → pixel tem que voltar ao mesmo lugar."""
    rays = cam.pixel_rays()
    H, W = rays.shape[:2]
    amostra = rays[:: H // 7, :: W // 9].reshape(-1, 3)
    px, _ = cam.project_cam(amostra)
    vv, uu = np.meshgrid(np.arange(0, H, H // 7), np.arange(0, W, W // 9), indexing="ij")
    esperado = np.stack([uu.ravel() + 0.5, vv.ravel() + 0.5], axis=1)
    assert np.abs(px - esperado).max() < 1e-6


def test_inversao_com_distorcao_converge(params):
    """Newton inverte θ_d(θ) mesmo com k1..k4 diferentes de zero."""
    d = params.__dict__.copy()
    d["dist"] = (-0.05, 0.01, -0.002, 0.0004)
    cam = FisheyeCamera(type(params)(**d))
    theta = np.linspace(0, 1.1, 50)
    assert cam._undistort_theta(cam._distort_theta(theta)) == pytest.approx(theta, abs=1e-9)


def test_bate_com_o_cv2_fisheye(params):
    """O modelo em NumPy tem que dar o MESMO pixel que o cv2.fisheye.

    É isso que garante que, quando a calibração real (cv2.fisheye.calibrate)
    for gravada em intrinsics.yaml, o simulador use exatamente a mesma câmera.
    """
    cv2 = pytest.importorskip("cv2")
    d = params.__dict__.copy()
    d["dist"] = (-0.04, 0.012, -0.003, 0.0005)
    cam = FisheyeCamera(type(params)(**d))
    rng = np.random.default_rng(1)
    pts = np.column_stack([rng.uniform(-1, 1, 200), rng.uniform(-0.8, 0.8, 200), rng.uniform(0.6, 3, 200)])
    ours, _ = cam.project_cam(pts)
    ref, _ = cv2.fisheye.projectPoints(pts.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), cam.K, cam.dist)
    assert np.abs(ours - ref.reshape(-1, 2)).max() < 1e-6


def test_fov_estimado_bate_com_o_claude_md(params):
    """120° na diagonal (equidistante) ⇒ ≈ 96° H × 72° V. O CLAUDE.md estima 92° × 75°."""
    full = params.scaled(params.ref_width, params.ref_height)
    hfov = math.degrees(2 * (full.width / 2) / full.fx)
    vfov = math.degrees(2 * (full.height / 2) / full.fy)
    assert hfov == pytest.approx(96, abs=1)
    assert vfov == pytest.approx(72, abs=1)


def test_reescalar_preserva_o_fov(params):
    a = params.scaled(1640, 1232)
    b = params.scaled(320, 240)
    assert a.width / a.fx == pytest.approx(b.width / b.fx, rel=1e-6)


# --- chão -----------------------------------------------------------------


def test_pixel_de_baixo_no_centro_ve_o_chao_logo_a_frente(cam):
    lut = cam.ground_lookup()
    H, W = lut.valid.shape
    assert lut.valid[H - 1, W // 2]
    fwd = float(lut.forward[H - 1, W // 2])
    assert 0.254 < fwd < 0.6                        # à frente da lente, perto
    assert abs(float(lut.left[H - 1, W // 2])) < 0.01


def test_pixel_do_topo_nao_ve_o_chao(cam):
    assert not cam.ground_lookup().valid[0, cam.p.width // 2]


def test_chao_a_esquerda_da_imagem_fica_a_esquerda_do_carro(cam):
    """Convenção: x_cam para a direita = −y do carro. Errar isso espelha a pista."""
    lut = cam.ground_lookup()
    H, W = lut.valid.shape
    assert lut.left[H - 1, 5] > 0.0
    assert lut.left[H - 1, W - 6] < 0.0


# --- a descoberta que destravou o pitch -----------------------------------


def _elevacao(altura, distancia, z_cam=0.156):
    return math.degrees(math.atan2(altura - z_cam, distancia))


def test_lente_vermelha_cabe_com_pitch_de_15_graus(params):
    """Com os centros REAIS (530/390/250 mm), a vermelha fica a 19,0° na largada.

    Com VFOV ≈ 72°, a borda superior com pitch 15° fica a 21° — a lente cabe.
    O 'ALERTA GEOMÉTRICO' do CLAUDE.md partia de 725 mm (27,7°) e concluía o
    contrário.
    """
    cam = FisheyeCamera(params.with_pitch(15.0))
    topo = math.degrees(cam.top_edge_elevation())
    vermelha = _elevacao(0.530, 1.086)
    assert vermelha == pytest.approx(19.0, abs=0.1)
    assert topo > vermelha


def test_lente_vermelha_sai_do_quadro_acima_de_17_graus(params):
    cam = FisheyeCamera(params.with_pitch(18.0))
    assert math.degrees(cam.top_edge_elevation()) < _elevacao(0.530, 1.086)
