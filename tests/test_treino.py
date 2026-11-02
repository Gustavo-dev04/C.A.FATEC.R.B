"""Testes do pipeline de treino: dados, rótulos, detector, controle e a rede.

Os testes da rede precisam de PyTorch e são pulados sem ele (a CI só instala
o básico). Rode localmente com ``pip install -e ".[treino]"``.
"""

from __future__ import annotations

import csv
import json
import math

import numpy as np
import pytest

from treino.rotulos import CLASSES_DETECCAO, COLUNAS, MANOBRAS, dividir_sementes
from vista_sim.config import load_camera
from vista_sim.env import SimConfig, VistaSimEnv


@pytest.fixture(scope="module")
def dataset(tmp_path_factory):
    from tools.gerar_dataset import gerar

    out = tmp_path_factory.mktemp("ds")
    gerar(out, episodios=2, semente_inicial=5, passo=3, workers=2, res=(160, 120), max_quadros=420,
          log=lambda *_: None)
    return out


# --- dados -----------------------------------------------------------------


def test_divisao_por_episodio_nao_mistura():
    tr, va = dividir_sementes(list(range(20)) * 3, 0.1)
    assert tr and va and not (tr & va)
    assert tr | va == set(range(20))


def test_dataset_tem_imagens_rotulos_e_meta(dataset):
    meta = json.loads((dataset / "meta.json").read_text())
    assert meta["episodios"] == 2 and meta["imagens"] > 0
    assert (meta["largura"], meta["altura"]) == (160, 120)
    with open(dataset / "rotulos.csv", encoding="utf-8") as fh:
        linhas = list(csv.DictReader(fh))
    assert tuple(linhas[0].keys()) == COLUNAS
    assert len(linhas) == meta["imagens"]
    for r in linhas:
        assert (dataset / r["arquivo"]).exists()
        assert r["manobra"] in MANOBRAS
        assert 0.0 <= float(r["dist_cruzamento"]) <= 3.0
    vistas = {r["vista"] for r in linhas}
    assert vistas == {"real", "deslocada"}
    desl = [abs(float(r["e_y"])) for r in linhas if r["vista"] == "deslocada"]
    real = [abs(float(r["e_y"])) for r in linhas if r["vista"] == "real"]
    assert np.mean(desl) > 2 * np.mean(real)          # as deslocadas cobrem o "fora do centro"


def test_rotulos_em_qualquer_pose_batem_com_a_verdade():
    env = VistaSimEnv(config=SimConfig(lighting=False, tl_red_s=(0.1, 0.1), tl_yellow_s=(0.1, 0.1)),
                      camera=load_camera().scaled(96, 72), seed=0)
    env.reset(seed=3)
    for _ in range(60):
        _, _, tr, _, _ = env.step(150.0, 150.0, render=False)
    r = env.rotulos_em(tr.x, tr.y, tr.yaw, tr.route_s)
    assert r["e_y"] == pytest.approx(tr.e_y, abs=1e-9)
    assert r["e_psi"] == pytest.approx(tr.e_psi, abs=1e-9)
    assert r["kappa"] == pytest.approx(tr.kappa_ahead, abs=1e-9)
    # deslocar o carro 8 cm para a esquerda soma ~8 cm ao e_y
    d = 0.08
    r2 = env.rotulos_em(tr.x - d * math.sin(tr.yaw), tr.y + d * math.cos(tr.yaw), tr.yaw, tr.route_s)
    assert r2["e_y"] - r["e_y"] == pytest.approx(d * math.cos(tr.e_psi), abs=0.005)


def test_yolo_rotulos_validos(dataset, tmp_path):
    from treino.deteccao import montar_yolo

    r = montar_yolo(dataset, tmp_path / "yolo", val_frac=0.5)
    assert "nc: 9" in (tmp_path / "yolo" / "data.yaml").read_text()
    n = 0
    for f in (tmp_path / "yolo" / "labels").rglob("*.txt"):
        img = tmp_path / "yolo" / "images" / f.parent.name / (f.stem + ".jpg")
        assert img.exists()
        for ln in f.read_text().splitlines():
            k, cx, cy, w, h = ln.split()
            assert 0 <= int(k) < len(CLASSES_DETECCAO)
            assert all(0.0 <= float(v) <= 1.0 for v in (cx, cy, w, h))
            n += 1
    assert n == sum(r["objetos"].values()) and n > 0


# --- controle --------------------------------------------------------------


def test_piloto_de_percepcao_com_a_verdade_completa_a_rota():
    """O teto: PilotoFaixa alimentado com a verdade (o que a rede tenta imitar)."""
    from treino.avaliacao import rodar

    r = rodar(None, [25], camera=load_camera().scaled(64, 48), config=SimConfig(lighting=False),
              log=lambda *_: None)
    assert r["completas"] == 1, r["por_rota"]


# --- rede (PyTorch) ----------------------------------------------------------


def test_rede_de_faixa_treina_exporta_e_dirige(dataset, tmp_path):
    torch = pytest.importorskip("torch")
    pytest.importorskip("torchvision")
    from treino.faixa import (
        FaixaDataset,
        PreditorTorch,
        RedeFaixa,
        aumentar_lote,
        carregar,
        exportar_onnx,
        ler_rotulos,
        treinar,
    )

    linhas = ler_rotulos(dataset)
    ds = FaixaDataset(dataset, linhas[:4], (96, 128))
    x, man, y, cz = ds[0]
    assert x.shape == (3, 96, 128) and x.dtype == torch.uint8
    assert man.sum() == 1 and y.shape == (4,)
    xa = aumentar_lote(x[None].float().repeat(8, 1, 1, 1) / 255.0)
    assert xa.shape == (8, 3, 96, 128) and float(xa.min()) >= 0.0 and float(xa.max()) <= 1.0
    out = RedeFaixa(pretrained=False)(xa[:1], man[None])
    assert out.shape == (1, 5)

    res = treinar(dataset, tmp_path, epocas=1, lote=16, tamanho=(96, 128), workers=0, device="cpu",
                  pretrained=False, limite=64, log=lambda *_: None)
    assert res["melhor"].exists() and len(res["historico"]) == 1
    modelo, tam = carregar(res["melhor"])
    assert tam == (96, 128)

    pytest.importorskip("onnx")
    info = exportar_onnx(modelo, tam, tmp_path / "faixa.onnx")
    assert (tmp_path / "faixa.onnx").exists() and (tmp_path / "faixa.json").exists()
    if info["dif_max_torch_onnx"] is not None:
        assert info["dif_max_torch_onnx"] < 1e-3

    pred = PreditorTorch(modelo, tam, device="cpu")
    o = pred(np.zeros((120, 160, 3), np.uint8), "esquerda")
    assert o.shape == (5,) and 0.0 <= o[4] <= 1.0
    with torch.no_grad():
        assert np.isfinite(o).all()
