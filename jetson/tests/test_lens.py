"""Testes da geometria da lente e da câmera virtual.

Toda a matemática de FOV, intrínsecos e apontamento é Python puro justamente
para poder ser verificada sem OpenCV e sem câmera. Errar um sinal de yaw aqui
faria a câmera virtual mirar o lado ERRADO da rua — e as placas ficam sempre à
direita.
"""

from __future__ import annotations

import math

import pytest

from robocar.sensors import lens

# --- conversões de FOV -----------------------------------------------------


def test_focal_e_fov_sao_inversos():
    for fov in (30.0, 62.0, 90.0, 108.0, 120.0, 160.0):
        focal = lens.focal_px_from_fov(fov, 1640)
        assert lens.fov_deg_from_focal_px(focal, 1640) == pytest.approx(fov)


def test_fov_de_90_graus_tem_focal_igual_a_meia_largura():
    # tan(45°) = 1, então f = (w/2)/1 = w/2. Âncora simples para o modelo.
    assert lens.focal_px_from_fov(90.0, 1000) == pytest.approx(500.0)


def test_fov_maior_significa_focal_menor():
    assert lens.focal_px_from_fov(120.0, 1640) < lens.focal_px_from_fov(62.0, 1640)


@pytest.mark.parametrize("fov", [0.0, -10.0, 180.0, 200.0])
def test_fov_invalido_levanta(fov):
    with pytest.raises(ValueError):
        lens.focal_px_from_fov(fov, 1640)


def test_focal_invalida_levanta():
    with pytest.raises(ValueError):
        lens.fov_deg_from_focal_px(0.0, 1640)


# --- tamanho aparente ------------------------------------------------------


def test_placa_a_2m_bate_com_a_tabela_do_doc():
    """Confere a linha "1640×1232, 108°, 2,0 m = 45 px" de docs/02-hardware.md."""
    focal = lens.focal_px_from_fov(108.0, 1640)
    assert lens.object_px(150.0, 2000.0, focal) == pytest.approx(45, abs=1)


def test_placa_no_pior_caso_de_120_graus():
    focal = lens.focal_px_from_fov(120.0, 1640)
    assert lens.object_px(150.0, 2000.0, focal) == pytest.approx(36, abs=1)


def test_1280x720_ficaria_abaixo_do_limite_de_32px():
    # É a justificativa para NÃO usar 1280x720 (que ainda por cima é modo
    # recortado no IMX219). Ver docs/02-hardware.md.
    focal = lens.focal_px_from_fov(120.0, 1280)
    assert lens.object_px(150.0, 2000.0, focal) < 32


def test_tamanho_aparente_cai_com_a_distancia():
    focal = lens.focal_px_from_fov(108.0, 1640)
    tamanhos = [lens.object_px(150.0, d, focal) for d in (1000, 2000, 3000, 4000)]
    assert tamanhos == sorted(tamanhos, reverse=True)


def test_distancia_invalida_levanta():
    with pytest.raises(ValueError):
        lens.object_px(150.0, 0.0, 600.0)


# --- apontamento da câmera virtual -----------------------------------------


def test_sem_rotacao_olha_para_frente():
    assert lens.view_direction(0.0, 0.0) == pytest.approx((0.0, 0.0, 1.0))


def test_yaw_positivo_olha_para_a_DIREITA():
    """x cresce para a direita. As placas ficam sempre à direita (item 3.5.2).

    Se este teste inverter, a câmera virtual mira a calçada esquerda e o
    detector nunca vê placa nenhuma.
    """
    x, _, z = lens.view_direction(30.0, 0.0)
    assert x > 0
    assert x == pytest.approx(math.sin(math.radians(30.0)))
    assert z == pytest.approx(math.cos(math.radians(30.0)))


def test_yaw_negativo_olha_para_a_esquerda():
    x, _, _ = lens.view_direction(-30.0, 0.0)
    assert x < 0


def test_pitch_positivo_olha_para_CIMA():
    """y aponta para baixo em coordenadas de câmera, então 'cima' é y negativo."""
    _, y, _ = lens.view_direction(0.0, 15.0)
    assert y < 0


def test_direcao_e_sempre_unitaria():
    for yaw in (-45.0, 0.0, 30.0, 75.0):
        for pitch in (-20.0, 0.0, 8.0, 25.0):
            x, y, z = lens.view_direction(yaw, pitch)
            assert math.sqrt(x * x + y * y + z * z) == pytest.approx(1.0)


def test_matriz_de_rotacao_e_ortonormal():
    r = lens.rotation_matrix(30.0, 8.0)
    for i in range(3):
        norma = math.sqrt(sum(r[j][i] ** 2 for j in range(3)))
        assert norma == pytest.approx(1.0)
    for a in range(3):
        for b in range(a + 1, 3):
            produto = sum(r[j][a] * r[j][b] for j in range(3))
            assert produto == pytest.approx(0.0, abs=1e-12)


# --- intrínsecos -----------------------------------------------------------


def _intrinsics(**kwargs):
    base = {
        "fx": 596.0,
        "fy": 596.0,
        "cx": 820.0,
        "cy": 616.0,
        "width": 1640,
        "height": 1232,
        "dist": [-0.32, 0.11, 0.0, 0.0, -0.02],
    }
    base.update(kwargs)
    return lens.CameraIntrinsics(**base)


def test_fov_derivado_dos_intrinsecos():
    intr = _intrinsics()
    assert intr.hfov_deg == pytest.approx(108, abs=1)
    assert intr.vfov_deg == pytest.approx(92, abs=2)
    # O fabricante anuncia 120° — que é a DIAGONAL.
    assert intr.dfov_deg == pytest.approx(120, abs=3)


def test_focal_invalida_reprova():
    with pytest.raises(lens.CalibrationError):
        _intrinsics(fx=0.0)
    with pytest.raises(lens.CalibrationError):
        _intrinsics(width=0)


def test_escala_para_outra_resolucao_preserva_o_fov():
    """Calibrar em 1640x1232 e usar em 3280x2464 (mesmo FOV) deve funcionar."""
    intr = _intrinsics()
    grande = intr.scale_to(3280, 2464)
    assert grande.hfov_deg == pytest.approx(intr.hfov_deg)
    assert grande.vfov_deg == pytest.approx(intr.vfov_deg)
    assert grande.fx == pytest.approx(intr.fx * 2)
    assert grande.cx == pytest.approx(intr.cx * 2)
    # A distorção é adimensional (opera em coordenadas normalizadas).
    assert grande.dist == intr.dist


def test_ida_e_volta_em_yaml(tmp_path):
    original = _intrinsics(rms_error_px=0.42, sample_count=20)
    caminho = tmp_path / "calib" / "intrinsics.yaml"
    original.save(caminho)

    lido = lens.CameraIntrinsics.load(caminho)
    assert lido.fx == pytest.approx(original.fx)
    assert lido.dist == pytest.approx(original.dist)
    assert lido.sample_count == 20


def test_campos_informativos_nao_atrapalham_a_leitura(tmp_path):
    # to_dict() grava _hfov_deg etc. só para humanos; from_dict deve ignorar.
    caminho = tmp_path / "intrinsics.yaml"
    _intrinsics().save(caminho)
    assert "_hfov_deg" in caminho.read_text()
    lens.CameraIntrinsics.load(caminho)  # não deve levantar


def test_calibracao_ausente_da_erro_util(tmp_path):
    with pytest.raises(lens.CalibrationError, match="robocar calib camera"):
        lens.CameraIntrinsics.load(tmp_path / "nao_existe.yaml")


def test_calibracao_incompleta_levanta(tmp_path):
    caminho = tmp_path / "ruim.yaml"
    caminho.write_text("fx: 600\nfy: 600\n", encoding="utf-8")
    with pytest.raises(lens.CalibrationError, match="cx"):
        lens.CameraIntrinsics.load(caminho)


# --- especificação da câmera virtual ---------------------------------------


def test_tamanho_automatico_preserva_densidade_de_pixels():
    """A saída automática deve ter a mesma densidade px/grau da fonte no centro."""
    intr = _intrinsics()
    spec = lens.VirtualCameraSpec(hfov_deg=50.0).resolve(intr)
    assert spec.width > 0 and spec.height > 0
    # f da câmera virtual deve bater com f da real: nada de detalhe jogado fora.
    assert spec.intrinsics().fx == pytest.approx(intr.fx, rel=0.01)


def test_tamanho_explicito_e_respeitado():
    spec = lens.VirtualCameraSpec(width=320, height=320).resolve(_intrinsics())
    assert (spec.width, spec.height) == (320, 320)


def test_camera_virtual_nao_inventa_resolucao():
    """Com tamanho automático, a placa na vista virtual tem ~o mesmo tamanho
    que teria no quadro bruto. Retificar corrige a geometria, não cria pixels.
    """
    intr = _intrinsics()
    spec = lens.VirtualCameraSpec(hfov_deg=50.0).resolve(intr)
    na_virtual = spec.sign_px_at(2000)
    no_bruto = lens.object_px(150.0, 2000.0, intr.fx)
    assert na_virtual == pytest.approx(no_bruto, rel=0.02)


def test_intrinsics_antes_de_resolve_levanta():
    with pytest.raises(lens.CalibrationError):
        lens.VirtualCameraSpec(width=0, height=0).intrinsics()


def test_padrao_mira_a_direita_e_para_cima():
    """Os padrões precisam corresponder à geometria do regulamento: placa
    sempre à direita (3.5.2), a 475 mm do chão, com a câmera a 180 mm."""
    spec = lens.VirtualCameraSpec()
    assert spec.yaw_deg > 0, "deve mirar à direita"
    assert spec.pitch_deg > 0, "deve mirar acima do horizonte"


def test_config_do_repositorio_constroi_a_camera_virtual():
    from robocar.config import find_repo_root, load_config

    config = load_config(find_repo_root())
    camera = lens.virtual_camera_from_config(config, _intrinsics())
    assert camera.spec.yaw_deg > 0
    assert camera.spec.width > 0
    assert "placa de 150 mm" in camera.describe()


# --- validação com OpenCV (pulada onde ele não existe) ---------------------


def test_camera_virtual_mira_onde_deveria():
    """Valida de ponta a ponta o apontamento, com remapeamento de verdade.

    A matemática pura não consegue verificar uma coisa: a convenção de
    ``cv2.initUndistortRectifyMap``, que aplica ``R⁻¹`` ao raio do destino e
    por isso exige passar a rotação **transposta**. Errar essa transposição
    faria a câmera virtual mirar espelhada — para a calçada esquerda, onde
    nunca há placa.

    O teste projeta um ponto na direção de mira, desenha uma "placa" ali no
    quadro bruto e confere que ela cai no centro da vista virtual.
    """
    cv2 = pytest.importorskip("cv2")
    np = pytest.importorskip("numpy")

    intr = _intrinsics()
    spec = lens.VirtualCameraSpec(yaw_deg=30.0, pitch_deg=8.0, hfov_deg=50.0)
    virtual = lens.VirtualCamera(intr, spec).build()

    # Onde, no quadro bruto, cai um ponto a 2 m na direção de mira?
    direction = np.array(
        lens.view_direction(spec.yaw_deg, spec.pitch_deg), dtype=np.float64
    )
    point = (direction * 2000.0).reshape(1, 1, 3)
    projected, _ = cv2.projectPoints(
        point,
        np.zeros(3),
        np.zeros(3),
        np.array(intr.matrix(), dtype=np.float64),
        np.array(intr.dist, dtype=np.float64),
    )
    source_px = projected.ravel()

    # Desenha a "placa" nesse ponto e extrai a vista virtual.
    frame = np.zeros((intr.height, intr.width, 3), np.uint8)
    cv2.circle(frame, tuple(source_px.astype(int)), 22, (255, 255, 255), -1)
    view = virtual.apply(frame)

    ys, xs = np.where(view[:, :, 0] > 128)
    assert len(xs) > 0, "a placa sumiu da vista virtual — apontamento errado"

    erro = math.hypot(
        xs.mean() - virtual.target.cx, ys.mean() - virtual.target.cy
    )
    assert erro < 12, f"placa {erro:.1f} px fora do centro da vista virtual"


def test_vista_virtual_tem_o_tamanho_pedido():
    pytest.importorskip("cv2")
    np = pytest.importorskip("numpy")

    intr = _intrinsics()
    spec = lens.VirtualCameraSpec(hfov_deg=50.0, width=320, height=240)
    virtual = lens.VirtualCamera(intr, spec).build()

    view = virtual.apply(np.zeros((intr.height, intr.width, 3), np.uint8))
    assert view.shape[:2] == (240, 320)
