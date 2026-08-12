"""Testes do carregamento e da validação de configuração.

A validação existe para transformar erro de calibração em mensagem clara na
bancada, em vez de comportamento estranho na pista.
"""

from __future__ import annotations

import pytest
import yaml

from robocar import config as cfg


@pytest.fixture
def config_dir(tmp_path):
    """Cria um ``config/`` mínimo, porém válido."""
    root = tmp_path / "config"
    (root / "profiles").mkdir(parents=True)

    (root / "vehicle.yaml").write_text(
        yaml.safe_dump(
            {
                "vehicle": {"name": "teste", "length_mm": 420, "width_mm": 195},
                "steering": {"center_us": 1500, "min_us": 1150, "max_us": 1850},
                "throttle": {
                    "neutral_us": 1500,
                    "min_us": 1000,
                    "max_us": 2000,
                    "limit_collect": 0.3,
                    "limit_auto": 0.45,
                    "limit_race": 0.6,
                },
                "safety": {
                    "command_timeout_ms": 250,
                    "command_hz": 50,
                    "obstacle_stop_mm": 250,
                    "obstacle_slow_mm": 600,
                },
            }
        )
    )
    (root / "camera.yaml").write_text(
        yaml.safe_dump({"camera": {"backend": "csi", "capture": {"fps": 30}}})
    )
    (root / "capture.yaml").write_text(
        yaml.safe_dump({"capture": {"rate_hz": 20, "root": "data/sessions"}})
    )
    return tmp_path


def test_carrega_e_mescla(config_dir):
    config = cfg.load_config(config_dir)
    assert config["vehicle.name"] == "teste"
    assert config["camera.backend"] == "csi"
    assert config.get_int("capture.rate_hz") == 20


def test_perfil_sobrescreve(config_dir):
    (config_dir / "config" / "profiles" / "race.yaml").write_text(
        yaml.safe_dump({"throttle_limit": 0.5, "vehicle": {"name": "prova"}})
    )
    config = cfg.load_config(config_dir, profile="race")
    assert config.get_float("throttle_limit") == 0.5
    assert config["vehicle.name"] == "prova"
    # A mesclagem é profunda: chaves não citadas no perfil sobrevivem.
    assert config.get_int("steering.center_us") == 1500


def test_chave_ausente_da_erro_legivel(config_dir):
    config = cfg.load_config(config_dir)
    with pytest.raises(cfg.ConfigError, match="steering.nao_existe"):
        config["steering.nao_existe"]


def test_default_e_respeitado(config_dir):
    config = cfg.load_config(config_dir)
    assert config.get("nada.aqui", "padrao") == "padrao"
    assert config.get("nada.aqui", None) is None


def test_deep_merge_nao_muta_a_entrada():
    base = {"a": {"b": 1, "c": 2}}
    override = {"a": {"b": 9}}
    resultado = cfg.deep_merge(base, override)
    assert resultado == {"a": {"b": 9, "c": 2}}
    assert base == {"a": {"b": 1, "c": 2}}


def test_get_bool_aceita_texto(config_dir):
    config = cfg.Config({"x": {"sim": "sim", "nao": "false", "um": 1}})
    assert config.get_bool("x.sim") is True
    assert config.get_bool("x.nao") is False
    assert config.get_bool("x.um") is True


# --- validação -------------------------------------------------------------


def _validar(**overrides):
    base = {
        "vehicle": {"length_mm": 420, "width_mm": 195},
        "steering": {"center_us": 1500, "min_us": 1150, "max_us": 1850},
        "throttle": {
            "neutral_us": 1500,
            "min_us": 1000,
            "max_us": 2000,
            "limit_collect": 0.3,
            "limit_auto": 0.45,
            "limit_race": 0.6,
        },
        "safety": {
            "command_timeout_ms": 250,
            "command_hz": 50,
            "obstacle_stop_mm": 250,
            "obstacle_slow_mm": 600,
        },
        "camera": {"capture": {"fps": 30}},
        "capture": {"rate_hz": 20},
    }
    for path, value in overrides.items():
        node = base
        parts = path.split(".")
        for part in parts[:-1]:
            node = node[part]
        node[parts[-1]] = value
    return cfg.Config(base)


def test_config_valida_passa():
    cfg.validate(_validar())


def test_centro_fora_dos_batentes_reprova():
    with pytest.raises(cfg.ConfigError, match="center_us"):
        cfg.validate(_validar(**{"steering.center_us": 1900}))


def test_pulso_de_servo_absurdo_reprova():
    with pytest.raises(cfg.ConfigError, match="faixa segura"):
        cfg.validate(_validar(**{"steering.min_us": 300}))


def test_timeout_curto_demais_reprova():
    # 50 Hz = 20 ms por comando; 30 ms de timeout dispararia failsafe sozinho.
    with pytest.raises(cfg.ConfigError, match="command_timeout_ms"):
        cfg.validate(_validar(**{"safety.command_timeout_ms": 30}))


def test_limiares_de_obstaculo_invertidos_reprovam():
    with pytest.raises(cfg.ConfigError, match="obstacle_stop_mm"):
        cfg.validate(_validar(**{"safety.obstacle_stop_mm": 900}))


def test_carro_maior_que_o_regulamento_reprova():
    # Júnior/Master: máximo 500 x 250 mm.
    with pytest.raises(cfg.ConfigError, match="500 mm"):
        cfg.validate(_validar(**{"vehicle.length_mm": 520}))
    with pytest.raises(cfg.ConfigError, match="250 mm"):
        cfg.validate(_validar(**{"vehicle.width_mm": 260}))


def test_taxa_de_captura_acima_do_fps_reprova():
    with pytest.raises(cfg.ConfigError, match="rate_hz"):
        cfg.validate(_validar(**{"capture.rate_hz": 60}))


def test_limite_de_aceleracao_fora_de_faixa_reprova():
    with pytest.raises(cfg.ConfigError, match="limit_race"):
        cfg.validate(_validar(**{"throttle.limit_race": 1.5}))


# --- limites por modo ------------------------------------------------------


def test_throttle_limit_por_modo():
    config = _validar()
    assert cfg.throttle_limit_for(config, "collect") == 0.3
    assert cfg.throttle_limit_for(config, "auto") == 0.45
    assert cfg.throttle_limit_for(config, "race") == 0.6


def test_perfil_so_consegue_apertar_o_limite():
    # `throttle_limit` de um perfil nunca deve AUMENTAR o teto do modo.
    config = _validar(**{"throttle.limit_race": 0.6})
    config.data["throttle_limit"] = 0.9
    assert cfg.throttle_limit_for(config, "race") == 0.6

    config.data["throttle_limit"] = 0.4
    assert cfg.throttle_limit_for(config, "race") == 0.4


def test_modo_desconhecido_levanta():
    with pytest.raises(cfg.ConfigError):
        cfg.throttle_limit_for(_validar(), "turbo")


# --- arquivos reais do repositório ----------------------------------------


def test_config_do_repositorio_e_valida():
    """Os YAML versionados precisam passar na própria validação."""
    config = cfg.load_config(cfg.find_repo_root())
    assert config["vehicle.category"] in {"junior", "master"}


def test_perfil_race_do_repositorio_e_valido():
    config = cfg.load_config(cfg.find_repo_root(), profile="race")
    # O regulamento proíbe controle e sensoriamento externos na volta oficial.
    assert config.get_bool("external.rc_receiver") is False
    assert config.get_bool("external.teleop") is False
    assert config.get_bool("external.wifi") is False


def test_captura_usa_modo_de_campo_completo_do_imx219():
    """A câmera é uma IMX219 com lente de 120°.

    Os modos 1920x1080 e 1280x720 do sensor são RECORTE, não redução: usá-los
    joga fora parte do campo de visão da lente e, junto, os pixels que a placa
    de trânsito precisa ter para ser classificada. 1640x1232 é binning 2x2 do
    array inteiro — campo completo a 30 fps. Ver docs/02-hardware.md.
    """
    config = cfg.load_config(cfg.find_repo_root())
    largura = config.get_int("camera.capture.width")
    altura = config.get_int("camera.capture.height")

    modos_recortados = {(1920, 1080), (1280, 720), (640, 480)}
    assert (largura, altura) not in modos_recortados, (
        f"{largura}x{altura} é modo recortado do IMX219 — use 1640x1232"
    )
    # Modos de campo completo: o nativo e seus binnings inteiros.
    assert (largura, altura) in {(3280, 2464), (1640, 1232)}


def test_roi_das_placas_cobre_o_lado_direito():
    """O regulamento garante a placa sempre à DIREITA da rota (item 3.5.2)."""
    config = cfg.load_config(cfg.find_repo_root())
    roi = config.get("camera.roi_signs")
    assert roi["right"] == 1.0
    assert 0.0 < roi["left"] < 1.0
    assert roi["top"] < roi["bottom"]


def test_placa_de_150mm_tem_pixels_suficientes_a_2m():
    """Confere a viabilidade do MASTER com a lente e a resolução escolhidas.

    Uma placa de 150 mm precisa de ~32 px para ser classificada com folga, na
    janela de decisão de 1,5 a 2,5 m. Se alguém reduzir a resolução de captura
    ou trocar por uma lente mais aberta, este teste falha antes de a coleta
    inteira ser feita com um dado que não serve.
    """
    import math

    config = cfg.load_config(cfg.find_repo_root())
    largura_px = config.get_int("camera.capture.width")
    hfov_deg = config.get_float("camera.hardware.fov_horizontal_deg")

    distancia_mm = 2000.0
    largura_cena_mm = 2 * distancia_mm * math.tan(math.radians(hfov_deg) / 2)
    placa_px = 150.0 / largura_cena_mm * largura_px

    assert placa_px >= 32, (
        f"placa de 150 mm a 2 m ocuparia só {placa_px:.0f} px — insuficiente "
        f"para o detector de placas do MASTER"
    )
