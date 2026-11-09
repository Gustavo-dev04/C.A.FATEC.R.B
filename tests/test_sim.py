"""Testes da geometria, do carro (rotação de roda), do ambiente e do renderizador."""

from __future__ import annotations

import math

import numpy as np
import pytest

from vista_sim.car import RPM, DiffDriveCar, wheel_rpm
from vista_sim.config import load_camera, load_car
from vista_sim.env import SimConfig, VistaSimEnv
from vista_sim.geometry import Polyline, rounded_rect
from vista_sim.oracle import OraclePilot


@pytest.fixture(scope="module")
def car_params():
    return load_car()


def _run(car: DiffDriveCar, seconds: float, dt: float = 0.002) -> None:
    for _ in range(int(round(seconds / dt))):
        car.step(dt)


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


def test_projecao_em_janela_nao_pula_para_outra_passagem():
    """Rota que passa duas vezes pelo mesmo lugar: a janela escolhe a passagem certa."""
    ida = np.column_stack([np.linspace(0, 4, 41), np.zeros(41)])
    volta = np.column_stack([np.linspace(4, 0, 41), np.full(41, 0.02)])
    line = Polyline(np.vstack([ida, volta]), closed=False)
    assert line.project(1.0, 0.01, 0.0, 2.0).s == pytest.approx(1.0, abs=0.02)
    assert line.project(1.0, 0.01, 6.0, 8.0).s == pytest.approx(7.0, abs=0.03)


def test_curvatura_do_anel_tem_o_sinal_certo():
    """Anti-horário = curvas à ESQUERDA = curvatura positiva."""
    lane = Polyline(rounded_rect(0, 0, 3, 2, 0.75, 0.02), closed=True)
    ks = [lane.project(*lane.point_at(s)[:2]).curvature for s in np.linspace(0, lane.length, 200)]
    assert max(ks) == pytest.approx(1 / 0.75, rel=0.15)
    assert min(ks) > -0.1


# --- carro: tudo em rotação de roda ---------------------------------------


def test_velocidade_maxima_bate_com_o_motor(car_params):
    """530 rpm a 12 V → ~486 rpm a 11 V → ~1,65 m/s na roda Ø65 (CLAUDE.md §5A)."""
    assert car_params.wheel_speed_max == pytest.approx(1.65, abs=0.02)


def test_bitola_e_dimensoes_do_desenho(car_params):
    assert car_params.track_width == pytest.approx(0.214)
    assert car_params.wheel_radius == pytest.approx(0.0325)
    assert car_params.length == pytest.approx(0.400)
    assert car_params.width == pytest.approx(0.241)
    assert car_params.length <= 0.500 and car_params.width <= 0.250   # regulamento


def test_rotacao_igual_anda_reto(car_params):
    car = DiffDriveCar(car_params)
    car.reset(0, 0, 0)
    car.command_wheels(150.0, 150.0)
    _run(car, 2.0)
    s = car.state
    assert s.y == pytest.approx(0.0, abs=1e-9)
    assert s.left.rpm == pytest.approx(150.0, rel=0.03)
    assert s.v == pytest.approx(150.0 / RPM * car_params.wheel_radius, rel=0.03)


def test_rotacao_diferente_curva_para_o_lado_mais_lento(car_params):
    car = DiffDriveCar(car_params)
    car.reset(0, 0, 0)
    car.command_wheels(100.0, 200.0)                 # direita mais rápida → esquerda
    _run(car, 1.0)
    assert car.state.yaw > 0.3 and car.state.y > 0


def test_gira_no_proprio_eixo_com_rotacoes_opostas(car_params):
    car = DiffDriveCar(car_params)
    car.reset(0, 0, 0)
    car.command_wheels(-120.0, 120.0)
    _run(car, 1.0)
    assert math.hypot(car.state.x, car.state.y) < 0.01
    assert abs(car.state.yaw) > 0.5


def test_pid_alcanca_o_rpm_pedido_rapido(car_params):
    """Degrau: o PID do ESP32 chega a 90 % em menos de 0,1 s."""
    car = DiffDriveCar(car_params)
    car.reset(0, 0, 0)
    car.command_wheels(147.0, 147.0)
    t90 = None
    for i in range(200):
        car.step(0.001)
        if t90 is None and car.state.left.rpm >= 0.9 * 147.0:
            t90 = (i + 1) * 0.001
    assert t90 is not None and t90 < 0.1


def test_pid_vence_a_zona_morta_em_baixa_rotacao(car_params):
    """Sem PID, um duty pequeno não gira a roda (zona morta). Com PID, 8 rpm saem."""
    car = DiffDriveCar(car_params)
    car.reset(0, 0, 0)
    car.command_wheels(8.0, 8.0)
    _run(car, 1.5)
    assert car.state.left.rpm == pytest.approx(8.0, abs=1.5)


def test_curva_com_raio_v_sobre_omega(car_params):
    car = DiffDriveCar(car_params)
    car.reset(0, 0, 0)
    v, w = 0.4, 0.8                                   # raio 0,5 m
    car.command(v, w)
    _run(car, 4.0, 0.0025)
    # Volta COMPLETA (2π/ω ≈ 7,9 s): com um arco só, o centro da caixa
    # envolvente não é o centro do círculo.
    xs, ys = [], []
    for _ in range(int(2 * math.pi / w / 0.0025) + 50):
        car.step(0.0025)
        xs.append(car.state.x)
        ys.append(car.state.y)
    xs, ys = np.array(xs), np.array(ys)
    cx, cy = (xs.max() + xs.min()) / 2, (ys.max() + ys.min()) / 2
    assert np.hypot(xs - cx, ys - cy).mean() == pytest.approx(v / w, rel=0.03)


def test_saturacao_preserva_a_curvatura(car_params):
    car = DiffDriveCar(car_params)
    car.command_wheels(2000.0, 1000.0)               # pede o impossível
    tl, tr = car.state.left.target, car.state.right.target
    assert max(abs(tl), abs(tr)) == pytest.approx(car_params.target_limit * car_params.omega_wheel_max)
    assert tl / tr == pytest.approx(2.0)


def test_cinematica_inversa(car_params):
    b, r = car_params.track_width, car_params.wheel_radius
    el, dr = wheel_rpm(0.5, 1.0, b, r)
    assert (el + dr) / 2 / RPM * r == pytest.approx(0.5)
    assert (dr - el) / RPM * r / b == pytest.approx(1.0)


def test_encoder_conta_a_rotacao(car_params):
    car = DiffDriveCar(car_params)
    car.reset(0, 0, 0)
    car.command_wheels(200.0, 200.0)
    _run(car, 3.0)
    voltas = car.state.x / (2 * math.pi * car_params.wheel_radius)
    assert car.state.ticks_left == pytest.approx(voltas * car_params.counts_per_rev, abs=3)


def test_raios_diferentes_puxam_para_um_lado(car_params):
    """Rotação igual NÃO é andar reto se uma roda é 0,5 % maior — por isso o
    Vista corrige pela câmera, não por "rpm iguais"."""
    car = DiffDriveCar(car_params)
    car.reset(0, 0, 0)
    car.state.right.radius *= 1.005
    car.command_wheels(150.0, 150.0)
    _run(car, 6.0)
    assert car.state.y > 0.05                          # direita maior → puxa à esquerda


def test_patinacao_engana_a_odometria(car_params):
    car = DiffDriveCar(car_params)
    car.reset(0, 0, 0)
    car.state.left.slip = car.state.right.slip = 0.03
    car.command_wheels(150.0, 150.0)
    _run(car, 3.0)
    pelos_ticks = car.state.ticks_left / car_params.counts_per_rev * 2 * math.pi * car_params.wheel_radius
    assert pelos_ticks > car.state.x * 1.02            # encoder conta mais do que andou


# --- ambiente -------------------------------------------------------------


@pytest.fixture(scope="module")
def env():
    cfg = SimConfig(latency_s=0.10, lighting=False, randomize_car=False)
    return VistaSimEnv(config=cfg, camera=load_camera().scaled(160, 120), seed=0)


def test_reset_devolve_imagem_e_verdade(env):
    obs, info = env.reset(seed=1)
    assert obs.shape == (120, 160, 3)
    tr = info["truth"]
    assert tr.on_carpet and tr.in_own_lane
    assert tr.traffic_light == "vermelho"
    assert abs(tr.e_y) < 0.01
    assert tr.next_junction is not None


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
    _, _, tr, _, _ = env.step(150.0, 150.0)           # 33 ms < 100 ms de atraso
    assert tr.v == pytest.approx(0.0, abs=1e-9)
    for _ in range(10):
        _, tel, tr, _, _ = env.step(150.0, 150.0)
    assert tr.v > 0.3
    assert tel.rpm_left == pytest.approx(150.0, rel=0.1)   # o encoder mede


def _espera_calibrar(env, segundos=1.0):
    """Parado (como no vermelho): a odometria estima o viés do giroscópio."""
    for _ in range(int(segundos * env.cfg.camera_hz)):
        _, tel, tr, _, _ = env.step(0.0, 0.0)
    return tel, tr


def test_odometria_vem_dos_sensores_e_nao_da_verdade(env):
    env.reset(seed=4)
    tel, _ = _espera_calibrar(env)
    assert tel.imu_calibrated
    for _ in range(45):
        _, tel, tr, _, _ = env.step(120.0, 120.0)
    erro = math.hypot(tel.x - tr.x, tel.y - tr.y)
    assert erro < 0.01                                # reto: odometria quase exata
    assert (tel.x, tel.y) != (tr.x, tr.y)             # mas é OUTRA medida


def test_parado_no_vermelho_calibra_o_vies_do_giroscopio():
    """MPU6050 sai de fábrica com até ±20 °/s de viés. Parado, a média do
    giroscópio É o viés — e o rumo não anda enquanto as rodas não andam."""
    env = VistaSimEnv(config=SimConfig(lighting=False, tl_red_s=(3.0, 3.0)),
                      camera=load_camera().scaled(96, 72), seed=0)
    _, info = env.reset(seed=11)
    psi0 = info["telemetry"].psi
    tel, tr = _espera_calibrar(env, 2.5)
    assert abs(tr.gyro_bias) > math.radians(0.5)      # o viés sorteado é grande...
    assert tel.imu_calibrated
    assert math.degrees(abs(tel.gyro_bias_est - tr.gyro_bias)) < 0.02   # ...e foi medido
    assert tel.psi == pytest.approx(psi0)             # parado, o rumo não girou
    assert math.degrees(tel.gyro_z) == pytest.approx(math.degrees(tr.gyro_bias), abs=0.1)


def test_giroscopio_corrige_o_rumo_que_os_encoders_erram():
    """Roda direita 0,5 % maior e rpm iguais: o carro curva sem os encoders
    perceberem. O giroscópio mede a rotação do corpo e acompanha."""
    env = VistaSimEnv(config=SimConfig(lighting=False, randomize_car=False, tl_red_s=(1.0, 1.0)),
                      camera=load_camera().scaled(96, 72), seed=0)
    env.reset(seed=12)
    env.imu.gyro_scale = 1.0                          # isola o efeito do viés calibrado
    env.car.state.right.radius *= 1.005
    _espera_calibrar(env, 1.5)
    for _ in range(150):                              # 5 s a ~0,5 m/s
        _, tel, tr, _, _ = env.step(147.0, 147.0)
    so_enc = env.odometry.enc_pose[2]
    real = tr.yaw
    assert abs(math.degrees(math.remainder(so_enc - real, 2 * math.pi))) > 2.0
    assert abs(math.degrees(math.remainder(tel.psi - real, 2 * math.pi))) < 0.5


def test_sair_da_pista_encerra_o_episodio(env):
    env.reset(seed=5)
    done, reason = False, ""
    for _ in range(300):
        _, _, _, done, info = env.step(250.0, 60.0)   # roda esquerda mais rápida → direita
        if done:
            reason = info["reason"]
            break
    assert done and reason in ("saiu_da_pista", "saiu_da_rota")


def _drive(env, pilot, info, max_steps=20000):
    tr = info["truth"]
    pilot.reset(tr.x, tr.y, tr.yaw)
    for _ in range(max_steps):
        go = tr.traffic_light == "verde"
        _, _, tr, done, inf = env.step(*pilot.act(tr.x, tr.y, tr.yaw, env.t, go))
        if done:
            return inf["reason"], tr
    return "nao_terminou", tr


def test_oraculo_completa_a_rota_respeitando_todas_as_placas(env):
    _, info = env.reset(seed=7)
    sc = info["scenario"]
    reason, tr = _drive(env, OraclePilot(env.track.network, sc.signs, env.car_params), info)
    assert reason == "rota_completa"
    tipos = [e["tipo"] for e in env.events]
    assert "desrespeitou_placa" not in tipos
    passou = [e for e in env.events if e["tipo"] in ("placa_respeitada", "cruzamento_sem_placa")]
    assert len(passou) == len(env.route.junctions)
    assert any(e["no"].startswith("C_") for e in passou)        # entrou no centro
    assert "largou_antes_do_verde" not in tipos


def test_ignorar_as_placas_e_desrespeitar(env):
    """Sem ler placas (tudo "em frente"), o carro erra a primeira conversão."""
    _, info = env.reset(seed=7)
    j = next(j for j in env.route.junctions if j.maneuver != "frente")
    reason, _ = _drive(env, OraclePilot(env.track.network, [], env.car_params), info)
    assert reason == "desrespeitou_placa"
    ev = next(e for e in env.events if e["tipo"] == "desrespeitou_placa")
    assert ev["no"] == j.node and ev["esperado"] == j.maneuver and ev["feito"] == "frente"


def test_placa_da_aproximacao_aparece_rotulada(env):
    """2 m antes de um cruzamento com placa, ela vem no rótulo, com a aproximação certa."""
    env.reset(seed=7)
    rt = env.route
    j = next(j for j in rt.junctions if j.sign)
    x, y, h = rt.line.point_at(j.s_in - 2.0)
    _, info = env.renderer.render((x, y, h), "verde")
    mine = [o for o in info["objects"] if o["kind"] == "placa" and tuple(o["arrival"]) == j.arrival]
    assert mine, "a placa da aproximação deveria estar visível"
    assert mine[0]["class"] == j.sign and mine[0]["on_route"]
    assert 1.0 < mine[0]["distance_m"] < 2.0
    assert 0.0 <= mine[0]["visible_frac"] <= 1.0


def test_render_e_deterministico(env):
    a, _ = env.reset(seed=8)
    b, _ = env.reset(seed=8)
    assert np.array_equal(a, b)


# --- MPU6050 ---------------------------------------------------------------


@pytest.fixture()
def mpu():
    from vista_sim.config import load_imu
    from vista_sim.imu import Mpu6050

    return Mpu6050(load_imu(), rng=np.random.default_rng(3), randomize=True)


def test_giroscopio_parado_tem_o_ruido_do_datasheet(mpu):
    """0,005 °/s/√Hz com filtro de 98 Hz → ~0,05 °/s RMS (datasheet: 0,05)."""
    g = np.array([mpu.sample(0.0, 0.0, 0.0, 0.0, 0.005)[0] for _ in range(4000)])
    assert math.degrees(g.std()) == pytest.approx(0.05, rel=0.2)
    assert abs(math.degrees(mpu.p.gyro_initial_bias)) == pytest.approx(20.0)


def test_giroscopio_satura_no_fundo_de_escala(mpu):
    gz, _ = mpu.sample(0.0, 8.0, 0.0, 0.0, 0.005)     # 458 °/s > 250
    assert math.degrees(gz) == pytest.approx(250.0, abs=0.01)


def test_fator_de_escala_sobra_depois_de_calibrar_o_vies(mpu):
    mpu.gyro_scale, mpu.gyro_bias = 1.03, 0.1
    g = np.mean([mpu.sample(0.3, 1.0, 0.0, 0.0, 0.005)[0] for _ in range(400)])
    assert g - mpu.gyro_bias == pytest.approx(1.03, abs=0.005)


def test_acelerometro_parado_le_a_gravidade_e_na_curva_a_centripeta(mpu):
    mpu.accel_bias = np.zeros(3)
    a0 = np.mean([mpu.sample(0.0, 0.0, 0.0, 0.0, 0.005)[1] for _ in range(400)], axis=0)
    assert a0 == pytest.approx([0.0, 0.0, 9.80665], abs=0.01)
    v, w = 0.4, 0.8
    a = np.mean([mpu.sample(v, w, 0.0, 0.0, 0.005)[1] for _ in range(400)], axis=0)
    assert a[1] == pytest.approx(v * w, abs=0.01)                   # v·ω para a esquerda
    assert a[0] == pytest.approx(-w * w * mpu.p.x, abs=0.01)        # braço de alavanca
