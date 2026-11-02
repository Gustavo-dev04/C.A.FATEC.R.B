"""Ambiente do Vista-Sim — API no estilo ``gymnasium``.

    env = VistaSimEnv()
    obs, info = env.reset(seed=0)            # sorteia um cenário inteiro
    while True:
        obs, telem, truth, done, info = env.step(rpm_left, rpm_right)

**O comando é rotação de roda.** O carro não tem direção: anda reto com as
rodas iguais e curva com a diferença entre elas. ``step`` recebe o rpm alvo de
cada roda — o mesmo que o ESP32 recebe e persegue com o PID. ``step_vw(v, ω)``
existe só como atalho e converte para rpm antes (``car.wheel_rpm``).

O que o Vista recebe (``obs`` + ``telem``) é o que receberia do carro real: a
imagem da câmera e a telemetria — rpm MEDIDO pelo encoder, ticks, leitura bruta
da MPU6050 e a odometria (deslocamento dos encoders, rumo do giroscópio com o
viés calibrado na espera do vermelho). A pose verdadeira só aparece em ``truth``, que
serve para medir o Vista e rotular dados; o Vista nunca deve lê-la.

A cada ``reset(seed)`` sai um cenário novo (``vista_sim.scenario``): rota pela
cidade, placas, faixas de pedestre, prédios, público e luz. A rota é verificada
quadro a quadro: virar errado num cruzamento encerra o episódio com
``desrespeitou_placa``.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

from .car import RPM, DiffDriveCar, wheel_rpm
from .config import CameraParams, CarParams, load_camera, load_car, load_imu, repo_root
from .geometry import wrap_angle
from .imu import Mpu6050, WheelGyroOdometry
from .render_fast import FastRenderer, PostFx
from .route import RoutePath
from .scenario import Scenario, ScenarioConfig, generate
from .track import Track


@dataclass
class SimConfig:
    camera_hz: float = 30.0
    physics_dt: float = 0.005
    latency_s: float = 0.10
    # Semáforo: vermelho → amarelo → verde (regulamento 3.6). Tempos sorteados
    # no reset para o Vista não decorar um instante fixo.
    tl_red_s: tuple[float, float] = (2.0, 6.0)
    tl_yellow_s: tuple[float, float] = (1.0, 3.0)
    randomize_car: bool = True           # raio de roda e patinação por episódio
    randomize_imu: bool = True           # viés, fator de escala e viés do acelerômetro por episódio
    lighting: bool = True
    randomize_fx: bool = False
    off_route_m: float = 0.30            # |e_y| acima disso = fora da rota
    max_episode_s: float = 400.0
    scenario: ScenarioConfig = field(default_factory=ScenarioConfig)


@dataclass
class Telemetry:
    """Espelha o quadro ``T,...`` do ESP32 (CLAUDE.md §4.1) + a MPU6050."""

    seq: int
    t: float
    x: float                  # odometria (m): deslocamento dos encoders,
    y: float                  #   rumo do giroscópio (vista_sim.imu)
    psi: float                # rad
    rpm_left: float           # rpm MEDIDO pelo encoder (quantizado)
    rpm_right: float
    ticks_left: int
    ticks_right: int
    gyro_z: float             # rad/s, leitura BRUTA da MPU6050 (média do quadro, com viés)
    accel: tuple[float, float, float]   # m/s² (x frente, y esquerda, z cima), bruto
    gyro_bias_est: float      # rad/s, viés estimado com o carro parado
    imu_calibrated: bool      # já houve parada suficiente para estimar o viés?


@dataclass
class Truth:
    """Verdade de campo exportada em todo quadro (CLAUDE.md §5A)."""

    t: float
    x: float
    y: float
    yaw: float
    v: float
    omega: float
    rpm_left: float           # rotação REAL de cada roda
    rpm_right: float
    route_s: float            # progresso ao longo da rota (m), do centro do carro
    route_length: float
    edge: str | None          # rua atual (None = dentro de um cruzamento)
    e_y: float                # m, + = centro do carro à esquerda do centro da faixa
    e_psi: float              # rad, + = apontando para a esquerda da faixa
    curvature: float
    on_carpet: bool
    in_own_lane: bool
    traffic_light: str
    next_junction: dict[str, Any] | None
    objects: list[dict[str, Any]] = field(default_factory=list)
    gyro_bias: float = 0.0    # rad/s, viés VERDADEIRO da MPU6050 agora


class VistaSimEnv:
    def __init__(
        self,
        track: Track | None = None,
        car: CarParams | None = None,
        camera: CameraParams | None = None,
        config: SimConfig | None = None,
        map_path: Path | str | None = None,
        seed: int | None = None,
    ) -> None:
        root = repo_root()
        self.track = track or Track.load(map_path or root / "maps" / "robocar_master.yaml")
        self.car_params = car or load_car(root)
        self.cam_params = camera or load_camera(root)
        self.cfg = config or SimConfig()
        self.rng = np.random.default_rng(seed)
        self.car = DiffDriveCar(self.car_params, rng=self.rng, randomize=self.cfg.randomize_car)
        self.imu_params = load_imu(root)
        self.imu = Mpu6050(self.imu_params, rng=self.rng, randomize=self.cfg.randomize_imu)
        p = self.car_params
        self.odometry = WheelGyroOdometry(p.track_width, p.wheel_radius, p.counts_per_rev)
        p = self.car_params
        body = np.array([[p.front_overhang, p.width / 2], [-p.rear_overhang, p.width / 2],
                         [-p.rear_overhang, -p.width / 2], [p.front_overhang, -p.width / 2]])
        self.renderer = FastRenderer(self.track, self.cam_params, rng=self.rng, car_body=(body, p.height))
        self.scenario: Scenario | None = None
        self.route: RoutePath | None = None
        self._reset_state()

    # ---------------- ciclo ----------------

    def _reset_state(self) -> None:
        self.t = 0.0
        self.seq = 0
        self._queue: deque[tuple[float, float, float]] = deque()
        self._applied = (0.0, 0.0)
        self._imu_acc = [0.0, np.zeros(3), 0]      # soma do giro, soma da aceleração, n
        self._gz_last, self._acc_last = 0.0, np.array([0.0, 0.0, 9.80665])
        self._tl_green_at = 0.0
        self._tl_yellow_at = 0.0
        self._s = 0.0
        self._passed = 0                     # cruzamentos já vencidos
        self._in_lane = True
        self.events: list[dict[str, Any]] = []
        self._new_events: list[dict[str, Any]] = []

    def reset(self, seed: int | None = None, scenario: Scenario | None = None):
        """Novo episódio. ``seed`` sorteia o cenário; ``scenario`` fixa um pronto."""
        if seed is not None:
            self.rng = np.random.default_rng(seed)
            self.car.rng = self.rng
            self.imu.rng = self.rng
            self.renderer.rng = self.rng
        self._reset_state()
        if scenario is None:
            scen_seed = int(self.rng.integers(2**31)) if seed is None else seed
            scenario = generate(self.track.data, self.track.network, scen_seed, self.cfg.scenario)
        self.scenario = scenario
        self.renderer.set_scenario(scenario, lighting=self.cfg.lighting)
        signs = {tuple(s["arrival"]): s["class"] for s in scenario.signs if s["on_route"]}
        self.route = RoutePath(self.track.network, scenario.route, scenario.start_s, scenario.start_s, signs)

        st = self.track.start
        self.car.reset(st["x"], st["y"], st["yaw"])
        self.imu.reset()
        self.odometry.reset(st["x"], st["y"], st["yaw"])
        self._tl_yellow_at = float(self.rng.uniform(*self.cfg.tl_red_s))
        self._tl_green_at = self._tl_yellow_at + float(self.rng.uniform(*self.cfg.tl_yellow_s))
        if self.cfg.randomize_fx:
            self.renderer.fx = PostFx(
                brightness=float(self.rng.uniform(0.85, 1.15)),
                noise_std=float(self.rng.uniform(0.0, 6.0)),
                vignette=float(self.rng.uniform(0.0, 0.4)),
            )
        obs, objs = self._observe()
        return obs, {"truth": self._truth(objs), "telemetry": self._telemetry(), "scenario": scenario}

    def step(self, rpm_left: float, rpm_right: float):
        """Envia o rpm alvo de cada roda — o contrato do ESP32 — e avança um quadro."""
        self._queue.append((self.t + self.cfg.latency_s, float(rpm_left), float(rpm_right)))
        frame_dt = 1.0 / self.cfg.camera_hz
        n = max(1, int(round(frame_dt / self.cfg.physics_dt)))
        dt = frame_dt / n
        for _ in range(n):
            while self._queue and self._queue[0][0] <= self.t:
                _, l_c, r_c = self._queue.popleft()
                self._applied = (l_c, r_c)
                self.car.command_wheels(l_c, r_c)
            s0 = self.car.state
            v0, w0 = s0.v, self._yaw_rate()
            self.car.step(dt)
            v1, w1 = self.car.state.v, self._yaw_rate()
            gz, acc = self.imu.sample(v1, w1, (v1 - v0) / dt, (w1 - w0) / dt, dt)
            st = self.car.state
            self.odometry.update(st.ticks_left, st.ticks_right, gz, dt)
            self._imu_acc[0] += gz
            self._imu_acc[1] = self._imu_acc[1] + acc
            self._imu_acc[2] += 1
            self.t += dt

        self.seq += 1
        obs, objs = self._observe()
        truth = self._truth(objs)
        done, reason = self._check_done(truth)
        info = {"reason": reason, "events": self._new_events}
        self._new_events = []
        return obs, self._telemetry(), truth, done, info

    def step_vw(self, v: float, omega: float):
        """Atalho: (v, ω) → rpm de cada roda (raio NOMINAL, como o ESP32) → ``step``."""
        p = self.car_params
        return self.step(*wheel_rpm(v, omega, p.track_width, p.wheel_radius))

    # ---------------- semáforo ----------------

    def traffic_light_state(self) -> str:
        if self.t < self._tl_yellow_at:
            return "vermelho"
        if self.t < self._tl_green_at:
            return "amarelo"
        return "verde"

    @property
    def green_at(self) -> float:
        return self._tl_green_at

    # ---------------- internos ----------------

    def _event(self, kind: str, **kw: Any) -> None:
        ev = {"t": round(self.t, 3), "tipo": kind, **kw}
        self.events.append(ev)
        self._new_events.append(ev)

    def _observe(self):
        s = self.car.state
        img, info = self.renderer.render((s.x, s.y, s.yaw), self.traffic_light_state())
        return img, info["objects"]

    def _yaw_rate(self) -> float:
        s = self.car.state
        return (s.right.ground_speed - s.left.ground_speed) / self.car_params.track_width

    def _telemetry(self) -> Telemetry:
        s = self.car.state
        gz_sum, acc_sum, n = self._imu_acc
        if n:
            self._gz_last, self._acc_last = gz_sum / n, acc_sum / n
            self._imu_acc = [0.0, np.zeros(3), 0]
        od = self.odometry
        return Telemetry(self.seq, round(self.t, 4), *od.pose, s.left.rpm_meas, s.right.rpm_meas,
                         s.ticks_left, s.ticks_right, gyro_z=float(self._gz_last),
                         accel=tuple(float(a) for a in self._acc_last),  # type: ignore[arg-type]
                         gyro_bias_est=od.bias, imu_calibrated=od.calibrated)

    def _truth(self, objs) -> Truth:
        s, rt = self.car.state, self.route
        assert rt is not None
        # e_y/e_ψ/progresso medidos no CENTRO do carro (o ponto que deve seguir a faixa)
        d = self.car_params.center_x
        proj = rt.project(s.x + d * math.cos(s.yaw), s.y + d * math.sin(s.yaw), self._s)
        self._s = proj.s
        e_psi = wrap_angle(s.yaw - proj.heading)
        on = bool(np.all(self.track.on_carpet(self.car.footprint(per_edge=12))))   # ~2–3 cm
        edge = rt.edge_at(proj.s)
        nj = rt.next_junction(proj.s)
        nxt = None
        if nj is not None:
            nxt = {"node": nj.node, "distance_m": round(nj.s_in - proj.s, 3), "options": nj.options,
                   "sign": nj.sign, "maneuver": nj.maneuver, "index": nj.index}
        return Truth(
            t=round(self.t, 4), x=s.x, y=s.y, yaw=s.yaw, v=s.v,
            omega=(s.right.ground_speed - s.left.ground_speed) / self.car_params.track_width,
            rpm_left=s.left.omega * RPM, rpm_right=s.right.omega * RPM,
            route_s=proj.s, route_length=rt.length, edge=edge[0] if edge else None,
            e_y=proj.e_y, e_psi=e_psi, curvature=proj.curvature,
            on_carpet=on, in_own_lane=abs(proj.e_y) <= rt.half_lane(proj.s),
            traffic_light=self.traffic_light_state(), next_junction=nxt, objects=objs,
            gyro_bias=self.imu.gyro_bias,
        )

    def _check_done(self, tr: Truth) -> tuple[bool, str]:
        rt = self.route
        assert rt is not None
        if self.traffic_light_state() != "verde" and tr.v > 0.02 and tr.route_s < 1.0 \
                and not any(e["tipo"] == "largou_antes_do_verde" for e in self.events):
            self._event("largou_antes_do_verde")
        if tr.in_own_lane != self._in_lane:
            self._in_lane = tr.in_own_lane
            self._event("entrou_na_contramao" if not tr.in_own_lane else "voltou_para_a_faixa",
                        e_y=round(tr.e_y, 3))

        # cruzamentos vencidos na rota certa
        while self._passed < len(rt.junctions) and tr.route_s > rt.junctions[self._passed].s_out + 0.3:
            j = rt.junctions[self._passed]
            if abs(tr.e_y) <= self.cfg.off_route_m:
                self._event("placa_respeitada" if j.sign else "cruzamento_sem_placa",
                            no=j.node, placa=j.sign, manobra=j.maneuver)
            self._passed += 1

        if not tr.on_carpet:
            self._event("saiu_da_pista")
            return True, "saiu_da_pista"
        if abs(tr.e_y) > self.cfg.off_route_m:
            j = rt.junction_near(tr.route_s)
            if j is not None:
                done_m = self._maneuver_taken(j)
                self._event("desrespeitou_placa", no=j.node, placa=j.sign, esperado=j.maneuver, feito=done_m)
                return True, "desrespeitou_placa"
            self._event("saiu_da_rota")
            return True, "saiu_da_rota"
        if tr.route_s >= rt.length - 0.05:
            self._event("rota_completa")
            return True, "rota_completa"
        if self.t >= self.cfg.max_episode_s:
            return True, "tempo_maximo"
        return False, ""

    def _maneuver_taken(self, j) -> str | None:
        """Para onde o carro foi, de fato, ao sair da rota num cruzamento."""
        s = self.car.state
        net = self.track.network
        best, best_d = None, 1e9
        for m, dep in net.options(j.arrival).items():
            pts = net.movement(j.arrival, dep)
            d = float(np.min(np.hypot(pts[:, 0] - s.x, pts[:, 1] - s.y)))
            if d < best_d:
                best, best_d = m, d
        return best
