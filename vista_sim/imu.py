"""IMU MPU6050 simulada e odometria rodas + giroscópio.

Duas partes, separadas de propósito:

* :class:`Mpu6050` — o SENSOR (só existe no simulador). Reproduz o que o
  datasheet promete e o que ele avisa: viés de fábrica de até ±20 °/s, fator
  de escala ±3 %, ruído branco, viés que anda com o tempo, quantização de
  16 bits e saturação no fundo de escala.
* :class:`WheelGyroOdometry` — a LÓGICA que vai para o carro (ESP32 ou
  Jetson). Calibra o viés com o carro parado — a espera no vermelho é a janela
  natural — e integra o rumo pelo giroscópio e o deslocamento pelos encoders.

Por que o giroscópio: o rumo pelos encoders erra com raio de roda diferente e
patinação (uma roda 0,5 % maior: 29 cm de desvio em 5 m, ADR 003). O
giroscópio mede a rotação do CORPO, que não liga para a roda. Mas ele tem seus
erros, e a calibração parada só corrige um deles (o viés); o fator de escala
precisa de outra calibração (girar o carro N voltas no lugar).
"""

from __future__ import annotations

import math

import numpy as np

from .config import ImuParams

G = 9.80665


def _quantize(v: float, full_scale: float) -> float:
    """ADC de 16 bits com sinal: passo = fundo de escala / 32768, satura nas pontas."""
    lsb = full_scale / 32768.0
    return float(min(max(round(v / lsb), -32768), 32767)) * lsb


class Mpu6050:
    def __init__(self, p: ImuParams, rng: np.random.Generator | None = None, randomize: bool = True) -> None:
        self.p = p
        self.rng = rng or np.random.default_rng()
        self.randomize = randomize
        bw = math.sqrt(p.dlpf_hz)
        self._sig_g = p.gyro_noise_density * bw          # ≈ 0,05 °/s RMS com DLPF de 98 Hz
        self._sig_a = p.accel_noise_density * bw
        self._rw = p.gyro_bias_drift / math.sqrt(100.0)  # passeio aleatório: anda isso em ~100 s
        self.reset()

    def reset(self) -> None:
        p = self.p
        if self.randomize:
            self.gyro_bias = float(self.rng.uniform(-p.gyro_initial_bias, p.gyro_initial_bias))
            self.gyro_scale = 1.0 + float(self.rng.uniform(-p.gyro_scale_tolerance, p.gyro_scale_tolerance))
            self.accel_bias = self.rng.uniform(-1.0, 1.0, 3) * np.asarray(p.accel_initial_bias)
        else:
            self.gyro_bias, self.gyro_scale = 0.0, 1.0
            self.accel_bias = np.zeros(3)

    def sample(self, v: float, omega: float, v_dot: float, omega_dot: float, dt: float) -> tuple[float, np.ndarray]:
        """Uma leitura. Entra o movimento VERDADEIRO do carro (no eixo);
        sai o que o sensor reporta: giro z (rad/s) e aceleração (m/s², x/y/z)."""
        p = self.p
        n = self.rng.normal(0.0, 1.0, 5)
        if self._rw > 0:
            self.gyro_bias += self._rw * math.sqrt(dt) * n[0]
        gz = _quantize(omega * self.gyro_scale + self.gyro_bias + self._sig_g * n[1], p.gyro_range)
        # Aceleração no PONTO da IMU (x à frente do eixo): braço de alavanca.
        ax = v_dot - omega * omega * p.x
        ay = v * omega + omega_dot * p.x
        acc = np.array([ax, ay, G]) + self.accel_bias + self._sig_a * n[2:5]
        acc = np.array([_quantize(a, p.accel_range) for a in acc])
        return gz, acc


class WheelGyroOdometry:
    """Odometria do carro: deslocamento pelos encoders, rumo pelo giroscópio.

    * **Parado** (nenhum tick há ``still_s``): o carro de tração diferencial
      não gira sem girar as rodas, então o rumo NÃO é integrado (evita que o
      viés ainda não calibrado gire o carro no vermelho) e o giroscópio é
      promediado para estimar o viés.
    * **Andando**: rumo += (giro − viés)·dt; posição pelos ticks.

    Guarda também a odometria só por encoders (``enc``), para comparação.
    """

    def __init__(self, track_width: float, wheel_radius: float, counts_per_rev: int,
                 still_s: float = 0.3, min_calib_s: float = 0.5) -> None:
        self.b = track_width
        self.m_per_tick = 2.0 * math.pi * wheel_radius / counts_per_rev
        self.still_s, self.min_calib_s = still_s, min_calib_s
        self.reset(0.0, 0.0, 0.0)

    def reset(self, x: float, y: float, psi: float) -> None:
        self.x, self.y, self.psi = x, y, psi
        self.enc = [x, y, psi]
        self.bias = 0.0
        self.calibrated = False
        # Na largada o carro está parado por definição: já começa "parado",
        # senão os primeiros 0,3 s integrariam o viés cru (até 20 °/s).
        self._still = self.still_s
        self._sum = 0.0
        self._n = 0
        self._last = (0, 0)

    def update(self, ticks_left: int, ticks_right: int, gyro_z: float, dt: float) -> None:
        dl_t, dr_t = ticks_left - self._last[0], ticks_right - self._last[1]
        self._last = (ticks_left, ticks_right)
        if dl_t == 0 and dr_t == 0:
            self._still += dt
            if self._still >= self.still_s:
                self._sum += gyro_z
                self._n += 1
                if self._n * dt >= self.min_calib_s:
                    self.bias = self._sum / self._n
                    self.calibrated = True
                return                                   # parado: rumo não muda
        else:
            self._still, self._sum, self._n = 0.0, 0.0, 0

        dl, dr = dl_t * self.m_per_tick, dr_t * self.m_per_tick
        ds = 0.5 * (dl + dr)
        # só encoders
        ex, ey, ep = self.enc
        dp_enc = (dr - dl) / self.b
        self.enc = [ex + ds * math.cos(ep + dp_enc / 2), ey + ds * math.sin(ep + dp_enc / 2), ep + dp_enc]
        # encoders + giroscópio
        dp = (gyro_z - self.bias) * dt
        self.x += ds * math.cos(self.psi + dp / 2)
        self.y += ds * math.sin(self.psi + dp / 2)
        self.psi += dp

    @property
    def pose(self) -> tuple[float, float, float]:
        return self.x, self.y, _wrap(self.psi)

    @property
    def enc_pose(self) -> tuple[float, float, float]:
        return self.enc[0], self.enc[1], _wrap(self.enc[2])


def _wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi
