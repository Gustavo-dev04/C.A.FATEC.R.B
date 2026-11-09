"""O Vista inteiro: imagem + telemetria → rpm de cada roda.

Máquina de estados:

    AGUARDANDO_VERDE ──(detector: verde N vezes seguidas)──▶ FAIXA
    FAIXA ──(mapa: chegou à entrada do cruzamento)──▶ CRUZAMENTO
    CRUZAMENTO ──(entrou na faixa de saída)──▶ FAIXA (próxima rua do grafo)

* **FAIXA** — a rede de faixa dirige: imagem → rede → filtro (encoders +
  giroscópio) → ``PilotoFaixa``. A mesma medida amarra a pose ao mapa
  (``mapa.py``). O detector roda a cada ``detector_cada`` quadros e vota a
  placa da aproximação (``placas.py``); a manobra provisória (que condiciona a
  rede e a frenagem) sai de ``rules.decide`` com o voto até agora.
* **CRUZAMENTO** — a manobra é congelada e a curva do mapa é seguida com a
  pose da odometria (``cruzamento.py``). A rede não é consultada.

Só entra o que o carro tem: imagem, telemetria, mapa oficial, calibração da
câmera. Nenhuma verdade de campo.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from vista_sim.car import RPM
from vista_sim.config import CameraParams, CarParams
from vista_sim.controle import ControleConfig, FiltroFaixa, Percepcao, PilotoFaixa
from vista_sim.track import Track

from .cruzamento import CruzamentoConfig, ManobraOdometria
from .mapa import LocalizacaoConfig, LocalizadorGrafo
from .placas import Detector, LeitorPlacas, Leitura, PlacasConfig

# rede de faixa: (imagem_bgr, manobra) → [e_y, e_ψ, κ, dist, p_dentro]
RedeFaixa = Callable[[np.ndarray, str], np.ndarray]

AGUARDANDO_VERDE, FAIXA, CRUZAMENTO = "aguardando_verde", "faixa", "cruzamento"


@dataclass
class VistaConfig:
    v_cruzeiro: float = 0.50        # m/s na faixa (subir depois de 10/10 a 0,5)
    v_curva: float = 0.28           # m/s nas conversões
    entra_m: float = 0.15           # começa a curva do mapa a esta distância da entrada do cruzamento
    detector_cada: int = 2          # roda o detector a cada N quadros (orçamento de 33 ms)
    calibra_apos_m: float = 0.40    # mede a escala do giroscópio isto depois de sair da curva
    filtro: bool = True
    localizacao: LocalizacaoConfig = field(default_factory=LocalizacaoConfig)
    placas: PlacasConfig = field(default_factory=PlacasConfig)
    cruzamento: CruzamentoConfig = field(default_factory=CruzamentoConfig)


class VistaPiloto:
    def __init__(self, track: Track, car: CarParams, camera: CameraParams, rede_faixa: RedeFaixa,
                 detector: Detector, cfg: VistaConfig | None = None) -> None:
        self.cfg = cfg or VistaConfig()
        c = self.cfg
        self.net = track.network
        self.car = car
        self.track = track
        self.rede = rede_faixa
        self.detector = detector
        self.piloto = PilotoFaixa(car, ControleConfig(v_cruise=c.v_cruzeiro, v_turn=c.v_curva))
        self.filtro = FiltroFaixa(car) if c.filtro else None
        self.mapa = LocalizadorGrafo(self.net, car, c.localizacao)
        c.cruzamento.v_frente, c.cruzamento.v_curva = c.v_cruzeiro, c.v_curva
        self.manobra_cruz = ManobraOdometria(self.net, car, c.cruzamento)
        self.placas = LeitorPlacas(self.net, track.data, camera, c.placas)

    def reset(self, x: float | None = None, y: float | None = None, psi: float | None = None) -> None:
        """Largada. Sem argumentos: a pose de largada do mapa (o carro é posto nela)."""
        st = self.track.start
        x = st["x"] if x is None else x
        y = st["y"] if y is None else y
        psi = st["yaw"] if psi is None else psi
        self.estado = AGUARDANDO_VERDE
        self.quadro = 0
        self.t = 0.0
        self.piloto.reset()
        if self.filtro:
            self.filtro.reset()
        self.mapa.reset(x, y, psi)
        self._curva_aberta, self._s_saida = False, 0.0
        self.placas.reiniciar()
        self.placas.nova_aproximacao(self.mapa.edge)
        self.manobra = "frente"
        self.leituras: list[Leitura] = []
        self.percepcao: Percepcao | None = None
        self.log: list[dict[str, Any]] = []

    # ---------------- passo ----------------

    def act(self, obs: np.ndarray, tel, dt: float = 1 / 30) -> tuple[float, float]:
        """Um quadro: imagem + telemetria → (rpm_esquerda, rpm_direita)."""
        self.quadro += 1
        self.t = tel.t
        self.mapa.prever((tel.x, tel.y, tel.psi))
        # O detector roda também DENTRO do cruzamento: a placa da próxima
        # aproximação (rua curta) pode aparecer ainda durante a curva.
        if self.estado == AGUARDANDO_VERDE or self.quadro % self.cfg.detector_cada == 0:
            self.placas.observar(self.detector(obs), self.mapa.pose)
        if self.estado == CRUZAMENTO:
            return self._cruzamento(dt)

        if self.estado == AGUARDANDO_VERDE:
            if not self.placas.verde:
                return 0.0, 0.0
            self.estado = FAIXA
            self._registrar("largada")
        return self._faixa(obs, tel, dt)

    def _faixa(self, obs: np.ndarray, tel, dt: float) -> tuple[float, float]:
        opcoes = self.mapa.opcoes()
        self.manobra, placa, reserva = self.placas.decidir(set(opcoes))
        o = self.rede(obs, self.manobra)
        p = Percepcao(float(o[0]), float(o[1]), float(o[2]), float(o[3]), bool(o[4] > 0.5))
        if self.filtro is not None:
            v = 0.5 * (tel.rpm_left + tel.rpm_right) / RPM * self.car.wheel_radius
            w = tel.gyro_z - tel.gyro_bias_est
            p = self.filtro.atualizar(p, v, w, dt, kappa_atual=self.mapa.curvatura)
        self.percepcao = p
        self.mapa.corrigir(p)
        # escala do giroscópio: a curva anterior fecha quando o rumo já se
        # amarrou de novo à faixa (``calibra_apos_m`` depois de sair dela)
        entrando = self.mapa.dist_cruzamento <= self.cfg.entra_m
        if self._curva_aberta and (entrando or self.mapa.s - self._s_saida >= self.cfg.calibra_apos_m):
            amostra = self.mapa.fechar_curva()
            self._curva_aberta = False
            if amostra is not None:
                self._registrar("escala_giro", amostra=round(amostra, 4), escala=round(self.mapa.escala_giro, 4))

        if entrando:
            chegada = self.mapa.edge
            assert chegada is not None
            self.leituras.append(Leitura(self.mapa.no, chegada, placa, sorted(opcoes), self.manobra,
                                         dict(self.placas.votos), reserva, round(self.t, 3)))
            self._registrar("cruzamento", no=self.mapa.no, placa=placa, manobra=self.manobra, reserva=reserva)
            self.mapa.marcar_curva()
            self.manobra_cruz.iniciar(chegada, opcoes[self.manobra], self.manobra, self.mapa.s, self.piloto._v)
            self.placas.nova_aproximacao(opcoes[self.manobra])     # decisão congelada: já lê a próxima
            self.estado = CRUZAMENTO
            return self._cruzamento(dt)
        return self.piloto.act(p, self.manobra, True, dt)

    def _cruzamento(self, dt: float) -> tuple[float, float]:
        m = self.manobra_cruz
        cmd = m.act(*self.mapa.pose, dt)
        if m.terminou:
            self.mapa.entrar_na_rua(m.saida)
            self._curva_aberta, self._s_saida = True, self.mapa.s
            if self.filtro:
                self.filtro.reset()
            self.piloto._v = m.v
            self.estado = FAIXA
            self._registrar("saiu_do_cruzamento", rua=m.saida[0])
        return cmd

    def _registrar(self, tipo: str, **kw: Any) -> None:
        self.log.append({"t": round(self.t, 3), "tipo": tipo, **kw})
