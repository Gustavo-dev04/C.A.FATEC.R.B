"""Detector no laço: qual placa é da MINHA aproximação, e o semáforo.

O detector (YOLO11n) acha placas na imagem inteira — inclusive as das outras
ruas do cruzamento, que o regulamento põe ali de propósito para confundir
(o gerador de cenários também põe). O que decide a manobra é só a placa da
aproximação em que o carro está.

Como o Vista sabe qual é: pelo mapa. A regra 3.5.2 fixa onde a placa fica
(400 mm antes do cruzamento, à direita, na calçada, base a 400 mm); com a pose
estimada (``mapa.py``) e a calibração da câmera, dá para prever EM QUE PIXEL a
placa da nossa aproximação deve aparecer. Detecção longe desse ponto é de outra
rua e é ignorada. As detecções aceitas votam (soma da confiança) ao longo da
aproximação; a decisão sai de ``rules.decide`` sobre a mais votada.

Semáforo: ``verde`` em ``n_verde`` leituras seguidas do detector → larga.
"""

from __future__ import annotations

import ast
import json
import math
from collections import Counter, deque
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from vista_sim.camera import FisheyeCamera
from vista_sim.config import CameraParams
from vista_sim.roads import Edge, RoadNetwork
from vista_sim.rules import MANEUVERS, PROHIBITION, SignalError, decide
from vista_sim.scenario import road_test
from vista_sim.track import SIGN_CLASSES


@dataclass
class Deteccao:
    classe: str
    conf: float
    caixa: tuple[float, float, float, float]       # x0, y0, x1, y1 (px da imagem da câmera)

    @property
    def centro(self) -> tuple[float, float]:
        return 0.5 * (self.caixa[0] + self.caixa[2]), 0.5 * (self.caixa[1] + self.caixa[3])

    @property
    def lado(self) -> float:
        return max(self.caixa[2] - self.caixa[0], self.caixa[3] - self.caixa[1])


Detector = Callable[[np.ndarray], list[Deteccao]]


# --------------------------------------------------------------------------
# Detectores
# --------------------------------------------------------------------------


def letterbox(img: np.ndarray, tamanho: tuple[int, int]) -> tuple[np.ndarray, float, tuple[float, float]]:
    """Redimensiona mantendo a proporção e completa com cinza 114 — igual ao
    Ultralytics (``LetterBox(auto=False)``), senão as caixas saem tortas."""
    H, W = tamanho
    h, w = img.shape[:2]
    r = min(H / h, W / w)
    nw, nh = int(round(w * r)), int(round(h * r))
    dw, dh = (W - nw) / 2, (H - nh) / 2
    if (nw, nh) != (w, h):
        img = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=(114, 114, 114))
    return img, r, (left, top)


def decodificar_yolo(saida: np.ndarray, r: float, pad: tuple[float, float], nomes: dict[int, str],
                     conf: float = 0.35, iou: float = 0.5) -> list[Deteccao]:
    """Saída crua do YOLO11 exportado sem NMS, ``(1, 4 + nc, N)`` (cx, cy, w, h
    em pixels da entrada + probabilidade de cada classe) → detecções na imagem
    original, com NMS por classe."""
    p = np.asarray(saida)[0].T                      # (N, 4 + nc)
    scores = p[:, 4:]
    cls = np.argmax(scores, axis=1)
    sc = scores[np.arange(len(cls)), cls]
    keep = sc >= conf
    if not np.any(keep):
        return []
    p, cls, sc = p[keep], cls[keep], sc[keep]
    cx, cy, w, h = p[:, 0], p[:, 1], p[:, 2], p[:, 3]
    x0 = (cx - w / 2 - pad[0]) / r
    y0 = (cy - h / 2 - pad[1]) / r
    ww, hh = w / r, h / r
    caixas = np.stack([x0, y0, ww, hh], axis=1)
    out: list[Deteccao] = []
    for c in np.unique(cls):
        idx = np.where(cls == c)[0]
        k = cv2.dnn.NMSBoxes(caixas[idx].tolist(), sc[idx].tolist(), conf, iou)
        for i in np.asarray(k).reshape(-1):
            j = idx[int(i)]
            bx, by, bw, bh = caixas[j]
            out.append(Deteccao(nomes[int(c)], float(sc[j]), (float(bx), float(by), float(bx + bw), float(by + bh))))
    return out


class DetectorOnnx:
    """YOLO11n em ONNX (o MESMO arquivo que vira TensorRT na Jetson), via onnxruntime."""

    def __init__(self, caminho: Path | str, conf: float = 0.35, iou: float = 0.5, threads: int | None = None) -> None:
        import onnxruntime as ort

        so = ort.SessionOptions()
        if threads:
            so.intra_op_num_threads = threads
            so.inter_op_num_threads = 1
        prov = [p for p in ("CUDAExecutionProvider", "CPUExecutionProvider") if p in ort.get_available_providers()]
        self.sess = ort.InferenceSession(str(caminho), so, providers=prov)
        meta = self.sess.get_modelmeta().custom_metadata_map
        self.nomes = {int(k): v for k, v in ast.literal_eval(meta["names"]).items()}
        imgsz = ast.literal_eval(meta.get("imgsz", "[640, 640]"))
        self.tamanho = (int(imgsz[0]), int(imgsz[1]))
        self.entrada = self.sess.get_inputs()[0].name
        self.conf, self.iou = conf, iou

    def __call__(self, img_bgr: np.ndarray) -> list[Deteccao]:
        img, r, pad = letterbox(img_bgr, self.tamanho)
        x = (cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0).transpose(2, 0, 1)[None]
        out = self.sess.run(None, {self.entrada: np.ascontiguousarray(x)})[0]
        return decodificar_yolo(out, r, pad, self.nomes, self.conf, self.iou)


class DetectorVerdade:
    """Detector PERFEITO, a partir dos rótulos do simulador — só para isolar a
    rede de faixa nos testes. Passa as mesmas regras de rótulo do dataset do
    YOLO (≥ 6 px, ≥ 50 % à vista). Antes de cada chamada: ``det.objetos = truth.objects``."""

    def __init__(self, min_px: int = 6, min_visivel: float = 0.5) -> None:
        self.objetos: list[dict[str, Any]] = []
        self.min_px, self.min_visivel = min_px, min_visivel

    def __call__(self, img_bgr: np.ndarray | None = None) -> list[Deteccao]:
        out = []
        for o in self.objetos:
            if o["kind"] == "placa":
                c = o["class"]
            elif o["kind"] == "semaforo" and o["class"] in ("vermelho", "amarelo", "verde"):
                c = f"semaforo_{o['class']}"
            else:
                continue
            x0, y0, x1, y1 = o["bbox"]
            if min(x1 - x0, y1 - y0) < self.min_px or o.get("visible_frac", 1.0) < self.min_visivel:
                continue
            out.append(Deteccao(c, 1.0, (float(x0), float(y0), float(x1), float(y1))))
        return out


# --------------------------------------------------------------------------
# Leitura da placa da aproximação
# --------------------------------------------------------------------------


@dataclass
class PlacasConfig:
    janela_px: float = 20.0       # tolerância horizontal fixa em volta do pixel previsto
    janela_lados: float = 1.5     # + este tanto × o tamanho previsto da placa
    janela_v_px: float = 10.0     # na vertical a altura da placa é conhecida: janela menor
    janela_v_lados: float = 0.8
    tamanho_min: float = 0.6      # tamanho detectado / previsto: placa de OUTRO cruzamento, mais
    tamanho_max: float = 1.7      #   longe na mesma direção, aparece menor
    min_votos: float = 1.5        # soma de confiança para valer (≈ 2 leituras boas)
    min_conf: float = 0.35
    n_verde: int = 2              # leituras seguidas de verde para largar


@dataclass
class Leitura:
    """O que o Vista decidiu num cruzamento (para o registro e a avaliação)."""

    no: str
    chegada: Edge
    placa: str | None
    opcoes: list[str]
    manobra: str
    votos: dict[str, float] = field(default_factory=dict)
    alternativa: bool = False       # True = a regra não decidia; usou a política de reserva
    t: float = 0.0


class LeitorPlacas:
    """Atribui cada placa detectada à APROXIMAÇÃO do mapa que melhor a explica.

    Não só a aproximação atual: com a câmera a 15,6 cm e 15° para baixo, a
    placa (centro a 47,5 cm do chão) sai do campo de visão a menos de ~0,8 m.
    Nas ruas curtas do centro (faixa de 1,0 m) a placa do próximo cruzamento
    NUNCA aparece na própria aproximação — só de longe, antes do cruzamento
    anterior. Então todo quadro do detector vota em todas as aproximações
    visíveis, e a decisão usa o que se acumulou para a aproximação da vez.
    De quebra, a placa distratora de outra rua vai para a rua dela.
    """

    def __init__(self, net: RoadNetwork, map_data: dict[str, Any], camera: CameraParams,
                 cfg: PlacasConfig | None = None) -> None:
        self.net = net
        self.cam = FisheyeCamera(camera)
        self.cfg = cfg or PlacasConfig()
        spec = map_data["sign_spec"]
        self.diametro = float(spec["diameter"])
        z = float(spec["bottom_height"]) + self.diametro / 2
        is_road = road_test(map_data)
        # onde a placa de CADA aproximação fica, pela regra 3.5.2 (mesmo cálculo do simulador)
        self.ponto: dict[Edge, np.ndarray] = {}
        normais = []
        for nid in net.nodes:
            for arr in net.arrivals(nid):
                sp = net.sign_pose(arr, float(spec["before_crossing"]), float(spec["outward"]), is_road)
                self.ponto[arr] = np.array([sp["x"], sp["y"], z])
                normais.append((math.cos(sp["facing_yaw"]), math.sin(sp["facing_yaw"])))
        self._arr = list(self.ponto)
        self._pts = np.stack([self.ponto[a] for a in self._arr])
        self._normal = np.array(normais)
        self.semaforo_hist: deque[str | None] = deque(maxlen=max(self.cfg.n_verde, 1))
        self.reiniciar()

    def reiniciar(self) -> None:
        """Novo episódio: esquece votos e semáforo."""
        self.votos_por: dict[Edge | None, Counter[str]] = {}
        self.semaforo_hist.clear()
        self.verde = False
        self.nova_aproximacao(None)

    def nova_aproximacao(self, chegada: Edge | None) -> None:
        """Passa a decidir pela placa de ``chegada`` (os votos já acumulados ficam)."""
        self.chegada = chegada
        self.ultima_previsao: tuple[float, float, float] | None = None

    @property
    def votos(self) -> Counter[str]:
        """Votos da aproximação atual."""
        return self.votos_por.setdefault(self.chegada, Counter())

    def prever_todas(self, pose: tuple[float, float, float]) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Pixel (N, 2), visível (N,) e lado esperado em px (N,) da placa de cada aproximação."""
        pc = self.cam.car_to_cam(self.cam.world_to_car(self._pts, pose))
        uv, vis = self.cam.project_cam(pc)
        # de frente para a câmera (o verso da placa não é a placa)
        para_cam = np.array([pose[0], pose[1]]) - self._pts[:, :2]
        vis = vis & (np.einsum("ij,ij->i", self._normal, para_cam) > 0)
        dist = np.maximum(np.linalg.norm(pc, axis=1), 0.05)
        # escala do fisheye equidistante ≈ fx por radiano: lado ≈ fx · diâmetro / distância
        return uv, vis, self.cam.p.fx * self.diametro / dist

    def prever_pixel(self, pose: tuple[float, float, float]) -> tuple[float, float, float] | None:
        """(u, v, lado esperado em px) da placa da aproximação atual, ou None se fora da imagem."""
        if self.chegada is None:
            return None
        uv, vis, lado = self.prever_todas(pose)
        i = self._arr.index(self.chegada)
        return (float(uv[i, 0]), float(uv[i, 1]), float(lado[i])) if vis[i] else None

    def observar(self, deteccoes: list[Deteccao], pose: tuple[float, float, float]) -> None:
        c = self.cfg
        # semáforo: só existe um (na largada)
        tl = [d for d in deteccoes if d.classe.startswith("semaforo_") and d.conf >= c.min_conf]
        estado = max(tl, key=lambda d: d.conf).classe[len("semaforo_"):] if tl else None
        self.semaforo_hist.append(estado)
        if len(self.semaforo_hist) == self.semaforo_hist.maxlen and all(e == "verde" for e in self.semaforo_hist):
            self.verde = True

        uv, vis, lado = self.prever_todas(pose)
        if self.chegada is not None:
            i = self._arr.index(self.chegada)
            self.ultima_previsao = (float(uv[i, 0]), float(uv[i, 1]), float(lado[i])) if vis[i] else None
        cand = np.flatnonzero(vis)
        if len(cand) == 0:
            return
        jan_u = c.janela_px + c.janela_lados * lado[cand]
        jan_v = c.janela_v_px + c.janela_v_lados * lado[cand]
        for d in deteccoes:
            if d.classe not in SIGN_CLASSES or d.conf < c.min_conf:
                continue
            u, v = d.centro
            dist = np.hypot((u - uv[cand, 0]) / jan_u, (v - uv[cand, 1]) / jan_v)
            razao = d.lado / np.maximum(lado[cand], 1e-6)
            dist[(razao < c.tamanho_min) | (razao > c.tamanho_max)] = np.inf
            k = int(np.argmin(dist))
            if dist[k] <= 1.0:              # a aproximação MAIS PERTO do previsto
                self.votos_por.setdefault(self._arr[cand[k]], Counter())[d.classe] += d.conf

    def placa(self) -> str | None:
        """A placa lida para a aproximação atual (None = nenhuma)."""
        if not self.votos:
            return None
        cls, v = self.votos.most_common(1)[0]
        return cls if v >= self.cfg.min_votos else None

    def decidir(self, opcoes: set[str]) -> tuple[str, str | None, bool]:
        """(manobra, placa usada, usou_reserva).

        Tenta as placas por ordem de votos; a primeira que ``rules.decide``
        aceita vale (uma leitura impossível — "virar à esquerda" onde não há
        esquerda — é erro de detecção e cai para a próxima). Sem placa lida,
        ``decide(None)`` = em frente (política confirmada pela equipe).

        Reserva — POLÍTICA PROVISÓRIA, não é regra do regulamento: se houve
        placa lida mas nenhuma decide (proibição que deixa duas saídas, leitura
        impossível) ou não há placa nem "frente", escolhe na ordem frente →
        direita → esquerda entre as saídas que a placa não proíbe. Fica
        registrado (``alternativa=True``) para a equipe ver.
        """
        lidas = [cls for cls, v in self.votos.most_common() if v >= self.cfg.min_votos]
        for cls in lidas:
            try:
                return decide(cls, opcoes), cls, False
            except SignalError:
                continue
        if not lidas:
            try:
                return decide(None, opcoes), None, False
            except SignalError:
                pass
        lida = lidas[0] if lidas else None
        proibida = PROHIBITION.get(lida) if lida else None
        for m in ("frente", "direita", "esquerda"):
            if m in opcoes and m != proibida:
                return m, lida, True
        return sorted(opcoes, key=MANEUVERS.index)[0], lida, True

    def resumo(self) -> str:
        return json.dumps(dict(self.votos), ensure_ascii=False)
