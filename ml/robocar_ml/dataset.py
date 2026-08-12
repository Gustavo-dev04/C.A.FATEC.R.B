"""Carregamento do dataset coletado para treinamento.

Lê o formato descrito em ``docs/05-formato-dataset.md`` (JPEG + JSONL) e
entrega tensores prontos para a rede.

Duas decisões que evitam métricas enganosas:

1. **Divisão treino/validação por SESSÃO, nunca por frame.** Frames vizinhos
   da mesma volta são quase idênticos; dividir aleatoriamente coloca imagens
   quase iguais nos dois conjuntos, a validação fica ótima e o carro não anda.
2. **Balanceamento por histograma de esterço.** Um percurso qualquer é
   majoritariamente reta. Sem balancear, o modelo aprende que prever ~0
   minimiza a perda — e não faz curva.
"""

from __future__ import annotations

import logging
import random
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import torch
from torch.utils.data import Dataset

# Reaproveita o esquema do pacote do Jetson: uma definição só, dos dois lados.
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "jetson"))
from robocar.capture.schema import (  # noqa: E402
    find_sessions,
    iter_records,
    read_session_meta,
)

log = logging.getLogger(__name__)


@dataclass
class Sample:
    """Uma amostra indexada: caminho da imagem + rótulos."""

    image_path: Path
    steer: float
    throttle: float
    session: str


def load_samples(
    roots: list[Path],
    *,
    include_discarded: bool = False,
    min_throttle: float = 0.0,
) -> list[Sample]:
    """Varre diretórios de sessão e monta a lista de amostras."""
    samples: list[Sample] = []
    for root in roots:
        for session_dir in find_sessions(Path(root)):
            meta = read_session_meta(session_dir)
            if meta.discarded and not include_discarded:
                log.info("pulando sessão descartada %s (%s)", meta.session_id, meta.discard_reason)
                continue

            count = 0
            for record in iter_records(session_dir):
                if abs(record.throttle) < min_throttle:
                    continue
                if not record.usable_for_training:
                    continue
                image_path = session_dir / record.image
                if not image_path.exists():
                    continue
                samples.append(
                    Sample(
                        image_path=image_path,
                        steer=record.steer,
                        throttle=record.throttle,
                        session=meta.session_id,
                    )
                )
                count += 1
            log.info("%s: %d amostras", meta.session_id, count)

    if not samples:
        raise RuntimeError(
            "nenhuma amostra encontrada. Confira o caminho e rode "
            "`robocar dataset stats` para ver o que existe."
        )
    return samples


def split_by_session(
    samples: list[Sample], val_fraction: float = 0.2, seed: int = 42
) -> tuple[list[Sample], list[Sample]]:
    """Separa treino e validação por sessão inteira.

    Dividir por frame vazaria informação: quadros consecutivos da mesma volta
    são quase a mesma imagem, e a validação passaria a medir memorização em
    vez de generalização.
    """
    sessions = sorted({s.session for s in samples})
    if len(sessions) < 2:
        log.warning(
            "só há %d sessão(ões). A validação vai usar frames da mesma sessão "
            "e a métrica ficará otimista. Colete em condições variadas.",
            len(sessions),
        )
        rng = random.Random(seed)
        shuffled = samples[:]
        rng.shuffle(shuffled)
        cut = int(len(shuffled) * (1 - val_fraction))
        return shuffled[:cut], shuffled[cut:]

    rng = random.Random(seed)
    shuffled_sessions = sessions[:]
    rng.shuffle(shuffled_sessions)
    n_val = max(1, round(len(sessions) * val_fraction))
    val_sessions = set(shuffled_sessions[:n_val])

    train = [s for s in samples if s.session not in val_sessions]
    val = [s for s in samples if s.session in val_sessions]
    log.info(
        "treino: %d amostras (%d sessões) | validação: %d amostras (%d sessões)",
        len(train),
        len(sessions) - len(val_sessions),
        len(val),
        len(val_sessions),
    )
    return train, val


def steering_histogram(samples: list[Sample], bins: int = 25) -> tuple[np.ndarray, np.ndarray]:
    values = np.array([s.steer for s in samples], dtype=np.float32)
    counts, edges = np.histogram(values, bins=bins, range=(-1.0, 1.0))
    return counts, edges


def balance_samples(
    samples: list[Sample],
    bins: int = 25,
    keep_factor: float = 1.5,
    seed: int = 42,
) -> list[Sample]:
    """Reduz a super-representação das amostras de esterço próximo de zero.

    Mantém no máximo ``keep_factor * média`` amostras por faixa do histograma.
    Sem isso, a rede converge para "vai reto sempre" — que minimiza a perda e
    tira o carro da pista na primeira curva.
    """
    counts, edges = steering_histogram(samples, bins)
    nonzero = counts[counts > 0]
    if nonzero.size == 0:
        return samples

    limit = int(nonzero.mean() * keep_factor)
    rng = random.Random(seed)
    buckets: dict[int, list[Sample]] = {}
    for sample in samples:
        index = min(int((sample.steer + 1.0) / 2.0 * bins), bins - 1)
        buckets.setdefault(index, []).append(sample)

    balanced: list[Sample] = []
    for index, bucket in buckets.items():
        if len(bucket) > limit:
            balanced.extend(rng.sample(bucket, limit))
        else:
            balanced.extend(bucket)

    log.info(
        "balanceamento: %d -> %d amostras (teto de %d por faixa)",
        len(samples),
        len(balanced),
        limit,
    )
    return balanced


class DrivingDataset(Dataset):
    """Dataset de condução: imagem -> (esterço, aceleração)."""

    def __init__(
        self,
        samples: list[Sample],
        *,
        width: int = 200,
        height: int = 66,
        roi: tuple[float, float] = (0.45, 0.95),
        color_space: str = "yuv",
        augment: bool = False,
        predict_throttle: bool = False,
        horizontal_flip: bool = False,
    ) -> None:
        self.samples = samples
        self.width = width
        self.height = height
        self.roi = roi
        self.color_space = color_space
        self.augment = augment
        self.predict_throttle = predict_throttle
        self.horizontal_flip = horizontal_flip

        if horizontal_flip:
            log.warning(
                "espelhamento horizontal ATIVADO. Cuidado: na minicidade o "
                "carro anda pela DIREITA e as placas ficam sempre à direita — "
                "espelhar inverte essa semântica e ensina a regra errada. "
                "Ver docs/06-treinamento.md."
            )

    def __len__(self) -> int:
        return len(self.samples)

    def _load_image(self, path: Path) -> np.ndarray:
        import cv2

        image = cv2.imread(str(path))
        if image is None:
            raise RuntimeError(f"não foi possível ler {path}")

        height = image.shape[0]
        top = int(height * self.roi[0])
        bottom = int(height * self.roi[1])
        image = image[top:bottom, :]

        image = cv2.resize(image, (self.width, self.height), interpolation=cv2.INTER_AREA)
        if self.color_space == "yuv":
            image = cv2.cvtColor(image, cv2.COLOR_BGR2YUV)
        else:
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        return image

    def _augment(self, image: np.ndarray, steer: float) -> tuple[np.ndarray, float]:
        import cv2

        # Brilho: o ginásio pode ter iluminação bem diferente da coleta.
        if random.random() < 0.5:
            factor = random.uniform(0.6, 1.4)
            image = np.clip(image.astype(np.float32) * factor, 0, 255).astype(np.uint8)

        # Sombras retangulares: simulam vigas do teto e público em volta.
        if random.random() < 0.3:
            h, w = image.shape[:2]
            x1, x2 = sorted(random.sample(range(w), 2))
            overlay = image.copy()
            overlay[:, x1:x2] = (overlay[:, x1:x2] * random.uniform(0.4, 0.8)).astype(np.uint8)
            image = overlay

        # Deslocamento horizontal com correção proporcional do rótulo:
        # ensina o carro a voltar ao centro quando sai dele.
        if random.random() < 0.4:
            h, w = image.shape[:2]
            shift = random.randint(-int(w * 0.08), int(w * 0.08))
            matrix = np.float32([[1, 0, shift], [0, 1, 0]])
            image = cv2.warpAffine(image, matrix, (w, h), borderMode=cv2.BORDER_REPLICATE)
            steer = float(np.clip(steer + shift / w * 0.6, -1.0, 1.0))

        if self.horizontal_flip and random.random() < 0.5:
            image = np.fliplr(image).copy()
            steer = -steer

        return image, steer

    def __getitem__(self, index: int):
        sample = self.samples[index]
        image = self._load_image(sample.image_path)
        steer = sample.steer

        if self.augment:
            image, steer = self._augment(image, steer)

        tensor = torch.from_numpy(image.transpose(2, 0, 1).copy()).float() / 255.0
        tensor = (tensor - 0.5) / 0.5  # normaliza para [-1, 1]

        if self.predict_throttle:
            target = torch.tensor([steer, sample.throttle], dtype=torch.float32)
        else:
            target = torch.tensor([steer], dtype=torch.float32)
        return tensor, target


__all__ = [
    "DrivingDataset",
    "Sample",
    "balance_samples",
    "load_samples",
    "split_by_session",
    "steering_histogram",
]
