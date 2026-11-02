"""Rede de faixa: imagem + manobra → onde o carro está na trilha.

Entrada: a imagem da câmera (RGB, 0–1) e a manobra do próximo cruzamento
(one-hot ``frente/esquerda/direita``, decidida pela placa em ``rules.py``).
Saída, em unidades SI:

* ``e_y``  — erro lateral do centro do carro (m, + = à esquerda);
* ``e_ψ``  — erro de rumo (rad, + = apontando para a esquerda);
* ``κ``    — curvatura média nos próximos 40 cm (1/m);
* ``dist`` — distância até a entrada do próximo cruzamento (m, satura em 3);
* ``p``    — probabilidade de já estar DENTRO do cruzamento.

Por que a manobra entra: fora do cruzamento ela não muda nada (a faixa é a
mesma); dentro, a trilha depende de virar ou seguir — sem a manobra, a mesma
imagem teria três respostas certas. É o "imitation learning condicional".

Backbone ResNet18 pré-treinado na ImageNet (torchvision): aprende rápido com
poucos dados e roda em ~2–3 ms na Orin Nano com TensorRT FP16.

Essas saídas alimentam ``vista_sim.controle.PilotoFaixa`` → rpm de cada roda.
"""

from __future__ import annotations

import csv
import json
import math
import time
from collections.abc import Callable
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import torch
from torch import nn
from torch.utils.data import DataLoader, Dataset

from .rotulos import ALVOS, DIST_MAX, ESCALA, KAPPA_M, MANOBRAS, dividir_sementes

IMAGENET_MEAN = (0.485, 0.456, 0.406)
IMAGENET_STD = (0.229, 0.224, 0.225)


# --------------------------------------------------------------------------
# dados
# --------------------------------------------------------------------------


def ler_rotulos(pasta: Path) -> list[dict[str, str]]:
    with open(Path(pasta) / "rotulos.csv", encoding="utf-8") as fh:
        return list(csv.DictReader(fh))


class FaixaDataset(Dataset):
    """Imagens + rótulos de ``tools/gerar_dataset.py``.

    Devolve a imagem em **uint8** (os workers só decodificam e redimensionam):
    4× menos memória compartilhada que float32, e o aumento de dados roda em
    lote na GPU (:func:`aumentar_lote`).
    """

    def __init__(self, pasta: Path, linhas: list[dict[str, str]], tamanho: tuple[int, int]) -> None:
        self.pasta = Path(pasta)
        self.linhas = linhas
        self.h, self.w = tamanho
        self.y = np.array([[float(r[a]) / ESCALA[a] for a in ALVOS] for r in linhas], np.float32)
        self.cruz = np.array([float(r["no_cruzamento"]) for r in linhas], np.float32)
        self.man = np.array([MANOBRAS.index(r["manobra"]) for r in linhas], np.int64)

    def __len__(self) -> int:
        return len(self.linhas)

    def __getitem__(self, i: int):
        img = cv2.imread(str(self.pasta / self.linhas[i]["arquivo"]), cv2.IMREAD_COLOR)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        if img.shape[:2] != (self.h, self.w):
            img = cv2.resize(img, (self.w, self.h), interpolation=cv2.INTER_AREA)
        x = torch.from_numpy(np.ascontiguousarray(img.transpose(2, 0, 1)))       # (3, H, W) uint8
        man = torch.zeros(len(MANOBRAS))
        man[self.man[i]] = 1.0
        return x, man, torch.from_numpy(self.y[i]), torch.tensor(self.cruz[i])


def aumentar_lote(x: torch.Tensor) -> torch.Tensor:
    """Aumento de dados em LOTE, no dispositivo do lote (GPU no Colab).

    Só FOTOMÉTRICO — cor, contraste, gama, desfoque, ruído e borrões
    (oclusão). Nada de espelhar (troca mão de direção e o sinal de e_y) nem
    girar (muda e_ψ). ``x``: (N, 3, H, W) float em [0, 1].
    """
    n, _, h, w = x.shape
    dev = x.device

    def r(*shape: int) -> torch.Tensor:
        return torch.rand(*shape, device=dev)

    um = torch.ones(n, 1, 1, 1, device=dev)
    sel = (r(n, 1, 1, 1) < 0.8).float()                                   # brilho, contraste, tom
    media = x.mean(dim=(1, 2, 3), keepdim=True)
    xa = ((x - media) * (0.7 + 0.6 * r(n, 1, 1, 1)) + media) * (0.7 + 0.6 * r(n, 1, 1, 1)) * (0.9 + 0.2 * r(n, 3, 1, 1))
    x = sel * xa + (1 - sel) * x
    gama = torch.where(r(n, 1, 1, 1) < 0.3, 0.7 + 0.6 * r(n, 1, 1, 1), um)
    x = x.clamp(0.0, 1.0) ** gama
    borra = r(n) < 0.2                                                    # desfoque
    if bool(borra.any()):
        x[borra] = nn.functional.avg_pool2d(x[borra], 3, 1, 1, count_include_pad=False)
    sigma = torch.where(r(n, 1, 1, 1) < 0.5, 0.01 + 0.04 * r(n, 1, 1, 1), 0 * um)
    x = x + torch.randn_like(x) * sigma                                   # ruído
    # borrão retangular (algo na frente da câmera) em 25 % das imagens
    hh = (h * (0.05 + 0.2 * r(n))).long()
    ww = (w * (0.05 + 0.2 * r(n))).long()
    y0 = (r(n) * (h - hh)).long()
    x0 = (r(n) * (w - ww)).long()
    yy = torch.arange(h, device=dev).view(1, h, 1)
    xx = torch.arange(w, device=dev).view(1, 1, w)
    caixa = ((yy >= y0.view(-1, 1, 1)) & (yy < (y0 + hh).view(-1, 1, 1))
             & (xx >= x0.view(-1, 1, 1)) & (xx < (x0 + ww).view(-1, 1, 1))
             & (r(n) < 0.25).view(-1, 1, 1)).unsqueeze(1)
    x = torch.where(caixa, r(n, 3, 1, 1).expand(-1, -1, h, w), x)
    return x.clamp_(0.0, 1.0)


# --------------------------------------------------------------------------
# modelo
# --------------------------------------------------------------------------


class RedeFaixa(nn.Module):
    """ResNet18 + manobra → 4 regressões normalizadas + 1 logit."""

    def __init__(self, pretrained: bool = True) -> None:
        super().__init__()
        from torchvision.models import ResNet18_Weights, resnet18

        net = resnet18(weights=ResNet18_Weights.IMAGENET1K_V1 if pretrained else None)
        net.fc = nn.Identity()
        self.backbone = net
        self.head = nn.Sequential(
            nn.Linear(512 + len(MANOBRAS), 256), nn.ReLU(inplace=True), nn.Dropout(0.1),
            nn.Linear(256, len(ALVOS) + 1),
        )
        self.register_buffer("mean", torch.tensor(IMAGENET_MEAN).view(1, 3, 1, 1))
        self.register_buffer("std", torch.tensor(IMAGENET_STD).view(1, 3, 1, 1))

    def forward(self, img01: torch.Tensor, manobra: torch.Tensor) -> torch.Tensor:
        f = self.backbone((img01 - self.mean) / self.std)
        return self.head(torch.cat([f, manobra], dim=1))


class RedeFaixaExport(nn.Module):
    """O que vai para a Jetson: saída já em unidades SI e probabilidade.

    Entradas: ``imagem`` (N, 3, H, W) RGB em [0, 1] e ``manobra`` (N, 3) one-hot.
    Saída: (N, 5) = e_y [m], e_ψ [rad], κ [1/m], dist [m], p(dentro do cruzamento).
    """

    def __init__(self, rede: RedeFaixa) -> None:
        super().__init__()
        self.rede = rede
        self.register_buffer("escala", torch.tensor([ESCALA[a] for a in ALVOS]))

    def forward(self, imagem: torch.Tensor, manobra: torch.Tensor) -> torch.Tensor:
        out = self.rede(imagem, manobra)
        reg = out[:, : len(ALVOS)] * self.escala
        return torch.cat([reg, torch.sigmoid(out[:, len(ALVOS):])], dim=1)


# --------------------------------------------------------------------------
# treino
# --------------------------------------------------------------------------

PESOS = torch.tensor([1.0, 1.0, 0.5, 0.5])        # e_y, e_ψ, κ, dist
PESO_CRUZ = 0.2


def _perda(out: torch.Tensor, y: torch.Tensor, cruz: torch.Tensor) -> torch.Tensor:
    reg = nn.functional.smooth_l1_loss(out[:, :4], y, reduction="none", beta=0.5)
    reg = (reg * PESOS.to(out.device)).sum(dim=1).mean()
    bce = nn.functional.binary_cross_entropy_with_logits(out[:, 4], cruz)
    return reg + PESO_CRUZ * bce


@torch.no_grad()
def avaliar(modelo: nn.Module, loader: DataLoader, device: str) -> dict[str, float]:
    """Erros em unidades físicas, separados fora × dentro do cruzamento."""
    modelo.eval()
    err, cruz_real, cruz_pred, perdas = [], [], [], []
    for x, man, y, cz in loader:
        x = x.to(device, non_blocking=True).float().div_(255.0)
        man, y, cz = man.to(device), y.to(device), cz.to(device)
        with torch.autocast(device_type="cuda", enabled=device.startswith("cuda")):
            out = modelo(x, man)
        out = out.float()
        perdas.append(float(_perda(out, y, cz)) * len(x))
        err.append(((out[:, :4] - y) * torch.tensor([ESCALA[a] for a in ALVOS], device=device)).abs().cpu())
        cruz_real.append(cz.cpu())
        cruz_pred.append((out[:, 4] > 0).float().cpu())
    e = torch.cat(err)
    cr, cp = torch.cat(cruz_real), torch.cat(cruz_pred)
    fora, dentro = cr < 0.5, cr >= 0.5
    m: dict[str, float] = {"perda": sum(perdas) / max(len(e), 1)}
    for nome, mask in (("faixa", fora), ("cruzamento", dentro)):
        if mask.any():
            m[f"e_y_cm_{nome}"] = float(e[mask, 0].mean()) * 100
            m[f"e_psi_graus_{nome}"] = math.degrees(float(e[mask, 1].mean()))
            m[f"kappa_{nome}"] = float(e[mask, 2].mean())
    m["dist_cm"] = float(e[fora, 3].mean()) * 100 if fora.any() else float("nan")
    m["acerto_cruzamento"] = float((cp == cr).float().mean())
    m["e_y_ate_2cm"] = float((e[:, 0] < 0.02).float().mean())
    return m


def treinar(pasta: Path, saida: Path, epocas: int = 20, lote: int = 128, lr: float = 5e-4,
            tamanho: tuple[int, int] = (240, 320), val_frac: float = 0.1, workers: int = 4,
            device: str | None = None, pretrained: bool = True, limite: int | None = None,
            inicial: Path | None = None, log: Callable[[str], Any] = print) -> dict[str, Any]:
    """Treina, guarda o melhor (perda de validação) em ``saida/faixa_melhor.pt``.

    ``inicial``: checkpoint para CONTINUAR (ajuste fino de uma rodada de
    DAgger) em vez de partir da ImageNet. Use ``lr`` menor (~2e-4).
    """
    device = device or ("cuda" if torch.cuda.is_available() else "cpu")
    saida = Path(saida)
    saida.mkdir(parents=True, exist_ok=True)
    linhas = ler_rotulos(pasta)
    tr_s, va_s = dividir_sementes([int(r["semente"]) for r in linhas], val_frac)
    tr = [r for r in linhas if int(r["semente"]) in tr_s]
    va = [r for r in linhas if int(r["semente"]) in va_s]
    if limite:
        tr, va = tr[:limite], va[: max(1, limite // 5)]
    log(f"treino: {len(tr)} imagens de {len(tr_s)} episódios | validação: {len(va)} de {len(va_s)} episódios")
    pin = device.startswith("cuda")
    if pin:
        torch.backends.cudnn.benchmark = True
    dl_tr = DataLoader(FaixaDataset(pasta, tr, tamanho), lote, shuffle=True,
                       num_workers=workers, pin_memory=pin, drop_last=len(tr) > lote,
                       persistent_workers=workers > 0)
    dl_va = DataLoader(FaixaDataset(pasta, va, tamanho), lote, num_workers=workers, pin_memory=pin)

    modelo = RedeFaixa(pretrained and inicial is None)
    if inicial is not None:
        ck = torch.load(inicial, map_location="cpu", weights_only=False)
        if tuple(ck["tamanho"]) != tuple(tamanho):
            raise ValueError(f"o checkpoint é de {tuple(ck['tamanho'])}, não {tuple(tamanho)}")
        modelo.load_state_dict(ck["modelo"])
        log(f"continuando de {inicial}")
    modelo = modelo.to(device).to(memory_format=torch.channels_last)
    otim = torch.optim.AdamW(modelo.parameters(), lr=lr, weight_decay=1e-4)
    passos = max(1, epocas * len(dl_tr))
    aquec = min(500, passos // 10)
    sched = torch.optim.lr_scheduler.LambdaLR(
        otim, lambda k: min(1.0, (k + 1) / max(aquec, 1)) * 0.5 * (1 + math.cos(math.pi * min(k / passos, 1.0))))
    scaler = torch.amp.GradScaler("cuda", enabled=pin)

    hist, melhor = [], float("inf")
    for ep in range(1, epocas + 1):
        modelo.train()
        t0, soma, n = time.perf_counter(), 0.0, 0
        for x, man, y, cz in dl_tr:
            x = aumentar_lote(x.to(device, non_blocking=True).float().div_(255.0))
            x = x.contiguous(memory_format=torch.channels_last)
            man, y, cz = man.to(device), y.to(device), cz.to(device)
            with torch.autocast(device_type="cuda", enabled=pin):
                perda = _perda(modelo(x, man).float(), y, cz)
            otim.zero_grad(set_to_none=True)
            scaler.scale(perda).backward()
            scaler.step(otim)
            scaler.update()
            sched.step()
            soma += float(perda.detach()) * len(x)
            n += len(x)
        m = avaliar(modelo, dl_va, device)
        m.update(epoca=ep, perda_treino=soma / max(n, 1), segundos=round(time.perf_counter() - t0, 1))
        hist.append(m)
        marca = ""
        if m["perda"] < melhor:
            melhor = m["perda"]
            torch.save({"modelo": modelo.state_dict(), "tamanho": tamanho, "metricas": m}, saida / "faixa_melhor.pt")
            marca = "  ← melhor"
        log(f"época {ep:2d}/{epocas}  treino {m['perda_treino']:.4f}  val {m['perda']:.4f}  "
            f"e_y {m.get('e_y_cm_faixa', float('nan')):.2f} cm (cruz. {m.get('e_y_cm_cruzamento', float('nan')):.2f})  "
            f"e_ψ {m.get('e_psi_graus_faixa', float('nan')):.2f}°  dist {m['dist_cm']:.1f} cm  "
            f"cruz. {m['acerto_cruzamento'] * 100:.1f}%  {m['segundos']:.0f}s{marca}")
    (saida / "faixa_historico.json").write_text(json.dumps(hist, indent=2), encoding="utf-8")
    return {"historico": hist, "melhor": saida / "faixa_melhor.pt"}


def carregar(caminho: Path, device: str = "cpu") -> tuple[RedeFaixa, tuple[int, int]]:
    ck = torch.load(caminho, map_location=device, weights_only=False)
    modelo = RedeFaixa(pretrained=False)
    modelo.load_state_dict(ck["modelo"])
    return modelo.to(device).eval(), tuple(ck["tamanho"])  # type: ignore[return-value]


def exportar_onnx(modelo: RedeFaixa, tamanho: tuple[int, int], caminho: Path) -> dict[str, Any]:
    """ONNX (opset 17, lote dinâmico) + JSON com o contrato de entrada/saída.

    O motor TensorRT é gerado NA JETSON a partir deste ONNX (é específico do
    dispositivo): ``trtexec --onnx=faixa.onnx --fp16 --saveEngine=faixa.engine``.
    """
    caminho = Path(caminho)
    exp = RedeFaixaExport(modelo.cpu().eval()).eval()
    h, w = tamanho
    img = torch.rand(1, 3, h, w)
    man = torch.tensor([[1.0, 0.0, 0.0]])
    kw = dict(input_names=["imagem", "manobra"], output_names=["saida"],
              dynamic_axes={"imagem": {0: "n"}, "manobra": {0: "n"}, "saida": {0: "n"}})
    # Opset 17 pelo exportador CLÁSSICO: é o que qualquer TensorRT ≥ 8.6
    # (JetPack 6.0 em diante) lê. O exportador novo (torch.export) só gera
    # opset ≥ 18 — fica de reserva para quando o clássico sumir do PyTorch.
    import warnings

    try:
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", DeprecationWarning)
            torch.onnx.export(exp, (img, man), str(caminho), dynamo=False, opset_version=17, **kw)
        opset = 17
    except Exception:  # noqa: BLE001 — clássico removido/quebrado: tenta o novo
        torch.onnx.export(exp, (img, man), str(caminho), dynamo=True, opset_version=18, **kw)
        opset = 18
    meta = {
        "entrada": {"imagem": f"(N, 3, {h}, {w}) RGB float32 em [0, 1], redimensionada da câmera com INTER_AREA",
                    "manobra": f"(N, 3) one-hot na ordem {list(MANOBRAS)}"},
        "saida": ["e_y_m", "e_psi_rad", "kappa_1_m", "dist_cruzamento_m", "p_dentro_cruzamento"],
        "altura": h, "largura": w, "manobras": list(MANOBRAS), "dist_max": DIST_MAX, "kappa_janela_m": KAPPA_M,
        "opset": opset,
    }
    caminho.with_suffix(".json").write_text(json.dumps(meta, indent=2, ensure_ascii=False), encoding="utf-8")
    dif = None
    try:
        import onnxruntime as ort

        sess = ort.InferenceSession(str(caminho), providers=["CPUExecutionProvider"])
        x = torch.rand(4, 3, h, w)
        mm = torch.eye(3)[[0, 1, 2, 0]]
        ref = exp(x, mm).detach().numpy()
        got = sess.run(None, {"imagem": x.numpy(), "manobra": mm.numpy()})[0]
        dif = float(np.abs(ref - got).max())
    except ImportError:
        pass
    return {"onnx": str(caminho), "meta": str(caminho.with_suffix(".json")), "opset": opset, "dif_max_torch_onnx": dif}


class PreditorTorch:
    """Adapta o modelo PyTorch à avaliação no simulador (GPU se houver)."""

    def __init__(self, modelo: RedeFaixa, tamanho: tuple[int, int], device: str | None = None) -> None:
        self.device = device or ("cuda" if torch.cuda.is_available() else "cpu")
        self.exp = RedeFaixaExport(modelo).to(self.device).eval()
        self.h, self.w = tamanho

    @torch.no_grad()
    def __call__(self, img_bgr: np.ndarray, manobra: str) -> np.ndarray:
        img = cv2.cvtColor(cv2.resize(img_bgr, (self.w, self.h), interpolation=cv2.INTER_AREA), cv2.COLOR_BGR2RGB)
        x = torch.from_numpy(img).permute(2, 0, 1)[None].float().div_(255.0).to(self.device)
        m = torch.zeros(1, 3, device=self.device)
        m[0, MANOBRAS.index(manobra)] = 1.0
        return self.exp(x, m)[0].float().cpu().numpy()
