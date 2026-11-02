"""Monta o dataset do detector de placas e semáforo no formato YOLO (Ultralytics).

Classes: as 6 placas da Figura 8 + o semáforo em cada estado
(``treino.rotulos.CLASSES_DETECCAO``).

Regras de rótulo, a partir de ``objetos.jsonl``:

* rotula o objeto se ele tem ≥ ``min_px`` px no menor lado e ≥ ``min_visivel``
  da área à vista (o resto atrás de prédio/pessoa);
* objeto **ambíguo** — grande o bastante mas meio escondido — tira a imagem
  do dataset: sem rótulo ele viraria "fundo" e ensinaria a rede a NÃO ver uma
  placa que está lá. Objeto minúsculo (longe) é só ignorado;
* imagens sem nenhum objeto entram só numa fração (``frac_vazias``).

Treinar com ``fliplr=0``: virar à esquerda espelhada É virar à direita.
"""

from __future__ import annotations

import json
import os
import random
from pathlib import Path
from typing import Any

from .rotulos import CLASSES_DETECCAO, dividir_sementes


def _classe(o: dict[str, Any]) -> str | None:
    if o["kind"] == "placa":
        return o["class"]
    if o["kind"] == "semaforo" and o["class"] in ("vermelho", "amarelo", "verde"):
        return f"semaforo_{o['class']}"
    return None


def montar_yolo(pasta: Path, saida: Path, val_frac: float = 0.1, min_px: int = 6, min_visivel: float = 0.5,
                frac_vazias: float = 0.1, semente: int = 0, copiar: bool = False) -> dict[str, Any]:
    pasta, saida = Path(pasta).resolve(), Path(saida)
    meta = json.loads((pasta / "meta.json").read_text(encoding="utf-8"))
    W, H = meta["largura"], meta["altura"]
    linhas = [json.loads(s) for s in (pasta / "objetos.jsonl").read_text(encoding="utf-8").splitlines() if s]
    sementes = [int(r["arquivo"].split("/")[0][3:]) for r in linhas]
    tr_s, va_s = dividir_sementes(sementes, val_frac)
    rng = random.Random(semente)
    cont = {"imagens": {"train": 0, "val": 0}, "objetos": {c: 0 for c in CLASSES_DETECCAO},
            "descartadas_ambiguas": 0, "vazias_usadas": 0}
    for parte in ("train", "val"):
        (saida / "images" / parte).mkdir(parents=True, exist_ok=True)
        (saida / "labels" / parte).mkdir(parents=True, exist_ok=True)

    for r, sem in zip(linhas, sementes, strict=True):
        rot, ambigua = [], False
        for o in r["objetos"]:
            c = _classe(o)
            if c is None:
                continue
            x0, y0, x1, y1 = o["bbox"]
            lado = min(x1 - x0, y1 - y0)
            vis = o["visible_frac"]
            if lado >= min_px and vis >= min_visivel:
                rot.append((CLASSES_DETECCAO.index(c), (x0 + x1) / 2 / W, (y0 + y1) / 2 / H,
                            (x1 - x0) / W, (y1 - y0) / H))
            elif lado >= min_px and vis >= 0.1:
                ambigua = True                   # grande, mas meio escondido
            # minúsculo (< min_px): longe demais para qualquer detector — ignora
        if ambigua:
            cont["descartadas_ambiguas"] += 1
            continue
        if not rot:
            if rng.random() >= frac_vazias:
                continue
            cont["vazias_usadas"] += 1
        parte = "val" if sem in va_s else "train"
        nome = r["arquivo"].replace("/", "_")
        dst = saida / "images" / parte / nome
        if not dst.exists():
            src = pasta / r["arquivo"]
            if copiar:
                dst.write_bytes(src.read_bytes())
            else:
                os.symlink(src, dst)
        with open(saida / "labels" / parte / (Path(nome).stem + ".txt"), "w", encoding="utf-8") as fh:
            for k, cx, cy, w, h in rot:
                fh.write(f"{k} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}\n")
                cont["objetos"][CLASSES_DETECCAO[k]] += 1
        cont["imagens"][parte] += 1

    yaml_txt = (f"path: {saida.resolve()}\ntrain: images/train\nval: images/val\n"
                f"nc: {len(CLASSES_DETECCAO)}\nnames:\n" + "".join(f"  {i}: {c}\n" for i, c in enumerate(CLASSES_DETECCAO)))
    (saida / "data.yaml").write_text(yaml_txt, encoding="utf-8")
    cont["data_yaml"] = str(saida / "data.yaml")
    cont["episodios_treino"], cont["episodios_val"] = len(tr_s), len(va_s)
    return cont
