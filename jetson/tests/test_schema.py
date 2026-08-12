"""Testes do esquema do dataset.

O que se garante aqui: um registro gravado hoje continua legível depois, e um
arquivo truncado por queda de energia não derruba a sessão inteira.
"""

from __future__ import annotations

import json

import pytest

from robocar.capture import schema


def _record(**kwargs):
    base = {
        "index": 0,
        "image": "images/000000.jpg",
        "t_host_ns": 1_000_000_000,
        "steer": 0.25,
        "throttle": 0.4,
    }
    base.update(kwargs)
    return schema.Record(**base)


def test_ida_e_volta_json():
    original = _record(
        distances_mm=[1200, 800, 1500], flags=0x01, vbat=7.8, source=2
    )
    recuperado = schema.Record.from_dict(json.loads(original.to_json()))
    assert recuperado == original


def test_campos_none_nao_vao_para_o_json():
    # cmd_steer/cmd_throttle são opcionais; ausentes não devem poluir o arquivo.
    data = json.loads(_record().to_json())
    assert "cmd_steer" not in data
    assert "cmd_throttle" not in data


def test_campo_desconhecido_eh_ignorado_na_leitura():
    # Compatibilidade para frente: um campo novo não quebra um leitor antigo.
    data = json.loads(_record().to_json())
    data["campo_do_futuro"] = 42
    assert schema.Record.from_dict(data).index == 0


def test_campo_obrigatorio_ausente_levanta():
    data = json.loads(_record().to_json())
    del data["steer"]
    with pytest.raises(schema.SchemaError):
        schema.Record.from_dict(data)


@pytest.mark.parametrize("valor", [-1.5, 1.5, 2.0])
def test_steer_fora_de_faixa_levanta(valor):
    with pytest.raises(schema.SchemaError):
        _record(steer=valor).validate()


def test_distancia_negativa_levanta():
    with pytest.raises(schema.SchemaError):
        _record(distances_mm=[100, -5]).validate()


@pytest.mark.parametrize(
    "flags,esperado",
    [
        (0x00, True),
        (0x01, True),   # ARMED
        (0x02, False),  # FAILSAFE
        (0x08, False),  # OBSTACLE_STOP
        (0x40, False),  # BATT_CRITICAL
        (0x10, True),   # OBSTACLE_SLOW: apenas limita, não invalida
        (0x20, True),   # BATT_LOW
    ],
)
def test_usable_for_training(flags, esperado):
    assert _record(flags=flags).usable_for_training is esperado


def test_mascara_bate_com_o_protocolo():
    # schema.py não importa protocol.py (é stdlib puro), então a máscara está
    # duplicada. Este teste impede que as duas se separem em silêncio.
    from robocar.comms.protocol import UNSAFE_FOR_TRAINING

    assert int(UNSAFE_FOR_TRAINING) == 0x4A


# --- leitura e escrita em disco -------------------------------------------


def test_escreve_e_le_sessao(tmp_path):
    meta = schema.SessionMeta(session_id="20261204_140000_teste", track="minicidade")
    schema.write_session_meta(tmp_path, meta)
    assert schema.read_session_meta(tmp_path).track == "minicidade"


def test_iter_records_pula_linha_truncada(tmp_path):
    # Cenário real: a bateria acaba no meio da gravação e a última linha fica
    # pela metade. Perder a sessão inteira por causa disso seria inaceitável.
    linhas = [_record(index=i).to_json() for i in range(3)]
    conteudo = "\n".join(linhas) + "\n" + '{"index":3,"ima'
    (tmp_path / schema.RECORDS_FILENAME).write_text(conteudo, encoding="utf-8")

    registros = schema.read_records(tmp_path)
    assert [r.index for r in registros] == [0, 1, 2]


def test_iter_records_strict_levanta_na_linha_ruim(tmp_path):
    (tmp_path / schema.RECORDS_FILENAME).write_text(
        _record().to_json() + "\n{lixo}\n", encoding="utf-8"
    )
    with pytest.raises(schema.SchemaError):
        schema.read_records(tmp_path, strict=True)


def test_iter_records_ignora_linhas_em_branco(tmp_path):
    (tmp_path / schema.RECORDS_FILENAME).write_text(
        _record().to_json() + "\n\n\n", encoding="utf-8"
    )
    assert len(schema.read_records(tmp_path)) == 1


def test_sessao_sem_records_levanta(tmp_path):
    with pytest.raises(FileNotFoundError):
        schema.read_records(tmp_path)


def test_find_sessions_ordenado_e_so_com_metadados(tmp_path):
    for nome in ("20261204_150000_b", "20261204_140000_a"):
        (tmp_path / nome).mkdir()
        schema.write_session_meta(tmp_path / nome, schema.SessionMeta(session_id=nome))
    (tmp_path / "pasta_solta").mkdir()  # sem session.json: não é sessão

    encontrados = [p.name for p in schema.find_sessions(tmp_path)]
    assert encontrados == ["20261204_140000_a", "20261204_150000_b"]


def test_find_sessions_em_raiz_inexistente():
    assert schema.find_sessions("/caminho/que/nao/existe") == []


def test_yield_ratio():
    meta = schema.SessionMeta(
        session_id="x",
        frame_count=80,
        dropped_frames=5,
        skipped_stopped=10,
        skipped_stale=3,
        skipped_unsafe=2,
    )
    assert meta.yield_ratio == pytest.approx(0.8)


def test_yield_ratio_sessao_vazia_nao_divide_por_zero():
    assert schema.SessionMeta(session_id="x").yield_ratio == 0.0
