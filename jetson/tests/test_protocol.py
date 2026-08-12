"""Testes do codec do protocolo.

Estes testes são o contrato entre o Python e o C++ do ESP32. Os exemplos
literais de quadro saem de ``docs/03-protocolo-serial.md`` — se um deles
quebrar, os dois lados saíram de sincronia.
"""

from __future__ import annotations

import pytest

from robocar.comms import protocol as p

# --- checksum --------------------------------------------------------------


def test_checksum_conhecido():
    # Valor de referência usado também no firmware.
    assert p.checksum("CMD,142,-0.3125,0.2000,2") == "65"


def test_checksum_de_payload_vazio_eh_zero():
    assert p.checksum("") == "00"


def test_checksum_sempre_dois_digitos_maiusculos():
    for payload in ("A", "PING,1", "TLM,0,0,0.0000,0.0000,0,0,0,0,0.00,00"):
        cs = p.checksum(payload)
        assert len(cs) == 2
        assert cs == cs.upper()


# --- codificação -----------------------------------------------------------


def test_encode_command_bate_com_a_especificacao():
    frame = p.encode_command(142, -0.3125, 0.2, p.Mode.AUTO)
    assert frame == "$CMD,142,-0.3125,0.2000,2*65\n"


def test_encode_command_limita_valores_fora_de_faixa():
    # Um bug no controle não pode virar comando absurdo na serial.
    frame = p.encode_command(1, -5.0, 9.0, p.Mode.TELEOP)
    assert ",-1.0000,1.0000," in frame


def test_seq_faz_wrap_em_16_bits():
    frame = p.encode_command(65536 + 7, 0.0, 0.0)
    assert frame.startswith("$CMD,7,")


def test_encode_config_formata_por_tipo():
    assert p.encode_config("rc_enable", True).startswith("$CFG,rc_enable,1*")
    assert p.encode_config("timeout_ms", 250).startswith("$CFG,timeout_ms,250*")
    assert p.encode_config("slew_thr", 2.0).startswith("$CFG,slew_thr,2.0000*")


def test_quadro_grande_demais_levanta():
    with pytest.raises(p.ProtocolError) as exc:
        p.encode_frame("LOG,I," + "x" * 200)
    assert exc.value.code is p.ErrorCode.FRAME_TOO_LONG


# --- decodificação ---------------------------------------------------------


def test_parse_telemetria_da_especificacao():
    line = "$TLM,142,84213,-0.3000,0.2000,1,1230,890,1540,7.92,01*61"
    tlm = p.parse(line)
    assert isinstance(tlm, p.Telemetry)
    assert tlm.seq == 142
    assert tlm.t_ms == 84213
    assert tlm.steer == pytest.approx(-0.3)
    assert tlm.throttle == pytest.approx(0.2)
    assert tlm.source is p.Source.SERIAL
    assert tlm.distances_mm == (1230, 890, 1540)
    assert tlm.vbat == pytest.approx(7.92)
    assert tlm.flags is p.Flags.ARMED
    assert tlm.armed and not tlm.failsafe


def test_telemetria_aceita_numero_variavel_de_sensores():
    for count in (1, 3, 5):
        tlm = p.Telemetry(
            seq=1,
            t_ms=10,
            steer=0.0,
            throttle=0.0,
            source=p.Source.SERIAL,
            distances_mm=tuple(range(100, 100 + count)),
            vbat=7.4,
            flags=p.Flags.NONE,
        )
        assert p.parse(tlm.encode()).distances_mm == tlm.distances_mm


def test_distancia_zero_vira_none():
    tlm = p.Telemetry(
        seq=1,
        t_ms=1,
        steer=0.0,
        throttle=0.0,
        source=p.Source.SERIAL,
        distances_mm=(0, 500, 0),
        vbat=7.4,
        flags=p.Flags.NONE,
    )
    # 0 significa "sem eco", não "obstáculo colado no sensor".
    assert tlm.distance(0) is None
    assert tlm.distance(1) == 500
    assert tlm.distance(9) is None


@pytest.mark.parametrize(
    "flags,esperado",
    [
        (p.Flags.ARMED, True),
        (p.Flags.ARMED | p.Flags.RC_VALID, True),
        (p.Flags.FAILSAFE, False),
        (p.Flags.OBSTACLE_STOP, False),
        (p.Flags.BATT_CRITICAL, False),
        (p.Flags.OBSTACLE_SLOW, True),
        (p.Flags.BATT_LOW, True),
    ],
)
def test_usable_for_training_reflete_as_flags(flags, esperado):
    tlm = p.Telemetry(
        seq=0,
        t_ms=0,
        steer=0.0,
        throttle=0.5,
        source=p.Source.RC,
        distances_mm=(0,),
        vbat=7.4,
        flags=flags,
    )
    assert tlm.usable_for_training is esperado


def test_checksum_invalido_levanta():
    with pytest.raises(p.ProtocolError) as exc:
        p.parse("$PING,1*FF")
    assert exc.value.code is p.ErrorCode.BAD_CHECKSUM


def test_tipo_desconhecido_levanta():
    with pytest.raises(p.ProtocolError) as exc:
        p.parse(p.encode_frame("XYZ,1"))
    assert exc.value.code is p.ErrorCode.UNKNOWN_TYPE


def test_numero_errado_de_campos_levanta():
    with pytest.raises(p.ProtocolError) as exc:
        p.parse(p.encode_frame("CMD,1,0.0"))
    assert exc.value.code is p.ErrorCode.BAD_FIELD_COUNT


def test_lixo_antes_do_cifrao_eh_ignorado():
    # O ESP32 imprime mensagens de boot antes do firmware assumir a serial.
    line = "rst:0x1 (POWERON_RESET),boot:0x13$PING,7*" + p.checksum("PING,7")
    assert p.parse(line) == p.Ping(seq=7)


@pytest.mark.parametrize(
    "message",
    [
        p.Command(seq=1, steer=0.5, throttle=-0.25, mode=p.Mode.TELEOP),
        p.Arm(armed=True),
        p.Arm(armed=False),
        p.Config(key="timeout_ms", value="250"),
        p.Ping(seq=9),
        p.Pong(seq=9, t_ms=1234),
        p.Ack(kind="CFG", detail="timeout_ms"),
        p.Err(code=4, detail="steer"),
        p.LogMessage(level="W", text="bateria fraca"),
    ],
)
def test_ida_e_volta(message):
    assert p.parse(message.encode()) == message


# --- parser incremental ----------------------------------------------------


def test_parser_junta_pedacos():
    parser = p.FrameParser()
    frame = p.encode_command(1, 0.5, 0.5).encode("ascii")
    meio = len(frame) // 2
    assert parser.feed(frame[:meio]) == []
    mensagens = parser.feed(frame[meio:])
    assert len(mensagens) == 1
    assert isinstance(mensagens[0], p.Command)


def test_parser_devolve_varias_mensagens_de_uma_vez():
    parser = p.FrameParser()
    data = b"".join(p.encode_command(i, 0.0, 0.0).encode("ascii") for i in range(5))
    assert len(parser.feed(data)) == 5
    assert parser.stats.frames_ok == 5


def test_parser_nao_quebra_com_linha_corrompida():
    parser = p.FrameParser()
    bom = p.encode_command(1, 0.0, 0.0).encode("ascii")
    ruim = b"$CMD,2,0.0000,0.0000,2*00\n"  # checksum errado
    mensagens = parser.feed(ruim + bom)
    # A linha boa passa; a ruim vira contador, não exceção.
    assert len(mensagens) == 1
    assert parser.stats.checksum_errors == 1
    assert parser.stats.frames_ok == 1


def test_parser_ignora_bytes_nao_ascii():
    parser = p.FrameParser()
    assert parser.feed(b"\xff\xfe ruido binario\n") == []
    assert parser.stats.malformed == 1


def test_parser_nao_cresce_sem_limite_sem_terminador():
    parser = p.FrameParser()
    for _ in range(100):
        parser.feed(b"x" * 64)
    assert parser.stats.dropped_bytes > 0
    # Continua funcionando depois de descartar o lixo.
    parser.feed(b"\n")
    assert len(parser.feed(p.encode_ping(1).encode("ascii"))) == 1


def test_parser_aceita_crlf():
    parser = p.FrameParser()
    frame = p.encode_ping(3).replace("\n", "\r\n").encode("ascii")
    assert len(parser.feed(frame)) == 1


# --- handshake -------------------------------------------------------------


def _vehicle_config():
    return {
        "steering": {
            "center_us": 1500,
            "min_us": 1150,
            "max_us": 1850,
            "invert": False,
            "slew_rate_per_s": 6.0,
        },
        "throttle": {
            "neutral_us": 1500,
            "min_us": 1000,
            "max_us": 2000,
            "deadband": 0.05,
            "limit_reverse": 0.2,
            "slew_rate_per_s": 2.0,
        },
        "safety": {
            "command_timeout_ms": 250,
            "obstacle_stop_mm": 250,
            "obstacle_slow_mm": 600,
            "telemetry_hz": 50,
        },
    }


def test_build_config_frames_gera_quadros_validos():
    frames = p.build_config_frames(
        _vehicle_config(), throttle_limit=0.3, rc_enable=True
    )
    chaves = [key for key, _ in frames]
    assert "thr_limit_fwd" in chaves
    assert "rc_enable" in chaves
    for key, frame in frames:
        mensagem = p.parse(frame)
        assert isinstance(mensagem, p.Config)
        assert mensagem.key == key


def test_build_config_frames_reclama_de_chave_ausente():
    incompleto = _vehicle_config()
    del incompleto["safety"]["command_timeout_ms"]
    with pytest.raises(KeyError):
        p.build_config_frames(incompleto, throttle_limit=0.3)


def test_rc_desabilitado_por_padrao():
    # Em prova, rádio ligado é desclassificação. O padrão precisa ser seguro.
    frames = dict(p.build_config_frames(_vehicle_config(), throttle_limit=0.3))
    assert p.parse(frames["rc_enable"]).value == "0"
