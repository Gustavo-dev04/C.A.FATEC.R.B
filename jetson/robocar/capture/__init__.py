"""Coleta de dados: esquema do dataset, sessões e gravação."""

from .recorder import SessionRecorder
from .schema import Record, SessionMeta, find_sessions, iter_records, read_session_meta
from .session import create_session

__all__ = [
    "Record",
    "SessionMeta",
    "SessionRecorder",
    "create_session",
    "find_sessions",
    "iter_records",
    "read_session_meta",
]
