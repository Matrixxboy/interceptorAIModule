"""Categorized system logger with live streaming support."""

from __future__ import annotations

import json
import logging
import time
from collections import deque
from dataclasses import asdict, dataclass, field
from enum import Enum
from pathlib import Path
from typing import Any, Callable

LOGS_DIR = Path(__file__).resolve().parent.parent / "logs"
LOGS_DIR.mkdir(exist_ok=True, parents=True)


class LogSeverity(str, Enum):
    DEBUG = "DEBUG"
    INFO = "INFO"
    WARNING = "WARNING"
    ERROR = "ERROR"
    CRITICAL = "CRITICAL"


class LogCategory(str, Enum):
    AI_DETECTION = "AI Detection"
    TRACKING = "Tracking"
    CAMERA = "Camera"
    DRONE = "Drone Communication"
    TELEMETRY = "Telemetry"
    FLIGHT = "Flight Controller"
    NAVIGATION = "Navigation"
    PERFORMANCE = "Performance"
    SYSTEM = "System Events"
    DEBUG = "Debug"


@dataclass
class LogEntry:
    timestamp: float
    module: str
    severity: LogSeverity
    category: LogCategory
    message: str
    target_id: str = ""
    fps: float = 0.0
    latency_ms: float = 0.0
    memory_mb: float = 0.0
    extra: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        d = asdict(self)
        d["severity"] = self.severity.value
        d["category"] = self.category.value
        return d

    @property
    def time_str(self) -> str:
        from datetime import datetime

        return datetime.fromtimestamp(self.timestamp).strftime("%H:%M:%S.%f")[:-3]


class SystemLogger:
    """Thread-safe categorized logger with buffer and live subscribers."""

    _instance: SystemLogger | None = None

    def __new__(cls) -> SystemLogger:
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self, max_buffer: int = 10000) -> None:
        if self._initialized:
            return
        self._initialized = True
        self.max_buffer = max_buffer
        self.buffer: deque[LogEntry] = deque(maxlen=max_buffer)
        self._subscribers: list[Callable[[LogEntry], None]] = []
        self._py_logger = logging.getLogger("arjuna")
        if not self._py_logger.handlers:
            handler = logging.StreamHandler()
            handler.setFormatter(logging.Formatter("%(asctime)s | %(levelname)s | %(message)s"))
            self._py_logger.addHandler(handler)
            self._py_logger.setLevel(logging.DEBUG)

    def subscribe(self, callback: Callable[[LogEntry], None]) -> None:
        self._subscribers.append(callback)

    def unsubscribe(self, callback: Callable[[LogEntry], None]) -> None:
        if callback in self._subscribers:
            self._subscribers.remove(callback)

    def log(
        self,
        category: LogCategory,
        message: str,
        severity: LogSeverity = LogSeverity.INFO,
        module: str = "Arjuna",
        target_id: str = "",
        fps: float = 0.0,
        latency_ms: float = 0.0,
        memory_mb: float = 0.0,
        **extra: Any,
    ) -> LogEntry:
        entry = LogEntry(
            timestamp=time.time(),
            module=module,
            severity=severity,
            category=category,
            message=message,
            target_id=target_id,
            fps=fps,
            latency_ms=latency_ms,
            memory_mb=memory_mb,
            extra=extra,
        )
        self.buffer.appendleft(entry)

        py_level = getattr(logging, severity.value, logging.INFO)
        self._py_logger.log(py_level, f"[{category.value}] {message}")

        for cb in self._subscribers:
            try:
                cb(entry)
            except Exception:
                pass

        return entry

    def get_entries(
        self,
        category: LogCategory | None = None,
        severity: LogSeverity | None = None,
        search: str = "",
        limit: int = 500,
    ) -> list[LogEntry]:
        results = list(self.buffer)
        if category:
            results = [e for e in results if e.category == category]
        if severity:
            results = [e for e in results if e.severity == severity]
        if search:
            q = search.lower()
            results = [
                e for e in results
                if q in e.message.lower() or q in e.target_id.lower() or q in e.module.lower()
            ]
        return results[:limit]

    def export_json(self, filepath: Path | None = None) -> Path:
        filepath = filepath or (LOGS_DIR / f"system_log_{int(time.time())}.json")
        data = [e.to_dict() for e in reversed(list(self.buffer))]
        with open(filepath, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)
        return filepath

    def export_csv(self, filepath: Path | None = None) -> Path:
        import csv

        filepath = filepath or (LOGS_DIR / f"system_log_{int(time.time())}.csv")
        fields = ["timestamp", "module", "severity", "category", "message", "target_id", "fps", "latency_ms", "memory_mb"]
        with open(filepath, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
            writer.writeheader()
            for e in reversed(list(self.buffer)):
                writer.writerow(e.to_dict())
        return filepath

    def clear(self) -> None:
        self.buffer.clear()
