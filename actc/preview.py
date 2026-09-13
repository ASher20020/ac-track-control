from __future__ import annotations

import json
import os
import time
from pathlib import Path


class MpcPreviewPublisher:
    def __init__(
        self,
        path: Path,
        min_interval_s: float = 0.08,
    ) -> None:
        self.path = path
        self.min_interval_s = max(0.02, min_interval_s)
        self._last_publish = 0.0

    def publish(
        self,
        *,
        current: tuple[float, float],
        reference: list[tuple[float, float]],
        predicted: list[tuple[float, float]],
        indices: list[int],
        lateral_errors: list[float],
        heading_errors: list[float],
    ) -> None:
        now = time.monotonic()
        if now - self._last_publish < self.min_interval_s:
            return
        self._last_publish = now
        payload = {
            "timestamp": now,
            "current": [float(current[0]), float(current[1])],
            "reference": reference,
            "predicted": predicted,
            "indices": indices,
            "lateral_errors": lateral_errors,
            "heading_errors": heading_errors,
        }
        temporary = self.path.with_suffix(
            self.path.suffix + ".tmp"
        )
        try:
            temporary.write_text(
                json.dumps(payload, separators=(",", ":")),
                encoding="utf-8",
            )
            os.replace(temporary, self.path)
        except OSError:
            try:
                temporary.unlink(missing_ok=True)
            except OSError:
                pass
