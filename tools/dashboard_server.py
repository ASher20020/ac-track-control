from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from actc.mpc import vehicle_state_to_frenet  # noqa: E402
from actc.shared_memory import (  # noqa: E402
    SharedMemoryReader,
    SharedMemoryUnavailable,
    VehicleState,
)
from actc.track import (  # noqa: E402
    TrackPath,
    physics_heading_to_world,
    resolve_fast_lane,
)


class TelemetryStore:
    def __init__(self, history_s: float, sample_hz: float) -> None:
        self.capacity = max(100, int(history_s * sample_hz))
        self.samples: deque[dict[str, float | int | str]] = deque(
            maxlen=self.capacity
        )
        self.lock = threading.Lock()
        self.lap_times: list[float] = []
        self.lap_started_at: float | None = None
        self.previous_normalized = 0.0
        self.connected = False
        self.error: str | None = None
        self.car = ""
        self.track = ""
        self.sequence = 0
        self.track_points: list[list[float]] = []
        self.finish_point: list[float] = [0.0, 0.0]

    def set_track(self, path: TrackPath | None) -> None:
        with self.lock:
            if path is None:
                self.track_points = []
                self.finish_point = [0.0, 0.0]
                return
            step = max(1, len(path.points) // 1800)
            self.track_points = [
                [float(x), float(z)]
                for x, z in path.points[::step]
            ]
            self.finish_point = [
                float(path.points[0][0]),
                float(path.points[0][1]),
            ]

    def add(
        self,
        state: VehicleState,
        path: TrackPath | None,
    ) -> None:
        now = time.monotonic()
        lateral_error = 0.0
        heading_error = 0.0
        path_index = 0
        if path is not None:
            path_index, frenet = vehicle_state_to_frenet(
                state,
                path,
                None,
            )
            lateral_error = float(frenet[0])
            heading_error = float(frenet[1])

        vx = float(state.local_velocity_m_s[2])
        vy = -float(state.local_velocity_m_s[0])
        body_slip_deg = math.degrees(
            math.atan2(vy, max(abs(vx), 0.5))
        )
        heading_error_deg = math.degrees(heading_error)
        yaw_rate_deg_s = math.degrees(-float(state.yaw_rate_rad_s))
        steer_deg = math.degrees(float(state.steer))
        lateral_g = -float(state.acceleration_g[0])
        longitudinal_g = float(state.acceleration_g[2])
        normalized = float(state.normalized_position)

        with self.lock:
            if (
                self.previous_normalized > 0.90
                and normalized < 0.10
            ):
                if self.lap_started_at is not None:
                    self.lap_times.append(now - self.lap_started_at)
                    self.lap_times = self.lap_times[-20:]
                self.lap_started_at = now
            self.previous_normalized = normalized
            self.connected = True
            self.error = None
            self.car = state.car_model
            self.track = state.track
            self.sequence += 1
            self.samples.append(
                {
                    "seq": self.sequence,
                    "time": now,
                    "speed": float(state.speed_kmh),
                    "lateral_error": lateral_error,
                    "heading_error": heading_error,
                    "heading_error_deg": heading_error_deg,
                    "yaw_rate": -float(state.yaw_rate_rad_s),
                    "yaw_rate_deg_s": yaw_rate_deg_s,
                    "body_slip": body_slip_deg,
                    "steer": float(state.steer),
                    "steer_deg": steer_deg,
                    "g_lat": lateral_g,
                    "g_long": longitudinal_g,
                    "position_x": float(state.position[0]),
                    "position_z": float(state.position[2]),
                    "heading_world": physics_heading_to_world(
                        float(state.heading_rad)
                    ),
                    "normalized_position": normalized,
                    "path_index": path_index,
                    "tyres_out": int(state.tyres_out),
                }
            )

    def snapshot(self) -> dict:
        with self.lock:
            now = time.monotonic()
            current_lap_s = (
                now - self.lap_started_at
                if self.lap_started_at is not None
                else 0.0
            )
            return {
                "connected": self.connected,
                "error": self.error,
                "car": self.car,
                "track": self.track,
                "now": now,
                "current_lap_s": current_lap_s,
                "lap_times_s": list(self.lap_times),
                "samples": list(self.samples),
            }

    def set_error(self, message: str) -> None:
        with self.lock:
            self.connected = False
            self.error = message

    def track_snapshot(self) -> dict:
        with self.lock:
            return {
                "points": list(self.track_points),
                "finish": list(self.finish_point),
            }


class DashboardHandler(BaseHTTPRequestHandler):
    store: TelemetryStore
    html_path: Path

    def do_GET(self) -> None:
        if self.path.startswith("/api/telemetry"):
            self._send_json(self.store.snapshot())
            return
        if self.path.startswith("/api/track"):
            self._send_json(self.store.track_snapshot())
            return
        if self.path.startswith("/api/preview"):
            preview_path = (
                PROJECT_ROOT / "logs" / "mpc_preview.json"
            )
            try:
                payload = json.loads(
                    preview_path.read_text(encoding="utf-8")
                )
            except (OSError, json.JSONDecodeError):
                payload = {}
            self._send_json(payload)
            return
        if self.path in ("/", "/index.html"):
            payload = self.html_path.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)
            return
        self.send_error(404)

    def _send_json(self, payload: dict) -> None:
        encoded = json.dumps(
            payload,
            separators=(",", ":"),
        ).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(encoded)))
        self.end_headers()
        self.wfile.write(encoded)

    def log_message(self, _format: str, *_args) -> None:
        return


def sampler(
    store: TelemetryStore,
    sample_hz: float,
    stop_event: threading.Event,
) -> None:
    reader = SharedMemoryReader()
    path: TrackPath | None = None
    path_track = ""
    last_connect_attempt = 0.0
    connected_now = False
    period = 1.0 / max(1.0, sample_hz)
    while not stop_event.is_set():
        try:
            if not connected_now:
                now = time.monotonic()
                if now - last_connect_attempt >= 1.0:
                    last_connect_attempt = now
                    reader.connect(wait_seconds=0.2)
                    connected_now = True
            if not connected_now:
                stop_event.wait(period)
                continue
            state = reader.snapshot()
            track_key = f"{state.track}:{state.track_configuration}"
            if track_key != path_track:
                try:
                    fast_lane = resolve_fast_lane(
                        state.track,
                        state.track_configuration,
                        None,
                    )
                    path = TrackPath.load(
                        fast_lane,
                        centerline=True,
                        max_speed_kmh=180.0,
                        min_speed_kmh=25.0,
                        lateral_accel_mps2=3.92,
                        accel_mps2=3.0,
                        brake_mps2=4.5,
                    )
                    path_track = track_key
                    store.set_track(path)
                except Exception:
                    path = None
                    path_track = ""
                    store.set_track(None)
            store.add(state, path)
        except SharedMemoryUnavailable as exc:
            connected_now = False
            path = None
            path_track = ""
            store.set_error(str(exc))
            reader.close()
        except Exception as exc:
            store.set_error(str(exc))
        stop_event.wait(period)
    reader.close()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--history", type=float, default=30.0)
    parser.add_argument("--hz", type=float, default=20.0)
    args = parser.parse_args()

    store = TelemetryStore(args.history, args.hz)
    stop_event = threading.Event()
    worker = threading.Thread(
        target=sampler,
        args=(store, args.hz, stop_event),
        daemon=True,
    )
    worker.start()

    DashboardHandler.store = store
    DashboardHandler.html_path = (
        PROJECT_ROOT / "tools" / "dashboard.html"
    )
    server = ThreadingHTTPServer(
        (args.host, args.port),
        DashboardHandler,
    )
    print(
        f"Dashboard: http://{args.host}:{args.port}",
        flush=True,
    )
    try:
        server.serve_forever(poll_interval=0.25)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
