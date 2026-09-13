from __future__ import annotations

import ctypes
import mmap
import os
import time
from dataclasses import dataclass
from typing import Iterable


PHYSICS_MAP = r"Local\acpmf_physics"
GRAPHICS_MAP = r"Local\acpmf_graphics"
STATIC_MAP = r"Local\acpmf_static"


class SPageFilePhysics(ctypes.Structure):
    _pack_ = 4
    _fields_ = [
        ("packetId", ctypes.c_int),
        ("gas", ctypes.c_float),
        ("brake", ctypes.c_float),
        ("fuel", ctypes.c_float),
        ("gear", ctypes.c_int),
        ("rpms", ctypes.c_int),
        ("steerAngle", ctypes.c_float),
        ("speedKmh", ctypes.c_float),
        ("velocity", ctypes.c_float * 3),
        ("accG", ctypes.c_float * 3),
        ("wheelSlip", ctypes.c_float * 4),
        ("wheelLoad", ctypes.c_float * 4),
        ("wheelsPressure", ctypes.c_float * 4),
        ("wheelAngularSpeed", ctypes.c_float * 4),
        ("tyreWear", ctypes.c_float * 4),
        ("tyreDirtyLevel", ctypes.c_float * 4),
        ("tyreCoreTemperature", ctypes.c_float * 4),
        ("camberRAD", ctypes.c_float * 4),
        ("suspensionTravel", ctypes.c_float * 4),
        ("drs", ctypes.c_float),
        ("tc", ctypes.c_float),
        ("heading", ctypes.c_float),
        ("pitch", ctypes.c_float),
        ("roll", ctypes.c_float),
        ("cgHeight", ctypes.c_float),
        ("carDamage", ctypes.c_float * 5),
        ("numberOfTyresOut", ctypes.c_int),
        ("pitLimiterOn", ctypes.c_int),
        ("abs", ctypes.c_float),
        ("kersCharge", ctypes.c_float),
        ("kersInput", ctypes.c_float),
        ("autoShifterOn", ctypes.c_int),
        ("rideHeight", ctypes.c_float * 2),
        ("turboBoost", ctypes.c_float),
        ("ballast", ctypes.c_float),
        ("airDensity", ctypes.c_float),
        ("airTemp", ctypes.c_float),
        ("roadTemp", ctypes.c_float),
        ("localAngularVel", ctypes.c_float * 3),
        ("finalFF", ctypes.c_float),
        ("performanceMeter", ctypes.c_float),
        ("engineBrake", ctypes.c_int),
        ("ersRecoveryLevel", ctypes.c_int),
        ("ersPowerLevel", ctypes.c_int),
        ("ersHeatCharging", ctypes.c_int),
        ("ersIsCharging", ctypes.c_int),
        ("kersCurrentKJ", ctypes.c_float),
        ("drsAvailable", ctypes.c_int),
        ("waterTemp", ctypes.c_float),
        ("brakeTemp", ctypes.c_float * 4),
        ("clutch", ctypes.c_float),
        ("tyreTempI", ctypes.c_float * 4),
        ("tyreTempM", ctypes.c_float * 4),
        ("tyreTempO", ctypes.c_float * 4),
        ("isAIControlled", ctypes.c_int),
        ("tyreContactPoint", (ctypes.c_float * 3) * 4),
        ("tyreContactNormal", (ctypes.c_float * 3) * 4),
        ("tyreContactHeading", (ctypes.c_float * 3) * 4),
        ("brakeBias", ctypes.c_float),
        ("localVelocity", ctypes.c_float * 3),
    ]


class SPageFileGraphic(ctypes.Structure):
    _pack_ = 4
    _fields_ = [
        ("packetId", ctypes.c_int),
        ("status", ctypes.c_int),
        ("session", ctypes.c_int),
        ("currentTime", ctypes.c_wchar * 15),
        ("lastTime", ctypes.c_wchar * 15),
        ("bestTime", ctypes.c_wchar * 15),
        ("split", ctypes.c_wchar * 15),
        ("completedLaps", ctypes.c_int),
        ("position", ctypes.c_int),
        ("iCurrentTime", ctypes.c_int),
        ("iLastTime", ctypes.c_int),
        ("iBestTime", ctypes.c_int),
        ("sessionTimeLeft", ctypes.c_float),
        ("distanceTraveled", ctypes.c_float),
        ("isInPit", ctypes.c_int),
        ("currentSectorIndex", ctypes.c_int),
        ("lastSectorTime", ctypes.c_int),
        ("numberOfLaps", ctypes.c_int),
        ("tyreCompound", ctypes.c_wchar * 33),
        ("replayTimeMultiplier", ctypes.c_float),
        ("normalizedCarPosition", ctypes.c_float),
        ("carCoordinates", ctypes.c_float * 3),
        ("penaltyTime", ctypes.c_float),
        ("flag", ctypes.c_int),
        ("idealLineOn", ctypes.c_int),
        ("isInPitLane", ctypes.c_int),
        ("surfaceGrip", ctypes.c_float),
        ("mandatoryPitDone", ctypes.c_int),
        ("windSpeed", ctypes.c_float),
        ("windDirection", ctypes.c_float),
    ]


class SPageFileStatic(ctypes.Structure):
    _pack_ = 4
    _fields_ = [
        ("smVersion", ctypes.c_wchar * 15),
        ("acVersion", ctypes.c_wchar * 15),
        ("numberOfSessions", ctypes.c_int),
        ("numCars", ctypes.c_int),
        ("carModel", ctypes.c_wchar * 33),
        ("track", ctypes.c_wchar * 33),
        ("playerName", ctypes.c_wchar * 33),
        ("playerSurname", ctypes.c_wchar * 33),
        ("playerNick", ctypes.c_wchar * 33),
        ("sectorCount", ctypes.c_int),
        ("maxTorque", ctypes.c_float),
        ("maxPower", ctypes.c_float),
        ("maxRpm", ctypes.c_int),
        ("maxFuel", ctypes.c_float),
        ("suspensionMaxTravel", ctypes.c_float * 4),
        ("tyreRadius", ctypes.c_float * 4),
        ("trackConfiguration", ctypes.c_wchar * 33),
    ]


@dataclass(frozen=True)
class VehicleState:
    timestamp: float
    packet_id: int
    status: int
    speed_kmh: float
    speed_ms: float
    gas: float
    brake: float
    steer: float
    gear: int
    rpm: float
    auto_shifter_on: bool
    heading_rad: float
    yaw_rate_rad_s: float
    local_angular_velocity_rad_s: tuple[float, float, float]
    velocity_m_s: tuple[float, float, float]
    local_velocity_m_s: tuple[float, float, float]
    acceleration_g: tuple[float, float, float]
    wheel_angular_speed_rad_s: tuple[float, float, float, float]
    wheel_load_n: tuple[float, float, float, float]
    tyre_slip: tuple[float, float, float, float]
    tyres_out: int
    is_ai_controlled: bool
    position: tuple[float, float, float]
    normalized_position: float
    completed_laps: int
    is_in_pit: bool
    is_in_pit_lane: bool
    surface_grip: float
    car_model: str
    track: str
    track_configuration: str


class SharedMemoryUnavailable(RuntimeError):
    pass


class SharedMemoryReader:
    """Read the standard Assetto Corsa shared-memory telemetry pages."""

    def __init__(self) -> None:
        self._physics_map: mmap.mmap | None = None
        self._graphics_map: mmap.mmap | None = None
        self._static_map: mmap.mmap | None = None
        self._physics: SPageFilePhysics | None = None
        self._graphics: SPageFileGraphic | None = None
        self._static: SPageFileStatic | None = None

    @staticmethod
    def _map_names(name: str) -> Iterable[str]:
        yield name
        if name.startswith("Local\\"):
            yield name.split("\\", 1)[1]

    def _open_named(self, name: str, structure: type[ctypes.Structure]) -> mmap.mmap:
        last_error: OSError | None = None
        for candidate in self._map_names(name):
            try:
                return mmap.mmap(-1, ctypes.sizeof(structure), tagname=candidate)
            except OSError as exc:
                last_error = exc
        raise SharedMemoryUnavailable(f"Cannot open shared memory {name}") from last_error

    def connect(
        self,
        wait_seconds: float = 0.0,
        poll_interval: float = 0.25,
        require_live: bool = True,
        live_probe_seconds: float = 0.35,
    ) -> None:
        if os.name != "nt":
            raise SharedMemoryUnavailable("Assetto Corsa shared memory is Windows-only")

        deadline = time.monotonic() + max(0.0, wait_seconds)
        last_error: Exception | None = None
        while True:
            try:
                self.close()
                self._physics_map = self._open_named(PHYSICS_MAP, SPageFilePhysics)
                self._graphics_map = self._open_named(GRAPHICS_MAP, SPageFileGraphic)
                self._static_map = self._open_named(STATIC_MAP, SPageFileStatic)
                self._physics = SPageFilePhysics.from_buffer(self._physics_map)
                self._graphics = SPageFileGraphic.from_buffer(self._graphics_map)
                self._static = SPageFileStatic.from_buffer(self._static_map)
                if require_live and not self._wait_for_live_probe(live_probe_seconds):
                    raise SharedMemoryUnavailable(
                        "Assetto Corsa shared memory is stale; waiting for a live session."
                    )
                return
            except (OSError, SharedMemoryUnavailable) as exc:
                last_error = exc
                self.close()
                if time.monotonic() >= deadline:
                    raise SharedMemoryUnavailable(
                        "Assetto Corsa telemetry is not available. "
                        "Start a session from Content Manager and try again."
                    ) from last_error
                time.sleep(poll_interval)

    def _wait_for_live_probe(self, probe_seconds: float) -> bool:
        if self._physics is None:
            return False
        deadline = time.monotonic() + max(0.05, probe_seconds)
        initial_packet_id = int(self._physics.packetId)
        while time.monotonic() < deadline:
            if int(self._physics.packetId) != initial_packet_id:
                return True
            time.sleep(0.01)
        return False

    def close(self) -> None:
        self._physics = None
        self._graphics = None
        self._static = None
        for mapping_name in ("_physics_map", "_graphics_map", "_static_map"):
            mapping = getattr(self, mapping_name)
            if mapping is not None:
                try:
                    mapping.close()
                except BufferError:
                    pass
                setattr(self, mapping_name, None)

    def __enter__(self) -> "SharedMemoryReader":
        self.connect()
        return self

    def __exit__(self, *_args: object) -> None:
        self.close()

    def snapshot(self) -> VehicleState:
        if self._physics is None or self._graphics is None or self._static is None:
            raise SharedMemoryUnavailable("Shared memory is not connected")

        physics = self._physics
        graphics = self._graphics
        static = self._static
        speed_ms = float(physics.speedKmh) / 3.6

        return VehicleState(
            timestamp=time.perf_counter(),
            packet_id=int(physics.packetId),
            status=int(graphics.status),
            speed_kmh=float(physics.speedKmh),
            speed_ms=speed_ms,
            gas=float(physics.gas),
            brake=float(physics.brake),
            steer=float(physics.steerAngle),
            gear=int(physics.gear),
            rpm=float(physics.rpms),
            auto_shifter_on=bool(physics.autoShifterOn),
            heading_rad=float(physics.heading),
            yaw_rate_rad_s=float(physics.localAngularVel[1]),
            local_angular_velocity_rad_s=tuple(
                float(v) for v in physics.localAngularVel
            ),
            velocity_m_s=tuple(float(v) for v in physics.velocity),
            local_velocity_m_s=tuple(float(v) for v in physics.localVelocity),
            acceleration_g=tuple(float(v) for v in physics.accG),
            wheel_angular_speed_rad_s=tuple(float(v) for v in physics.wheelAngularSpeed),
            wheel_load_n=tuple(float(v) for v in physics.wheelLoad),
            tyre_slip=tuple(float(v) for v in physics.wheelSlip),
            tyres_out=int(physics.numberOfTyresOut),
            is_ai_controlled=bool(physics.isAIControlled),
            position=tuple(float(v) for v in graphics.carCoordinates),
            normalized_position=float(graphics.normalizedCarPosition),
            completed_laps=int(graphics.completedLaps),
            is_in_pit=bool(graphics.isInPit),
            is_in_pit_lane=bool(graphics.isInPitLane),
            surface_grip=float(graphics.surfaceGrip),
            car_model=str(static.carModel).rstrip("\x00"),
            track=str(static.track).rstrip("\x00"),
            track_configuration=str(static.trackConfiguration).rstrip("\x00"),
        )
