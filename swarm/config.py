"""Configuration for the swarm runtime.

Two settings groups:

* :class:`MqttSettings` — broker location for the operator command/telemetry
  channel, MQTT v5 [17].
* :class:`SimSettings` — Pegasus / Isaac Sim run-time parameters: number of
  drones, scene source, default cruise speed used to size segment durations
  in :func:`swarm.trajectory.build_point_to_point`.

References:
    [17] OASIS Standard, "MQTT Version 5.0", 2019.
    [22] Jacinto, M. *et al.* (2024). "Pegasus Simulator: An Isaac Sim
         Framework for Multiple Aerial Vehicles Simulation", *ICUAS 2024*.
"""
from __future__ import annotations

import os
from functools import lru_cache
from pathlib import Path

from pydantic import Field, field_validator
from pydantic_settings import BaseSettings, SettingsConfigDict


def _env_files() -> tuple[str, ...]:
    """Prefer ``$SWARM_IOT_ROOT/.env`` when launched from Isaac Sim."""
    files: list[str] = []
    root = os.environ.get("SWARM_IOT_ROOT")
    if root:
        candidate = Path(root) / ".env"
        if candidate.is_file():
            files.append(str(candidate))
    files.append(".env")
    return tuple(files)


class MqttSettings(BaseSettings):
    model_config = SettingsConfigDict(
        env_prefix="MQTT_",
        env_file=_env_files(),
        extra="ignore",
    )

    host: str = "localhost"
    port: int = 1883
    username: str | None = None
    password: str | None = None
    topic_prefix: str = "swarm"
    keepalive: int = 30


class SimSettings(BaseSettings):
    """Runtime knobs for the Isaac Sim + Pegasus deployment."""

    model_config = SettingsConfigDict(
        env_prefix="SIM_",
        env_file=_env_files(),
        extra="ignore",
    )

    #: Number of multirotors spawned by ``isaac/swarm_app.py``.
    num_drones: int = Field(3, ge=1, le=32)

    #: Cruise speed (m/s) used as the default segment-duration scale in
    #: :mod:`swarm.trajectory` when an operator command does not override it.
    #: Default kept conservative (0.5 m/s) so motions are easy to follow
    #: visually and stay well below the linear-tracking regime the gains in
    #: Pegasus' ``NonlinearController`` are tuned for.
    cruise_speed_mps: float = Field(0.5, gt=0.0)

    #: Default takeoff altitude (m). Set >= 3.0 to clear 2 m warehouse rack tops.
    takeoff_altitude_m: float = Field(3.0, gt=0.05)

    #: Default sweep spacing (m). Set equal to aisle width so one boustrophedon
    #: pass covers the full aisle — per Choset (2000) [4].
    scan_sweep_spacing_m: float = Field(2.5, gt=0.0)

    #: Battery drain rate (% per second of active flight).
    #: Default 0.05 → ~33 min to deplete at cruise (realistic for indoor UAV).
    #: Set SIM_BATTERY_DRAIN_RATE_PPS=2.0 for a demo that triggers relay in ~30 s.
    battery_drain_rate_pps: float = Field(0.05, ge=0.0)

    #: Battery % below which the relay mission triggers a handoff to a fresh drone.
    battery_relay_threshold_pct: float = Field(40.0, ge=0.0, le=100.0)

    #: Scene source loaded into Isaac Sim:
    #:   * ``warehouse`` — a Pegasus built-in environment via Nucleus.
    #:   * ``usd``       — a user-provided ``.usd`` file.
    scene_style: str = "warehouse"

    #: When ``scene_style == "warehouse"``: which key from
    #: ``pegasus.simulator.params.SIMULATION_ENVIRONMENTS`` to load.
    pegasus_env_name: str = "Full Warehouse"

    #: When ``scene_style == "usd"``: absolute path to a USD file.
    usd_scene_path: str | None = None

    @field_validator("usd_scene_path", mode="before")
    @classmethod
    def _empty_path_is_none(cls, v: object) -> str | None:
        if v is None or v == "":
            return None
        return str(v)

    @field_validator("scene_style", mode="before")
    @classmethod
    def _normalize_scene_style(cls, v: object) -> str:
        if isinstance(v, str):
            return v.strip().lower()
        return "warehouse"


class Settings(BaseSettings):
    mqtt: MqttSettings = Field(default_factory=MqttSettings)
    sim: SimSettings = Field(default_factory=SimSettings)


@lru_cache(maxsize=1)
def get_settings() -> Settings:
    return Settings()
