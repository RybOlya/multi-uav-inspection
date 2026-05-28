"""MQTT command schema — operator → drone.

Commands (``cmd`` field):
  takeoff      climb to altitude_m
  goto         fly to (x, y, z)
  orbit        circle around (center_x, center_y) at radius_m
  inspect_poi  goto named POI + hover dwell + orbit
  scan         Boustrophedon sweep of a rectangle
  hover/stop   hold current pose
  land         descend to floor
"""
from __future__ import annotations

import json
from typing import Annotated, Literal, Union

from pydantic import BaseModel, ConfigDict, Field, model_validator


class _Cmd(BaseModel):
    model_config = ConfigDict(extra="forbid")

    seq: int | None = Field(
        default=None,
        description="Optional monotonic sequence id from the operator; echoed in state messages.",
    )


class TakeoffCmd(_Cmd):
    cmd: Literal["takeoff"]
    altitude_m: float = Field(2.5, gt=0.05)
    cruise_speed_mps: float | None = Field(default=None, gt=0.0)


class GoToCmd(_Cmd):
    cmd: Literal["goto"]
    x: float
    y: float
    z: float = Field(..., gt=0.0)
    yaw: float = 0.0
    cruise_speed_mps: float | None = Field(default=None, gt=0.0)


class ScanCmd(_Cmd):
    """Boustrophedon coverage of an axis-aligned rectangle [4]."""

    cmd: Literal["scan"]
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    altitude_m: float = Field(..., gt=0.05)
    sweep_spacing_m: float = Field(2.0, gt=0.0)
    sweep_axis: Literal["x", "y"] = "x"
    cruise_speed_mps: float | None = Field(default=None, gt=0.0)

    @model_validator(mode="after")
    def _check_bounds(self) -> "ScanCmd":
        if self.x_max <= self.x_min:
            raise ValueError(f"x_max ({self.x_max}) must be > x_min ({self.x_min})")
        if self.y_max <= self.y_min:
            raise ValueError(f"y_max ({self.y_max}) must be > y_min ({self.y_min})")
        return self


class OrbitCmd(_Cmd):
    """Circle around a centre point at a fixed altitude."""

    cmd: Literal["orbit"]
    center_x: float
    center_y: float
    altitude_m: float = Field(..., gt=0.05)
    radius_m: float = Field(2.0, gt=0.0)
    speed_mps: float = Field(0.5, gt=0.0)
    revolutions: float = Field(1.0, gt=0.0)


class InspectPoiCmd(_Cmd):
    """Go to named POI, hover briefly, then orbit once."""

    cmd: Literal["inspect_poi"]
    poi_id: str
    dwell_s: float = Field(5.0, ge=0.0)


class HoverCmd(_Cmd):
    cmd: Literal["hover", "stop"]


class LandCmd(_Cmd):
    cmd: Literal["land"]
    cruise_speed_mps: float | None = Field(default=None, gt=0.0)


Command = Annotated[
    Union[TakeoffCmd, GoToCmd, OrbitCmd, InspectPoiCmd, ScanCmd, HoverCmd, LandCmd],
    Field(discriminator="cmd"),
]


class CommandEnvelope(BaseModel):
    """Top-level wrapper used as the MQTT payload."""

    model_config = ConfigDict(extra="forbid")
    body: Command


def parse_command(payload: bytes | str) -> Command:
    """Parse the JSON wire format into a typed command.

    Accepts either ``{"cmd": "...", ...}`` directly or ``{"body": {...}}``.
    """
    if isinstance(payload, bytes):
        payload = payload.decode("utf-8")
    raw = json.loads(payload)
    if isinstance(raw, dict) and "body" in raw and "cmd" not in raw:
        env = CommandEnvelope.model_validate(raw)
        return env.body
    if isinstance(raw, dict) and "cmd" in raw:
        env = CommandEnvelope.model_validate({"body": raw})
        return env.body
    raise ValueError(f"command payload must be a JSON object with 'cmd' or 'body', got {type(raw).__name__}")


def encode_command(cmd: Command) -> str:
    """Serialise a typed command back to JSON (for the operator CLI)."""
    return cmd.model_dump_json(exclude_none=True)
