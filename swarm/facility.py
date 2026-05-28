"""Facility map — 2-drone warehouse inspection.

Coordinate frame (Isaac Sim): X right, Y forward, Z up.  Floor at Z = 0.

      X
  ←   |   →
  ====|====  rack row L  (x ≈ -5 … -2)
      |       aisle L     x = -3.25   ← uav-01
  ====|====  rack row C  (x ≈ -2 … +2)
      |       aisle R     x = +3.25   ← uav-02
  ====|====  rack row R  (x ≈ +2 … +5)

Aisles run along Y.  Tune _CX_LEFT/_CX_RIGHT to your exact scene.
"""
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Literal

from .commands import HoverCmd, LandCmd, ScanCmd, TakeoffCmd
from .config import MqttSettings, SimSettings
from .mqtt_cmd import publish_command


@dataclass(frozen=True)
class POI:
    """Named inspection marker."""
    id: str
    label: str
    x: float
    y: float
    z: float
    orbit_radius_m: float = 1.5
    dwell_s: float = 4.0


@dataclass(frozen=True)
class ScanZone:
    """Rectangular Boustrophedon sweep area for one drone."""
    key: str
    label: str
    drone_id: str
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    altitude_m: float
    sweep_spacing_m: float
    sweep_axis: Literal["x", "y"] = "x"

    def to_scan_cmd(self) -> ScanCmd:
        return ScanCmd(
            cmd="scan",
            x_min=self.x_min, x_max=self.x_max,
            y_min=self.y_min, y_max=self.y_max,
            altitude_m=self.altitude_m,
            sweep_spacing_m=self.sweep_spacing_m,
            sweep_axis=self.sweep_axis,
        )


# ---------------------------------------------------------------------------
# Scene constants — adjust to match your Isaac Sim scene
# ---------------------------------------------------------------------------
_CX_LEFT  = -3.25   # left aisle centre-line  (uav-01)
_CX_RIGHT =  3.25   # right aisle centre-line (uav-02)
_Y_NEAR   = -5.5    # near end of aisles
_Y_FAR    =  5.5    # far end of aisles
_ALT      =  3.0    # scan altitude (above ~2 m rack tops)


def default_pois() -> dict[str, POI]:
    """Far ends of the two rack rows — one POI per drone."""
    return {
        "rack_left_end": POI(
            id="rack_left_end", label="Left Rack — Far End",
            x=_CX_LEFT, y=_Y_FAR, z=_ALT,
        ),
        "rack_right_end": POI(
            id="rack_right_end", label="Right Rack — Far End",
            x=_CX_RIGHT, y=_Y_FAR, z=_ALT,
        ),
    }


def default_zones(
    sim: SimSettings | None = None,
    *,
    altitude_m: float | None = None,
) -> dict[str, ScanZone]:
    """Two aisle scan zones — one straight pass per aisle along Y.

    Single-pass trick: x_max = x_min + tiny_eps, sweep_spacing >> eps
    → boustrophedon fires exactly once → straight line down aisle centre.
    """
    sim = sim or SimSettings()
    alt = max(
        float(altitude_m if altitude_m is not None else sim.takeoff_altitude_m),
        3.0,
    )
    _EPS     = 0.05
    _SPACING = 10.0

    return {
        "aisle_left": ScanZone(
            key="aisle_left", label="Left Aisle (uav-01)",
            drone_id="uav-01",
            x_min=_CX_LEFT, x_max=_CX_LEFT + _EPS,
            y_min=_Y_NEAR,  y_max=_Y_FAR,
            altitude_m=alt,
            sweep_spacing_m=_SPACING,
            sweep_axis="y",
        ),
        "aisle_right": ScanZone(
            key="aisle_right", label="Right Aisle (uav-02)",
            drone_id="uav-02",
            x_min=_CX_RIGHT, x_max=_CX_RIGHT + _EPS,
            y_min=_Y_NEAR,   y_max=_Y_FAR,
            altitude_m=alt,
            sweep_spacing_m=_SPACING,
            sweep_axis="y",
        ),
    }
