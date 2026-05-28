"""Coverage path planning primitives.

Implements the classical *Boustrophedon Cellular Decomposition* coverage
primitive — back-and-forth sweep within a single rectangular cell — exactly
as introduced by Choset (2000).

Reference:
    [4] Choset, H. (2000). "Coverage of Known Spaces: The Boustrophedon
    Cellular Decomposition". Autonomous Robots 9, 247–253.
    https://doi.org/10.1023/A:1008958800904

This module produces a list of ``(x, y, z)`` waypoints that, when traversed
in order, cover the requested rectangular region with a sensor footprint of
``sweep_spacing`` metres at constant altitude. Cell decomposition itself
(splitting an obstacle-laden polygon into monotone cells) is out of scope:
we expose only the per-cell sweep, since the operator is the one who chooses
which rectangle to scan.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Rectangle:
    """Axis-aligned rectangle in the world ``XY`` plane (metres)."""

    x_min: float
    x_max: float
    y_min: float
    y_max: float

    def __post_init__(self) -> None:
        if self.x_max <= self.x_min:
            raise ValueError(f"x_max ({self.x_max}) must be > x_min ({self.x_min})")
        if self.y_max <= self.y_min:
            raise ValueError(f"y_max ({self.y_max}) must be > y_min ({self.y_min})")

    @property
    def length_x(self) -> float:
        return self.x_max - self.x_min

    @property
    def length_y(self) -> float:
        return self.y_max - self.y_min


def boustrophedon_path(
    region: Rectangle,
    altitude_m: float,
    sweep_spacing_m: float = 2.0,
    sweep_axis: str = "x",
) -> list[np.ndarray]:
    """Generate a Boustrophedon (lawnmower) sweep over ``region`` at ``altitude_m``.

    Args:
        region: Axis-aligned rectangle to cover.
        altitude_m: Constant flight altitude (m above the world ``Z=0`` floor).
        sweep_spacing_m: Distance between adjacent passes (m). For an inspection
            UAV, this is typically chosen as ``sensor_footprint × (1 - overlap)``.
        sweep_axis: ``"x"`` to make long passes along ``X`` (stripes spaced
            along ``Y``) or ``"y"`` for the transposed sweep.

    Returns:
        Ordered list of 3-vectors ``[x, y, z]`` describing the coverage path.

    The output begins at the ``(x_min, y_min)`` corner and alternates the
    sweep direction on every stripe so that consecutive stripes share a
    short transition leg. This matches the canonical sweep primitive
    described in Section 4 of Choset (2000) [4] for a single Boustrophedon
    cell.
    """
    if sweep_spacing_m <= 0.0:
        raise ValueError(f"sweep_spacing_m must be > 0, got {sweep_spacing_m}")
    axis = sweep_axis.strip().lower()
    if axis not in {"x", "y"}:
        raise ValueError(f"sweep_axis must be 'x' or 'y', got {sweep_axis!r}")

    waypoints: list[np.ndarray] = []
    z = float(altitude_m)

    if axis == "x":
        stripe_coord = region.y_min
        forward = True
        while stripe_coord <= region.y_max + 1e-6:
            y = float(min(stripe_coord, region.y_max))
            if forward:
                waypoints.append(np.array([region.x_min, y, z], dtype=float))
                waypoints.append(np.array([region.x_max, y, z], dtype=float))
            else:
                waypoints.append(np.array([region.x_max, y, z], dtype=float))
                waypoints.append(np.array([region.x_min, y, z], dtype=float))
            forward = not forward
            stripe_coord += sweep_spacing_m
    else:
        stripe_coord = region.x_min
        forward = True
        while stripe_coord <= region.x_max + 1e-6:
            x = float(min(stripe_coord, region.x_max))
            if forward:
                waypoints.append(np.array([x, region.y_min, z], dtype=float))
                waypoints.append(np.array([x, region.y_max, z], dtype=float))
            else:
                waypoints.append(np.array([x, region.y_max, z], dtype=float))
                waypoints.append(np.array([x, region.y_min, z], dtype=float))
            forward = not forward
            stripe_coord += sweep_spacing_m

    return waypoints


def path_length_m(waypoints: list[np.ndarray]) -> float:
    """Sum of Euclidean segment lengths (m)."""
    if len(waypoints) < 2:
        return 0.0
    arr = np.asarray(waypoints, dtype=float)
    diffs = arr[1:] - arr[:-1]
    return float(np.linalg.norm(diffs, axis=1).sum())
