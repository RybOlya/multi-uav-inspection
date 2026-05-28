"""Time-parameterised quintic polynomial trajectories for waypoint following.

Used by :class:`swarm.controller.CommandedNonlinearController` to feed
position, velocity, acceleration and jerk references to the Pegasus
``NonlinearController`` (Mellinger & Kumar 2011 [25]; Pinto et al. 2021 [24])
in lieu of the original CSV-loaded trajectory.

References:
    [25] Mellinger, D. and Kumar, V. (2011). "Minimum snap trajectory
    generation and control for quadrotors." In *2011 IEEE International
    Conference on Robotics and Automation (ICRA)*, pp. 2520–2525.
    doi:10.1109/ICRA.2011.5980409.

    [26] Lynch, K. M. and Park, F. C. (2017). *Modern Robotics: Mechanics,
    Planning, and Control*. Cambridge University Press. §9.2 Point-to-Point
    Trajectories — quintic time scaling that yields zero velocity and
    acceleration at the endpoints.

Why a quintic polynomial:
    Per Lynch & Park (§9.2), a 5th-order polynomial in time is the
    *lowest* order that simultaneously enforces position, velocity, and
    acceleration at both endpoints; lower-order alternatives (cubic) leave
    a step in acceleration which the controller of [24, 25] would convert
    into a step in body torque — physically poor for a quadrotor. We chain
    multiple quintic segments end-to-end at non-zero junction velocities so
    a back-and-forth Boustrophedon scan flies smoothly, while still
    starting and ending at rest as required by [26, eq. 9.13–9.16].
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


def _quintic_coeffs(p0: float, p1: float, T: float, v0: float = 0.0, v1: float = 0.0,
                    a0: float = 0.0, a1: float = 0.0) -> tuple[float, ...]:
    """Solve the quintic polynomial p(t)=Σ a_i t^i for the boundary data.

    Closed-form solution from Lynch & Park (2017), §9.2, eq. 9.13–9.16 [26].
    Returns ``(c0, c1, c2, c3, c4, c5)`` such that
    ``p(t) = c0 + c1 t + c2 t^2 + c3 t^3 + c4 t^4 + c5 t^5``.
    """
    if T <= 0.0:
        raise ValueError(f"segment duration T must be > 0, got {T}")

    c0 = p0
    c1 = v0
    c2 = 0.5 * a0
    T2, T3, T4, T5 = T * T, T**3, T**4, T**5
    c3 = (20.0 * (p1 - p0) - (8.0 * v1 + 12.0 * v0) * T - (3.0 * a0 - a1) * T2) / (2.0 * T3)
    c4 = (30.0 * (p0 - p1) + (14.0 * v1 + 16.0 * v0) * T + (3.0 * a0 - 2.0 * a1) * T2) / (2.0 * T4)
    c5 = (12.0 * (p1 - p0) - (6.0 * v1 + 6.0 * v0) * T - (a0 - a1) * T2) / (2.0 * T5)
    return (c0, c1, c2, c3, c4, c5)


@dataclass
class QuinticSegment:
    """A 3-D quintic-polynomial segment between two boundary states.

    Boundary data are independently specified for ``x``, ``y``, and ``z``.
    Yaw is interpolated linearly between ``yaw0`` and ``yaw1`` over the same
    duration ``T``; rotational accelerations are not constrained because
    Pegasus' geometric controller drives yaw through a separate channel
    [24, 25].
    """

    p0: np.ndarray
    p1: np.ndarray
    v0: np.ndarray
    v1: np.ndarray
    a0: np.ndarray
    a1: np.ndarray
    yaw0: float
    yaw1: float
    T: float
    coeffs: list[tuple[float, ...]]  # one (c0..c5) tuple per axis

    @classmethod
    def build(
        cls,
        p0: np.ndarray,
        p1: np.ndarray,
        T: float,
        v0: np.ndarray | None = None,
        v1: np.ndarray | None = None,
        a0: np.ndarray | None = None,
        a1: np.ndarray | None = None,
        yaw0: float = 0.0,
        yaw1: float = 0.0,
    ) -> "QuinticSegment":
        p0 = np.asarray(p0, dtype=float).reshape(3)
        p1 = np.asarray(p1, dtype=float).reshape(3)
        v0 = np.zeros(3) if v0 is None else np.asarray(v0, dtype=float).reshape(3)
        v1 = np.zeros(3) if v1 is None else np.asarray(v1, dtype=float).reshape(3)
        a0 = np.zeros(3) if a0 is None else np.asarray(a0, dtype=float).reshape(3)
        a1 = np.zeros(3) if a1 is None else np.asarray(a1, dtype=float).reshape(3)

        coeffs = [
            _quintic_coeffs(p0[i], p1[i], T, v0[i], v1[i], a0[i], a1[i])
            for i in range(3)
        ]
        return cls(p0=p0, p1=p1, v0=v0, v1=v1, a0=a0, a1=a1,
                   yaw0=float(yaw0), yaw1=float(yaw1), T=float(T), coeffs=coeffs)

    def sample(self, tau: float) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, float, float]:
        """Evaluate ``(p, v, a, j, yaw, yaw_rate)`` at local time ``tau ∈ [0, T]``."""
        t = float(np.clip(tau, 0.0, self.T))
        p = np.empty(3)
        v = np.empty(3)
        a = np.empty(3)
        j = np.empty(3)
        for i, (c0, c1, c2, c3, c4, c5) in enumerate(self.coeffs):
            p[i] = c0 + c1 * t + c2 * t**2 + c3 * t**3 + c4 * t**4 + c5 * t**5
            v[i] = c1 + 2 * c2 * t + 3 * c3 * t**2 + 4 * c4 * t**3 + 5 * c5 * t**4
            a[i] = 2 * c2 + 6 * c3 * t + 12 * c4 * t**2 + 20 * c5 * t**3
            j[i] = 6 * c3 + 24 * c4 * t + 60 * c5 * t**2

        if self.T > 0.0:
            s = t / self.T
            yaw = self.yaw0 + (self.yaw1 - self.yaw0) * s
            yaw_rate = (self.yaw1 - self.yaw0) / self.T
        else:
            yaw, yaw_rate = self.yaw1, 0.0
        return p, v, a, j, yaw, yaw_rate


def _segment_duration(distance: float, cruise: float, min_T: float = 2.0) -> float:
    """Time budget for a segment of ``distance`` metres at ``cruise`` m/s."""
    if cruise <= 0.0:
        return max(min_T, 2.0)
    return max(min_T, distance / cruise)


def build_point_to_point(
    start_pos: np.ndarray,
    end_pos: np.ndarray,
    cruise_speed_mps: float = 0.5,
    yaw_start: float = 0.0,
    yaw_end: float = 0.0,
    min_duration_s: float = 2.0,
) -> list[QuinticSegment]:
    """Single-segment quintic from ``start_pos`` to ``end_pos`` with v=a=0 at endpoints.

    Implements the canonical point-to-point time-scaling of Lynch & Park
    (2017), §9.2 [26]. Suitable for ``goto``, ``takeoff``, ``land``, ``hover``.
    """
    distance = float(np.linalg.norm(np.asarray(end_pos) - np.asarray(start_pos)))
    T = _segment_duration(distance, cruise_speed_mps, min_T=min_duration_s)
    return [QuinticSegment.build(start_pos, end_pos, T, yaw0=yaw_start, yaw1=yaw_end)]


def build_chained_waypoints(
    waypoints: list[np.ndarray],
    cruise_speed_mps: float = 0.5,
    yaw_per_segment: list[float] | None = None,
    min_duration_s: float = 2.0,
) -> list[QuinticSegment]:
    """Chain multiple waypoints into a sequence of quintic segments.

    Boundary conditions:
        * Start of the first segment and end of the last segment have zero
          velocity and acceleration (the swarm stops cleanly at both ends),
          per Lynch & Park (2017), §9.2 [26, eq. 9.13–9.16].
        * Intermediate junctions inherit a non-zero velocity tangent to the
          incoming-outgoing waypoint chord, with magnitude clipped at
          ``cruise_speed_mps``. Acceleration is set to zero at junctions —
          a pragmatic relaxation of the full minimum-snap formulation of
          Mellinger & Kumar (2011) [25] that retains C¹ continuity in the
          delivered reference (sufficient for the ``NonlinearController``,
          which differentiates only up to jerk).

    This is the trajectory used to traverse a Boustrophedon coverage path
    produced by :func:`swarm.coverage.boustrophedon_path` when the operator
    issues a ``scan`` command.
    """
    if len(waypoints) < 2:
        raise ValueError("need at least 2 waypoints to build a chained trajectory")

    pts = [np.asarray(p, dtype=float).reshape(3) for p in waypoints]
    n = len(pts)

    junction_velocities: list[np.ndarray] = [np.zeros(3)]
    for i in range(1, n - 1):
        in_dir = pts[i] - pts[i - 1]
        out_dir = pts[i + 1] - pts[i]
        in_norm = np.linalg.norm(in_dir)
        out_norm = np.linalg.norm(out_dir)
        if in_norm < 1e-9 or out_norm < 1e-9:
            junction_velocities.append(np.zeros(3))
            continue
        avg_dir = in_dir / in_norm + out_dir / out_norm
        avg_norm = np.linalg.norm(avg_dir)
        if avg_norm < 1e-9:
            junction_velocities.append(np.zeros(3))
            continue
        v_dir = avg_dir / avg_norm
        junction_velocities.append(v_dir * float(cruise_speed_mps))
    junction_velocities.append(np.zeros(3))

    yaws = yaw_per_segment if yaw_per_segment is not None else [0.0] * (n - 1)
    if len(yaws) != n - 1:
        raise ValueError(
            f"yaw_per_segment length {len(yaws)} != number of segments {n - 1}"
        )

    segs: list[QuinticSegment] = []
    for i in range(n - 1):
        T = _segment_duration(
            float(np.linalg.norm(pts[i + 1] - pts[i])),
            cruise_speed_mps,
            min_T=min_duration_s,
        )
        segs.append(
            QuinticSegment.build(
                p0=pts[i],
                p1=pts[i + 1],
                T=T,
                v0=junction_velocities[i],
                v1=junction_velocities[i + 1],
                a0=np.zeros(3),
                a1=np.zeros(3),
                yaw0=yaws[i],
                yaw1=yaws[i],
            )
        )
    return segs


def build_orbit(
    center: np.ndarray,
    radius_m: float,
    altitude_m: float,
    speed_mps: float,
    revolutions: float = 1.0,
    n_waypoints: int = 16,
) -> list[QuinticSegment]:
    """Circular orbit around ``center`` at ``altitude_m``.

    Generates ``ceil(n_waypoints * revolutions)`` equally-spaced waypoints on
    a circle of ``radius_m`` and chains them with :func:`build_chained_waypoints`.
    The path is closed: the last waypoint equals the first so the drone
    completes full orbit(s) and stops cleanly.

    Yaw is set to face the centre on every segment so the camera always
    points inward.
    """
    import math

    center = np.asarray(center, dtype=float).reshape(3)
    n_pts = max(4, int(math.ceil(n_waypoints * revolutions)))
    angles = [2.0 * math.pi * i / n_waypoints for i in range(n_waypoints)]
    # Replicate angles for multiple revolutions, then close the loop.
    all_angles = []
    for _ in range(int(math.ceil(revolutions))):
        all_angles.extend(angles)
    all_angles = all_angles[:n_pts]
    all_angles.append(all_angles[0])  # close loop

    waypoints = [
        np.array([
            center[0] + radius_m * math.cos(a),
            center[1] + radius_m * math.sin(a),
            altitude_m,
        ], dtype=float)
        for a in all_angles
    ]

    # Yaw toward centre at each segment (angle + π).
    yaws = [a + math.pi for a in all_angles[:-1]]

    return build_chained_waypoints(
        waypoints,
        cruise_speed_mps=speed_mps,
        yaw_per_segment=yaws,
        min_duration_s=1.0,
    )


@dataclass
class TrajectoryFollower:
    """Stateful evaluator for a sequence of :class:`QuinticSegment`.

    The controller pulls ``(p_ref, v_ref, a_ref, j_ref, yaw, yaw_rate)`` at
    each physics step. After the last segment, the follower latches the
    final waypoint as a hover setpoint (``v=a=j=0``).
    """

    segments: list[QuinticSegment]
    elapsed_s: float = 0.0
    name: str = "idle"
    finished: bool = False

    @property
    def total_duration_s(self) -> float:
        return float(sum(s.T for s in self.segments))

    def reset(self, segments: list[QuinticSegment], name: str = "trajectory") -> None:
        self.segments = list(segments)
        self.elapsed_s = 0.0
        self.name = name
        self.finished = len(self.segments) == 0

    def hover_at(self, position: np.ndarray, yaw: float = 0.0) -> None:
        position = np.asarray(position, dtype=float).reshape(3)
        seg = QuinticSegment.build(position, position, T=1.0, yaw0=yaw, yaw1=yaw)
        self.segments = [seg]
        self.elapsed_s = 1.0
        self.name = "hover"
        self.finished = True

    def advance(self, dt: float) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, float, float]:
        """Step the follower by ``dt`` seconds and return the current reference."""
        if not self.segments:
            return (
                np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(3), 0.0, 0.0,
            )

        self.elapsed_s += max(0.0, float(dt))
        t = self.elapsed_s
        cum = 0.0
        for seg in self.segments:
            if t <= cum + seg.T + 1e-9:
                p, v, a, j, yaw, yaw_rate = seg.sample(t - cum)
                return p, v, a, j, yaw, yaw_rate
            cum += seg.T

        last = self.segments[-1]
        self.finished = True
        return last.p1.copy(), np.zeros(3), np.zeros(3), np.zeros(3), last.yaw1, 0.0
