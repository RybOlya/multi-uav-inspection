"""Unit tests for swarm.trajectory (quintic time-scaling per Lynch & Park 2017 [26])."""
from __future__ import annotations

import math

import numpy as np
import pytest

from swarm.trajectory import (
    QuinticSegment,
    TrajectoryFollower,
    build_chained_waypoints,
    build_orbit,
    build_point_to_point,
)


def test_quintic_endpoint_position_velocity_acceleration_zero():
    seg = QuinticSegment.build(
        p0=np.array([0.0, 0.0, 0.0]),
        p1=np.array([5.0, -2.0, 3.0]),
        T=2.5,
    )
    p0, v0, a0, *_ = seg.sample(0.0)
    p1, v1, a1, *_ = seg.sample(seg.T)
    assert np.allclose(p0, [0.0, 0.0, 0.0])
    assert np.allclose(v0, [0.0, 0.0, 0.0], atol=1e-9)
    assert np.allclose(a0, [0.0, 0.0, 0.0], atol=1e-9)
    assert np.allclose(p1, [5.0, -2.0, 3.0])
    assert np.allclose(v1, [0.0, 0.0, 0.0], atol=1e-9)
    assert np.allclose(a1, [0.0, 0.0, 0.0], atol=1e-9)


def test_point_to_point_duration_scales_with_distance():
    short = build_point_to_point(np.zeros(3), np.array([1.0, 0.0, 0.0]), cruise_speed_mps=1.0)
    long = build_point_to_point(np.zeros(3), np.array([10.0, 0.0, 0.0]), cruise_speed_mps=1.0)
    assert long[0].T > short[0].T
    assert long[0].T == pytest.approx(10.0, rel=0.05)


def test_chained_waypoints_endpoints_are_at_rest():
    pts = [
        np.array([0.0, 0.0, 4.0]),
        np.array([5.0, 0.0, 4.0]),
        np.array([5.0, 3.0, 4.0]),
        np.array([0.0, 3.0, 4.0]),
    ]
    segs = build_chained_waypoints(pts, cruise_speed_mps=1.5)
    assert len(segs) == 3
    assert np.allclose(segs[0].v0, 0.0)
    assert np.allclose(segs[-1].v1, 0.0)
    junction_v = segs[0].v1
    assert np.linalg.norm(junction_v) > 0.0


def test_trajectory_follower_holds_final_pose_after_completion():
    segs = build_point_to_point(np.zeros(3), np.array([2.0, 0.0, 1.0]), cruise_speed_mps=1.0)
    follower = TrajectoryFollower(segments=segs)
    dt = 0.1
    for _ in range(int(segs[0].T / dt) + 5):
        p, v, a, j, yaw, yaw_rate = follower.advance(dt)
    assert follower.finished
    assert np.allclose(p, [2.0, 0.0, 1.0], atol=1e-6)
    assert np.allclose(v, 0.0, atol=1e-6)


def test_hover_resets_follower_to_completion():
    follower = TrajectoryFollower(segments=[], name="x")
    follower.hover_at(np.array([1.0, 2.0, 3.0]))
    assert follower.finished
    p, v, *_ = follower.advance(0.1)
    assert np.allclose(p, [1.0, 2.0, 3.0])
    assert np.allclose(v, 0.0)


class TestBuildOrbit:
    def test_orbit_returns_nonempty_segments(self):
        segs = build_orbit(
            center=np.array([0.0, 0.0, 2.5]),
            radius_m=2.0,
            altitude_m=2.5,
            speed_mps=0.5,
            revolutions=1.0,
        )
        assert len(segs) >= 4

    def test_orbit_waypoints_lie_on_circle(self):
        cx, cy = 1.0, -2.0
        r = 2.0
        alt = 3.0
        segs = build_orbit(
            center=np.array([cx, cy, alt]),
            radius_m=r,
            altitude_m=alt,
            speed_mps=0.5,
        )
        for seg in segs:
            # start point of each segment should be ~radius from centre
            d = math.sqrt((seg.p0[0] - cx) ** 2 + (seg.p0[1] - cy) ** 2)
            assert abs(d - r) < 0.1, f"point not on circle: dist={d:.3f} radius={r}"

    def test_orbit_all_segments_at_correct_altitude(self):
        alt = 2.5
        segs = build_orbit(
            center=np.zeros(3), radius_m=1.5, altitude_m=alt, speed_mps=0.4
        )
        for seg in segs:
            assert abs(seg.p0[2] - alt) < 1e-6
            assert abs(seg.p1[2] - alt) < 1e-6

    def test_orbit_loop_is_closed(self):
        segs = build_orbit(
            center=np.zeros(3), radius_m=2.0, altitude_m=2.0, speed_mps=0.5
        )
        assert np.allclose(segs[0].p0, segs[-1].p1, atol=1e-6)
