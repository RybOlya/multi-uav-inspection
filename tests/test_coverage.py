"""Unit tests for swarm.coverage (Boustrophedon CPP, Choset 2000 [4])."""
from __future__ import annotations

import numpy as np
import pytest

from swarm.coverage import Rectangle, boustrophedon_path, path_length_m


def test_rectangle_rejects_invalid_bounds():
    with pytest.raises(ValueError):
        Rectangle(0.0, 0.0, 0.0, 1.0)
    with pytest.raises(ValueError):
        Rectangle(0.0, 1.0, 1.0, 0.0)


def test_boustrophedon_path_alternates_direction_per_stripe():
    region = Rectangle(0.0, 10.0, -2.0, 2.0)
    wps = boustrophedon_path(region, altitude_m=4.0, sweep_spacing_m=1.0, sweep_axis="x")
    assert len(wps) >= 4
    arr = np.asarray(wps)

    assert np.allclose(arr[:, 2], 4.0), "altitude must be constant"

    assert arr[0, 0] == pytest.approx(region.x_min)
    assert arr[1, 0] == pytest.approx(region.x_max)
    assert arr[2, 0] == pytest.approx(region.x_max)
    assert arr[3, 0] == pytest.approx(region.x_min)

    ys = arr[:, 1]
    assert ys[0] == pytest.approx(region.y_min)
    assert ys[1] == pytest.approx(region.y_min)
    assert ys[2] > ys[1] - 1e-6


def test_boustrophedon_path_y_axis():
    region = Rectangle(0.0, 6.0, -3.0, 3.0)
    wps = boustrophedon_path(region, altitude_m=3.0, sweep_spacing_m=2.0, sweep_axis="y")
    arr = np.asarray(wps)
    assert arr[0, 0] == pytest.approx(region.x_min)
    assert arr[1, 0] == pytest.approx(region.x_min)
    assert arr[0, 1] == pytest.approx(region.y_min)
    assert arr[1, 1] == pytest.approx(region.y_max)


def test_boustrophedon_path_length_lower_bound():
    region = Rectangle(0.0, 10.0, 0.0, 4.0)
    wps = boustrophedon_path(region, altitude_m=3.0, sweep_spacing_m=2.0, sweep_axis="x")
    length = path_length_m(wps)
    expected_min = region.length_x * 3
    assert length >= expected_min - 1e-6


def test_boustrophedon_path_rejects_bad_inputs():
    region = Rectangle(0.0, 5.0, 0.0, 5.0)
    with pytest.raises(ValueError):
        boustrophedon_path(region, altitude_m=2.0, sweep_spacing_m=0.0)
    with pytest.raises(ValueError):
        boustrophedon_path(region, altitude_m=2.0, sweep_axis="z")
