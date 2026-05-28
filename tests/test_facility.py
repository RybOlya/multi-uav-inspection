"""Tests for swarm.facility — POI registry and zone command generation."""
from __future__ import annotations

import pytest

from swarm.commands import ScanCmd
from swarm.facility import default_pois, default_zones


class TestPOIRegistry:
    def test_expected_poi_ids(self):
        pois = default_pois()
        assert set(pois) == {"rack_left_end", "rack_right_end"}

    def test_all_pois_have_unique_ids(self):
        pois = default_pois()
        ids = [p.id for p in pois.values()]
        assert len(ids) == len(set(ids))

    def test_poi_z_above_floor(self):
        for poi in default_pois().values():
            assert poi.z > 0.5, f"{poi.id} is too close to floor"

    def test_poi_orbit_radius_positive(self):
        for poi in default_pois().values():
            assert poi.orbit_radius_m > 0


class TestScanZoneRegistry:
    def test_two_zones(self):
        assert set(default_zones()) == {"aisle_left", "aisle_right"}

    def test_zones_centre_lines_distinct(self):
        centres = [z.x_min for z in default_zones().values()]
        assert len(centres) == len(set(centres)), "two zones share the same aisle centre"

    def test_sweep_axis_is_y_for_all_zones(self):
        for zone in default_zones().values():
            assert zone.sweep_axis == "y", (
                f"{zone.key} has sweep_axis={zone.sweep_axis!r}; aisles run along Y"
            )

    def test_scan_altitude_above_rack_tops(self):
        for zone in default_zones().values():
            assert zone.altitude_m >= 3.0, (
                f"{zone.key} altitude {zone.altitude_m} m is below 2 m rack tops"
            )

    def test_scan_cmd_rectangle_valid(self):
        for z in default_zones().values():
            cmd = z.to_scan_cmd()
            assert cmd.x_max > cmd.x_min
            assert cmd.y_max > cmd.y_min
