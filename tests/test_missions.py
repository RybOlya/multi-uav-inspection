"""Unit tests for swarm.facility (named scan zones and POIs)."""
from __future__ import annotations

from swarm.facility import default_pois, default_zones


def test_default_zones_have_two_aisles():
    zones = default_zones()
    assert set(zones) == {"aisle_left", "aisle_right"}


def test_zones_map_to_distinct_drones():
    zones = default_zones()
    drones = {z.drone_id for z in zones.values()}
    assert drones == {"uav-01", "uav-02"}


def test_zone_scan_cmd_validates():
    zone = default_zones()["aisle_right"]
    cmd = zone.to_scan_cmd()
    assert cmd.cmd == "scan"
    assert cmd.x_max > cmd.x_min
    assert cmd.y_max > cmd.y_min


def test_default_pois_have_two_markers():
    pois = default_pois()
    assert set(pois) == {"rack_left_end", "rack_right_end"}


def test_poi_orbit_radius_positive():
    for poi in default_pois().values():
        assert poi.orbit_radius_m > 0
        assert poi.z > 0
