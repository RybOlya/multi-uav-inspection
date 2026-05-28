"""Unit tests for swarm.commands (MQTT v5 wire schema)."""
from __future__ import annotations

import pytest

from swarm.commands import (
    GoToCmd,
    HoverCmd,
    InspectPoiCmd,
    LandCmd,
    OrbitCmd,
    ScanCmd,
    TakeoffCmd,
    encode_command,
    parse_command,
)


def test_takeoff_round_trip():
    payload = '{"cmd":"takeoff","altitude_m":3.0,"seq":7}'
    cmd = parse_command(payload)
    assert isinstance(cmd, TakeoffCmd)
    assert cmd.altitude_m == 3.0
    assert cmd.seq == 7
    assert "takeoff" in encode_command(cmd)


def test_goto_payload():
    cmd = parse_command('{"cmd":"goto","x":1.0,"y":2.0,"z":3.0,"yaw":0.5}')
    assert isinstance(cmd, GoToCmd)
    assert cmd.x == 1.0 and cmd.y == 2.0 and cmd.z == 3.0
    assert cmd.yaw == 0.5


def test_scan_validates_rectangle():
    good = parse_command(
        '{"cmd":"scan","x_min":0,"x_max":10,"y_min":-2,"y_max":2,"altitude_m":4}'
    )
    assert isinstance(good, ScanCmd)
    with pytest.raises(Exception):
        parse_command(
            '{"cmd":"scan","x_min":5,"x_max":1,"y_min":-2,"y_max":2,"altitude_m":4}'
        )


def test_hover_and_stop_aliases():
    assert isinstance(parse_command('{"cmd":"hover"}'), HoverCmd)
    assert isinstance(parse_command('{"cmd":"stop"}'), HoverCmd)


def test_land_default_cruise_is_none():
    cmd = parse_command('{"cmd":"land"}')
    assert isinstance(cmd, LandCmd)
    assert cmd.cruise_speed_mps is None


def test_envelope_form_is_accepted():
    cmd = parse_command('{"body":{"cmd":"hover"}}')
    assert isinstance(cmd, HoverCmd)


def test_unknown_cmd_rejected():
    with pytest.raises(Exception):
        parse_command('{"cmd":"explode"}')


def test_orbit_round_trip():
    payload = '{"cmd":"orbit","center_x":0.0,"center_y":2.0,"altitude_m":2.5,"radius_m":1.5,"speed_mps":0.4,"revolutions":1.0}'
    cmd = parse_command(payload)
    assert isinstance(cmd, OrbitCmd)
    assert cmd.center_x == 0.0
    assert cmd.radius_m == 1.5
    assert "orbit" in encode_command(cmd)


def test_orbit_defaults():
    cmd = parse_command('{"cmd":"orbit","center_x":1.0,"center_y":2.0,"altitude_m":3.0}')
    assert isinstance(cmd, OrbitCmd)
    assert cmd.radius_m == 2.0
    assert cmd.speed_mps == 0.5
    assert cmd.revolutions == 1.0


def test_inspect_poi_round_trip():
    payload = '{"cmd":"inspect_poi","poi_id":"rack_left_end","dwell_s":4.0}'
    cmd = parse_command(payload)
    assert isinstance(cmd, InspectPoiCmd)
    assert cmd.poi_id == "rack_left_end"
    assert cmd.dwell_s == 4.0
    assert "inspect_poi" in encode_command(cmd)


def test_inspect_poi_default_dwell():
    cmd = parse_command('{"cmd":"inspect_poi","poi_id":"rack_right_end"}')
    assert isinstance(cmd, InspectPoiCmd)
    assert cmd.dwell_s == 5.0
