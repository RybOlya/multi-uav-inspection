"""MQTT line-protocol formatter tests (no broker required)."""
from __future__ import annotations

import re

from swarm.mqtt_publisher import telemetry_to_line_protocol


def test_line_protocol_starts_with_measurement_and_tag():
    line = telemetry_to_line_protocol(
        measurement="drone_telemetry",
        drone_id="uav-01",
        fields={"x": 1.5, "battery": 95.5, "alive": True, "wp": 3},
    )
    assert line.startswith("drone_telemetry,drone_id=uav-01")


def test_line_protocol_field_typing():
    line = telemetry_to_line_protocol(
        measurement="m", drone_id="d", fields={"f": 1.234, "i": 7, "b": False}
    )
    body = line.split(" ", 2)[1]
    fields = dict(part.split("=") for part in body.split(","))
    assert fields["f"] == "1.234000"
    assert fields["i"] == "7i"
    assert fields["b"] == "false"


def test_line_protocol_tag_escaping_for_spaces():
    line = telemetry_to_line_protocol(
        measurement="m",
        drone_id="d 1",
        fields={"x": 1.0},
        tags={"site": "hall A"},
    )
    assert "drone_id=d\\ 1" in line
    assert "site=hall\\ A" in line


def test_line_protocol_has_nanosecond_timestamp_at_the_end():
    line = telemetry_to_line_protocol(
        measurement="m", drone_id="d", fields={"x": 1.0}, timestamp_ns=1_700_000_000_000_000_000
    )
    assert line.endswith(" 1700000000000000000")
    assert re.search(r" \d{18,19}$", line)
