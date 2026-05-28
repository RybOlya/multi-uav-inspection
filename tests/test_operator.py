"""Operator CLI tests."""
from __future__ import annotations

from unittest.mock import MagicMock, patch

import pytest

from swarm.commands import TakeoffCmd


def test_publish_command_uses_mqtt():
    with patch("swarm.mqtt_cmd.MqttPublisher") as pub_cls:
        pub = MagicMock()
        pub.is_connected = True
        pub_cls.return_value = pub
        from swarm.mqtt_cmd import publish_command

        ok = publish_command("all", TakeoffCmd(cmd="takeoff", altitude_m=2.5))
        assert ok is True
        pub.publish_command.assert_called_once()


def test_operator_scan_zone_publishes():
    with patch("swarm.operator.publish_command", return_value=True) as pub:
        from swarm.operator import main
        assert main(["scan-zone", "aisle_left"]) == 0
        pub.assert_called_once()


def test_operator_inspect_poi_publishes():
    with patch("swarm.operator.publish_command", return_value=True) as pub:
        from swarm.operator import main
        assert main(["inspect-poi", "uav-01", "rack_left_end"]) == 0
        pub.assert_called_once()


def test_operator_orbit_publishes():
    with patch("swarm.operator.publish_command", return_value=True) as pub:
        from swarm.operator import main
        assert main([
            "orbit", "uav-01",
            "--cx", "0", "--cy", "2", "--altitude", "2.5", "--radius", "2"
        ]) == 0
        pub.assert_called_once()
