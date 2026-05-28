"""Shared MQTT command publishing for CLI, UI, and mission scripts."""
from __future__ import annotations

import time
from typing import TYPE_CHECKING

from .commands import encode_command
from .config import MqttSettings
from .mqtt_publisher import MqttPublisher

if TYPE_CHECKING:
    from .commands import Command


def publish_command(
    target: str,
    cmd: "Command",
    *,
    settings: MqttSettings | None = None,
    client_id: str | None = None,
    wait_s: float = 0.15,
) -> bool:
    """Publish one command to ``swarm/cmd/{target}``. Returns True if broker OK."""
    pub = MqttPublisher(
        settings or MqttSettings(),
        client_id=client_id or f"operator-{int(time.time() * 1000)}",
    )
    pub.connect()
    if not pub.is_connected:
        return False
    try:
        pub.publish_command(target, encode_command(cmd))
        if wait_s > 0:
            time.sleep(wait_s)
    finally:
        pub.disconnect()
    return True


def require_broker(settings: MqttSettings | None = None) -> MqttPublisher:
    """Connect or raise with a clear operator message."""
    pub = MqttPublisher(settings or MqttSettings(), client_id="swarm-check")
    pub.connect()
    if not pub.is_connected:
        settings = settings or MqttSettings()
        raise ConnectionError(
            f"MQTT broker unreachable at {settings.host}:{settings.port}. "
            "Run: make up"
        )
    return pub
