"""MQTT v5 client wrapper for the swarm telemetry/command channel.

Two topic families are used (see :mod:`swarm.commands`):

* ``{prefix}/cmd/{drone_id}``      — operator → drone control commands (JSON).
* ``{prefix}/cmd/all``             — broadcast to all drones.
* ``{prefix}/state/{drone_id}``    — drone → operator state (JSON).
* ``{prefix}/telemetry/{drone_id}``— drone → cloud, InfluxDB Line Protocol.
* ``{prefix}/events/{kind}``       — lifecycle events (JSON).

References:
    [17] OASIS Standard, "MQTT Version 5.0", 2019.
         https://docs.oasis-open.org/mqtt/mqtt/v5.0/mqtt-v5.0.pdf
"""
from __future__ import annotations

import json
import logging
import time
from typing import Any, Callable

import paho.mqtt.client as mqtt

from .config import MqttSettings

log = logging.getLogger(__name__)


def _escape_tag(value: str) -> str:
    return value.replace(",", r"\,").replace(" ", r"\ ").replace("=", r"\=")


def telemetry_to_line_protocol(
    *,
    measurement: str,
    drone_id: str,
    fields: dict[str, float | int | bool],
    tags: dict[str, str] | None = None,
    timestamp_ns: int | None = None,
) -> str:
    """Format a fields dict into InfluxDB Line Protocol.

    Used so any downstream consumer (Telegraf, ``mosquitto_sub``, custom
    subscriber) can ingest telemetry without further parsing.
    """
    tags = {"drone_id": drone_id, **(tags or {})}
    tag_str = ",".join(f"{_escape_tag(k)}={_escape_tag(v)}" for k, v in tags.items())
    field_parts: list[str] = []
    for k, v in fields.items():
        if isinstance(v, bool):
            field_parts.append(f"{k}={'true' if v else 'false'}")
        elif isinstance(v, int):
            field_parts.append(f"{k}={v}i")
        else:
            field_parts.append(f"{k}={float(v):.6f}")
    field_str = ",".join(field_parts)
    ts = timestamp_ns if timestamp_ns is not None else time.time_ns()
    return f"{measurement},{tag_str} {field_str} {ts}"


class MqttPublisher:
    """MQTT v5 publisher + subscriber. Reuses one ``paho`` client.

    On connect failure we log a warning and degrade silently — the
    simulation must keep running even if the broker is unreachable.
    """

    def __init__(self, settings: MqttSettings | None = None, *, client_id: str | None = None) -> None:
        self.settings = settings or MqttSettings()
        cid = client_id or f"swarm-{int(time.time_ns())}"
        self._client = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2,
            client_id=cid,
            protocol=mqtt.MQTTv5,
        )
        if self.settings.username:
            self._client.username_pw_set(self.settings.username, self.settings.password or "")
        self._connected = False
        self._handlers: dict[str, list[Callable[[str, bytes], None]]] = {}
        self._client.on_message = self._on_message

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def topic_prefix(self) -> str:
        return self.settings.topic_prefix

    def connect(self) -> None:
        try:
            self._client.connect(self.settings.host, self.settings.port, self.settings.keepalive)
            self._client.loop_start()
            self._connected = True
            log.info("MQTT connected to %s:%s", self.settings.host, self.settings.port)
        except Exception as exc:
            log.warning(
                "MQTT connect to %s:%s failed: %s — running in offline mode",
                self.settings.host,
                self.settings.port,
                exc,
            )
            self._connected = False

    def disconnect(self) -> None:
        if self._connected:
            self._client.loop_stop()
            self._client.disconnect()
            self._connected = False

    def publish(self, topic: str, payload: str | bytes, qos: int = 1, retain: bool = False) -> None:
        if not self._connected:
            return
        try:
            self._client.publish(topic, payload, qos=qos, retain=retain)
        except Exception as exc:
            log.debug("MQTT publish failed: %s", exc)

    def subscribe(self, topic: str, handler: Callable[[str, bytes], None], qos: int = 1) -> None:
        """Subscribe to ``topic`` and route incoming messages to ``handler(topic, payload)``."""
        self._handlers.setdefault(topic, []).append(handler)
        if self._connected:
            try:
                self._client.subscribe(topic, qos=qos)
            except Exception as exc:
                log.debug("MQTT subscribe(%s) failed: %s", topic, exc)

    def _on_message(self, _client, _userdata, msg) -> None:
        for sub_topic, handlers in self._handlers.items():
            if mqtt.topic_matches_sub(sub_topic, msg.topic):
                for h in handlers:
                    try:
                        h(msg.topic, msg.payload)
                    except Exception:
                        log.exception("handler for %s raised", msg.topic)

    def publish_telemetry(self, drone_id: str, fields: dict[str, Any]) -> None:
        topic = f"{self.settings.topic_prefix}/telemetry/{drone_id}"
        line = telemetry_to_line_protocol(
            measurement="drone_telemetry",
            drone_id=drone_id,
            fields=fields,
        )
        self.publish(topic, line, qos=0)

    def publish_state(self, drone_id: str, payload: dict[str, Any]) -> None:
        topic = f"{self.settings.topic_prefix}/state/{drone_id}"
        self.publish(topic, json.dumps(payload), qos=0, retain=False)

    def publish_event(self, kind: str, payload: dict[str, Any]) -> None:
        topic = f"{self.settings.topic_prefix}/events/{kind}"
        self.publish(topic, json.dumps(payload), qos=1, retain=False)

    def publish_command(self, target: str, payload: str) -> None:
        """Operator-side helper: publish a JSON command to ``swarm/cmd/{target}``."""
        topic = f"{self.settings.topic_prefix}/cmd/{target}"
        self.publish(topic, payload, qos=1, retain=False)
