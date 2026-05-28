"""Battery-aware relay handoff between two drones.

When drone A's battery drops below a threshold, drone B flies to drone A's
last known **world pose** (X, Y, Z in metres) and resumes the scan from
that exact position.

All target coordinates are explicit world-frame XYZ — no abstract IDs,
no relative offsets.  Every position printed during the mission shows the
actual (X, Y, Z) numbers the drone will fly to.

Typical use::

    # terminal 1 — fast battery drain for demo
    SIM_BATTERY_DRAIN_RATE_PPS=2.0 python -m swarm.mock_swarm

    # terminal 2
    python missions/relay_demo.py
"""
from __future__ import annotations

import json
import logging
import threading
import time
from dataclasses import dataclass, field
from typing import Callable

import numpy as np

from .commands import GoToCmd, HoverCmd, LandCmd, TakeoffCmd
from .config import MqttSettings, SimSettings
from .mqtt_cmd import publish_command
from .mqtt_publisher import MqttPublisher

log = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# World-pose snapshot
# ---------------------------------------------------------------------------

@dataclass
class WorldPose:
    """Drone position in world frame (metres) plus live status."""
    drone_id: str
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    battery_pct: float = 100.0
    mode: str = "unknown"
    ts: float = field(default_factory=time.time)

    @property
    def xyz(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z], dtype=float)

    def __str__(self) -> str:
        return (
            f"({self.drone_id})  "
            f"X={self.x:+7.2f} m  Y={self.y:+7.2f} m  Z={self.z:+6.2f} m  "
            f"battery={self.battery_pct:5.1f}%  mode={self.mode}"
        )


# ---------------------------------------------------------------------------
# Live state tracker (subscribes to swarm/state/{drone_id})
# ---------------------------------------------------------------------------

class DroneStateTracker:
    """Subscribe to ``swarm/state/{drone_id}`` and maintain latest world pose.

    The paho-mqtt network loop runs in a background thread, so
    :meth:`snapshot` is safe to call from any thread.
    """

    def __init__(self, drone_id: str, settings: MqttSettings | None = None) -> None:
        self._drone_id = drone_id
        self._settings = settings or MqttSettings()
        self._lock = threading.Lock()
        self._pose = WorldPose(drone_id=drone_id)
        self._pub = MqttPublisher(
            self._settings, client_id=f"relay-tracker-{drone_id}-{int(time.time())}"
        )

    def connect(self) -> None:
        self._pub.connect()
        topic = f"{self._pub.topic_prefix}/state/{self._drone_id}"
        self._pub.subscribe(topic, self._on_state, qos=0)
        log.debug("DroneStateTracker subscribed to %s", topic)

    def disconnect(self) -> None:
        self._pub.disconnect()

    def snapshot(self) -> WorldPose:
        """Return a copy of the latest known world pose (thread-safe)."""
        with self._lock:
            p = self._pose
            return WorldPose(
                drone_id=p.drone_id,
                x=p.x, y=p.y, z=p.z,
                battery_pct=p.battery_pct,
                mode=p.mode,
                ts=p.ts,
            )

    def _on_state(self, _topic: str, payload: bytes) -> None:
        try:
            data = json.loads(payload)
            pos = data.get("position", [0.0, 0.0, 0.0])
            with self._lock:
                self._pose = WorldPose(
                    drone_id=self._drone_id,
                    x=float(pos[0]),
                    y=float(pos[1]),
                    z=float(pos[2]),
                    battery_pct=float(data.get("battery_pct", 100.0)),
                    mode=str(data.get("mode", "unknown")),
                    ts=float(data.get("ts", time.time())),
                )
        except Exception as exc:
            log.warning("DroneStateTracker parse error: %s", exc)


# ---------------------------------------------------------------------------
# Relay mission
# ---------------------------------------------------------------------------

class RelayMission:
    """Single-aisle battery-relay between drone_a and drone_b.

    Mission flow
    ------------
    1.  drone_a   takeoff → Z=altitude
    2.  drone_a   goto world pose (cx, y_start, altitude)   ← aisle entry
    3.  drone_a   goto world pose (cx, y_end,   altitude)   ← full aisle scan
        [monitoring battery every poll_interval seconds]

    When drone_a.battery_pct < threshold_pct:
    4.  drone_a   stop  →  capture handoff world pose (X, Y, Z)
    5.  drone_a   land
    6.  drone_b   takeoff → Z=altitude  (relay speed for urgency)
    7.  drone_b   goto   handoff world pose (X, Y, Z)
    8.  drone_b   goto   world pose (cx, y_end, altitude)   ← resume scan
    9.  drone_b   land

    If drone_a completes without hitting the threshold, it lands normally
    and no relay is triggered.

    Parameters
    ----------
    drone_a / drone_b : str
        MQTT drone IDs, e.g. "uav-01".
    cx : float
        Aisle centre-line X coordinate (world metres).
        Must match ``_CX_*`` in ``swarm/facility.py`` for the chosen aisle.
    y_start / y_end : float
        Near and far ends of the aisle in world Y (metres).
    altitude : float
        Constant scan altitude in world Z (metres, above floor).
    threshold_pct : float
        Battery % that triggers the handoff (0–100).
    relay_speed_mps : float
        Cruise speed for drone_b's intercept leg (can be higher than normal
        scan speed so it reaches the handoff point quickly).
    """

    def __init__(
        self,
        *,
        drone_a: str = "uav-01",
        drone_b: str = "uav-02",
        cx: float = -3.25,
        y_start: float = -5.5,
        y_end: float = 5.5,
        altitude: float = 3.0,
        threshold_pct: float | None = None,
        relay_speed_mps: float = 1.0,
        poll_interval_s: float = 0.5,
        mqtt_settings: MqttSettings | None = None,
        sim_settings: SimSettings | None = None,
        on_step: Callable[[str], None] | None = None,
    ) -> None:
        self.drone_a = drone_a
        self.drone_b = drone_b
        self.cx = cx
        self.y_start = y_start
        self.y_end = y_end
        self.altitude = altitude
        self.relay_speed_mps = relay_speed_mps
        self.poll_interval_s = poll_interval_s
        self.settings = mqtt_settings or MqttSettings()
        self.sim = sim_settings or SimSettings()
        self.threshold_pct = (
            threshold_pct
            if threshold_pct is not None
            else float(self.sim.battery_relay_threshold_pct)
        )
        self._log = on_step or print
        self._tracker = DroneStateTracker(drone_a, self.settings)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def run(self) -> None:
        """Execute the relay mission.  Blocks until both drones have landed."""
        step = self._log
        cruise = float(self.sim.cruise_speed_mps)

        entry = (self.cx, self.y_start, self.altitude)
        far   = (self.cx, self.y_end,   self.altitude)

        step("=" * 60)
        step("Relay Mission")
        step(f"  Drone A : {self.drone_a}  (scans, hands off when low)")
        step(f"  Drone B : {self.drone_b}  (waits, then resumes)")
        step(f"  Aisle centre  X = {self.cx:+.2f} m")
        step(f"  Near end      Y = {self.y_start:+.2f} m   "
             f"Far end Y = {self.y_end:+.2f} m")
        step(f"  Altitude      Z = {self.altitude:.2f} m")
        step(f"  Handoff at battery < {self.threshold_pct:.0f} %")
        step("=" * 60)

        # connect tracker before sending any commands
        self._tracker.connect()
        time.sleep(0.8)   # let MQTT subscription register

        # ── Phase 1: drone A takes off and begins aisle scan ──────────
        step("")
        step(f"[phase 1]  {self.drone_a}  takeoff → Z={self.altitude:.2f} m")
        self._pub(self.drone_a, TakeoffCmd(cmd="takeoff", altitude_m=self.altitude))
        self._wait(self.altitude / cruise + 4.0)

        step(f"[phase 1]  {self.drone_a}  goto aisle entry")
        step(f"           world pose  X={entry[0]:+.2f}  Y={entry[1]:+.2f}  Z={entry[2]:.2f}")
        self._pub(self.drone_a, GoToCmd(cmd="goto", x=entry[0], y=entry[1], z=entry[2]))
        self._wait(5.0)

        step(f"[phase 1]  {self.drone_a}  scan toward far end")
        step(f"           world pose  X={far[0]:+.2f}  Y={far[1]:+.2f}  Z={far[2]:.2f}")
        self._pub(self.drone_a, GoToCmd(cmd="goto", x=far[0], y=far[1], z=far[2]))

        # ── Monitor battery; wait for threshold or completion ─────────
        step(f"[phase 1]  monitoring {self.drone_a} "
             f"(threshold = {self.threshold_pct:.0f}%) …")

        aisle_len = abs(self.y_end - self.y_start)
        deadline = time.monotonic() + aisle_len / cruise + 20.0
        handoff: WorldPose | None = None

        # Max plausible battery drop per poll — 5× nominal rate guards against
        # ghost mock_swarm processes injecting stale 0 % readings.
        max_drop_per_poll = float(self.sim.battery_drain_rate_pps) * self.poll_interval_s * 5 + 2.0
        prev_battery: float | None = None

        while time.monotonic() < deadline:
            snap = self._tracker.snapshot()

            # ── anti-spike: ignore implausible sudden drop ────────────
            if prev_battery is not None:
                drop = prev_battery - snap.battery_pct
                if drop > max_drop_per_poll:
                    step(f"           [skip] battery spike "
                         f"{prev_battery:.0f}% → {snap.battery_pct:.0f}% "
                         f"(drop {drop:.0f}% > max {max_drop_per_poll:.0f}%) "
                         f"— likely ghost process, ignoring")
                    time.sleep(self.poll_interval_s)
                    continue
            prev_battery = snap.battery_pct

            step(f"           {snap}")

            if snap.battery_pct < self.threshold_pct:
                # stop drone_a in place, let it settle 1 s, read final pose
                self._pub(self.drone_a, HoverCmd(cmd="stop"))
                time.sleep(1.0)
                handoff = self._tracker.snapshot()
                break

            # reached far end regardless of scan direction
            scanning_negative = self.y_end < self.y_start
            at_end = (snap.y <= self.y_end + 0.5) if scanning_negative else (snap.y >= self.y_end - 0.5)
            if snap.mode in ("hover", "idle") and at_end:
                step(f"[phase 1]  {self.drone_a} completed scan without hitting threshold")
                break

            time.sleep(self.poll_interval_s)

        # ── No relay needed ──────────────────────────────────────────
        if handoff is None:
            step(f"[done]  {self.drone_a} landing (no relay needed)")
            self._pub(self.drone_a, LandCmd(cmd="land"))
            self._tracker.disconnect()
            return

        # ── Phase 2: relay ────────────────────────────────────────────
        hp = handoff
        step("")
        step("*** HANDOFF TRIGGERED ***")
        step(f"  {self.drone_a}  battery = {hp.battery_pct:.1f}%  "
             f"< threshold {self.threshold_pct:.0f}%")
        step(f"  Handoff world pose:  X={hp.x:+.2f} m  Y={hp.y:+.2f} m  Z={hp.z:.2f} m")
        step(f"  Remaining aisle:     Y={hp.y:+.2f} → Y={self.y_end:+.2f}  "
             f"({abs(self.y_end - hp.y):.1f} m)")
        step("")

        step(f"[phase 2]  {self.drone_a}  land in place  "
             f"(X={hp.x:+.2f}  Y={hp.y:+.2f})")
        self._pub(self.drone_a, LandCmd(cmd="land"))

        step(f"[phase 2]  {self.drone_b}  takeoff → Z={self.altitude:.2f} m")
        self._pub(self.drone_b, TakeoffCmd(cmd="takeoff", altitude_m=self.altitude))
        self._wait(self.altitude / self.relay_speed_mps + 4.0)

        # fly to handoff world pose (explicit X, Y, Z)
        step(f"[phase 2]  {self.drone_b}  intercept handoff world pose")
        step(f"           X={hp.x:+.2f} m  Y={hp.y:+.2f} m  Z={self.altitude:.2f} m  "
             f"(relay speed {self.relay_speed_mps} m/s)")
        self._pub(self.drone_b, GoToCmd(
            cmd="goto",
            x=hp.x, y=hp.y, z=self.altitude,
            cruise_speed_mps=self.relay_speed_mps,
        ))
        # conservative wait: distance from drone_b spawn to handoff
        # uav-02 spawns 2 m to the LEFT of uav-01's aisle (cx-2), same near end Y
        b_spawn = np.array([self.cx - 2.0, self.y_start, self.altitude])
        dist_intercept = float(np.linalg.norm(
            np.array([hp.x, hp.y]) - b_spawn[:2]
        ))
        self._wait(dist_intercept / self.relay_speed_mps + 5.0)

        # resume scan from handoff Y to y_end
        remaining = abs(self.y_end - hp.y)
        step(f"[phase 2]  {self.drone_b}  resume scan to far end")
        step(f"           world pose  X={self.cx:+.2f}  Y={self.y_end:+.2f}  Z={self.altitude:.2f}")
        self._pub(self.drone_b, GoToCmd(
            cmd="goto",
            x=self.cx, y=self.y_end, z=self.altitude,
        ))
        self._wait(remaining / cruise + 5.0)

        step(f"[phase 2]  {self.drone_b}  land")
        self._pub(self.drone_b, LandCmd(cmd="land"))
        self._wait(self.altitude / cruise + 4.0)

        self._tracker.disconnect()
        step("")
        step("=" * 60)
        step("Relay mission complete.")
        step(f"  {self.drone_a} covered  Y={self.y_start:+.2f} → Y={hp.y:+.2f} m  → landed in place")
        step(f"  {self.drone_b} covered  Y={hp.y:+.2f} → Y={self.y_end:+.2f} m")
        step("=" * 60)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _pub(self, target: str, cmd) -> None:
        if not publish_command(target, cmd, settings=self.settings, wait_s=0.15):
            raise ConnectionError("MQTT broker unreachable — run: make up")

    def _wait(self, seconds: float) -> None:
        time.sleep(max(1.0, seconds))
