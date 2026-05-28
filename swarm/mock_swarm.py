"""Headless swarm simulator — MQTT commands without Isaac Sim.

Runs the same trajectory + coverage logic as the Isaac controller but
without the physics engine. Publishes identical telemetry (including
battery_pct and coverage_pct) so the Grafana dashboard works identically.

Usage::

    make up
    python -m swarm.mock_swarm          # terminal 1
    python missions/inspect.py          # terminal 2
    make grafana
"""
from __future__ import annotations

import logging
import sys
import threading
import time
from dataclasses import dataclass, field

import numpy as np

from .commands import (
    Command, GoToCmd, HoverCmd, InspectPoiCmd, LandCmd,
    OrbitCmd, ScanCmd, TakeoffCmd, parse_command,
)
from .config import MqttSettings, SimSettings
from .coverage import Rectangle, boustrophedon_path
from .mqtt_publisher import MqttPublisher
from .trajectory import (
    QuinticSegment,
    TrajectoryFollower,
    build_chained_waypoints,
    build_orbit,
    build_point_to_point,
)

log = logging.getLogger(__name__)


@dataclass
class DroneSim:
    drone_id: str
    position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    follower: TrajectoryFollower = field(
        default_factory=lambda: TrajectoryFollower(segments=[], name="idle", finished=True)
    )
    mode: str = "idle"
    battery_pct: float = 100.0
    coverage_pct: float = 0.0
    scan_total_segs: int = 0
    scan_passed_segs: int = 0
    # drain rate read from SimSettings so SIM_BATTERY_DRAIN_RATE_PPS env var works
    battery_drain_rate: float = 0.05


class MockSwarm:
    """Kinematic stand-in for Pegasus drones (same MQTT topics as Isaac)."""

    def __init__(self, num_drones: int = 2) -> None:
        self.sim = SimSettings()
        self.mqtt = MqttSettings()
        self._publisher = MqttPublisher(self.mqtt, client_id="mock-swarm")
        self._lock = threading.Lock()
        self._running = False
        self._drones: dict[str, DroneSim] = {}

        # uav-01: left aisle  uav-02: 2 m to the LEFT of uav-01, same near end
        _aisle_entries = [
            np.array([8.25, 3.5, 0.1], dtype=float),   # uav-01
            np.array([8.25, 5.5, 0.1], dtype=float),   # uav-02  (2 m left of uav-01)
        ]
        drain = float(self.sim.battery_drain_rate_pps)
        for i in range(num_drones):
            drone_id = f"uav-{i + 1:02d}"
            if i < len(_aisle_entries):
                pos = _aisle_entries[i].copy()
            else:
                pos = np.array([float(i % 3) * 1.5 - 1.5, -7.0 - float(i // 3) * 1.5, 0.1])
            self._drones[drone_id] = DroneSim(
                drone_id=drone_id, position=pos, battery_drain_rate=drain
            )

    def start(self) -> None:
        self._publisher.connect()
        if not self._publisher.is_connected:
            raise ConnectionError(
                f"Cannot connect to MQTT at {self.mqtt.host}:{self.mqtt.port}. "
                "Run: make up"
            )
        prefix = self._publisher.topic_prefix
        for drone_id in self._drones:
            self._publisher.subscribe(
                f"{prefix}/cmd/{drone_id}", self._on_command, qos=1,
            )
        self._publisher.subscribe(f"{prefix}/cmd/all", self._on_command, qos=1)
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        print(f"[mock_swarm] {len(self._drones)} drones listening")
        print("[mock_swarm] run: python missions/inspect.py")

    def stop(self) -> None:
        self._running = False
        self._publisher.disconnect()

    def _on_command(self, topic: str, payload: bytes) -> None:
        try:
            cmd = parse_command(payload)
        except Exception as exc:
            log.warning("bad command on %s: %s", topic, exc)
            return

        if topic.endswith("/all"):
            targets = list(self._drones.keys())
        else:
            drone_id = topic.rsplit("/", 1)[-1]
            targets = [drone_id] if drone_id in self._drones else []

        for drone_id in targets:
            with self._lock:
                self._apply(drone_id, cmd)
            self._publisher.publish_event(
                "command_accepted",
                {"drone_id": drone_id, "cmd": cmd.cmd, "ts": time.time()},
            )

    def _apply(self, drone_id: str, cmd: Command) -> None:
        drone = self._drones[drone_id]
        cur = drone.position.copy()
        cruise = float(self.sim.cruise_speed_mps)

        if isinstance(cmd, TakeoffCmd):
            # reset battery and coverage so re-runs don't start depleted
            drone.battery_pct = 100.0
            drone.coverage_pct = 0.0
            drone.scan_total_segs = 0
            drone.scan_passed_segs = 0
            target = np.array([cur[0], cur[1], cmd.altitude_m])
            drone.follower.reset(
                build_point_to_point(cur, target, cruise_speed_mps=cruise),
                name="takeoff",
            )
            drone.mode = "takeoff"
            return

        if isinstance(cmd, GoToCmd):
            target = np.array([cmd.x, cmd.y, cmd.z])
            drone.follower.reset(
                build_point_to_point(cur, target, cruise_speed_mps=cruise),
                name="goto",
            )
            drone.mode = "goto"
            return

        if isinstance(cmd, OrbitCmd):
            segs = build_orbit(
                center=np.array([cmd.center_x, cmd.center_y, cmd.altitude_m]),
                radius_m=cmd.radius_m,
                altitude_m=cmd.altitude_m,
                speed_mps=cmd.speed_mps,
                revolutions=cmd.revolutions,
            )
            drone.follower.reset(segs, name="orbit")
            drone.mode = "orbit"
            return

        if isinstance(cmd, InspectPoiCmd):
            from .facility import default_pois
            pois = default_pois()
            if cmd.poi_id not in pois:
                log.warning("unknown POI: %s", cmd.poi_id)
                return
            poi = pois[cmd.poi_id]
            poi_pos = np.array([poi.x, poi.y, poi.z], dtype=float)
            goto_segs = build_point_to_point(cur, poi_pos, cruise_speed_mps=cruise)
            hover_seg = QuinticSegment.build(poi_pos, poi_pos, T=max(1.0, float(cmd.dwell_s)))
            orbit_segs = build_orbit(poi_pos, poi.orbit_radius_m, poi.z, cruise)
            drone.follower.reset(goto_segs + [hover_seg] + orbit_segs, name="inspect_poi")
            drone.mode = "inspect_poi"
            return

        if isinstance(cmd, ScanCmd):
            region = Rectangle(cmd.x_min, cmd.x_max, cmd.y_min, cmd.y_max)
            wps = boustrophedon_path(
                region,
                altitude_m=cmd.altitude_m,
                sweep_spacing_m=cmd.sweep_spacing_m,
                sweep_axis=cmd.sweep_axis,
            )
            entry = wps[0]
            chained = [cur] + wps if np.linalg.norm(entry - cur) > 1e-3 else wps
            segs = build_chained_waypoints(chained, cruise_speed_mps=cruise)
            drone.follower.reset(segs, name="scan")
            drone.mode = "scan"
            drone.coverage_pct = 0.0
            drone.scan_total_segs = len(segs)
            drone.scan_passed_segs = 0
            return

        if isinstance(cmd, HoverCmd):
            drone.follower.hover_at(cur)
            drone.mode = "hover"
            return

        if isinstance(cmd, LandCmd):
            target = np.array([cur[0], cur[1], 0.05])
            drone.follower.reset(
                build_point_to_point(cur, target, cruise_speed_mps=cruise),
                name="land",
            )
            drone.mode = "land"
            return

    def _loop(self) -> None:
        dt = 0.05
        while self._running:
            with self._lock:
                for drone in self._drones.values():
                    p_ref, v_ref, _, _, _, _ = drone.follower.advance(dt)
                    drone.position = p_ref.copy()

                    # battery drain while flying
                    if drone.mode not in ("idle", "hover"):
                        drone.battery_pct = max(
                            0.0, drone.battery_pct - drone.battery_drain_rate * dt
                        )

                    # scan coverage progress
                    if drone.mode == "scan" and drone.scan_total_segs > 0:
                        seg_idx = 0
                        cum = 0.0
                        for seg in drone.follower.segments:
                            if drone.follower.elapsed_s > cum:
                                seg_idx += 1
                            cum += seg.T
                        drone.scan_passed_segs = min(seg_idx, drone.scan_total_segs)
                        drone.coverage_pct = 100.0 * drone.scan_passed_segs / drone.scan_total_segs

                    if drone.follower.finished and drone.mode not in ("idle", "hover"):
                        if drone.mode == "scan":
                            drone.coverage_pct = 100.0
                        drone.follower.hover_at(drone.position)
                        drone.mode = "hover"
                    self._emit(drone, v_ref)
            time.sleep(dt)

    def _emit(self, drone: DroneSim, velocity: np.ndarray) -> None:
        speed = float(np.linalg.norm(velocity))
        payload = {
            "drone_id": drone.drone_id,
            "mode": drone.mode,
            "position": drone.position.tolist(),
            "velocity": velocity.tolist(),
            "yaw": 0.0,
            "battery_pct": round(drone.battery_pct, 2),
            "coverage_pct": round(drone.coverage_pct, 1),
            "trajectory_finished": bool(drone.follower.finished),
            "trajectory_total_s": float(drone.follower.total_duration_s),
            "trajectory_elapsed_s": float(drone.follower.elapsed_s),
            "ts": time.time(),
        }
        self._publisher.publish_state(drone.drone_id, payload)
        self._publisher.publish_telemetry(
            drone.drone_id,
            {
                "x": float(drone.position[0]),
                "y": float(drone.position[1]),
                "z": float(drone.position[2]),
                "vx": float(velocity[0]),
                "vy": float(velocity[1]),
                "vz": float(velocity[2]),
                "speed": speed,
                "battery_pct": round(drone.battery_pct, 2),
                "coverage_pct": round(drone.coverage_pct, 1),
                "finished": bool(drone.follower.finished),
            },
        )


def main() -> int:
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    sim = SimSettings()
    swarm = MockSwarm(num_drones=sim.num_drones)
    try:
        swarm.start()
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\n[mock_swarm] stopped")
    finally:
        swarm.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
