"""Commanded nonlinear controller backend for Pegasus / Isaac Sim.

This module subclasses the *nonlinear trajectory-tracking controller* shipped
with Pegasus Simulator (``examples/utils/nonlinear_controller.py``) and
replaces only its **reference source** — instead of reading a CSV trajectory
or evaluating a hard-coded analytic curve, it pulls the reference from a
:class:`swarm.trajectory.TrajectoryFollower` whose contents are filled by
operator commands arriving over MQTT.

The control law itself (outer-loop position PD + integral, inner-loop
SO(3) attitude PD with body-frame torque allocation) is **left untouched**
inside the Pegasus parent class. It implements the trajectory-tracking
controller of:

    [25] Mellinger, D. and Kumar, V. (2011). "Minimum snap trajectory
         generation and control for quadrotors", *2011 IEEE ICRA*,
         pp. 2520–2525. doi:10.1109/ICRA.2011.5980409.
    [24] Pinto, J., Guerreiro, B. J. and Cunha, R. (2021). "Planning
         Parcel Relay Manoeuvres for Quadrotors", *2021 ICUAS*,
         pp. 137–145. doi:10.1109/ICUAS51884.2021.9476757.

For the canonical SE(3) trajectory-tracking formulation that this
controller is based on see also:

    [9]  Lee, T., Leok, M. and McClamroch, N. H. (2010). "Geometric
         Tracking Control of a Quadrotor UAV on SE(3)", *49th IEEE CDC*,
         pp. 5420–5425. arXiv:1003.2005.

The runtime is supplied by:

    [22] Jacinto, M. *et al.* (2024). "Pegasus Simulator: An Isaac Sim
         Framework for Multiple Aerial Vehicles Simulation", *ICUAS 2024*.
         arXiv:2404.15923.

We do not implement any localisation in this project. The state used by
the controller is the ground-truth ``State`` provided by Pegasus' physics
backend; in a real deployment this is the slot to plug in a
visual-inertial estimator such as ORB-SLAM3 [7] or VINS-Fusion [8].
"""
from __future__ import annotations

import json
import logging
import threading
import time
from typing import Any

import numpy as np

from .commands import (
    Command,
    GoToCmd,
    HoverCmd,
    InspectPoiCmd,
    LandCmd,
    OrbitCmd,
    ScanCmd,
    TakeoffCmd,
    parse_command,
)
from .config import MqttSettings, SimSettings
from .coverage import Rectangle, boustrophedon_path
from .mqtt_publisher import MqttPublisher
from .trajectory import (
    TrajectoryFollower,
    build_chained_waypoints,
    build_orbit,
    build_point_to_point,
)

log = logging.getLogger(__name__)


class _BackendStub:
    """Compile-time placeholder used outside Isaac Sim (unit tests)."""


try:
    from pegasus.simulator.logic.state import State as PegasusState  # type: ignore
    PEGASUS_AVAILABLE = True
except Exception:
    PegasusState = object  # type: ignore[misc, assignment]
    PEGASUS_AVAILABLE = False


def _resolve_parent_controller():
    """Locate Pegasus' ``NonlinearController`` example class.

    Different Pegasus checkouts ship the controller in slightly different
    locations; we try each in turn so the same code path works on a
    developer machine and inside Isaac Sim's bundled Python.
    """
    candidates = (
        "utils.nonlinear_controller",
        "nonlinear_controller",
        "controllers.nonlinear_controller",
        "pegasus.simulator.logic.backends.controller.nonlinear_controller",
        "pegasus.simulator.logic.backends.tools.controllers.nonlinear_controller",
    )
    for module_path in candidates:
        try:
            mod = __import__(module_path, fromlist=["NonlinearController"])
            cls = getattr(mod, "NonlinearController", None)
            if cls is not None:
                log.info("loaded Pegasus NonlinearController from %s", module_path)
                return cls
        except Exception:
            continue
    return None


_NonlinearController = _resolve_parent_controller()


class _ControllerBase(_NonlinearController if _NonlinearController is not None else _BackendStub):  # type: ignore[misc]
    """Internal helper: bind to Pegasus parent if available, stub otherwise."""


class CommandedNonlinearController(_ControllerBase):
    """Pegasus controller backend driven by MQTT operator commands.

    One instance per drone. Behaviour:

    1. Subscribes to ``{prefix}/cmd/{drone_id}`` and ``{prefix}/cmd/all``
       and translates each incoming command into a fresh
       :class:`TrajectoryFollower` reference stream.
    2. On every physics step, replaces the parent's CSV/built-in lookup
       with a sample from that follower (position, velocity, acceleration,
       jerk, yaw, yaw-rate). The parent's tracking control law (Mellinger
       & Kumar 2011 [25]; Pinto et al. 2021 [24]) is invoked unchanged.
    3. Publishes the drone's current state at ``state_publish_hz`` to
       ``{prefix}/state/{drone_id}`` and InfluxDB-style telemetry to
       ``{prefix}/telemetry/{drone_id}``.
    """

    def __init__(
        self,
        drone_id: str,
        *,
        mqtt_settings: MqttSettings | None = None,
        sim_settings: SimSettings | None = None,
        Kp: list[float] | None = None,
        Kd: list[float] | None = None,
        Ki: list[float] | None = None,
        Kr: list[float] | None = None,
        Kw: list[float] | None = None,
        state_publish_hz: float = 5.0,
    ) -> None:
        if _NonlinearController is None:
            raise RuntimeError(
                "Pegasus NonlinearController not importable — run inside Isaac Sim with PEGASUS_PATH set."
            )

        super().__init__(
            trajectory_file=None,
            results_file=None,
            Kp=Kp or [10.0, 10.0, 10.0],
            Kd=Kd or [8.5, 8.5, 8.5],
            Ki=Ki or [1.5, 1.5, 1.5],
            Kr=Kr or [3.5, 3.5, 3.5],
            Kw=Kw or [0.5, 0.5, 0.5],
        )

        self.drone_id = drone_id
        self.sim = sim_settings or SimSettings()
        self.mqtt_settings = mqtt_settings or MqttSettings()

        self._lock = threading.Lock()
        self._follower = TrajectoryFollower(segments=[], name="idle", finished=True)
        self._mode: str = "idle"
        self._active_cmd_seq: int | None = None
        self._initial_pos = np.zeros(3)
        self._initial_pos_set = False

        # battery simulation — rate from config so relay demo can set it fast
        self._battery_pct: float = 100.0
        self._battery_drain_rate: float = float(self.sim.battery_drain_rate_pps)

        # scan coverage tracking
        self._coverage_pct: float = 0.0
        self._scan_total_segs: int = 0
        self._scan_passed_segs: int = 0

        # camera presence flag — set to True by swarm_app.py after attaching Camera prim
        self._camera_active: bool = False

        self._publisher = MqttPublisher(self.mqtt_settings, client_id=f"drone-{drone_id}")
        self._publisher.connect()
        self._publisher.subscribe(
            f"{self._publisher.topic_prefix}/cmd/{drone_id}", self._on_command, qos=1
        )
        self._publisher.subscribe(
            f"{self._publisher.topic_prefix}/cmd/all", self._on_command, qos=1
        )

        self._state_publish_period_s = 1.0 / max(0.1, state_publish_hz)
        self._last_state_publish = 0.0
        self._publisher.publish_event(
            "drone_online",
            {"drone_id": drone_id, "ts": time.time()},
        )

    # -----------------------------------------------------------------
    # Pegasus Backend overrides
    # -----------------------------------------------------------------
    def update_state(self, state: "PegasusState") -> None:  # type: ignore[override]
        super().update_state(state)
        if not self._initial_pos_set:
            self._initial_pos = np.array(self.p, dtype=float).copy()
            self._initial_pos_set = True
            with self._lock:
                self._follower.hover_at(self._initial_pos)
                self._mode = "idle"

    def update(self, dt: float) -> None:  # type: ignore[override]
        if not self.reveived_first_state:  # parent attribute (sic - upstream typo)
            return

        with self._lock:
            p_ref, v_ref, a_ref, j_ref, yaw_ref, yaw_rate_ref = self._follower.advance(dt)
            mode = self._mode
            finished = self._follower.finished
            traj_name = self._follower.name

            # battery drain while flying
            if mode not in ("idle", "hover"):
                self._battery_pct = max(0.0, self._battery_pct - self._battery_drain_rate * dt)

            # coverage progress for scan mode
            if mode == "scan" and self._scan_total_segs > 0:
                seg_idx = 0
                cum = 0.0
                for seg in self._follower.segments:
                    if self._follower.elapsed_s > cum:
                        seg_idx += 1
                    cum += seg.T
                self._scan_passed_segs = min(seg_idx, self._scan_total_segs)
                self._coverage_pct = 100.0 * self._scan_passed_segs / self._scan_total_segs

        self._apply_reference_and_step(
            dt, p_ref, v_ref, a_ref, j_ref, yaw_ref, yaw_rate_ref
        )

        if mode != "idle" and finished and traj_name not in ("hover", "idle"):
            with self._lock:
                self._follower.hover_at(np.array(self.p, dtype=float), yaw=yaw_ref)
                self._mode = "idle"
                if traj_name == "scan":
                    self._coverage_pct = 100.0
            self._publisher.publish_event(
                "trajectory_finished",
                {"drone_id": self.drone_id, "previous_mode": mode, "ts": time.time()},
            )

        now = time.monotonic()
        if now - self._last_state_publish >= self._state_publish_period_s:
            self._last_state_publish = now
            self._emit_state(traj_name)

    def reset(self) -> None:  # type: ignore[override]
        super().reset()
        with self._lock:
            self._follower = TrajectoryFollower(segments=[], name="idle", finished=True)
            self._mode = "idle"
            self._initial_pos_set = False

    def stop(self) -> None:  # type: ignore[override]
        try:
            self._publisher.publish_event(
                "drone_offline", {"drone_id": self.drone_id, "ts": time.time()}
            )
            self._publisher.disconnect()
        except Exception:
            pass
        try:
            super().stop()
        except Exception:
            pass

    # -----------------------------------------------------------------
    # Internal: replace the parent's CSV/built-in reference
    # -----------------------------------------------------------------
    def _apply_reference_and_step(
        self,
        dt: float,
        p_ref: np.ndarray,
        v_ref: np.ndarray,
        a_ref: np.ndarray,
        j_ref: np.ndarray,
        yaw_ref: float,
        yaw_rate_ref: float,
    ) -> None:
        """Run the parent's Mellinger/Pinto control law on our reference.

        We reuse the exact body of ``NonlinearController.update`` from
        Pegasus (PD on position, integral, SO(3) attitude PD, allocation
        through ``vehicle.force_and_torques_to_velocities``); only the
        reference is supplied externally instead of being looked up from a
        CSV row. See [25, eq. 11–24] for the underlying derivation.
        """
        from scipy.spatial.transform import Rotation as _Rotation  # noqa: WPS433

        ep = self.p - p_ref
        ev = self.v - v_ref
        self.int = self.int + (ep * dt)
        ei = self.int

        F_des = (
            -(self.Kp @ ep)
            - (self.Kd @ ev)
            - (self.Ki @ ei)
            + np.array([0.0, 0.0, self.m * self.g])
            + (self.m * a_ref)
        )

        Z_B = self.R.as_matrix()[:, 2]
        u_1 = float(F_des @ Z_B)

        norm_F = float(np.linalg.norm(F_des))
        if norm_F < 1e-6:
            return
        Z_b_des = F_des / norm_F

        X_c_des = np.array([np.cos(yaw_ref), np.sin(yaw_ref), 0.0])
        Z_b_cross_X_c = np.cross(Z_b_des, X_c_des)
        cross_norm = float(np.linalg.norm(Z_b_cross_X_c))
        if cross_norm < 1e-6:
            return
        Y_b_des = Z_b_cross_X_c / cross_norm
        X_b_des = np.cross(Y_b_des, Z_b_des)

        R_des = np.c_[X_b_des, Y_b_des, Z_b_des]
        R = self.R.as_matrix()

        e_R = 0.5 * self.vee((R_des.T @ R) - (R.T @ R_des))

        if abs(u_1) < 1e-6:
            self.a = np.zeros(3)
        else:
            self.a = (u_1 * Z_B) / self.m - np.array([0.0, 0.0, self.g])
            hw = (self.m / u_1) * (j_ref - np.dot(Z_b_des, j_ref) * Z_b_des)
            w_des = np.array(
                [-np.dot(hw, Y_b_des), np.dot(hw, X_b_des), yaw_rate_ref * Z_b_des[2]]
            )
            e_w = self.w - w_des
            tau = -(self.Kr @ e_R) - (self.Kw @ e_w)
            if getattr(self, "vehicle", None):
                self.input_ref = self.vehicle.force_and_torques_to_velocities(u_1, tau)

        # Pegasus' upstream class also keeps statistics; we skip them since
        # they are not needed for the operator-driven flow.
        _ = _Rotation  # quiet unused-import linters when scipy is preloaded

    # -----------------------------------------------------------------
    # Operator command channel
    # -----------------------------------------------------------------
    def _on_command(self, topic: str, payload: bytes) -> None:
        try:
            cmd = parse_command(payload)
        except Exception as exc:
            log.warning("[%s] dropping malformed command on %s: %s", self.drone_id, topic, exc)
            self._publisher.publish_event(
                "command_rejected",
                {"drone_id": self.drone_id, "reason": str(exc), "topic": topic},
            )
            return

        if not self._initial_pos_set:
            log.info("[%s] command queued — waiting for first state tick", self.drone_id)

        try:
            with self._lock:
                self._dispatch(cmd)
            self._publisher.publish_event(
                "command_accepted",
                {
                    "drone_id": self.drone_id,
                    "cmd": cmd.cmd,
                    "seq": cmd.seq,
                    "topic": topic,
                    "ts": time.time(),
                },
            )
        except Exception as exc:
            log.exception("[%s] command %s failed", self.drone_id, cmd.cmd)
            self._publisher.publish_event(
                "command_failed",
                {"drone_id": self.drone_id, "cmd": cmd.cmd, "reason": str(exc)},
            )

    def _dispatch(self, cmd: Command) -> None:
        cur_pos = (
            np.array(self.p, dtype=float) if self._initial_pos_set else self._initial_pos.copy()
        )
        default_cruise = float(self.sim.cruise_speed_mps)

        if isinstance(cmd, TakeoffCmd):
            target = np.array([cur_pos[0], cur_pos[1], cmd.altitude_m], dtype=float)
            cruise = float(cmd.cruise_speed_mps or default_cruise)
            segs = build_point_to_point(cur_pos, target, cruise_speed_mps=cruise)
            self._follower.reset(segs, name="takeoff")
            self._mode = "takeoff"
            self._active_cmd_seq = cmd.seq
            return

        if isinstance(cmd, GoToCmd):
            target = np.array([cmd.x, cmd.y, cmd.z], dtype=float)
            cruise = float(cmd.cruise_speed_mps or default_cruise)
            segs = build_point_to_point(
                cur_pos, target, cruise_speed_mps=cruise,
                yaw_start=cmd.yaw, yaw_end=cmd.yaw,
            )
            self._follower.reset(segs, name="goto")
            self._mode = "goto"
            self._active_cmd_seq = cmd.seq
            return

        if isinstance(cmd, OrbitCmd):
            segs = build_orbit(
                center=np.array([cmd.center_x, cmd.center_y, cmd.altitude_m]),
                radius_m=cmd.radius_m,
                altitude_m=cmd.altitude_m,
                speed_mps=cmd.speed_mps,
                revolutions=cmd.revolutions,
            )
            self._follower.reset(segs, name="orbit")
            self._mode = "orbit"
            self._active_cmd_seq = cmd.seq
            return

        if isinstance(cmd, InspectPoiCmd):
            from .facility import default_pois
            pois = default_pois()
            if cmd.poi_id not in pois:
                raise ValueError(f"unknown POI: {cmd.poi_id!r}. Known: {list(pois)}")
            poi = pois[cmd.poi_id]
            poi_pos = np.array([poi.x, poi.y, poi.z], dtype=float)
            goto_segs = build_point_to_point(
                cur_pos, poi_pos, cruise_speed_mps=default_cruise
            )
            # dwell: hover at POI for dwell_s by a zero-length hover segment
            hover_seg_s = max(1.0, float(cmd.dwell_s))
            from .trajectory import QuinticSegment
            hover_seg = QuinticSegment.build(poi_pos, poi_pos, T=hover_seg_s)
            orbit_segs = build_orbit(
                center=poi_pos,
                radius_m=poi.orbit_radius_m,
                altitude_m=poi.z,
                speed_mps=default_cruise,
                revolutions=1.0,
            )
            self._follower.reset(goto_segs + [hover_seg] + orbit_segs, name="inspect_poi")
            self._mode = "inspect_poi"
            self._active_cmd_seq = cmd.seq
            return

        if isinstance(cmd, ScanCmd):
            cruise = float(cmd.cruise_speed_mps or default_cruise)
            region = Rectangle(cmd.x_min, cmd.x_max, cmd.y_min, cmd.y_max)
            wps = boustrophedon_path(
                region,
                altitude_m=cmd.altitude_m,
                sweep_spacing_m=cmd.sweep_spacing_m,
                sweep_axis=cmd.sweep_axis,
            )
            entry = wps[0]
            chained = [cur_pos] + wps if np.linalg.norm(entry - cur_pos) > 1e-3 else wps
            segs = build_chained_waypoints(chained, cruise_speed_mps=cruise)
            self._follower.reset(segs, name="scan")
            self._mode = "scan"
            self._active_cmd_seq = cmd.seq
            self._coverage_pct = 0.0
            self._scan_total_segs = len(segs)
            self._scan_passed_segs = 0
            return

        if isinstance(cmd, HoverCmd):
            self._follower.hover_at(cur_pos)
            self._mode = "hover"
            self._active_cmd_seq = cmd.seq
            return

        if isinstance(cmd, LandCmd):
            target = np.array([cur_pos[0], cur_pos[1], 0.05], dtype=float)
            cruise = float(cmd.cruise_speed_mps or default_cruise)
            segs = build_point_to_point(cur_pos, target, cruise_speed_mps=cruise)
            self._follower.reset(segs, name="land")
            self._mode = "land"
            self._active_cmd_seq = cmd.seq
            return

        raise ValueError(f"unsupported command type: {type(cmd).__name__}")

    # -----------------------------------------------------------------
    # State publishing
    # -----------------------------------------------------------------
    def _emit_state(self, mode: str) -> None:
        try:
            yaw = float(self.R.as_euler("zyx", degrees=False)[0])
        except Exception:
            yaw = 0.0

        state_payload: dict[str, Any] = {
            "drone_id": self.drone_id,
            "mode": mode,
            "seq": self._active_cmd_seq,
            "position": [float(self.p[0]), float(self.p[1]), float(self.p[2])],
            "velocity": [float(self.v[0]), float(self.v[1]), float(self.v[2])],
            "yaw": yaw,
            "battery_pct": round(self._battery_pct, 2),
            "coverage_pct": round(self._coverage_pct, 1),
            "trajectory_finished": bool(self._follower.finished),
            "trajectory_total_s": float(self._follower.total_duration_s),
            "trajectory_elapsed_s": float(self._follower.elapsed_s),
            "ts": time.time(),
        }
        self._publisher.publish_state(self.drone_id, state_payload)

        self._publisher.publish_telemetry(
            self.drone_id,
            {
                "x": float(self.p[0]),
                "y": float(self.p[1]),
                "z": float(self.p[2]),
                "vx": float(self.v[0]),
                "vy": float(self.v[1]),
                "vz": float(self.v[2]),
                "speed": float(np.linalg.norm(self.v)),
                "yaw": yaw,
                "battery_pct": round(self._battery_pct, 2),
                "coverage_pct": round(self._coverage_pct, 1),
                "camera_active": int(self._camera_active),
                "elapsed_s": float(self._follower.elapsed_s),
                "total_s": float(self._follower.total_duration_s),
                "finished": bool(self._follower.finished),
            },
        )
        _ = json
