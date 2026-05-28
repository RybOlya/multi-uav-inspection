"""Operator CLI — send MQTT commands to the swarm.

Quick-start::

    python missions/relay_demo.py       # full 2-drone relay mission

Individual commands::

    python -m swarm.operator takeoff --all --altitude 3.0
    python -m swarm.operator inspect-poi uav-01 rack_left_end
    python -m swarm.operator orbit uav-01 --cx -3.25 --cy 0 --altitude 3.0 --radius 1.5
    python -m swarm.operator scan-zone aisle_left
    python -m swarm.operator scan-zone aisle_right
    python -m swarm.operator stop --all
    python -m swarm.operator land --all
"""
from __future__ import annotations

import argparse
import sys

from pydantic import ValidationError

from .commands import (
    GoToCmd,
    HoverCmd,
    InspectPoiCmd,
    LandCmd,
    OrbitCmd,
    ScanCmd,
    TakeoffCmd,
    encode_command,
)
from .config import MqttSettings, SimSettings
from .facility import default_pois, default_zones
from .mqtt_cmd import publish_command


def _publish_cmd(target: str, cmd) -> int:
    payload = encode_command(cmd)
    ok = publish_command(target, cmd, wait_s=0.2)
    if not ok:
        print("[operator] MQTT broker unreachable — run: make up", file=sys.stderr)
        return 2
    print(f"[operator] -> {target} {payload}")
    return 0


def _resolve_target(args: argparse.Namespace) -> str:
    if getattr(args, "all", False):
        return "all"
    drone = getattr(args, "drone", None)
    if not drone:
        raise SystemExit("must pass either DRONE_ID or --all")
    return drone


def _common_args(p: argparse.ArgumentParser, *, with_drone: bool = True) -> None:
    if with_drone:
        p.add_argument("drone", nargs="?", help="drone id, e.g. uav-01")
        p.add_argument("--all", action="store_true", help="broadcast to every drone")
    p.add_argument("--seq", type=int, default=None)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="swarm-operator",
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    sub = parser.add_subparsers(dest="cmd", required=True)

    zone_names = ", ".join(default_zones().keys())
    poi_names = ", ".join(default_pois().keys())

    p_zone = sub.add_parser("scan-zone", help=f"scan named aisle ({zone_names})")
    p_zone.add_argument("zone", choices=list(default_zones().keys()))
    p_zone.add_argument("drone", nargs="?", help="override drone id")

    p_poi = sub.add_parser(
        "inspect-poi", help=f"goto POI + hover + orbit ({poi_names})"
    )
    p_poi.add_argument("drone", nargs="?", help="drone id, e.g. uav-01")
    p_poi.add_argument("--all", action="store_true")
    p_poi.add_argument("poi_id", choices=list(default_pois().keys()))
    p_poi.add_argument("--dwell", type=float, default=5.0, help="hover seconds at marker")
    p_poi.add_argument("--seq", type=int, default=None)

    p_orbit = sub.add_parser("orbit", help="fly circular orbit around (cx, cy)")
    _common_args(p_orbit)
    p_orbit.add_argument("--cx", type=float, required=True, help="centre X")
    p_orbit.add_argument("--cy", type=float, required=True, help="centre Y")
    p_orbit.add_argument("--altitude", type=float, required=True)
    p_orbit.add_argument("--radius", type=float, default=2.0)
    p_orbit.add_argument("--speed", type=float, default=0.5)
    p_orbit.add_argument("--revolutions", type=float, default=1.0)

    p_take = sub.add_parser("takeoff", help="climb vertically")
    _common_args(p_take)
    p_take.add_argument("--altitude", type=float, default=None)

    p_goto = sub.add_parser("goto", help="fly to (x, y, z)")
    _common_args(p_goto)
    p_goto.add_argument("--x", type=float, required=True)
    p_goto.add_argument("--y", type=float, required=True)
    p_goto.add_argument("--z", type=float, required=True)
    p_goto.add_argument("--yaw", type=float, default=0.0)

    p_scan = sub.add_parser("scan", help="raw rectangle scan — advanced")
    _common_args(p_scan)
    p_scan.add_argument("--xmin", type=float, required=True)
    p_scan.add_argument("--xmax", type=float, required=True)
    p_scan.add_argument("--ymin", type=float, required=True)
    p_scan.add_argument("--ymax", type=float, required=True)
    p_scan.add_argument("--altitude", type=float, required=True)
    p_scan.add_argument("--spacing", type=float, default=2.0)
    p_scan.add_argument("--axis", choices=["x", "y"], default="x")

    p_hov = sub.add_parser("hover", help="hold current pose")
    _common_args(p_hov)

    p_stop = sub.add_parser("stop", help="abort trajectory and hover")
    _common_args(p_stop)

    p_land = sub.add_parser("land", help="descend to floor")
    _common_args(p_land)

    return parser


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    sim = SimSettings()

    # ---- scan-zone -------------------------------------------------------
    if args.cmd == "scan-zone":
        zones = default_zones(sim)
        zone = zones[args.zone]
        target = args.drone or zone.drone_id
        ok = publish_command(target, zone.to_scan_cmd(), wait_s=0.2)
        if not ok:
            print("[operator] MQTT broker unreachable — run: make up", file=sys.stderr)
            return 2
        print(f"[operator] -> {target} {encode_command(zone.to_scan_cmd())}")
        return 0

    # ---- inspect-poi -----------------------------------------------------
    if args.cmd == "inspect-poi":
        target = "all" if getattr(args, "all", False) else args.drone
        if not target:
            raise SystemExit("must pass DRONE_ID or --all")
        cmd = InspectPoiCmd(
            cmd="inspect_poi", poi_id=args.poi_id, dwell_s=args.dwell, seq=args.seq
        )
        return _publish_cmd(target, cmd)

    # ---- orbit -----------------------------------------------------------
    if args.cmd == "orbit":
        target = _resolve_target(args)
        cmd = OrbitCmd(
            cmd="orbit",
            center_x=args.cx,
            center_y=args.cy,
            altitude_m=args.altitude,
            radius_m=args.radius,
            speed_mps=args.speed,
            revolutions=args.revolutions,
            seq=args.seq,
        )
        return _publish_cmd(target, cmd)

    # ---- simple commands ------------------------------------------------
    target = _resolve_target(args)

    if args.cmd == "takeoff":
        alt = args.altitude if args.altitude is not None else float(sim.takeoff_altitude_m)
        cmd = TakeoffCmd(cmd="takeoff", altitude_m=alt, seq=args.seq)

    elif args.cmd == "goto":
        cmd = GoToCmd(
            cmd="goto", x=args.x, y=args.y, z=args.z, yaw=args.yaw, seq=args.seq
        )

    elif args.cmd == "scan":
        try:
            cmd = ScanCmd(
                cmd="scan",
                x_min=args.xmin, x_max=args.xmax,
                y_min=args.ymin, y_max=args.ymax,
                altitude_m=args.altitude,
                sweep_spacing_m=args.spacing,
                sweep_axis=args.axis,
                seq=args.seq,
            )
        except ValidationError as exc:
            print("[operator] invalid scan:", file=sys.stderr)
            print(exc, file=sys.stderr)
            if args.xmax <= args.xmin:
                print(
                    f"Hint: xmax must be > xmin. "
                    f"Try: python -m swarm.operator scan-zone aisle_left",
                    file=sys.stderr,
                )
            return 1

    elif args.cmd in ("hover", "stop"):
        cmd = HoverCmd(cmd=args.cmd, seq=args.seq)

    elif args.cmd == "land":
        cmd = LandCmd(cmd="land", seq=args.seq)

    else:
        raise SystemExit(f"unknown command: {args.cmd}")

    return _publish_cmd(target, cmd)


if __name__ == "__main__":
    raise SystemExit(main())
