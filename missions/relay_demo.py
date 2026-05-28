#!/usr/bin/env python3
"""Battery-relay demo: uav-01 scans the left aisle, hands off to uav-02
when battery drops below the threshold; uav-02 flies to the exact world
pose of uav-01 and completes the scan.

All positions are explicit world (X, Y, Z) coordinates in metres.

World frame (Isaac Sim / Full Warehouse scene)
----------------------------------------------
  X  — left (-) / right (+)
  Y  — near end of warehouse (-) / far end (+)
  Z  — floor (0) / ceiling (~4 m)

  Left aisle centre:  X = -3.25 m
  Aisle near end:     Y = -5.50 m
  Aisle far end:      Y = +5.50 m
  Scan altitude:      Z =  3.00 m  (above 2 m rack tops)

  uav-01 spawn: (-3.25, -5.50, 0.10)  — left aisle entry
  uav-02 spawn: (5.25, -5.50, 0.10)  — 2 m to the LEFT of uav-01 (relay standby)

Relay timing (with SIM_BATTERY_DRAIN_RATE_PPS=2.0):
  battery 100% → 60% threshold in ~20 s of flight
  → handoff at approx. Y = 0.0 m  (midpoint of aisle)
  uav-02 intercepts at full relay speed (1.0 m/s) in ~15 s
  uav-02 completes remaining ~5.5 m scan

Usage:
    # terminal 1 — fast battery drain
    SIM_BATTERY_DRAIN_RATE_PPS=2.0 python -m swarm.mock_swarm

    # terminal 2
    python missions/relay_demo.py

    # optional Grafana dashboard (needs make up-full)
    make grafana
"""
from __future__ import annotations

import os
import signal
import sys
import time
from pathlib import Path

_ROOT = Path(__file__).resolve().parent.parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from swarm.config import MqttSettings, SimSettings
from swarm.relay import RelayMission

# ── Tuneable parameters ───────────────────────────────────────────────────
# World coordinates for the left aisle (Full Warehouse scene).
# Adjust to match your exact scene if needed.

CX        = -3.25   # aisle centre-line  X (metres, world frame)
Y_NEAR    = 5.50   # spawn end  (near, left side of scene)
Y_FAR     =  35.50   # far end    (right side — drones fly RIGHT → positive Y)
ALTITUDE  =  3.00   # scan altitude       Z (metres, above floor)

DRONE_A   = "uav-01"   # primary scanner
DRONE_B   = "uav-02"   # relay / relief drone

# Battery % that triggers handoff.
# Override via SIM_BATTERY_RELAY_THRESHOLD_PCT env var.
THRESHOLD = float(os.environ.get("SIM_BATTERY_RELAY_THRESHOLD_PCT", "30"))

# Speed for drone_b's intercept leg (may be faster than normal scan speed).
RELAY_SPEED = 1.0   # m/s
# ─────────────────────────────────────────────────────────────────────────


def _step(msg: str) -> None:
    ts = time.strftime("%H:%M:%S")
    print(f"[{ts}] {msg}")


def main() -> int:
    sim    = SimSettings()
    mqtt   = MqttSettings()

    mission = RelayMission(
        drone_a=DRONE_A,
        drone_b=DRONE_B,
        cx=CX,
        y_start=Y_NEAR,
        y_end=Y_FAR,
        altitude=ALTITUDE,
        threshold_pct=THRESHOLD,
        relay_speed_mps=RELAY_SPEED,
        mqtt_settings=mqtt,
        sim_settings=sim,
        on_step=_step,
    )

    def _abort(sig, frame):
        print("\n[relay_demo] Ctrl-C — stop all")
        from swarm.commands import HoverCmd
        from swarm.mqtt_cmd import publish_command
        publish_command("all", HoverCmd(cmd="stop"), settings=mqtt)
        sys.exit(130)

    signal.signal(signal.SIGINT, _abort)

    drain = float(sim.battery_drain_rate_pps)
    _step(f"Battery drain rate : {drain:.3f} %/s")
    _step(f"Handoff threshold  : {THRESHOLD:.0f}%")
    if drain < 0.5:
        _step(
            "TIP: drain rate is very low — relay may not trigger during the demo.\n"
            "     Run with:  SIM_BATTERY_DRAIN_RATE_PPS=2.0 python -m swarm.mock_swarm"
        )
    _step("")

    try:
        mission.run()
    except ConnectionError as exc:
        print(f"[relay_demo] {exc}", file=sys.stderr)
        return 2

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
