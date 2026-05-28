"""Pegasus + Isaac Sim entry point — operator-driven UAV swarm.

Spawns ``SIM_NUM_DRONES`` quadrotors inside the chosen Isaac Sim scene,
attaches a :class:`swarm.controller.CommandedNonlinearController` to each,
and runs the simulation loop. Every drone is idle (hovering at spawn) until
an operator publishes commands on ``swarm/cmd/<drone_id>``.

Architecture (everything below is literature-grounded):

    Operator CLI  --MQTT v5 cmd--> CommandedNonlinearController
                                     │
                                     ├─ TrajectoryFollower
                                     │  (quintic time scaling per
                                     │   Lynch & Park 2017 [26];
                                     │   Boustrophedon coverage per
                                     │   Choset 2000 [4])
                                     │
                                     └─ Pegasus NonlinearController
                                        (Mellinger & Kumar 2011 [25];
                                         Pinto, Guerreiro, Cunha 2021 [24])
                                          │
                                          ▼
                                        Pegasus Multirotor in Isaac Sim
                                        (Jacinto et al. 2024 [22])

Run inside Isaac Sim's bundled Python (NOT host python):

    cd ~/isaac-sim
    PEGASUS_PATH=~/PegasusSimulator ./python.sh \\
        /path/to/bridge-swarm-iot/isaac/swarm_app.py
"""
from __future__ import annotations

import os
import sys
import time
from pathlib import Path

# ---------------------------------------------------------------------------
# 1. Boot SimulationApp BEFORE any other omni.* / isaac.* import.
# ---------------------------------------------------------------------------
HEADLESS = bool(int(os.environ.get("HEADLESS", "0")))
RENDER = not HEADLESS

try:
    from omni.isaac.kit import SimulationApp  # noqa: E402

    simulation_app = SimulationApp({"headless": HEADLESS, "renderer": "RayTracedLighting"})
except ImportError as exc:
    sys.stderr.write(
        "[swarm_app] omni.isaac.kit not available — run with Isaac Sim's bundled Python.\n"
        f"Underlying error: {exc}\n"
    )
    sys.exit(1)

THIS_DIR = Path(__file__).resolve().parent
REPO_ROOT = THIS_DIR.parent
os.environ.setdefault("SWARM_IOT_ROOT", str(REPO_ROOT))
if (REPO_ROOT / "swarm").is_dir() and str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

# ---------------------------------------------------------------------------
# 2. Locate and enable the Pegasus extension (so its examples/utils path is
#    on sys.path before we import the controller subclass).
# ---------------------------------------------------------------------------
import omni.kit.app  # noqa: E402
import omni.timeline  # noqa: E402
from omni.isaac.core.world import World  # noqa: E402


_PEGASUS_ROOT = os.environ.get("PEGASUS_PATH", str(Path.home() / "PegasusSimulator"))


def _enable_pegasus_extension() -> None:
    candidates = [
        Path(_PEGASUS_ROOT) / "extensions",
        Path.home() / "PegasusSimulator" / "extensions",
        Path.home() / "pegasus_simulator" / "extensions",
        Path("/opt/PegasusSimulator/extensions"),
    ]
    manager = omni.kit.app.get_app().get_extension_manager()
    for cand in candidates:
        if cand.is_dir():
            print(f"[swarm_app] registering Pegasus ext path: {cand}")
            manager.add_path(str(cand))
    try:
        manager.set_extension_enabled_immediate("pegasus.simulator", True)
    except Exception as exc:
        print(f"[swarm_app] could not enable pegasus.simulator: {exc}")


_enable_pegasus_extension()

for cand in (
    Path(_PEGASUS_ROOT) / "examples",
    Path(_PEGASUS_ROOT) / "examples" / "utils",
):
    if cand.is_dir() and str(cand) not in sys.path:
        sys.path.insert(0, str(cand))

try:
    from pegasus.simulator.params import ROBOTS  # noqa: E402
    from pegasus.simulator.logic.vehicles.multirotor import (  # noqa: E402
        Multirotor,
        MultirotorConfig,
    )
    from pegasus.simulator.logic.interface.pegasus_interface import (  # noqa: E402
        PegasusInterface,
    )
except ImportError as exc:
    sys.stderr.write(
        "\n[swarm_app] Pegasus Simulator not found.\n"
        "  git clone https://github.com/PegasusSimulator/PegasusSimulator.git ~/PegasusSimulator\n"
        "  Then re-launch (set PEGASUS_PATH=~/PegasusSimulator if you cloned elsewhere).\n"
        f"\nUnderlying error: {exc}\n"
    )
    simulation_app.close()
    sys.exit(1)

import numpy as np  # noqa: E402

from isaac.scene_builder import build_sim_environment  # noqa: E402
from swarm.config import SimSettings  # noqa: E402
from swarm.controller import CommandedNonlinearController  # noqa: E402

try:
    from omni.isaac.sensor import Camera as IsaacCamera  # noqa: E402
    _CAMERA_AVAILABLE = True
except ImportError:
    _CAMERA_AVAILABLE = False


def _attach_nadir_camera(stage_prefix: str, drone_idx: int) -> bool:
    """Attach a nadir-facing Camera prim to a spawned drone body.

    Returns True on success, False if the sensor extension is unavailable.
    """
    if not _CAMERA_AVAILABLE:
        return False
    try:
        cam = IsaacCamera(
            prim_path=f"{stage_prefix}/body/InspectionCamera",
            translation=np.array([0.0, 0.0, -0.05]),
            orientation=np.array([0.0, 0.0, 0.0, 1.0]),
            resolution=(320, 240),
        )
        cam.initialize()
        print(f"[swarm_app] camera attached to drone {drone_idx} at {stage_prefix}/body/InspectionCamera")
        return True
    except Exception as exc:
        print(f"[swarm_app] camera attach failed for drone {drone_idx}: {exc}")
        return False


def _spawn_positions(num_drones: int) -> list[list[float]]:
    """Return spawn positions for up to 3 drones, one per aisle entry.

    Drones start at the near end of their assigned aisle (Y = −5.5 m,
    aisle centre-line in X, Z = 0.1 m above floor) so ``takeoff`` lifts
    them straight up into the aisle.

    For more than 3 drones the extras fall back to a compact grid near origin.
    Adjust the ``_AISLE_ENTRIES`` list to match your scene.
    """
    # (x, y, z) — aisle centre-line, near end of warehouse, just above floor.
    # These X values must match _CX_* constants in swarm/facility.py.
    # Spawn outside the aisle (Y=5.5); aisle entry is at Y=8.25
    _AISLE_ENTRIES = [
        [-3.25, 5.5, 0.1],   # uav-01 — left aisle centre-line
        [-5.25, 5.5, 0.1],   # uav-02 — 2 m to the LEFT of uav-01
        [ 3.25, 5.5, 0.1],   # uav-03 — right aisle
    ]
    out: list[list[float]] = []
    for i in range(num_drones):
        if i < len(_AISLE_ENTRIES):
            out.append(_AISLE_ENTRIES[i])
        else:
            # fallback compact grid for extra drones
            col = i % 3
            row = i // 3
            out.append([float(col) * 1.5 - 1.5, float(-row) * 1.5 - 7.0, 0.1])
    return out


class SwarmPegasusApp:
    """Single Isaac Sim app hosting N MQTT-controlled drones.

    Mirrors ``examples/5_python_multi_vehicle.py`` from PegasusSimulator [22],
    with the per-drone backend replaced by
    :class:`swarm.controller.CommandedNonlinearController`.
    """

    def __init__(self) -> None:
        self.sim = SimSettings()

        self.timeline = omni.timeline.get_timeline_interface()
        self.pg = PegasusInterface()
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        scene_style = (self.sim.scene_style or "warehouse").lower()
        if scene_style not in {"warehouse", "usd"}:
            self.world.scene.add_default_ground_plane()
        print(
            f"[swarm_app] scene_style={scene_style} "
            f"pegasus_env={getattr(self.sim, 'pegasus_env_name', '-')}"
        )
        build_sim_environment(self.world, self.sim, pg=self.pg)

        spawn = _spawn_positions(self.sim.num_drones)
        self.controllers: list[CommandedNonlinearController] = []
        for i, pos in enumerate(spawn):
            drone_id = f"uav-{i+1:02d}"
            controller = CommandedNonlinearController(drone_id=drone_id)

            config = MultirotorConfig()
            config.backends = [controller]

            stage_prefix = f"/World/Quadrotor_{i}"
            Multirotor(
                stage_prefix=stage_prefix,
                usd_file=ROBOTS["Iris"],
                vehicle_id=i,
                init_pos=pos,
                init_orientation=[0.0, 0.0, 0.0, 1.0],
                config=config,
            )
            cam_ok = _attach_nadir_camera(stage_prefix, i)
            self.controllers.append(controller)
            controller._camera_active = cam_ok
            print(f"[swarm_app] spawned {drone_id} at {pos}  camera={'OK' if cam_ok else 'N/A'}")

        self.world.reset()

    def run(self) -> int:
        print("[swarm_app] simulation loop started; waiting for MQTT commands "
              "(see `python -m swarm.operator --help`)")
        self.timeline.play()
        try:
            while simulation_app.is_running():
                self.world.step(render=RENDER)
                time.sleep(0.0)
        except KeyboardInterrupt:
            print("[swarm_app] interrupted — shutting down")
        finally:
            for c in self.controllers:
                try:
                    c.stop()
                except Exception:
                    pass
            self.timeline.stop()
            simulation_app.close()
        return 0


def main() -> int:
    return SwarmPegasusApp().run()


if __name__ == "__main__":
    raise SystemExit(main())
