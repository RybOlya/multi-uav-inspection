"""Scene loader for ``isaac/swarm_app.py``.

Two scene sources are supported via :data:`swarm.config.SimSettings.scene_style`:

* ``warehouse`` — a Pegasus built-in environment loaded by name from
  ``pegasus.simulator.params.SIMULATION_ENVIRONMENTS`` (e.g. ``Full Warehouse``,
  ``Warehouse with Shelves``). The asset comes from NVIDIA Nucleus and is
  loaded through ``PegasusInterface.load_environment(...)`` exactly as in
  ``examples/5_python_multi_vehicle.py`` of [22].

* ``usd`` — a user-supplied filesystem ``.usd`` / ``.usda`` / ``.usdc`` file.
  It is referenced under ``/World/UserUSDScene``.

If the requested source cannot be loaded (e.g. Nucleus unreachable, USD
file missing), Isaac's default ground plane is added so the simulation can
still proceed for the operator demo.

References:
    [22] Jacinto, M. *et al.* (2024). "Pegasus Simulator: An Isaac Sim
         Framework for Multiple Aerial Vehicles Simulation", *ICUAS 2024*.
"""
from __future__ import annotations


def _add_default_ground_plane(world) -> None:
    try:
        world.scene.add_default_ground_plane()
    except Exception as exc:  # pragma: no cover - simulator-only path
        print(f"[scene_builder] could not add default ground plane: {exc}")


def reference_usd_to_stage(usd_path: str, prim_path: str = "/World/UserUSDScene") -> str:
    """Attach a filesystem ``.usd`` payload under ``prim_path``."""
    from pathlib import Path

    resolved = Path(usd_path).expanduser().resolve()
    if not resolved.is_file():
        raise FileNotFoundError(f"USD file not found: {resolved}")
    fs = resolved.as_posix()

    from omni.isaac.core.utils.prims import create_prim
    from omni.isaac.core.utils.stage import add_reference_to_stage

    create_prim(prim_path=prim_path, prim_type="Xform")

    try:
        add_reference_to_stage(usd_path=fs, prim_path=prim_path)
    except Exception as primary_exc:
        try:
            add_reference_to_stage(usd_path="file://" + fs, prim_path=prim_path)
        except Exception as fallback_exc:
            raise RuntimeError(
                f"Could not reference USD '{fs}'. Primary: {primary_exc}; Fallback: {fallback_exc}"
            ) from primary_exc
    return fs


def load_pegasus_environment(pg, env_name: str) -> str:
    """Load a built-in Pegasus environment by name (e.g. ``Full Warehouse``)."""
    from pegasus.simulator.params import SIMULATION_ENVIRONMENTS

    if env_name not in SIMULATION_ENVIRONMENTS:
        available = ", ".join(sorted(SIMULATION_ENVIRONMENTS.keys()))
        raise KeyError(
            f"Pegasus environment '{env_name}' not found. Available: {available}"
        )

    usd_path = SIMULATION_ENVIRONMENTS[env_name]
    try:
        pg.load_environment(usd_path)
    except Exception as exc:
        raise RuntimeError(
            f"PegasusInterface.load_environment('{env_name}') failed: {exc}"
        ) from exc
    return str(usd_path)


def build_sim_environment(world, sim, pg=None) -> None:
    """Dispatch scene loading from :class:`swarm.config.SimSettings`.

    On any failure (missing Nucleus, missing USD), falls back to Isaac's
    default ground plane so the simulation still launches for a demo.
    """
    style = (getattr(sim, "scene_style", "warehouse") or "warehouse").lower()

    if style == "warehouse":
        if pg is None:
            print(
                "[scene_builder] scene_style=warehouse needs PegasusInterface; "
                "falling back to default ground plane."
            )
            _add_default_ground_plane(world)
            return
        env_name = getattr(sim, "pegasus_env_name", "Full Warehouse") or "Full Warehouse"
        try:
            resolved = load_pegasus_environment(pg, env_name)
            print(f"[scene_builder] loaded Pegasus environment '{env_name}': {resolved}")
            return
        except Exception as exc:
            print(
                f"[scene_builder] could not load Pegasus '{env_name}' ({exc}); "
                "falling back to default ground plane."
            )
            _add_default_ground_plane(world)
            return

    if style == "usd" and getattr(sim, "usd_scene_path", None):
        try:
            resolved = reference_usd_to_stage(sim.usd_scene_path)
            print(f"[scene_builder] referenced user USD: {resolved} -> /World/UserUSDScene")
            return
        except Exception as exc:
            print(
                f"[scene_builder] USD reference failed ({exc}); "
                "falling back to default ground plane."
            )

    _add_default_ground_plane(world)
