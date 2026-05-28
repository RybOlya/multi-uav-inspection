# Isaac Sim entry point

Run **`swarm_app.py`** inside Isaac Sim's Python (not host `python3`).

```bash
make up                    # from repo root
make isaac-run             # prints exact command
```

After launch: **Window → Swarm Operator** → **Demo mission**.

Prerequisites: Pegasus Simulator, `paho-mqtt pydantic pydantic-settings numpy scipy` in Isaac's Python.

Scene / coordinates: [docs/ISAAC_SCENE_UA.md](../docs/ISAAC_SCENE_UA.md)

No GPU? Use headless demo instead:

```bash
python -m swarm.mock_swarm
make demo
```
