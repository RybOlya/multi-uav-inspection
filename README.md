# Multi-UAV Warehouse Inspection Swarm

> Autonomous facility inspection with battery-aware relay handoff and live IoT telemetry.

Two quadrotors inspect a warehouse aisle. **uav-01** scans from entry to far end. When its battery drops below the threshold, **uav-02** flies to uav-01's exact world coordinates and continues — no operator input required. All telemetry streams live to Grafana.

Built on [NVIDIA Isaac Sim](https://docs.isaacsim.omniverse.nvidia.com/) + [Pegasus Simulator](https://github.com/PegasusSimulator/PegasusSimulator), MQTT v5, InfluxDB, and Grafana.

---

## Demo

### Flight — Isaac Sim

<video src="https://github.com/RybOlya/multi-uav-inspection/raw/master/docs/flight.webm" controls width="100%"></video>

### Telemetry — Grafana live dashboard

<video src="https://github.com/RybOlya/multi-uav-inspection/raw/master/docs/grafana.webm" controls width="100%"></video>

---

## How it works

```
uav-01  spawns at (-3.25, 5.5, 0.1)
         takeoff → Z = 3.0 m
         goto aisle entry  Y = 8.25 m
         goto far end      Y = 15.5 m   ← scans along the aisle
         battery drops to 50% threshold
         stop → world pose captured via MQTT → return to spawn → land

uav-02  spawns at (-5.25, 5.5, 0.1)   ← 2 m to the left of uav-01
         takeoff → goto handoff pose (exact XYZ from MQTT)
         goto far end      Y = 15.5 m   ← resumes scan
         land
```

The handoff pose is not pre-planned — it is the **live world position** of uav-01 read from the MQTT state topic at the moment it stops. No gaps in coverage.

### Architecture

```
Operator script  ──MQTT cmd (QoS 1)──►  Drone controllers
                                         ├── CommandedNonlinearController
                                         │     └── TrajectoryFollower (quintic poly)
                                         └── Pegasus NonlinearController (SE(3))

Drone controllers ──MQTT state/telemetry──► Mosquitto
                                              ├── DroneStateTracker  (relay logic)
                                              └── Telegraf → InfluxDB → Grafana
```

Control stack is grounded in:
- **Lee, Leok & McClamroch (2010)** — geometric tracking on SE(3)
- **Mellinger & Kumar (2011)** — minimum-snap trajectory generation
- **Lynch & Park (2017)** — quintic polynomial segments (§9.2)

---

## Quickstart — headless (no Isaac Sim needed)

```bash
# 1. Install dependencies
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt

# 2. Start cloud stack (Docker required)
make up

# 3. Start simulated drones — fast battery drain for demo
SIM_BATTERY_DRAIN_RATE_PPS=2.0 python3 -m swarm.mock_swarm

# 4. Run the relay mission (separate terminal)
python3 missions/relay_demo.py

# 5. Open Grafana dashboard
make grafana          # → http://localhost:3000
```

The relay triggers in ~25 s. Watch battery % fall on the gauge, then see uav-02 take over from uav-01's exact world coordinates.

> **Before each run:** `pkill -f mock_swarm` to ensure no stale processes.

---

## Quickstart — Isaac Sim + Pegasus

```bash
# Terminal 1 — cloud stack
make up

# Terminal 2 — Isaac Sim (use Isaac's bundled Python)
./python.sh /path/to/bridge-swarm-iot/isaac/swarm_app.py

# Terminal 3 — mission
python3 missions/relay_demo.py
```

---

## Configuration (`.env`)

| Variable | Default | Description |
|---|---|---|
| `SIM_NUM_DRONES` | `2` | Number of drones to spawn |
| `SIM_CRUISE_SPEED_MPS` | `0.5` | Cruise speed (m/s) |
| `SIM_TAKEOFF_ALTITUDE_M` | `3.0` | Scan altitude (m) |
| `SIM_BATTERY_DRAIN_RATE_PPS` | `0.05` | Battery drain (%/s); use `2.0` for fast demo |
| `SIM_BATTERY_RELAY_THRESHOLD_PCT` | `50.0` | Handoff trigger (%) |
| `MQTT_HOST` | `localhost` | Broker host |
| `MQTT_PORT` | `1883` | Broker port |

---

## Facility layout

```
      X = -5.25   X = -3.25
       uav-02 ●    uav-01 ●    ← spawn at Y = 5.5 (outside aisle)
               │        │
               │  aisle │      Y = 8.25  ← scan start
               │        │
               │        │      Y = 15.5  ← scan end
```

| ID | Type | World pose (X, Y, Z) |
|---|---|---|
| `rack_left_end` | POI | (−3.25, +15.5, 3.0) |
| `rack_right_end` | POI | (+3.25, +15.5, 3.0) |
| `aisle_left` | ScanZone | X = −3.25, Y ∈ [8.25, 15.5] |
| `aisle_right` | ScanZone | X = +3.25, Y ∈ [8.25, 15.5] |

---

## MQTT topics

| Topic | Direction | QoS | Payload |
|---|---|---|---|
| `swarm/cmd/{drone_id}` | Operator → Drone | 1 | JSON command |
| `swarm/cmd/all` | Operator → All | 1 | Broadcast |
| `swarm/state/{drone_id}` | Drone → Cloud | 0 | `{position, battery_pct, mode}` |
| `swarm/telemetry/{drone_id}` | Drone → Cloud | 0 | InfluxDB Line Protocol |

---

## Project structure

```
bridge-swarm-iot/
├── missions/
│   └── relay_demo.py        # battery-relay end-to-end mission
├── swarm/
│   ├── commands.py          # Pydantic command schemas
│   ├── config.py            # settings from .env
│   ├── controller.py        # CommandedNonlinearController
│   ├── trajectory.py        # quintic segments, TrajectoryFollower
│   ├── relay.py             # DroneStateTracker, RelayMission
│   ├── mock_swarm.py        # headless simulator (no Isaac Sim)
│   ├── facility.py          # POIs and scan zones
│   └── operator.py          # CLI for manual commands
├── isaac/
│   └── swarm_app.py         # Pegasus / Isaac Sim entry point
├── cloud/                   # Telegraf + InfluxDB + Grafana config
├── docs/
│   ├── diagrams/            # PlantUML architecture diagrams
│   ├── flight.webm          # Isaac Sim demo video
│   ├── grafana.webm         # Grafana dashboard demo
│   └── КУРСОВА_РОБОТА.txt   # Coursework (Ukrainian)
├── tests/                   # pytest unit tests
├── docker-compose.yml
├── Makefile
└── .env.example
```

---

## Tests

```bash
cd bridge-swarm-iot
python3 -m pytest tests/ -q
```

---

## References

1. Lee, T., Leok, M., McClamroch, N. H. — *Geometric Tracking Control of a Quadrotor UAV on SE(3)*. IEEE CDC 2010. [arXiv:1003.2005](https://arxiv.org/abs/1003.2005)
2. Mellinger, D., Kumar, V. — *Minimum snap trajectory generation and control for quadrotors*. IEEE ICRA 2011.
3. Lynch, K. M., Park, F. C. — *Modern Robotics*. Cambridge University Press, 2017. §9.2
4. Jacinto, M. et al. — *Pegasus Simulator: An Isaac Sim Framework for Multiple Aerial Vehicles Simulation*. ICUAS 2024. [arXiv:2404.15923](https://arxiv.org/abs/2404.15923)
5. Pinto, J., Guerreiro, B. J., Cunha, R. — *Planning Parcel Relay Manoeuvres for Quadrotors*. ICUAS 2021.
6. OASIS Standard — *MQTT Version 5.0*, 2019. [spec](https://docs.oasis-open.org/mqtt/mqtt/v5.0/mqtt-v5.0.pdf)

---

## License

MIT
