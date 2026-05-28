# Facility Inspection Swarm

Two drones inspect a warehouse. **uav-01** scans the left aisle.
When its battery drops below 60%, **uav-02** flies to that exact world position and finishes.
Telemetry (position, battery %, scan coverage) streams live to Grafana.

---

## Run it

**Terminal 1 — cloud stack + simulated drones:**
```bash
make up
SIM_BATTERY_DRAIN_RATE_PPS=2.0 python -m swarm.mock_swarm
```

**Terminal 2 — relay mission:**
```bash
python missions/relay_demo.py
```

**Terminal 3 — Grafana dashboard:**
```bash
make grafana    # opens http://localhost:3000
```

That's it. The relay triggers in ~20 s. Watch battery % drop on the gauge,
then see uav-02 take over from uav-01's exact world coordinates.

---

## Manual commands (operator CLI)

```bash
# takeoff / navigate
python -m swarm.operator takeoff --all --altitude 3.0
python -m swarm.operator goto uav-01 --x -3.25 --y 0.0 --z 3.0

# inspect a rack end (goto → hover 4 s → orbit)
python -m swarm.operator inspect-poi uav-01 rack_left_end
python -m swarm.operator inspect-poi uav-02 rack_right_end

# straight aisle scan
python -m swarm.operator scan-zone aisle_left    # uav-01
python -m swarm.operator scan-zone aisle_right   # uav-02

# stop / land
python -m swarm.operator stop --all
python -m swarm.operator land --all
```

---

## Layout

```
X = -3.25 m  ←  left aisle   (uav-01 spawn)
X = +3.25 m  ←  right aisle  (uav-02 spawn)
Y = -5.5 … +5.5 m  aisle length
Z =  3.0 m          scan altitude
```

| ID | Type | (X, Y, Z) |
|---|---|---|
| `rack_left_end`  | POI      | (−3.25, +5.5, 3.0) |
| `rack_right_end` | POI      | (+3.25, +5.5, 3.0) |
| `aisle_left`     | ScanZone | X=−3.25, Y∈[−5.5,+5.5] |
| `aisle_right`    | ScanZone | X=+3.25, Y∈[−5.5,+5.5] |

---

## Grafana panels

Battery %, scan coverage %, position, speed, mode, events — all live.

---

## Key `.env` settings

```bash
SIM_NUM_DRONES=2
SIM_CRUISE_SPEED_MPS=0.5
SIM_TAKEOFF_ALTITUDE_M=3.0
SIM_BATTERY_DRAIN_RATE_PPS=0.05       # 2.0 for fast demo
SIM_BATTERY_RELAY_THRESHOLD_PCT=40.0  # 60 for fast demo
```

---

## Isaac Sim

```bash
make up
./python.sh /path/to/bridge-swarm-iot/isaac/swarm_app.py
# then in another terminal:
python missions/relay_demo.py
```
