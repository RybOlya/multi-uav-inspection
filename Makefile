.PHONY: help venv install up up-full down check relay grafana workflow isaac-run test clean

PY ?= python3
VENV ?= .venv
ACT = . $(VENV)/bin/activate
REPO := $(CURDIR)

help:
	@echo "Facility Inspection Swarm — operator-driven multi-UAV simulation"
	@echo ""
	@echo "  make install     create venv + install deps"
	@echo "  make up          MQTT broker only (minimum required)"
	@echo "  make up-full     MQTT + InfluxDB + Telegraf + Grafana"
	@echo "  make check       verify MQTT broker is reachable"
	@echo ""
	@echo "  make relay       run 2-drone relay mission"
	@echo "  make grafana     open operator dashboard in browser"
	@echo "  make isaac-run   print Isaac Sim launch command"
	@echo "  make workflow    full two-path instructions"
	@echo ""
	@echo "  make test        run unit tests"
	@echo "  make clean       remove venv + caches"

venv:
	@$(PY) -m venv "$(VENV)" || { chmod -R u+w "$(VENV)" 2>/dev/null; rm -rf "$(VENV)"; $(PY) -m venv "$(VENV)"; }

install: venv
	$(ACT) && pip install -U pip wheel && pip install -r requirements.txt

up:
	docker compose up -d mosquitto
	@echo ""
	@echo "MQTT ready at tcp://localhost:1883"
	@echo "  make check"
	@echo "  make mock    →  then  make inspect"

up-full:
	docker compose --profile observability up -d
	@echo ""
	@echo "MQTT      : tcp://localhost:1883"
	@echo "Grafana   : http://localhost:3000  (admin / admin)"
	@echo "InfluxDB  : http://localhost:8086"

down:
	docker compose --profile observability down

check:
	@mosquitto_pub -h localhost -t swarm/ping -m ok 2>&1 && echo "OK  MQTT broker at localhost:1883" \
		|| echo "FAIL  MQTT broker not reachable — run: make up"

mock:
	@echo "Starting headless mock swarm — then run: make relay"
	@echo "Press Ctrl+C to stop."
	$(ACT) && SIM_BATTERY_DRAIN_RATE_PPS=2.0 python -m swarm.mock_swarm

relay:
	$(ACT) && python missions/relay_demo.py

grafana:
	@xdg-open "http://localhost:3000/d/swarm-operator/swarm-operator-dashboard?orgId=1&refresh=1s" 2>/dev/null \
		|| echo "Open: http://localhost:3000/d/swarm-operator/swarm-operator-dashboard"

workflow:
	@echo "=== Path A: headless (fastest, no Isaac Sim) ==="
	@echo "  make install && make up-full"
	@echo "  Terminal 1:  make mock"
	@echo "  Terminal 2:  make relay"
	@echo "  Browser:     make grafana"
	@echo ""
	@echo "=== Path B: Isaac Sim + Pegasus physics ==="
	@echo "  make install && make up-full"
	@echo "  Terminal 1:  see  make isaac-run"
	@echo "  Terminal 2:  make relay"
	@echo "  Browser:     make grafana"
	@echo ""
	@echo "  Abort:  python -m swarm.operator stop --all"
	@echo "  Land:   python -m swarm.operator land --all"

isaac-run:
	@echo "cd ~/isaac-sim"
	@echo "PEGASUS_PATH=~/PegasusSimulator SWARM_IOT_ROOT=$(REPO) \\"
	@echo "  ./python.sh $(REPO)/isaac/swarm_app.py"
	@echo ""
	@echo "One-time dep install (Isaac Python):"
	@echo "  ./python.sh -m pip install paho-mqtt pydantic pydantic-settings numpy scipy"

test:
	$(ACT) && pytest -q

clean:
	rm -rf $(VENV) .pytest_cache **/__pycache__
