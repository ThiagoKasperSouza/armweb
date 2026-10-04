.PHONY: help build up down logs ps status usd record gltf cmd pose web test clean shell arm

help:
	@echo "armweb - headless ROS 2 + OpenUSD arm simulation (no GPU required)"
	@echo ""
	@echo "  make build   build the Docker image"
	@echo "  make up      start the stack (sim + Foxglove bridge on :8765)"
	@echo "  make ps      show container status"
	@echo "  make logs    tail container logs"
	@echo "  make status  print topics, frames and joint states"
	@echo "  make usd     (re)generate data/usd/arm.usda"
	@echo "  make arm     choose the robot: demo | fr3 | ur5 (needs make up first)"
	@echo "  make record  record /joint_states -> animated USD (DURATION RATE)"
	@echo "  make gltf    export glTF .glb (bakes the animation if recorded)"
	@echo "  make web     serve web/viewer.html on :8080 for the three.js viewer"
	@echo "  make test    run the exporter + viewer test suites"
	@echo "  make cmd     send a demo joint command"
	@echo "  make pose    switch to a named pose: photo | home"
	@echo "  make shell   open a shell inside the sim container"
	@echo "  make down    stop and remove the stack"
	@echo "  make clean   stop and remove containers, images and workspace build"

build:
	docker build -t armweb:latest .

up:
	docker compose up -d --build
	@echo ""
	@echo "Foxglove: open https://app.foxglove.dev and connect to ws://localhost:8765"

down:
	docker compose down -v

ps:
	docker compose ps

logs:
	docker compose logs -f sim

status:
	./scripts/status.sh

# Choose which robot to use: demo | fr3 | ur5
# Usage: make arm ARM=fr3   (default), then `make up` to restart the sim.
arm:
	./scripts/prepare_arm.sh $(or $(ARM),fr3)

usd:
	./scripts/export_usd.sh

# Record live /joint_states into an animated .usda.
# Usage: make record DURATION=5 RATE=20
record:
	./scripts/record_anim.sh $(or $(DURATION),5) $(or $(RATE),20)

# Export glTF .glb for the three.js viewer (bakes the animation if present).
gltf:
	./scripts/export_gltf.sh

# Run every exporter/viewer test (python ones inside the sim container).
test:
	@bash ws/tools/run_all_tests.sh

# Static server for the browser viewer.
web:
	@echo "three.js viewer -> http://localhost:8080/web/viewer.html"
	@echo "(Ctrl+C to stop)"
	cd "$(CURDIR)" && python3 -m http.server 8080

cmd:
	./scripts/send_cmd.sh

# Switch to a named pose (resolved by joint name, clamped to URDF limits).
# Usage: make pose POSE=photo   (or: home)
pose:
	./scripts/set_pose.sh $(or $(POSE),photo)

shell:
	docker compose exec sim bash

clean: down
	-docker rmi armweb:latest
	-./scripts/clean_ws.sh
	-rm -rf data/usd/*.usd*