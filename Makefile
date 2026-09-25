export PROJECT_ROOT=$(shell pwd)
export USER_UID=$(shell id -u)
export USER_GID=$(shell id -g)

.PHONY: isaac_sim-fix-permissions

ros2-up:
	@xhost local:root; \
	docker compose up --build -d ros2

ros2-exec:
	@docker compose exec ros2 bash

ros2-down:
	@docker compose down ros2

isaac_sim-up:
	@xhost local:root; \
	mkdir -p ~/docker/isaac-sim/{cache/main,cache/computecache,cache/kit,config,logs,pkg}; \
	sudo chown -R 1234:1234 ~/docker/isaac-sim ~/.cache/ov/hub; \
	docker compose up --build -d isaac_sim_hub isaac_sim

isaac_sim-fix-permissions:
	@chown -R 1234:1234 ~/docker/isaac-sim ~/.cache/ov/hub

isaac_sim-exec: isaac_sim-fix-permissions
	@docker compose exec isaac_sim bash

isaac_sim-down:
	@sudo rm -rf ~/docker/isaac-sim; \
	docker compose down isaac_sim isaac_sim_hub
