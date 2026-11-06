# Настройка окружения

## Docker

> **Если Docker уже есть то можно ничего отсюда не делать, за исключением 4 пункта**

1. Удаляем полностью Docker если он есть:
```shell
sudo apt remove $(dpkg --get-selections docker.io docker-compose docker-compose-v2 docker-doc docker-buildx podman-docker containerd runc | cut -f1)
```

2. Настраиваем репозиторий:
```shell
# Add Docker's official GPG key:
sudo apt update
sudo apt install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# Add the repository to Apt sources:
sudo tee /etc/apt/sources.list.d/docker.sources <<EOF
Types: deb
URIs: https://download.docker.com/linux/ubuntu
Suites: $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}")
Components: stable
Architectures: $(dpkg --print-architecture)
Signed-By: /etc/apt/keyrings/docker.asc
EOF

sudo apt update
```

3. Устанавливаем последнюю версию:
```shell
sudo apt install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

4. Добавляем пользователя в группу docker для доступа без `sudo`:
```shell
sudo groupadd docker
sudo usermod -aG docker $USER
newgrp docker
```

## NVIDIA Container Toolkit

```shell
# Configure the repository
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
    && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list \
    && \
    sudo apt-get update

# Install the NVIDIA Container Toolkit packages
sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker

# Configure the container runtime
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

# Verify NVIDIA Container Toolkit
docker run --rm --runtime=nvidia --gpus all nvcr.io/nvidia/cuda:12.8.0-base-ubuntu24.04 nvidia-smi
```

# Работа с репозиторием

1. Клонируем репозиторий:
```shell
git clone git@github.com:mazhugich-labs/preentend.git
cd preentend
```

## Доступ к файловой системе из Isaac Sim

```shell
setfacl -m u:1234:rwx workspaces/isaac_sim_ws
setfacl -d -m u:1234:rwx workspaces/isaac_sim_ws
setfacl -d -m u:$(id -u):rwx workspaces/isaac_sim_ws
```

> С этими настройками **workspaces/isaac_sim_ws** доступна в контейнере **isaac_sim**, а **workspaces/ros2_ws** в контейнере **ros2**

## Запуск контейнеров через Makefile

1. Пример 1. Запуск контейнера ros2:
```shell
make ros2-up
```

2. Пример 2. Вход в контейнер:
```shell
make ros2-exec
```

3. Пример 3. Закрытие и удаление контейнера:
```shell
make ros2-down
```

# Работа с ROS из Docker

1. Билдим пакеты:
```shell
cd /ros2_ws
colcon build --symlink-install
```
Можно билдить выбранные пакеты если добавить флаг `--packages-select` а после него название пакета. Пример:
```shell
colcon build --symlink-install --packages-select preentend_description
```

2. Сурсим рабочее пространство:
```
source install/setup.bash
```

> С флагом `--symlink-install` при изменении файлов в уже забилженых пакетах заново их билдить не нужно

## Пакет preentend_description

Можно посмотреть ртк в **rviz**:
```shell
ros2 launch preentend_description view_preentend.launch.py
```

![preentend_description_rviz](docs/assets/preentend_description_rviz.png) ![preentend_description_jsp_gui](docs/assets/preentend_description_jsp_gui.png)

## Пакет preentend_moveit_config

Можно поуправлять роботами по отдельности:
```shell
ros2 launch preentend_moveit_config demo.launch.py
```

![preentend_moveit_config](docs/assets/preentend_moveit_config.gif)