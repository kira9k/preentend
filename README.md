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

## Запуск контейнера ROS 2

Из корня репозитория:

```shell
make ros2-up
make ros2-enter
```

Для остановки только контейнера ROS 2 выполните на хосте `docker compose stop ros2`.

# Работа с ROS 2

Команды ниже выполняются **в контейнере ROS 2**. Рабочее пространство хоста `workspaces/ros2_ws` смонтировано в контейнер по пути `/ros2_ws`. Образ содержит ROS 2 Jazzy, MoveIt 2, Gazebo и `gz_ros2_control`.

## Сборка рабочего пространства

```shell
cd /ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Для сборки пакета вместе с зависимостями можно использовать, например:

```shell
colcon build --symlink-install --packages-up-to preentend_moveit_config
source install/setup.bash
```

В новом терминале снова выполните `source /ros2_ws/install/setup.bash`. После добавления файлов, изменения `CMakeLists.txt` или `package.xml` пересоберите пакеты. `--symlink-install` создает символические ссылки на уже существующие файлы в пакете, что убирает необходимость в постоянной сборке при изменении старых файлов.

## Пакеты и сценарии запуска

| Пакет | Назначение |
| --- | --- |
| `preentend_description` | Xacro/URDF, геометрия и просмотр модели в RViz |
| `preentend_controller` | Конфигурация `ros2_control`, контроллеры левой и правой рук с линейными осями |
| `preentend_gazebo` | Мир Gazebo и запуск симуляции с созданием модели робота |
| `preentend_moveit_config` | SRDF, планирование MoveIt, конфигурация RViz и запуск с mock или Gazebo |

Запускайте **один** из следующих сценариев в отдельном терминале после сборки и `source install/setup.bash`.

### Просмотр модели без контроллеров

```shell
ros2 launch preentend_description preentend_rviz2_display.launch.py
```

Этот запуск поднимает `robot_state_publisher`, `joint_state_publisher` и RViz. Он предназначен для просмотра модели; контроллеры и MoveIt не запускаются.

### MoveIt с mock-оборудованием

```shell
ros2 launch preentend_moveit_config preentend_moveit_fake.launch.py
```

Запускаются `ros2_control_node` с `mock_components/GenericSystem`, `joint_state_broadcaster`, два контроллера траекторий, `move_group` и MoveIt RViz. Контроллеры принимают команды без Gazebo. Публикацию `/joint_states` обеспечивает `joint_state_broadcaster`.

### MoveIt с Gazebo

```shell
ros2 launch preentend_moveit_config preentend_moveit_gazebo.launch.py
```

Запускаются Gazebo с миром `preentend_gazebo/worlds/empty.sdf`, публикация описания робота, создание модели в Gazebo, `gz_ros2_control`, контроллеры, мост `/clock`, `move_group` и MoveIt RViz. В этом сценарии узлы используют симуляционное время.

Для проверки состояния контроллеров в другом терминале контейнера:

```shell
source /ros2_ws/install/setup.bash
ros2 control list_controllers
```

## Файлы конфигурации

- `preentend_controller/config/ros2_controllers.yaml` задаёт `joint_state_broadcaster` и контроллеры `left_arm_controller`/`right_arm_controller`.
- `preentend_moveit_config/config/moveit_controllers.yaml` связывает контроллеры MoveIt с действиями `FollowJointTrajectory`.
- `preentend_moveit_config/config/ompl_planning.yaml` задаёт OMPL и обработку траекторий перед выполнением.
- `preentend_moveit_config/srdf/preentend.srdf` задаёт группы `left_arm` и `right_arm` и исключения из проверки самостолкновений.

Вспомогательные launch-файлы `_ros2_control.launch.py` и `_moveit_common.launch.py` используются сценариями выше. Первый выбирает оборудование через `hardware_plugin`: `mock_components/GenericSystem` по умолчанию или `gz_ros2_control/GazeboSimSystem` при запуске с Gazebo.
