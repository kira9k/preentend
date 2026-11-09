# Сценарий сканирования автомобиля

Этот каталог содержит отдельный ROS-пакет `preentend_scan`. Он использует пакеты робота как зависимости; его настройки MoveIt, мир Gazebo с прозрачными стёклами, RViz и логика сканирования находятся здесь.

## Сборка и запуск

Из контейнера ROS 2:

```shell
cd /ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select preentend_scan
source install/setup.bash
ros2 launch preentend_scan preentend_scan.launch.py
```

Launch запускает Gazebo, MoveIt, RViz и окно управления. Дождитесь загрузки и нажмите «Начать сканирование». Остановка доступна кнопкой «Остановить» или сервисом `/scan/stop`.

Результаты по умолчанию сохраняются в `/ros2_ws/scans/scan_.../` и доступны на хосте в `workspaces/ros2_ws/scans/`. Для каждого прохода сохраняются `merged_raw.ply`, `measured.ply`, `reconstructed_only.ply`, итоговый `merged.ply`, отдельные кадры, `waypoints.json` и `manifest.json`.

Путь для результатов и YAML-конфигурацию можно переопределить:

```shell
ros2 launch preentend_scan preentend_scan.launch.py \
  scan_output:=/ros2_ws/scans_test scan_config:=/ros2_ws/src/scan_node/config/scan.yaml
```

Для работы нужен собранный основной workspace с пакетами `preentend_description`, `preentend_controller`, `preentend_gazebo` и `preentend_moveit_config`. Не запускайте одновременно второй Gazebo или второй экземпляр сценария: они будут публиковать конкурирующее время `/clock`.
