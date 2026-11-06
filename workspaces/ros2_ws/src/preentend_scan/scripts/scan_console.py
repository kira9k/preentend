#!/usr/bin/env python3
"""Button-driven, collision-planned scan of the space between the two rails."""

import json
import threading
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy
import yaml
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    PlanningSceneComponents,
)
from moveit_msgs.srv import GetPlanningScene
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import (
    QApplication,
    QLabel,
    QProgressBar,
    QPushButton,
    QVBoxLayout,
    QWidget,
)
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener


def save_ply(path, points):
    """Write XYZ points in metres as a compact binary PLY file."""
    points = np.asarray(points, dtype="<f4").reshape(-1, 3)
    header = (
        "ply\nformat binary_little_endian 1.0\n"
        f"element vertex {len(points)}\n"
        "property float x\nproperty float y\nproperty float z\nend_header\n"
    )
    with path.open("wb") as stream:
        stream.write(header.encode("ascii"))
        stream.write(points.tobytes())


def rotation_matrix(quaternion):
    x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
    length = np.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / length, y / length, z / length, w / length
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float32,
    )


class ScanNode(Node):
    def __init__(self):
        super().__init__("scan_console")
        self.declare_parameter("config_file", "")
        self.declare_parameter("output_root", "/ros2_ws/scans")
        config_file = self.get_parameter("config_file").value
        with open(config_file, encoding="utf-8") as stream:
            self.config = yaml.safe_load(stream)
        self.output_root = Path(self.get_parameter("output_root").value)
        self._validate_config()

        self.move_client = ActionClient(self, MoveGroup, "/move_action")
        self.controllers = {
            side: ActionClient(
                self, FollowJointTrajectory, f"/{side}_arm_controller/follow_joint_trajectory"
            )
            for side in ("left", "right")
        }
        self.scene_client = self.create_client(GetPlanningScene, "/get_planning_scene")
        self.tf_buffer = Buffer(cache_time=Duration(seconds=30))
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self._cloud_lock = threading.Condition()
        self._clouds = {}
        self._cloud_seq = {"left": 0, "right": 0}
        for side in ("left", "right"):
            self.create_subscription(
                PointCloud2,
                f"/{side}_camera/depth/points",
                lambda msg, name=side: self._on_cloud(name, msg),
                qos_profile_sensor_data,
            )

        self._state_lock = threading.Lock()
        self._active = False
        self._cancel = threading.Event()
        self._goal_handle = None
        self._worker = None
        self._phase = "Готов к сканированию"
        self._progress = 0
        self._total = len(self.config["rail_positions"]) * len(self.config["levels"]) * 2
        self._output_path = ""
        self.create_service(Trigger, "/scan/start", self._start_service)
        self.create_service(Trigger, "/scan/stop", self._stop_service)

    def _validate_config(self):
        cfg = self.config
        if not cfg["rail_positions"] or not cfg["levels"]:
            raise ValueError("scan.yaml needs rail_positions and levels")
        if len(cfg["scan_bounds"]) != 6 or cfg["point_stride"] < 1:
            raise ValueError("scan_bounds must have six values and point_stride must be positive")
        for position in cfg["rail_positions"]:
            if not -2.8 <= position <= 2.8:
                raise ValueError(f"Rail position {position} is outside the ±2.8 m travel")
        for level in cfg["levels"]:
            if len(level["left"]) != 6 or len(level["right"]) != 6:
                raise ValueError(f"Level {level['name']} needs six arm joints per robot")

    def _on_cloud(self, side, message):
        with self._cloud_lock:
            self._clouds[side] = message
            self._cloud_seq[side] += 1
            self._cloud_lock.notify_all()

    def state(self):
        with self._state_lock:
            return self._phase, self._progress, self._total, self._output_path, self._active

    def _set_state(self, phase=None, progress=None, output_path=None):
        with self._state_lock:
            if phase is not None:
                self._phase = phase
            if progress is not None:
                self._progress = progress
            if output_path is not None:
                self._output_path = str(output_path)

    def start_scan(self):
        with self._state_lock:
            if self._active:
                return False, "Сканирование уже идёт"
            self._active = True
            self._progress = 0
            self._phase = "Подключение к MoveIt…"
            self._output_path = ""
        self._cancel.clear()
        self._worker = threading.Thread(target=self._run_scan, daemon=True)
        self._worker.start()
        return True, "Сканирование запущено"

    def stop_scan(self):
        with self._state_lock:
            if not self._active:
                return False, "Сканирование не запущено"
            self._phase = "Остановка…"
            goal_handle = self._goal_handle
        self._cancel.set()
        if goal_handle is not None:
            goal_handle.cancel_goal_async()
        return True, "Остановка запрошена"

    def wait_for_scan_stop(self, timeout):
        worker = self._worker
        if worker is not None:
            worker.join(timeout=timeout)

    def _start_service(self, _request, response):
        response.success, response.message = self.start_scan()
        return response

    def _stop_service(self, _request, response):
        response.success, response.message = self.stop_scan()
        return response

    def _wait_future(self, future, timeout):
        deadline = time.monotonic() + timeout
        while not future.done():
            if self._cancel.is_set():
                raise InterruptedError("Сканирование остановлено")
            if time.monotonic() > deadline:
                raise TimeoutError("Превышено время ожидания ответа MoveIt")
            time.sleep(0.05)
        return future.result()

    @staticmethod
    def _joint_names(side):
        return [f"{side}_linear_axis_joint"] + [f"{side}_J{i}" for i in range(1, 7)]

    def _move_goal(self, waypoints):
        goal = MoveGroup.Goal()
        request = goal.request
        request.group_name = "dual_arm_camera"
        request.start_state.is_diff = True
        request.num_planning_attempts = 5
        request.allowed_planning_time = 10.0
        request.max_velocity_scaling_factor = float(self.config["velocity_scaling"])
        request.max_acceleration_scaling_factor = float(self.config["acceleration_scaling"])
        constraints = Constraints()
        for waypoint in waypoints:
            for joint, position in zip(
                self._joint_names(waypoint["robot"]), waypoint["joints"]
            ):
                item = JointConstraint()
                item.joint_name = joint
                item.position = float(position)
                item.tolerance_above = 0.02
                item.tolerance_below = 0.02
                item.weight = 1.0
                constraints.joint_constraints.append(item)
        request.goal_constraints.append(constraints)
        goal.planning_options.plan_only = False
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        return goal

    def _move_pair(self, waypoints):
        accepted = self._wait_future(
            self.move_client.send_goal_async(self._move_goal(waypoints)), 15.0
        )
        if accepted is None or not accepted.accepted:
            raise RuntimeError("MoveIt отклонил общую цель двух роботов")
        with self._state_lock:
            self._goal_handle = accepted
        try:
            result = self._wait_future(accepted.get_result_async(), 180.0)
        finally:
            with self._state_lock:
                self._goal_handle = None
        if result.result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(
                f"MoveIt не выполнил общую цель: код {result.result.error_code.val}"
            )

    def _fresh_cloud(self, side):
        timeout = float(self.config["cloud_timeout_seconds"])
        deadline = time.monotonic() + timeout
        with self._cloud_lock:
            previous = self._cloud_seq[side]
            while self._cloud_seq[side] <= previous:
                if self._cancel.is_set():
                    raise InterruptedError("Сканирование остановлено")
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(f"Нет нового облака от {side}_camera")
                self._cloud_lock.wait(timeout=min(remaining, 0.2))
            return self._clouds[side]

    def _points_in_world(self, message):
        transform = self.tf_buffer.lookup_transform(
            "world",
            message.header.frame_id,
            Time.from_msg(message.header.stamp),
            timeout=Duration(seconds=3),
        )
        points = point_cloud2.read_points_numpy(
            message, field_names=("x", "y", "z"), skip_nans=True
        )
        points = np.asarray(points[:: int(self.config["point_stride"])], dtype=np.float32)
        if not len(points):
            return points.reshape(0, 3), transform
        translation = transform.transform.translation
        origin = np.array([translation.x, translation.y, translation.z], dtype=np.float32)
        world = points @ rotation_matrix(transform.transform.rotation).T + origin
        xmin, xmax, ymin, ymax, zmin, zmax = self.config["scan_bounds"]
        inside = (
            (world[:, 0] >= xmin) & (world[:, 0] <= xmax)
            & (world[:, 1] >= ymin) & (world[:, 1] <= ymax)
            & (world[:, 2] >= zmin) & (world[:, 2] <= zmax)
        )
        return world[inside], transform

    @staticmethod
    def _waypoints(config):
        route = []
        for level_index, level in enumerate(config["levels"]):
            positions = list(config["rail_positions"])
            if level_index % 2:
                positions.reverse()
            for station_index, rail in enumerate(positions):
                for side in ("left", "right"):
                    route.append(
                        {
                            "level": level["name"],
                            "station": station_index + 1,
                            "robot": side,
                            "rail_position": rail,
                            "joints": [rail] + list(level[side]),
                        }
                    )
        return route

    def _run_scan(self):
        folder = None
        manifest = None
        all_points = []
        try:
            self._set_state(phase="Ожидание MoveIt и контроллеров…")
            if not self.move_client.wait_for_server(timeout_sec=30.0):
                raise RuntimeError("MoveIt /move_action недоступен")
            for side, controller in self.controllers.items():
                if not controller.wait_for_server(timeout_sec=45.0):
                    raise RuntimeError(f"Контроллер {side}_arm_controller недоступен")
            self._set_state(phase="Ожидание коллизии автомобиля…")
            if not self.scene_client.wait_for_service(timeout_sec=30.0):
                raise RuntimeError("MoveIt /get_planning_scene недоступен")
            deadline = time.monotonic() + 30.0
            while time.monotonic() < deadline:
                request = GetPlanningScene.Request()
                request.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
                scene = self._wait_future(self.scene_client.call_async(request), 5.0).scene
                if any(obj.id == "car" for obj in scene.world.collision_objects):
                    break
                time.sleep(0.2)
            else:
                raise RuntimeError("Объект коллизии car ещё не добавлен в MoveIt")
            self.output_root.mkdir(parents=True, exist_ok=True)
            folder = self.output_root / datetime.now().strftime("scan_%Y%m%d_%H%M%S_%f")
            folder.mkdir()
            self._set_state(output_path=folder)
            route = self._waypoints(self.config)
            (folder / "waypoints.json").write_text(
                json.dumps(route, ensure_ascii=False, indent=2), encoding="utf-8"
            )
            manifest = {
                "frame": "world",
                "units": "m",
                "scan_bounds": self.config["scan_bounds"],
                "captures": [],
                "status": "running",
            }
            for offset in range(0, len(route), 2):
                if self._cancel.is_set():
                    raise InterruptedError("Сканирование остановлено")
                pair = route[offset : offset + 2]
                self._set_state(
                    phase=(
                        f"{offset // 2 + 1}/{len(route) // 2}: оба робота, "
                        f"уровень {pair[0]['level']}, "
                        f"рельс {pair[0]['rail_position']:+.1f} м"
                    )
                )
                self._move_pair(pair)
                time.sleep(float(self.config["settle_seconds"]))
                for index, waypoint in enumerate(pair, start=offset + 1):
                    side = waypoint["robot"]
                    cloud = self._fresh_cloud(side)
                    points, transform = self._points_in_world(cloud)
                    filename = f"{index:02d}_{side}_{waypoint['level']}.ply"
                    save_ply(folder / filename, points)
                    if len(points):
                        all_points.append(points)
                    translation = transform.transform.translation
                    orientation = transform.transform.rotation
                    entry = {
                        **waypoint,
                        "file": filename,
                        "points": int(len(points)),
                        "stamp": [cloud.header.stamp.sec, cloud.header.stamp.nanosec],
                        "sensor_frame": cloud.header.frame_id,
                        "sensor_origin_world": [translation.x, translation.y, translation.z],
                        "sensor_orientation_world": [
                            orientation.x, orientation.y, orientation.z, orientation.w
                        ],
                    }
                    manifest["captures"].append(entry)
                    self._set_state(progress=index)
                    (folder / "manifest.json").write_text(
                        json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8"
                    )

            if not all_points:
                raise RuntimeError("Камеры не получили точек в заданной области scan_bounds")
            merged = np.concatenate(all_points, axis=0)
            voxel_size = float(self.config["voxel_size"])
            keys = np.floor(merged / voxel_size).astype(np.int32)
            _, unique = np.unique(keys, axis=0, return_index=True)
            merged = merged[np.sort(unique)]
            save_ply(folder / "merged.ply", merged)
            manifest["merged_points"] = int(len(merged))
            manifest["status"] = "complete"
            self._set_state(phase=f"Готово: {len(merged)} точек, {folder}")
        except InterruptedError:
            if manifest is not None:
                manifest["status"] = "cancelled"
            self._set_state(phase="Сканирование остановлено")
        except Exception as error:
            if manifest is not None:
                manifest["status"] = "error"
                manifest["error"] = str(error)
            self.get_logger().error(f"Scan failed: {error}")
            self._set_state(phase=f"Ошибка: {error}")
        finally:
            if folder is not None and manifest is not None:
                (folder / "manifest.json").write_text(
                    json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8"
                )
            with self._state_lock:
                self._active = False


class ScanWindow(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle("Сканирование автомобиля")
        self.resize(480, 190)
        layout = QVBoxLayout(self)
        self.status = QLabel()
        self.status.setWordWrap(True)
        self.progress = QProgressBar()
        self.start_button = QPushButton("Начать сканирование")
        self.stop_button = QPushButton("Остановить")
        self.path = QLabel()
        self.path.setWordWrap(True)
        for widget in (self.status, self.progress, self.start_button, self.stop_button, self.path):
            layout.addWidget(widget)
        self.start_button.clicked.connect(node.start_scan)
        self.stop_button.clicked.connect(node.stop_scan)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(200)
        self.refresh()

    def refresh(self):
        phase, done, total, path, active = self.node.state()
        self.status.setText(phase)
        self.progress.setMaximum(total)
        self.progress.setValue(done)
        self.start_button.setEnabled(not active)
        self.stop_button.setEnabled(active)
        self.path.setText(f"Файлы: {path}" if path else "Файлы появятся после запуска")


def main():
    rclpy.init()
    app = QApplication([])
    node = ScanNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()
    window = ScanWindow(node)
    window.show()
    try:
        app.exec_()
    finally:
        if node.state()[4]:
            node.stop_scan()
            node.wait_for_scan_stop(timeout=5.0)
        executor.shutdown(timeout_sec=2.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
