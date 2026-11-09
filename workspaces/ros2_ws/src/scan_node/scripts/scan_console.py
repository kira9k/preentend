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
from scipy import ndimage
from scipy.spatial import cKDTree
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
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
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


def filter_vehicle(points, min_z, cluster_voxel_size, min_cluster_points, min_cluster_fraction):
    """Keep substantial vehicle surfaces after removing floor and sparse returns."""
    points = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    points = points[np.isfinite(points).all(axis=1) & (points[:, 2] >= min_z)]
    if not len(points):
        return points
    cells = np.floor((points - points.min(axis=0)) / cluster_voxel_size).astype(np.int32)
    occupied = np.zeros(tuple(cells.max(axis=0) + 1), dtype=bool)
    occupied[cells[:, 0], cells[:, 1], cells[:, 2]] = True
    neighbors = np.ones((3, 3, 3), dtype=bool)
    joined = ndimage.binary_dilation(occupied, structure=neighbors)
    labels, count = ndimage.label(joined, structure=neighbors)
    if count == 0:
        return points[:0]
    point_labels = labels[cells[:, 0], cells[:, 1], cells[:, 2]]
    sizes = np.bincount(point_labels, minlength=count + 1)
    sizes[0] = 0
    largest = int(sizes.max())
    threshold = min(largest, max(min_cluster_points, int(largest * min_cluster_fraction)))
    keep = (sizes >= threshold) & (np.arange(len(sizes)) != 0)
    return points[keep[point_labels]]


def remove_sparse_points(points, radius, neighbors):
    """Discard isolated depth returns without eroding connected wheel surfaces."""
    if len(points) < neighbors:
        return points[:0]
    distances, _ = cKDTree(points).query(
        points, k=neighbors, distance_upper_bound=radius, workers=-1
    )
    return points[np.isfinite(distances[:, -1])]


def complete_projection(points, axes, normal_axis, outer_sign, spacing, lower_gap,
                        upper_gap, upper_threshold, min_distance, fill_upper_silhouette=False):
    """Fill enclosed gaps on one measured exterior envelope, in a 2D view."""
    coordinates = points[:, axes]
    origin = coordinates.min(axis=0)
    cells = np.floor((coordinates - origin) / spacing).astype(np.int32)
    shape = tuple(cells.max(axis=0) + 1)
    occupied = np.zeros(shape, dtype=bool)
    occupied[cells[:, 0], cells[:, 1]] = True
    envelope = np.full(shape, -np.inf, dtype=np.float32)
    np.maximum.at(envelope, (cells[:, 0], cells[:, 1]), points[:, normal_axis] * outer_sign)

    # A small closing bridges sampling seams; enclosed upper holes include glass.
    closed = ndimage.binary_closing(occupied, structure=np.ones((5, 5), dtype=bool))
    upper = origin[1] + (np.arange(shape[1]) + 0.5) * spacing >= upper_threshold
    target = closed | (ndimage.binary_fill_holes(closed) & upper[None, :])
    if fill_upper_silhouette:
        # Transparent windows can have no depth returns at all. Their holes
        # need not be closed loops, so span from the measured door/hood to
        # the measured roofline in each longitudinal column.
        rows = np.arange(shape[1])[None, :]
        highest = np.max(np.where(occupied, rows, -1), axis=1)
        lowest = np.min(np.where(occupied, rows, shape[1]), axis=1)
        has_both_edges = (lowest < shape[1]) & (
            origin[1] + (highest + 0.5) * spacing > upper_threshold + 0.08
        )
        target |= has_both_edges[:, None] & upper[None, :] & (rows <= highest[:, None])
    missing = np.argwhere(target & ~occupied)
    if not len(missing):
        return np.empty((0, 3), dtype=np.float32)
    known = np.argwhere(occupied)
    distances, nearest = cKDTree(known).query(missing, k=min(8, len(known)))
    distances = np.atleast_2d(distances).reshape(len(missing), -1)
    nearest = np.atleast_2d(nearest).reshape(len(missing), -1)
    limit = np.where(upper[missing[:, 1]], upper_gap, lower_gap)
    keep = distances[:, 0] * spacing <= limit
    missing, distances, nearest = missing[keep], distances[keep], nearest[keep]
    if not len(missing):
        return np.empty((0, 3), dtype=np.float32)
    weights = 1.0 / np.maximum(distances, 0.5) ** 2
    values = envelope[known[nearest, 0], known[nearest, 1]]
    normal = (weights * values).sum(axis=1) / weights.sum(axis=1)
    result = np.zeros((len(missing), 3), dtype=np.float32)
    result[:, axes] = origin + (missing + 0.5) * spacing
    result[:, normal_axis] = outer_sign * normal
    # Estimates are useful only where a measured point is genuinely missing.
    separated = cKDTree(points).query(result, workers=-1)[0] >= min_distance
    return result[separated]


def approximate_side_glass(points, sign, spacing, min_distance, glass_start, top):
    """Estimate an outer glass sheet from door tops and the roof edge."""
    side = points[points[:, 1] * sign > 0.3 * np.quantile(np.abs(points[:, 1]), 0.95)]
    if len(side) < 100:
        return points[:0]
    x0 = float(side[:, 0].min())
    xbin = np.floor((side[:, 0] - x0) / spacing).astype(np.int32)
    count = int(xbin.max()) + 1
    highest = np.full(count, -np.inf, dtype=np.float32)
    np.maximum.at(highest, xbin, side[:, 2])
    valid = highest > 0.78 * top
    if not valid.any():
        return points[:0]
    waist = np.full(count, -np.inf, dtype=np.float32)
    roof_edge = np.full(count, -np.inf, dtype=np.float32)
    waist_mask = (side[:, 2] >= glass_start) & (side[:, 2] <= glass_start + 0.15)
    roof_mask = side[:, 2] >= highest[xbin] - 0.08
    np.maximum.at(waist, xbin[waist_mask], side[waist_mask, 1] * sign)
    np.maximum.at(roof_edge, xbin[roof_mask], side[roof_mask, 1] * sign)
    columns = np.arange(count)
    for profile in (waist, roof_edge):
        known = np.isfinite(profile)
        if not known.any():
            return points[:0]
        profile[~known] = np.interp(columns[~known], columns[known], profile[known])
        profile[:] = ndimage.median_filter(profile, size=5)
    x_columns = columns[valid]
    z_rows = np.arange(int(np.ceil((top - glass_start) / spacing)) + 1)
    xx, zz = np.meshgrid(x_columns, z_rows, indexing="ij")
    zz = glass_start + zz * spacing
    keep = zz < highest[xx] - 0.02
    xx, zz = xx[keep], zz[keep]
    fraction = np.clip((zz - glass_start) / np.maximum(highest[xx] - glass_start, spacing), 0, 1)
    yy = sign * (waist[xx] * (1 - fraction) + roof_edge[xx] * fraction)
    estimates = np.stack((x0 + (xx + 0.5) * spacing, yy, zz), axis=1).astype(np.float32)
    return estimates[cKDTree(points).query(estimates, workers=-1)[0] >= min_distance]


def reconstruct_surface(points, spacing, min_distance):
    """Estimate missing side glass, small panel seams and roof patches."""
    if not len(points):
        return points[:0]
    pieces = []
    top = float(np.quantile(points[:, 2], 0.99))
    glass_start = max(0.5, 0.42 * top)
    side_limit = 0.48 * float(np.quantile(np.abs(points[:, 1]), 0.95))
    for sign in (1, -1):
        side = points[points[:, 1] * sign >= side_limit]
        if len(side) > 100:
            pieces.append(complete_projection(
                side, (0, 2), 1, sign, spacing, 0.07, 0.8,
                glass_start, min_distance, fill_upper_silhouette=True
            ))
            pieces.append(approximate_side_glass(
                points, sign, spacing, min_distance, glass_start, top
            ))
    roof = points[points[:, 2] >= glass_start]
    if len(roof) > 100:
        # For the roof, the second projected coordinate is y, so all gaps
        # receive the same conservative search radius.
        pieces.append(complete_projection(
            roof, (0, 1), 2, 1, spacing, 0.12, 0.12,
            float("inf"), min_distance
        ))
    estimates = np.concatenate(pieces) if pieces else points[:0]
    if not len(estimates):
        return estimates
    keys = np.floor(estimates / spacing).astype(np.int32)
    _, unique = np.unique(keys, axis=0, return_index=True)
    estimates = estimates[np.sort(unique)]
    return estimates[cKDTree(points).query(estimates, workers=-1)[0] >= min_distance]


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
        self.car_publisher = self.create_publisher(
            PointCloud2,
            "/scan/car_points",
            QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            ),
        )
        self.preview_publishers = {
            side: self.create_publisher(
                PointCloud2, f"/scan/{side}_preview", QoSProfile(depth=1)
            )
            for side in ("left", "right")
        }
        self._preview_last = {"left": 0.0, "right": 0.0}

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
        if cfg["cluster_voxel_size"] <= 0 or cfg["vehicle_min_z"] < 0:
            raise ValueError("cluster_voxel_size must be positive and vehicle_min_z nonnegative")
        if cfg["min_cluster_points"] < 1 or not 0 < cfg["min_cluster_fraction"] <= 1:
            raise ValueError("min_cluster_points and min_cluster_fraction must be positive")
        if cfg["outlier_radius"] <= 0 or cfg["outlier_neighbors"] < 2:
            raise ValueError("outlier_radius must be positive and outlier_neighbors >= 2")
        if cfg["completion_spacing"] <= 0 or cfg["completion_min_distance"] < 0:
            raise ValueError("completion_spacing must be positive and completion_min_distance nonnegative")
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
        now = time.monotonic()
        if self._active and now - self._preview_last[side] >= 0.5:
            self.preview_publishers[side].publish(message)
            self._preview_last[side] = now

    def _publish_car(self, points):
        header = Header()
        header.frame_id = "world"
        header.stamp = self.get_clock().now().to_msg()
        self.car_publisher.publish(point_cloud2.create_cloud_xyz32(header, points))

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
        self._publish_car(np.empty((0, 3), dtype=np.float32))
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

    def _move_goal(self, waypoints, speed_factor=1.0):
        goal = MoveGroup.Goal()
        request = goal.request
        request.group_name = "dual_arm_camera"
        request.start_state.is_diff = True
        request.num_planning_attempts = 5
        request.allowed_planning_time = 10.0
        request.max_velocity_scaling_factor = float(self.config["velocity_scaling"]) * speed_factor
        request.max_acceleration_scaling_factor = float(self.config["acceleration_scaling"]) * speed_factor
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
        for attempt in range(2):
            accepted = self._wait_future(
                self.move_client.send_goal_async(
                    self._move_goal(waypoints, speed_factor=1.0 if attempt == 0 else 0.5)
                ), 15.0
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
            code = result.result.error_code.val
            if code == MoveItErrorCodes.SUCCESS:
                return
            if code == MoveItErrorCodes.CONTROL_FAILED and attempt == 0:
                self.get_logger().warn("Контроллер не достиг цели; повтор с половинной скоростью")
                continue
            raise RuntimeError(
                f"MoveIt не выполнил общую цель: код {code}"
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
            save_ply(folder / "merged_raw.ply", merged)
            measured = filter_vehicle(
                merged,
                float(self.config["vehicle_min_z"]),
                float(self.config["cluster_voxel_size"]),
                int(self.config["min_cluster_points"]),
                float(self.config["min_cluster_fraction"]),
            )
            measured = remove_sparse_points(
                measured,
                float(self.config["outlier_radius"]),
                int(self.config["outlier_neighbors"]),
            )
            if not len(measured):
                raise RuntimeError("После фильтрации не осталось точек автомобиля")
            save_ply(folder / "measured.ply", measured)
            estimated = reconstruct_surface(
                measured,
                float(self.config["completion_spacing"]),
                float(self.config["completion_min_distance"]),
            )
            save_ply(folder / "reconstructed_only.ply", estimated)
            car_points = np.concatenate((measured, estimated))
            save_ply(folder / "merged.ply", car_points)
            manifest["raw_merged_points"] = int(len(merged))
            manifest["measured_points"] = int(len(measured))
            manifest["reconstructed_points"] = int(len(estimated))
            manifest["merged_points"] = int(len(car_points))
            self._publish_car(car_points)
            manifest["status"] = "complete"
            self._set_state(phase=f"Готово: {len(car_points)} точек автомобиля, {folder}")
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
