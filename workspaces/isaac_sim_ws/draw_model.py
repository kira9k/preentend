from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d


FOLDER = Path("test_data_2_cameras/20261006_195117_281688")
MAX_DISTANCE = 4.0

POSITION_1 = np.array([3.0, 0.0, 0.4])
POSITION_2 = np.array([-3.0, 0.0, 0.4])

ROTATION_1 = np.array([
    [ 0.0, 0.0, -1.0],
    [-1.0, 0.0,  0.0],
    [ 0.0, 1.0,  0.0],
])

ROTATION_2 = np.array([
    [0.0, 0.0, 1.0],
    [1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
])

MAX_DISTANCE = 4.0

def load_and_transform(path, rotation, position):
    cloud = o3d.io.read_point_cloud(str(path))
    points = np.asarray(cloud.points)

    valid = np.isfinite(points).all(axis=1)
    points = points[valid]

    # Ограничиваем расстояние до преобразования:
    # здесь начало координат — соответствующая камера.
    distance = np.linalg.norm(points, axis=1)
    points = points[(distance > 0) & (distance <= MAX_DISTANCE)]

    # Точки хранятся строками, поэтому используем R.T.
    return points @ rotation.T + position


points_1 = load_and_transform(
    FOLDER / "camera_1.ply", ROTATION_1, POSITION_1
)
points_2 = load_and_transform(
    FOLDER / "camera_2.ply", ROTATION_2, POSITION_2
)

if len(points_1) + len(points_2) == 0:
    raise RuntimeError("Оба облака пустые.")

combined = np.vstack([points_1, points_2])

fig = plt.figure(figsize=(12, 8))
ax = fig.add_subplot(111, projection="3d")

# Разные цвета позволяют проверить совмещение.
for points, color, label in [
    (points_1, "tab:blue", "Camera 1"),
    (points_2, "tab:orange", "Camera 2"),
]:
    ax.scatter(
        points[:, 0], points[:, 1], points[:, 2],
        color=color,
        label=label,
        s=0.3,
        linewidths=0,
        depthshade=False,
    )

lower = combined.min(axis=0)
upper = combined.max(axis=0)
center = (lower + upper) / 2
radius = max(float((upper - lower).max()) / 2, 0.01)

ax.set_xlim(center[0] - radius, center[0] + radius)
ax.set_ylim(center[1] - radius, center[1] + radius)
ax.set_zlim(center[2] - radius, center[2] + radius)

#ax.set_zlim(0, 0.5)

ax.set_box_aspect((1, 1, 1))

ax.set_xlabel("World X")
ax.set_ylabel("World Y")
ax.set_zlabel("World Z")
ax.set_title(f"Combined cloud — {len(combined):,} points")
ax.legend()

plt.tight_layout()
plt.show()