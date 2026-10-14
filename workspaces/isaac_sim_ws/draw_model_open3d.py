import itertools
from pathlib import Path

import numpy as np
import open3d as o3d
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering


class PoinsCloud:
    def __init__(self, path: Path):
        self.cloud_points = o3d.io.read_point_cloud(str(path))

    def cut_cloud(self,
                  min_bound=np.array([-1.0, -1.0, -1.0]),
                  max_bound=np.array([ 1.0,  1.0,  1.0])):
        aabb = o3d.geometry.AxisAlignedBoundingBox(
            min_bound=min_bound,
            max_bound=max_bound,
        )
        self.cloud_points = self.cloud_points.crop(aabb)

    def rotate(self, R: np.array):
        self.cloud_points = self.cloud_points.rotate(R, center=(0,0,0))

    def move(self, offset: np.array):
        self.cloud_points.translate(offset)


class MainGui:
    def __init__(self, height=800, width=1280):
        self.window = None
        self.scene_widget = None
        self.height = height
        self.width = width

        self._frame_counter = itertools.count()
        self._geometries = {}        # name -> (geometry, material)
        self._labels = {}            # name -> list[Label3D]

        self.offset = 0.15
        self.label_scale = 1.0
        self.axis_len = 0.6
        self.d_label = self.axis_len + self.offset

    def _init_gui(self):
        # initialize() должен вызываться ровно один раз на процесс
        gui.Application.instance.initialize()
        self.window = gui.Application.instance.create_window(
            "Облако точек", self.width, self.height
        )
        self.scene_widget = gui.SceneWidget()
        self.scene_widget.scene = rendering.Open3DScene(self.window.renderer)
        self.scene_widget.scene.set_background([0.1, 0.1, 0.15, 1.0])
        self.window.add_child(self.scene_widget)

    def _add_geometry(self, name, geometry, material):
        """Добавить/заменить геометрию по имени."""
        if name in self._geometries:
            self.scene_widget.scene.remove_geometry(name)
        self.scene_widget.scene.add_geometry(name, geometry, material)
        self._geometries[name] = (geometry, material)

    def add_label(self, position, text, color=(1.0, 1.0, 1.0)):
        label = self.scene_widget.add_3d_label(
            np.asarray(position, dtype=np.float64), text
        )
        label.color = gui.Color(*color)
        label.scale = self.label_scale
        return label

    def add_point_cloud(self, pcd, name, point_size=2.0, color=None):
        """Добавить облако точек. Если у pcd нет цветов — покрасит в color."""
        if not pcd.has_colors():
            pcd = o3d.geometry.PointCloud(pcd)  # копия, чтобы не мутировать оригинал
            pcd.paint_uniform_color(color if color is not None else [0.8, 0.8, 0.8])

        mat = rendering.MaterialRecord()
        mat.shader = "defaultUnlit"       
        mat.point_size = point_size       

        self._add_geometry(name, pcd, mat)

    def add_mesh(self, mesh, name, color=None):
        """Добавить меш."""
        if color is not None:
            mesh = o3d.geometry.TriangleMesh(mesh)
            mesh.paint_uniform_color(color)
        mesh.compute_vertex_normals()

        mat = rendering.MaterialRecord()
        mat.shader = "defaultLit"

        self._add_geometry(name, mesh, mat)

    def remove_geometry(self, name):
        """Удалить объект (и его подписи, если были)."""
        if name in self._geometries:
            self.scene_widget.scene.remove_geometry(name)
            del self._geometries[name]

        if name in self._labels:
            for lbl in self._labels[name]:
                self.scene_widget.remove_3d_label(lbl)
            del self._labels[name]

    def draw_frame(self, position=np.array([0.0, 0.0, 0.0]),
                   rotation=None, name=None, prefix=None):
        """Нарисовать координатный фрейм + подписи осей."""
        position = np.asarray(position, dtype=np.float64)

        if name is None:
            name = f"frame_{next(self._frame_counter)}"

        mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=self.axis_len, origin=position
        )
        if rotation is not None:
            mesh.rotate(rotation, center=position)

        mat = rendering.MaterialRecord()
        mat.shader = "defaultLit"
        self._add_geometry(name, mesh, mat)

        if name in self._labels:
            for lbl in self._labels[name]:
                self.scene_widget.remove_3d_label(lbl)

        d = self.d_label
        offsets = np.array([[d, 0, 0], [0, d, 0], [0, 0, d]], dtype=np.float64)
        if rotation is not None:
            offsets = offsets @ np.asarray(rotation).T

        colors = [(1.0, 0.3, 0.3), (0.3, 1.0, 0.3), (0.3, 0.5, 1.0)]
        axes = ["X", "Y", "Z"]
        label_prefix = prefix if prefix is not None else name

        labels = [
            self.add_label(position + off, f"{label_prefix}:{ax}", col)
            for off, col, ax in zip(offsets, colors, axes)
        ]
        self._labels[name] = labels

    def camera_setup(self,
                     eye=np.array([3.0, -3.0, 2.0]),
                     lookat=np.array([0.0, 0.0, 0.0]),
                     up=np.array([0.0, 0.0, 1.0]),
                     fov=60.0,
                     near=0.01,
                     far=1000.0):
        """Задать камеру вручную. Никакой автоматики по bounding box."""
        self.scene_widget.scene.camera.set_projection(
            fov, 1.0, near, far,
            rendering.Camera.FovType.Vertical,
        )
        self.scene_widget.scene.camera.look_at(lookat, eye, up)
        self.scene_widget.center_of_rotation = np.asarray(lookat, dtype=np.float64)

    def run(self):
        gui.Application.instance.run()


if __name__ == "__main__":
    # --- 1. Данные ---
    #cloud1 = PoinsCloud("test_data_2_cameras/20261006_195142_216771/camera_2.ply")
    #cloud2 = PoinsCloud("test_data_2_cameras/20261006_195142_216771/camera_2.ply").cloud_points
    cloud1 = PoinsCloud("/home/kira9k/robot_washing/preentend/workspaces/isaac_sim_ws/test_data_2_cameras/new/pointclouds/20261014_135231_762315/camera_2.ply")
    #cloud1.cut_cloud(min_bound=np.array([-1, -1, 0]),
    #                    max_bound=np.array([1,  2,  2]))

    R_z = np.array([[0, -1, 0],
              [1,  0, 0],
              [0,  0, 1]])

    R_x = np.array([[1, 0, 0],
                    [0, 0, -1],
                    [0, 1, 0]])
    cloud1.rotate(R_x)
    cloud1.rotate(R_z)
    cloud1.rotate(R_z)
    cloud1.rotate(R_z)
    offset_1 = np.array([3.0, 0.0, 0.4])
    offset_depth = np.array([0.0, -0.0475, 0.0])
    cloud1.move(offset_1)
    cloud1.move(offset_depth)
    cloud1 = cloud1.cloud_points
    print(cloud1)

    main_gui = MainGui()
    main_gui._init_gui()   

    main_gui.add_point_cloud(cloud1, name="cloud_cam1",
                             point_size=2.0, color=[0.9, 0.4, 0.3])
    #main_gui.add_point_cloud(cloud2, name="cloud_cam2",
    #                         point_size=2.0, color=[0.3, 0.6, 0.9])

    main_gui.draw_frame([0.0, 0.0, 0.0], name="world_cs", prefix="world")
    main_gui.draw_frame([3.0, 0.0, 0.4], name="camera_1", prefix="cam1")
    main_gui.draw_frame([-3.0, 0.0, 0.4], name="camera_2", prefix="cam2")

    main_gui.camera_setup(
        eye=np.array([3.0, -3.0, 2.0]),
        lookat=np.array([0.0, 0.0, 0.4]),
        up=np.array([0.0, 0.0, 1.0]),
        fov=60.0,
        near=0.01,
        far=1000.0,
    )

    main_gui.run()