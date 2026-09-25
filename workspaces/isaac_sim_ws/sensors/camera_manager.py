from pathlib import Path

import omni.replicator.core as rep
import omni.usd
import yaml
import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics
from isaacsim.sensors.experimental.rtx import RtxCamera, SingleViewDepthCameraSensor
from isaacsim.storage.native import get_assets_root_path
from isaacsim.core.utils import rotations as rot_utils


DEPTH_CAMERA_ASSETS = {
    "realsense_d455": {
        "label": "RealSense D455",
        "usd_path": "/Isaac/Sensors/RealSense/D455/rsd455.usd",
    },
    "orbbec_gemini2": {
        "label": "Orbbec Gemini 2",
        "usd_path": "/Isaac/Sensors/Orbbec/Gemini2/orbbec_gemini2_v1.0.usd",
        "depth_camera_suffix": "/Stream_rgb",
        "rgb_camera_suffix": "/Stream_rgb",
        "depth_parameters": {
            "baseline": 50.0,
            "focal_length": 628.0,
            "sensor_size": 1280.0,
            "maximum_disparity": 210.0,
            "disparity_confidence": 0.95,
            "minimum_distance": 0.15,
            "maximum_distance": 10.0,
            "noise_mean": 0.5,
            "noise_sigma": 1.0,
        },
    },
    "luxonis_oak_d_tof": {
        "label": "Luxonis OAK-D ToF",
        "usd_path": "/Isaac/Sensors/Luxonis/OAK-D_ToF/oak_d_tof.usd",
        "depth_camera_suffix": "/Camera_pseudo_depth_tof",
        "rgb_camera_suffix": "/Camera_33d",
    },
}


class CameraManager:
    def __init__(self, config_path):
        self.config_path = Path(config_path)
        self.cameras = {}

    def create_all(self):
        with self.config_path.open("r", encoding="utf-8") as config_file:
            config = yaml.safe_load(config_file) or {}

        for camera_config in config.get("cameras", []):
            if not camera_config.get("enabled", True):
                continue
            self._create_camera(camera_config)

        return self.cameras

    def _create_camera(self, config):
        name = config["name"]
        if name in self.cameras:
            raise ValueError(f"Camera name is not unique: {name}")

        model = config.get("model", "generic")
        if model in DEPTH_CAMERA_ASSETS:
            self._create_depth_camera(config)
            return
        if model != "generic":
            raise ValueError(f"Unsupported camera model: {model}")

        stage = omni.usd.get_context().get_stage()
        prim_path = config.get("prim_path")
        clipping_range = config.get("clipping_range", [0.1, 1000.0])

        if prim_path:
            camera_prim = stage.GetPrimAtPath(prim_path)
            if not camera_prim.IsValid() or not camera_prim.IsA(UsdGeom.Camera):
                raise ValueError(f"Camera prim does not exist: {prim_path}")

            UsdGeom.Camera(camera_prim).GetClippingRangeAttr().Set(
                Gf.Vec2f(*clipping_range)
            )
            camera = prim_path
        else:
            position = config["position"]
            parent_path = config.get("parent_path")
            relative_to = config.get("relative_to")

            if parent_path:
                parent_prim = stage.GetPrimAtPath(parent_path)
                if not parent_prim.IsValid():
                    raise ValueError(
                        f"Camera parent does not exist: {parent_path}"
                    )
            elif relative_to:
                reference_prim = stage.GetPrimAtPath(relative_to)
                if not reference_prim.IsValid():
                    raise ValueError(f"Relative prim does not exist: {relative_to}")

                world_transform = UsdGeom.XformCache().GetLocalToWorldTransform(
                    reference_prim
                )
                position = world_transform.Transform(Gf.Vec3d(*position))
                if "look_at" in config:
                    look_at = world_transform.Transform(
                        Gf.Vec3d(*config["look_at"])
                    )
                else:
                    look_at = None
            else:
                look_at = config.get("look_at")

            if "rotation" in config and parent_path:
                xform_path = f"{parent_path}/{name}_Xform"
                camera_path = f"{xform_path}/Camera"
                camera_xform = UsdGeom.Xform.Define(stage, xform_path)
                xformable = UsdGeom.Xformable(camera_xform.GetPrim())
                xformable.ClearXformOpOrder()
                xformable.AddTranslateOp().Set(Gf.Vec3d(*position))
                xformable.AddRotateXYZOp().Set(
                    Gf.Vec3f(*config["rotation"])
                )

                camera_schema = UsdGeom.Camera.Define(stage, camera_path)
                camera_schema.GetClippingRangeAttr().Set(
                    Gf.Vec2f(*clipping_range)
                )
                camera = camera_path
            elif look_at is not None:
                camera = rep.create.camera(
                    position=tuple(position),
                    look_at=tuple(look_at),
                    clipping_range=tuple(clipping_range),
                    parent=parent_path,
                    name=name,
                )
            else:
                raise ValueError(
                    f"Camera {name} requires rotation or look_at"
                )
        render_product = rep.create.render_product(
            camera,
            resolution=tuple(config.get("resolution", [640, 480])),
        )

        annotators = {}
        for annotator_name in config.get("annotators", ["rgb"]):
            annotator = rep.AnnotatorRegistry.get_annotator(annotator_name)
            annotator.attach([render_product])
            annotators[annotator_name] = annotator

        camera_path = (
            camera if isinstance(camera, str) else str(camera.get_prims()[0].GetPath())
        )
        resolution = list(config.get("resolution", [640, 480]))
        self.cameras[name] = {
            "model": "generic",
            "camera": camera,
            "camera_path": camera_path,
            "resolution": resolution,
            "intrinsics": self._get_intrinsics(
                stage, camera_path, resolution
            ),
            "render_product": render_product,
            "annotators": annotators,
            "depth_annotator": config.get(
                "depth_annotator", "distance_to_image_plane"
            ),
        }
        print(f"Camera ready: {name}")

    @staticmethod
    def _get_intrinsics(stage, camera_path, resolution):
        camera_prim = stage.GetPrimAtPath(camera_path)
        if not camera_prim.IsValid() or not camera_prim.IsA(UsdGeom.Camera):
            raise ValueError(f"Camera prim does not exist: {camera_path}")

        camera = UsdGeom.Camera(camera_prim)
        focal_length = float(camera.GetFocalLengthAttr().Get())
        horizontal_aperture = float(camera.GetHorizontalApertureAttr().Get())
        vertical_aperture = float(camera.GetVerticalApertureAttr().Get())
        if horizontal_aperture <= 0 or vertical_aperture <= 0:
            raise ValueError(f"Invalid camera aperture: {camera_path}")

        width, height = map(int, resolution)
        fx = width * focal_length / horizontal_aperture
        fy = height * focal_length / vertical_aperture
        cx = width * 0.5
        cy = height * 0.5
        return {
            "fx": fx,
            "fy": fy,
            "cx": cx,
            "cy": cy,
            "matrix": [
                [fx, 0.0, cx],
                [0.0, fy, cy],
                [0.0, 0.0, 1.0],
            ],
            "source_camera_path": camera_path,
            "projection": "pinhole",
        }

    def _create_depth_camera(self, config):
        name = config["name"]
        model = config["model"]
        profile = DEPTH_CAMERA_ASSETS[model]
        prim_path = config["prim_path"]
        assets_root = get_assets_root_path()
        if not assets_root:
            raise RuntimeError("Isaac Sim assets root is not available")

        asset_path = assets_root + profile["usd_path"]
        camera = RtxCamera.create(
            prim_path,
            usd_path=asset_path,
        )

        stage = omni.usd.get_context().get_stage()
        rig_prim = stage.GetPrimAtPath(prim_path)
        if not rig_prim.IsValid():
            raise RuntimeError(
                f"{profile['label']} rig prim was not created: {prim_path}"
            )

        rig_xform = UsdGeom.Xformable(rig_prim)
        rig_xform.ClearXformOpOrder()
        rig_xform.AddTranslateOp(opSuffix="config").Set(
            Gf.Vec3d(*config.get("translation", [0.0, 0.0, 0.0]))
        )
        if "orientation" in config:
            x, y, z = config["orientation"]
            w, x, y, z = rot_utils.euler_angles_to_quat(
                np.array([x,y,z]),
                degrees=True
            )

            rig_xform.AddOrientOp(opSuffix="config").Set(
                Gf.Quatf(w, Gf.Vec3f(x, y, z))
            )
        else:
            rig_xform.AddRotateXYZOp(opSuffix="config").Set(
                Gf.Vec3f(*config.get("rotation", [0.0, 0.0, 0.0]))
            )

        if config.get("fixed", True):
            rigid_body_count = 0
            for prim in stage.Traverse():
                if not str(prim.GetPath()).startswith(prim_path):
                    continue
                if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                    rigid_body = UsdPhysics.RigidBodyAPI(prim)
                    rigid_body.CreateKinematicEnabledAttr(True)
                    rigid_body_count += 1
            print(f"{profile['label']} fixed rigid bodies: {rigid_body_count}")

        camera_prims = [
            prim
            for prim in stage.Traverse()
            if str(prim.GetPath()).startswith(prim_path)
            and prim.IsA(UsdGeom.Camera)
        ]

        depth_camera_path = None
        render_product_paths = []
        for child in stage.Traverse():
            if child.GetTypeName() != "RenderProduct":
                continue
            render_product_paths.append(str(child.GetPath()))
            if not child.HasRelationship("camera"):
                continue
            targets = child.GetRelationship("camera").GetTargets()
            if len(targets) != 1 or not str(targets[0]).startswith(prim_path):
                continue
            depth_suffix = profile.get("depth_camera_suffix")
            if depth_suffix and not str(targets[0]).endswith(depth_suffix):
                continue
            has_depth_api = child.HasAPI("OmniSensorDepthSensorSingleViewAPI")
            has_depth_attributes = any(
                attr.GetName().startswith("omni:rtx:post:depthSensor:")
                for attr in child.GetAttributes()
            )
            if has_depth_api or has_depth_attributes:
                depth_camera_path = str(targets[0])
                break

        has_depth_template = depth_camera_path is not None
        camera_paths = [str(prim.GetPath()) for prim in camera_prims]
        if depth_camera_path is None:
            depth_suffix = profile.get("depth_camera_suffix")
            depth_camera_path = next(
                (
                    path
                    for path in camera_paths
                    if depth_suffix and path.endswith(depth_suffix)
                ),
                None,
            )

        if depth_camera_path is None:
            raise RuntimeError(
                f"No depth camera found in {profile['label']} asset. "
                f"Cameras: {camera_paths}; render products: {render_product_paths}"
            )

        depth_camera = RtxCamera(
            depth_camera_path,
            reset_xform_op_properties=False,
        )
        if has_depth_template:
            depth_camera._asset_root_path = prim_path

        width, height = config.get("resolution", [640, 480])
        annotator_names = config.get(
            "annotators", ["depth_sensor_distance"]
        )
        sensor = SingleViewDepthCameraSensor(
            depth_camera,
            resolution=(height, width),
            annotators=annotator_names,
        )

        uses_stereo_depth = any(
            item.startswith("depth_sensor_") for item in annotator_names
        )
        depth_parameters = profile.get("depth_parameters")
        if uses_stereo_depth and depth_parameters:
            sensor.set_sensor_baseline(depth_parameters["baseline"])
            sensor.set_sensor_focal_length(depth_parameters["focal_length"])
            sensor.set_sensor_size(depth_parameters["sensor_size"])
            sensor.set_sensor_maximum_disparity(
                depth_parameters["maximum_disparity"]
            )
            sensor.set_sensor_disparity_confidence(
                depth_parameters["disparity_confidence"]
            )
            sensor.set_sensor_distance_cutoffs(
                minimum_distance=depth_parameters["minimum_distance"],
                maximum_distance=depth_parameters["maximum_distance"],
            )
            sensor.set_sensor_noise_parameters(
                noise_mean=depth_parameters["noise_mean"],
                noise_sigma=depth_parameters["noise_sigma"],
            )
            sensor.set_sensor_disparity_noise_downscale(1.0)
        if uses_stereo_depth:
            sensor.set_enabled_post_processing(True)

        # The RTX sensor configures camera transforms during construction, so
        # apply the image-plane roll only after sensor initialization.
        optical_roll = float(config.get("optical_roll", 0.0))
        if optical_roll:
            for camera_prim in camera_prims:
                UsdGeom.Xformable(camera_prim).AddRotateZOp(
                    opSuffix="sensorRoll"
                ).Set(optical_roll)

        optical_yaw = float(config.get("optical_yaw", 0.0))
        if optical_yaw:
            for camera_prim in camera_prims:
                UsdGeom.Xformable(camera_prim).AddRotateYOp(
                    opSuffix="sensorYaw"
                ).Set(optical_yaw)

        rgb_suffix = profile.get("rgb_camera_suffix")
        viewport_camera_path = next(
            (
                path
                for path in camera_paths
                if (rgb_suffix and path.endswith(rgb_suffix))
                or "Pseudo_Depth" in path
            ),
            camera_paths[0] if camera_paths else None,
        )
        if viewport_camera_path is None:
            raise RuntimeError(f"No Camera prim found in {prim_path}")

        self.cameras[name] = {
            "model": model,
            "sensor_type": "single_view_depth",
            "camera": depth_camera,
            "camera_path": viewport_camera_path,
            "depth_camera_path": depth_camera_path,
            "resolution": [width, height],
            "intrinsics": self._get_intrinsics(
                stage, depth_camera_path, [width, height]
            ),
            "sensor": sensor,
            "annotators": {item: None for item in annotator_names},
            "depth_annotator": config.get(
                "depth_annotator", "depth_sensor_distance"
            ),
            "depth_visualization_range": config.get(
                "depth_visualization_range", [0.1, 10.0]
            ),
            "auto_depth_visualization": config.get(
                "auto_depth_visualization", True
            ),
            "noise_model": config.get("noise_model"),
            "noise_parameters": config.get("noise_parameters", {}),
            "noise_rng": np.random.default_rng(config.get("noise_seed", 42)),
        }
        print(
            f"{profile['label']} ready: {name}; "
            f"depth camera: {depth_camera_path}"
        )

    def get_data(self, camera_name, annotator_name):
        camera = self.cameras[camera_name]
        if camera.get("sensor_type") == "single_view_depth":
            data, _ = camera["sensor"].get_data(annotator_name)
            if camera.get("noise_model") == "tof":
                return self._apply_tof_noise(data, camera)
            return data
        return camera["annotators"][annotator_name].get_data()

    def _apply_tof_noise(self, depth, camera):
        if depth is None:
            return None
        if hasattr(depth, "numpy"):
            depth = depth.numpy()

        noisy = np.asarray(depth, dtype=np.float32).copy()
        values = noisy[..., 0] if noisy.ndim == 3 and noisy.shape[-1] == 1 else noisy
        params = camera["noise_parameters"]
        near, far = camera["depth_visualization_range"]
        rng = camera["noise_rng"]

        valid = np.isfinite(values) & (values >= near) & (values <= far)
        values[~valid] = 0.0

        # Approximate ToF range noise; edge artifacts and dropouts are added below.
        sigma = (
            float(params.get("sigma_base_m", 0.002))
            + float(params.get("sigma_per_m2", 0.001)) * values * values
        )
        gaussian = rng.normal(size=values.shape).astype(np.float32) * sigma
        values[valid] += gaussian[valid]

        quantization = float(params.get("quantization_m", 0.001))
        if quantization > 0:
            values[valid] = np.round(values[valid] / quantization) * quantization

        gradient_x = np.zeros_like(values)
        gradient_y = np.zeros_like(values)
        gradient_x[:, 1:] = np.abs(values[:, 1:] - values[:, :-1])
        gradient_y[1:, :] = np.abs(values[1:, :] - values[:-1, :])
        edges = np.maximum(gradient_x, gradient_y) > float(
            params.get("edge_threshold_m", 0.08)
        )
        flying_pixels = edges & valid & (
            rng.random(values.shape)
            < float(params.get("flying_pixel_probability", 0.15))
        )
        values[flying_pixels] += rng.normal(
            0.0,
            float(params.get("edge_error_sigma_m", 0.04)),
            size=np.count_nonzero(flying_pixels),
        ).astype(np.float32)

        dropout = valid & (
            rng.random(values.shape)
            < float(params.get("dropout_probability", 0.005))
        )
        values[dropout | (values < near) | (values > far)] = 0.0
        return noisy

    def close(self):
        for camera in self.cameras.values():
            if camera.get("sensor_type") == "single_view_depth":
                sensor = camera["sensor"]
                if hasattr(sensor, "destroy"):
                    sensor.destroy()
                continue
            render_product = camera["render_product"]
            for annotator in camera["annotators"].values():
                annotator.detach([render_product])
        self.cameras.clear()
