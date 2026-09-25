import os
import traceback
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import omni.timeline
import omni.ui as ui
import omni.usd
import isaacsim.core.utils.stage as stage_utils
from omni.kit.viewport.utility import create_viewport_window, get_active_viewport
from pxr import Sdf
from sensors import CameraManager, DepthCaptureController, DepthPreviewWindow

STAGE_PATH = "/isaac_sim/isaac_sim_ws/stages/test_stage.usd"
CAMERA_CONFIG_PATH = "/isaac_sim/isaac_sim_ws/configs/sensors.yaml"
CAMERA_COMMAND_DIR = "/isaac_sim/isaac_sim_ws/camera_commands"
CAMERA_OUTPUT_DIR = "/isaac_sim/isaac_sim_ws/camera_output"

try:
    if not os.path.isfile(STAGE_PATH):
        raise FileNotFoundError(STAGE_PATH)

    print(f"Открываю сцену: {STAGE_PATH}")

    success = stage_utils.open_stage(STAGE_PATH)

    if not success:
        raise RuntimeError(f"Не удалось открыть сцену: {STAGE_PATH}")

    while stage_utils.is_stage_loading():
        simulation_app.update()

    for _ in range(10):
        simulation_app.update()

    stage = omni.usd.get_context().get_stage()

    if stage is None:
        raise RuntimeError("USD-сцена не загружена")

    print(f"Сцена загружена: {STAGE_PATH}")

    camera_manager = CameraManager(CAMERA_CONFIG_PATH)
    print("Creating enabled camera sensors...", flush=True)
    cameras = camera_manager.create_all()
    if not cameras:
        raise RuntimeError("No enabled cameras in sensors.yaml")
    print("Camera sensors created", flush=True)
    capture_controller = DepthCaptureController(
        CAMERA_COMMAND_DIR,
        CAMERA_OUTPUT_DIR,
        cameras,
    )

    preview_name, preview_camera = next(iter(cameras.items()))
    preview_camera_path = preview_camera["camera_path"]

    main_viewport = get_active_viewport()
    if main_viewport is not None:
        main_viewport.camera_path = Sdf.Path("/OmniverseKit_Persp")

    timeline = omni.timeline.get_timeline_interface()
    timeline.set_looping(True)
    timeline.play()

    # Let the RTX sensor pipeline finish creating its render products first.
    print("Warming up RTX sensors...", flush=True)
    for _ in range(30):
        simulation_app.update()
    print("RTX sensors ready; creating camera viewport", flush=True)

    camera_viewport_window = create_viewport_window(
        name=f"Camera: {preview_name}",
        width=640,
        height=480,
        camera_path=Sdf.Path(preview_camera_path),
    )
    if camera_viewport_window is None:
        raise RuntimeError("Failed to create the camera viewport")

    simulation_app.update()
    docked = ui.dock_window_in_window(
        f"Camera: {preview_name}",
        "Viewport",
        ui.DockPosition.RIGHT,
        0.5,
    )
    if not docked:
        print("Camera viewport opened as a floating window")

    depth_preview = None
    if preview_camera.get("sensor_type") == "single_view_depth":
        near, far = preview_camera["depth_visualization_range"]
        depth_preview = DepthPreviewWindow(
            preview_name,
            near=near,
            far=far,
            auto_range=preview_camera["auto_depth_visualization"],
        )
        simulation_app.update()
        ui.dock_window_in_window(
            f"Depth: {preview_name}",
            f"Camera: {preview_name}",
            ui.DockPosition.BOTTOM,
            0.5,
        )

    frame = 0
    while simulation_app.is_running():
        simulation_app.update()
        frame += 1
        capture_controller.poll_commands()

        for camera_name, camera in cameras.items():
            depth = camera_manager.get_data(
                camera_name, camera["depth_annotator"]
            )
            if depth_preview is not None and camera_name == preview_name:
                depth_preview.update(depth)
            capture_controller.on_depth_frame(camera_name, depth, frame)
            if frame % 60 == 0 and depth is not None:
                print(
                    f"{camera_name}: {camera['depth_annotator']} frame received"
                )

except Exception:
    traceback.print_exc()
    raise

finally:
    if "timeline" in locals():
        timeline.stop()

    if "camera_manager" in locals():
        camera_manager.close()

    if "depth_preview" in locals() and depth_preview is not None:
        depth_preview.close()

    simulation_app.close()
