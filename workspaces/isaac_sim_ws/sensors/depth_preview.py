import cv2
import numpy as np
import omni.ui as ui


class DepthPreviewWindow:
    def __init__(self, camera_name, near=0.1, far=10.0, auto_range=True):
        self.near = near
        self.far = far
        self.auto_range = auto_range
        self.provider = ui.ByteImageProvider()
        self.frame_count = 0
        self.window = ui.Window(
            f"Depth: {camera_name}",
            width=640,
            height=480,
        )

        with self.window.frame:
            ui.ImageWithProvider(
                self.provider,
                fill_policy=ui.IwpFillPolicy.IWP_PRESERVE_ASPECT_FIT,
            )

    def update(self, depth):
        if depth is None:
            return

        if hasattr(depth, "numpy"):
            depth = depth.numpy()
        depth = np.asarray(depth).squeeze()
        if depth.ndim != 2 or depth.size == 0:
            print(f"Depth preview: unexpected frame shape {depth.shape}")
            return

        valid = np.isfinite(depth) & (depth > 0)
        if not np.any(valid):
            self.frame_count += 1
            if self.frame_count % 60 == 1:
                finite_count = int(np.isfinite(depth).sum())
                print(
                    "Depth preview: no valid pixels; "
                    f"shape={depth.shape}, dtype={depth.dtype}, "
                    f"finite={finite_count}/{depth.size}, "
                    f"min={depth.min():.3f}, max={depth.max():.3f}"
                )
            return

        self.frame_count += 1
        if self.frame_count % 60 == 1:
            values = depth[valid]
            p01, p50, p99 = np.percentile(values, [1, 50, 99])
            print(
                "Depth values: "
                f"shape={depth.shape}, min={values.min():.3f}, "
                f"p01={p01:.3f}, median={p50:.3f}, "
                f"p99={p99:.3f}, max={values.max():.3f}"
            )

        values = depth[valid]
        if self.auto_range:
            view_near, view_far = np.percentile(values, [1, 99])
            if view_far <= view_near:
                view_near, view_far = self.near, self.far
        else:
            view_near, view_far = self.near, self.far

        depth_for_view = np.where(valid, depth, view_far)
        depth_for_view = np.clip(depth_for_view, view_near, view_far)
        normalized = (
            (depth_for_view - view_near)
            / (view_far - view_near)
            * 255
        ).astype(np.uint8)

        colored_bgr = cv2.applyColorMap(255 - normalized, cv2.COLORMAP_TURBO)
        colored_rgba = cv2.cvtColor(colored_bgr, cv2.COLOR_BGR2RGBA)
        if colored_rgba.shape[1] > 640:
            preview_height = round(colored_rgba.shape[0] * 640 / colored_rgba.shape[1])
            colored_rgba = cv2.resize(
                colored_rgba, (640, preview_height), interpolation=cv2.INTER_AREA
            )
        height, width = colored_rgba.shape[:2]
        self.provider.set_bytes_data(
            colored_rgba.reshape(-1).tolist(),
            [width, height],
        )

    def close(self):
        self.window.visible = False
        self.provider.destroy()
