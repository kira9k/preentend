import json
from datetime import datetime, timezone
from pathlib import Path

import numpy as np


class DepthCaptureController:
    """Accept filesystem commands and save depth frames from the main Kit loop."""

    def __init__(self, command_root, output_root, cameras):
        self.command_root = Path(command_root)
        self.inbox = self.command_root / "inbox"
        self.status_dir = self.command_root / "status"
        self.output_root = Path(output_root)
        self.cameras = cameras
        self.jobs = {}

        self.inbox.mkdir(parents=True, exist_ok=True)
        self.status_dir.mkdir(parents=True, exist_ok=True)
        self.output_root.mkdir(parents=True, exist_ok=True)

    def poll_commands(self):
        for command_path in sorted(self.inbox.glob("*.json")):
            request_id = command_path.stem
            try:
                command = json.loads(command_path.read_text(encoding="utf-8"))
                request_id = str(command.get("id", request_id))
                self._start_capture(request_id, command)
            except Exception as error:
                self._write_status(
                    request_id,
                    {"state": "rejected", "error": str(error)},
                )
                print(f"Depth capture command rejected: {error}", flush=True)
            finally:
                command_path.unlink(missing_ok=True)

    def on_depth_frame(self, camera_name, depth, simulation_frame):
        if depth is None:
            return

        for request_id, job in list(self.jobs.items()):
            if job["camera"] != camera_name:
                continue

            job["seen"] += 1
            if (job["seen"] - 1) % job["every"] != 0:
                continue

            array = self._to_numpy(depth)
            frame_path = job["output_dir"] / f"depth_{job['saved']:04d}.npy"
            np.save(frame_path, array, allow_pickle=False)
            job["saved"] += 1

            status = {
                "state": "capturing",
                "camera": camera_name,
                "saved": job["saved"],
                "requested": job["frames"],
                "output_dir": str(job["output_dir"]),
                "last_simulation_frame": simulation_frame,
            }

            if job["saved"] >= job["frames"]:
                status["state"] = "completed"
                status["completed_at"] = self._now()
                del self.jobs[request_id]
                print(
                    f"Depth capture {request_id} completed: "
                    f"{job['frames']} frames -> {job['output_dir']}",
                    flush=True,
                )

            self._write_status(request_id, status)

    def _start_capture(self, request_id, command):
        if command.get("action") != "save_depth":
            raise ValueError("unsupported action; expected 'save_depth'")

        camera_name = str(command.get("camera", ""))
        if camera_name not in self.cameras:
            available = ", ".join(sorted(self.cameras))
            raise ValueError(
                f"unknown camera '{camera_name}'; available: {available}"
            )

        frames = int(command.get("frames", 10))
        every = int(command.get("every", 1))
        if frames < 1:
            raise ValueError("frames must be greater than zero")
        if every < 1:
            raise ValueError("every must be greater than zero")
        if request_id in self.jobs:
            raise ValueError(f"request '{request_id}' is already active")

        output_dir = self.output_root / camera_name / request_id
        output_dir.mkdir(parents=True, exist_ok=False)
        camera = self.cameras[camera_name]
        metadata = {
            "id": request_id,
            "camera": camera_name,
            "annotator": camera["depth_annotator"],
            "resolution": camera.get("resolution"),
            "intrinsics": camera.get("intrinsics"),
            "frames": frames,
            "every": every,
            "dtype": "float32",
            "units": "sensor_native",
            "created_at": self._now(),
        }
        self._write_json(output_dir / "metadata.json", metadata)

        self.jobs[request_id] = {
            "camera": camera_name,
            "frames": frames,
            "every": every,
            "seen": 0,
            "saved": 0,
            "output_dir": output_dir,
        }
        self._write_status(
            request_id,
            {
                "state": "capturing",
                "camera": camera_name,
                "saved": 0,
                "requested": frames,
                "output_dir": str(output_dir),
            },
        )
        print(
            f"Depth capture {request_id} started: "
            f"camera={camera_name}, frames={frames}, every={every}",
            flush=True,
        )

    @staticmethod
    def _to_numpy(depth):
        if hasattr(depth, "numpy"):
            depth = depth.numpy()
        array = np.asarray(depth, dtype=np.float32)
        if array.ndim == 3 and array.shape[-1] == 1:
            array = array[..., 0]
        if array.ndim != 2:
            raise ValueError(f"expected a 2D depth frame, got shape {array.shape}")
        return np.ascontiguousarray(array)

    def _write_status(self, request_id, status):
        payload = {"id": request_id, **status, "updated_at": self._now()}
        self._write_json(self.status_dir / f"{request_id}.json", payload)

    @staticmethod
    def _write_json(path, payload):
        temporary_path = path.with_suffix(path.suffix + ".tmp")
        temporary_path.write_text(
            json.dumps(payload, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
        temporary_path.replace(path)

    @staticmethod
    def _now():
        return datetime.now(timezone.utc).isoformat()
