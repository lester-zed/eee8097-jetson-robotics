# EEE8097 Persistent Camera patch v1

Purpose
-------
Keep the USB Camera and YOLO preview alive for the full lifetime of
`main_modular.py`, instead of opening and destroying the Camera inside every
`detect_target()` call.

Why this addresses the current reset problem
--------------------------------------------
The current implementation creates `cv2.VideoCapture` and OpenCV/Qt preview
windows inside the short-lived modular task worker. A second run after
`r -> IDLE -> s` creates a different task worker thread and tries to create the
Camera/Qt resources again. This is consistent with the observed second-run
Camera hang and the Qt "Timers cannot be stopped from another thread" warnings.

This patch changes Camera ownership to one long-lived thread:
- Camera opens once when `ExistingYoloCameraAdapter` is constructed.
- YOLO inference and preview continue during RPLIDAR, localization, planning,
  RoArm motion, COMPLETE/ERROR, reset, and later runs.
- `detect_target()` only waits for a fresh stable detection from that stream.
- `r` no longer closes/reopens the Camera.
- `q` / manager shutdown calls `vision.close()`, which stops the Camera worker.
- Camera release and OpenCV window destruction happen inside the same Camera
  worker thread that created/used the preview.

Files
-----
- `src/vision/yolo_camera.py`
- `src/vision/yolo_camera_adapter.py`
- `src/tools/test_camera_live.py`

No YAML replacement is included
-------------------------------
Your local `modular_pipeline.yaml` contains hardware calibration values that may
be newer than GitHub main (for example LiDAR minimum range and grasp Y offset).
This patch intentionally does NOT overwrite that file.

Apply
-----
From the directory where the ZIP is located:

    unzip -o eee8097_persistent_camera_v1_2026-08-09.zip -d /workspace

The archive paths start with `src/...`, so `/workspace/src/...` is updated.

Or copy the three files manually into `/workspace/src`.

Standalone Camera-only test
---------------------------
From `/workspace/src`:

    python3 tools/test_camera_live.py --config configs/modular_pipeline.yaml

Expected:
- YOLO model loads once.
- Camera opens once.
- Preview remains visible continuously.
- RPLIDAR and RoArm are not opened.
- Press `q` in the preview or Ctrl+C in the terminal to stop.

Full pipeline test
------------------
From `/workspace/src`:

    python3 main_modular.py --config configs/modular_pipeline.yaml

Expected behavior:
1. The Camera preview appears immediately after manager construction.
2. Camera stays live before `s`.
3. Camera stays live through DETECTING -> RANGING -> RECHECKING -> PLANNING ->
   EXECUTING -> COMPLETE/ERROR.
4. After `r`, Camera remains live.
5. A second `s` reuses the same Camera stream instead of reopening `/dev/video*`.

Important
---------
Do not run the standalone Camera test and the full pipeline simultaneously:
both would try to own the same USB Camera device.
