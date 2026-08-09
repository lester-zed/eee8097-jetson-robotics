# Calibration capture and experiment logging

This stage turns the current real-hardware grasp prototype into a measurable
baseline. It does not replace the current geometry or tune another physical
parameter.

## Safety boundary

`configs/calibration_capture.yaml` inherits the current Camera, RPLIDAR, and
planner settings from `modular_pipeline.yaml`, including the current
`grasp_y_offset_mm: -15.0`, then enforces:

```text
Camera:  REAL
RPLIDAR: REAL
RoArm:   MOCK
```

The calibration entry point does not execute the T=122 startup home, does not
open `/dev/ttyROARM`, and cannot construct the real RoArm adapter. The RPLIDAR
does rotate and the Camera/YOLO stream is live. Keep the target and cables clear
of the scanner.

The target coordinates used for calibration are recorded before the planner's
Y-offset compensation, so the retained `-15 mm` grasp adjustment does not bias
the localization error calculation.

## Validate without opening devices

```bash
cd /workspace/src

python3 tools/calibration_capture.py \
  --config configs/calibration_capture.yaml \
  --validate-config
```

Expected result:

```text
Camera=REAL, RPLIDAR=REAL, RoArm=MOCK, startup home=SKIPPED
```

## 3 x 3 capture protocol

1. Do not move the Camera, RPLIDAR, RoArm base, or work surface during the
   experiment.
2. Mark nine target centres across the usable workspace as `P11` through
   `P33`.
3. Measure every target centre in `base_link` millimetres. Positive X and Y
   must follow the coordinate convention in `FULL_GRASP_PIPELINE.md`.
4. Place one `tissue_pack` target at a marked centre with no second target in
   view.
5. Capture three samples at that point. `--repeat 3` means three requested
   measurements; a failed sample stops the command and is not retried.
6. Repeat for all nine points to obtain 27 successful measurements.

Example:

```bash
cd /workspace/src

./run_calibration_capture.sh \
  --point-id P11 \
  --truth-x-mm 220 \
  --truth-y-mm -60 \
  --truth-z-mm -110 \
  --repeat 3 \
  --notes "3x3 grid, front-left point"
```

Replace the example coordinates with measured values. Do not copy them as
calibration data.

## Log contents

Calibration capture writes:

```text
/workspace/logs/calibration/runs.jsonl
/workspace/logs/calibration/runs.csv
```

The normal real-grasp profile automatically writes equivalent records under:

```text
/workspace/logs/experiments/
```

JSONL is the source of truth. Each run contains:

- run ID, UTC timing, duration, Git commit, merged config snapshot and SHA-256;
- Camera/LiDAR/RoArm modes;
- YOLO label, confidence, bbox and centre;
- LiDAR distance, bearings, sample count, MAD and range span;
- target coordinates in `base_link` and `arm_base`;
- pre-execution Camera/range/target shifts;
- all four planned waypoints and the retained grasp Y offset;
- execution result, pipeline error, failure type and recording error;
- optional measured ground truth, repeat index, notes and human-judged grasp
  outcome;
- per-run X/Y/Z and planar localization errors when ground truth is present.

The host container launcher passes the checked-out Git commit through
`EEE8097_GIT_COMMIT`. If the code is started without that launcher and no Git
metadata is mounted, the record uses `"unknown"` while still preserving the
full config snapshot and hash.

## Generate the report

```bash
cd /workspace/src

./run_calibration_report.sh \
  --input /workspace/logs/calibration/runs.jsonl \
  --output /workspace/logs/calibration/summary.json
```

The report includes total/complete/failed runs, failure types, X/Y bias,
X/Y RMSE, planar RMSE, population standard deviation, per-point statistics,
and annotated grasp success rate.

## Annotate a real grasp after inspection

Automatic logging records the completed trajectory but cannot yet prove that
the object remained in the gripper. Add the human-observed outcome afterwards:

```bash
python3 tools/annotate_experiment.py \
  --run-id 20260810T120000.000000Z-ab12cd34 \
  --grasp-success yes \
  --notes "object remained secured after retreat"
```

The annotation atomically updates the JSONL record and regenerates the CSV.
Do not run annotation while another process is writing the same log.

## Acceptance threshold for the next PR

Do not edit the main hardware calibration from a single sample. First collect
27 successful no-motion observations, inspect failures, and report overall plus
per-point errors. Apply measured geometry in a separate
`fix/apply-measured-calibration` PR so code infrastructure and physical
parameters remain independently reviewable.
