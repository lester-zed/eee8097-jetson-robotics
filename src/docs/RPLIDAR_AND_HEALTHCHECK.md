# RPLIDAR C1 and startup health checks

## Device aliases

The Docker container exposes stable aliases:

```text
/dev/ttyROARM
/dev/ttyRPLIDAR
/dev/video0
```

`docker/docker-compose.yml` maps RoArm and RPLIDAR through separate environment
variables. Recreate the container after changing a host device path.

## SDK preparation

The official Slamtec SDK is tracked as a Git submodule. On a new checkout:

```bash
git submodule update --init --recursive
```

Then, inside Docker:

```bash
cd /workspace/src
./setup_rplidar_sdk.sh
```

## Read-only startup validation

```bash
cd /workspace/src
./run_robot_healthcheck.sh
```

The check verifies distinct device aliases, reads RoArm T=105 feedback, acquires
RPLIDAR scans, and reads Camera frames. It writes:

```text
/workspace/logs/robot_health_latest.json
```

YOLO inference can be included with the current runtime model:

```bash
./run_robot_healthcheck.sh \
  --check-yolo \
  --yolo-model /workspace/models/tissue_pack_yolov8n_v1.pt
```

## RPLIDAR-only measurement

```bash
python3 -m lidar.rplidar_distance \
  --device /dev/ttyRPLIDAR \
  --baudrate 460800 \
  --scans 4 \
  --front-angle 180 \
  --half-width 8 \
  --json
```

The committed values are hardware-specific. Verify the forward angle and scan
plane after moving either the sensor or the target.

## Limitation

RPLIDAR C1 provides a horizontal 2-D angle and range. The pipeline derives X/Y
from that scan, while `localization.target_z_mm` remains a configured,
physically measured grasp height. RPLIDAR alone does not supply object Z.

