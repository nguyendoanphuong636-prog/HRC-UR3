# Phase 5 — Kinect Front: 3D XYZ logging and validation (ROS 2 Jazzy)

Read-only research diagnostics. Do not connect to UR3 control, collision-stop decisions or any safety-rated protection. The current coordinate frame is `kinect_front_depth_optical_frame`, **not** UR3 `base_link`.

This bundle does **not** patch or rebuild the working Kinect driver, Pose2D or Pose3D nodes. All scripts are standalone Python 3 and are compatible with ROS 2 Jazzy system packages. The analysis script has **no ROS dependencies**.

## What is inside

| Script | Function |
|---|---|
| `record_pose3d_xyz.py` | Record exactly nine joint rows per 3D message; metric XYZ, valid flag, MediaPipe 2D visibility/presence and source stamp when matched |
| `analyze_pose3d_xyz.py` | Compute valid%, timestamp gaps, static-person XYZ standard deviation and robust displacement percentiles; write summary CSV |
| `depth_center_check.py` | Validate rectified depth ROI against an approximate tape-measured flat board distance along optical Z |

## Stage 0 — Preconditions / QoS

Start Kinect Front with `enable_registration:=true`, Pose2D with registered RGB input and Pose3D with prefix `/kinect_front`, exactly as in the previous successful test. A known-working Pose2D setting is `max_width:=384`, `max_fps:=8.0`, `publish_overlay:=false`.

With all three nodes running, check:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_hri_ws/install/setup.bash
ros2 topic info --verbose /kinect_front/depth_rect/image_raw
ros2 topic echo /kinect_front/pose3d/status --once
```

The registered depth publisher and Pose3D depth subscriber should both report `Reliability: RELIABLE` for the trial patch. Status should be `OBSERVING_3D_CAMERA_FRAME` when a person is visible. If the ROS topic is unknown, check that the nodes are running. Do **not** infer safety from tracking status.

## Stage 1 — 30 seconds stationary-person 3D capture

Keep the UR3 stopped. Fix the camera in place. Stand where the entire upper body, wrists and hips are visible, stay as still as practical for 30 s, and keep the three perception nodes running. Exit `rqt_image_view` to avoid overhead.

```bash
unzip -o ~/Downloads/front_kinect_phase5_measurement.zip -d ~/ur3_hri_ws/tools/
source /opt/ros/jazzy/setup.bash
source ~/ur3_hri_ws/install/setup.bash
python3 ~/ur3_hri_ws/tools/front_kinect_phase5_measurement/record_pose3d_xyz.py \
  --seconds 30 \
  --csv ~/ur3_hri_ws/data/pose3d/front_static_01_xyz.csv
```

Analyze:

```bash
python3 ~/ur3_hri_ws/tools/front_kinect_phase5_measurement/analyze_pose3d_xyz.py \
  ~/ur3_hri_ws/data/pose3d/front_static_01_xyz.csv
```

Outputs:

- `~/ur3_hri_ws/data/pose3d/front_static_01_xyz.csv` (9 rows per Pose3D frame)
- `~/ur3_hri_ws/data/pose3d/front_static_01_xyz_summary.csv` (9 rows, one per joint)

Raw CSV columns: `stamp_sec`, `stamp_nanosec`, `frame_id`, `joint_id`, `joint_name`, `valid_3d`, `x_m`, `y_m`, `z_m`, `mp_visibility`, `mp_presence`, `x_norm`, `y_norm`, `detected_2d`, `matched_2d_metadata`, `arrival_time_utc`.

IMPORTANT: `mp_visibility` and `mp_presence` are MediaPipe's 2D tracking estimates, **not** a calibrated 3D confidence probability. A missing 2D metadata match leaves these fields blank without affecting measured XYZ. For invalid 3D joints, XYZ cells are left blank intentionally. `orientation.w` is used only as an internal Pose3D validity marker.

The analysis includes per-joint `std_x_mm`, `std_y_mm`, `std_z_mm`, and `radial_p95_from_median_mm`. With a person standing still, these quantify **observed position variation**, which includes body sway, MediaPipe localization jitter and depth error; it does not isolate sensor noise and does not establish absolute XYZ accuracy.

## Stage 2 — Depth reference check with a flat target

Once the static-person capture is finished, **stop Pose2D and Pose3D only**, leave Kinect Front driver running. This avoids additional DDS and GPU load from the 3D pipeline. Position a flat matte non-reflective board at the center of the Kinect depth field, roughly perpendicular to the optical axis. Avoid a board edge, reflective surfaces, fingers, or other closer objects in the central 11 × 11-pixel ROI. View Kinect front registered RGB/depth if necessary to center the board, then close viewers.

Measure an approximate distance *along the camera's optical Z axis*, ideally from the depth/IR optical center to the board face; a tape from the plastic casing is only a rough reference. Replace `1.50` below with the **distance you actually measured**; if uncertain, omit `--reference-m`.

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_hri_ws/install/setup.bash
python3 ~/ur3_hri_ws/tools/front_kinect_phase5_measurement/depth_center_check.py \
  --seconds 12 --u 256 --v 212 --radius 5 --reference-m 1.50
```

The script prints the median measured `Z`, temporal standard deviation, median absolute deviation, an approximate XYZ at the ROI center (from CameraInfo) and the difference from the user-measured reference when provided. This is an *approximate consistency check*, not an intrinsic/extrinsic calibration. Try at least three distinct measured distances (e.g. approximately 1.0, 1.5 and 2.0 m), with actual measurements substituted.

The trial driver publishes `depth_rect/image_raw` with **RELIABLE QoS**; this subscriber uses RELIABLE, while `depth_rect/camera_info` still uses BEST_EFFORT (sensor QoS).

## Stage 3 — Dynamic capture (only after static test)

Restart Pose2D and Pose3D; keep UR3 stopped. For a separate 30-s trial, move arms slowly without major occlusion:

```bash
python3 ~/ur3_hri_ws/tools/front_kinect_phase5_measurement/record_pose3d_xyz.py \
  --seconds 30 \
  --csv ~/ur3_hri_ws/data/pose3d/front_slow_arms_01_xyz.csv
```

This recording is suitable as a first research dataset, but do **not** use the static-jitter statistics as a motion-accuracy score for a moving subject. Before GRU/LSTM training, add data-quality annotations, inspect the true sample-time gaps, assess ground truth and avoid arbitrary interpolation through lost tracking.

## Limitations / interpretation

1. Kinect v2 registered RGB and Depth use the IR depth optical frame at 512 × 424, not the robot base frame. The driver timestamps are from host receipt, not guaranteed hardware timing across two Kinect sensors.
2. A 100% 2D–3D timestamp match in an observation window does not prove zero sensor loss, zero measurement bias, or minimal end-to-end latency.
3. The 9 keypoints in `PoseArray` are index-ordered as 0 Nose; 11/12 shoulders; 13/14 elbows; 15/16 wrists; 23/24 hips. `orientation.w >= 0.5` plus finite XYZ and plausible Z determines the validity flag in the recording.
4. The current Kinect and MediaPipe perception stack is *not safety-rated*. Never treat "no person detected" as permission to move UR3.
5. If rates degrade while recording, reduce other monitoring tools, not thresholds or confidence settings blindly.
