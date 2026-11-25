# UR3 — Phase 6/7 preliminary research tooling

**Read-only. No robot command or protective stop. Do not use it to control moving hardware.**

Requires the already-built Phase 1 ur3_human_fusion, Phase 3 MediaPipe, Phase 4 registration + reliable depth patch, and `.venv-mediapipe` in `~/ur3_hri_ws`. Stop previous camera and pose processes before launching. No rebuild required. Use a fresh terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_hri_ws/install/setup.bash
export LD_LIBRARY_PATH="$HOME/freenect2/lib:${LD_LIBRARY_PATH:-}"
ros2 launch ~/ur3_hri_ws/tools/ur3_phase67_demo/dual_pose.launch.py
```

Check both `/kinect_front/human_skeleton_depth` and `/kinect_back/human_skeleton_depth` and both pose3d/status topics. Registration enabled for both. This may be demanding on shared USB and CPU.

Separate terminal (source ROS/workspace):

```bash
python3 ~/ur3_hri_ws/tools/ur3_phase67_demo/phase67_view_fuse.py
```

In raw mode there is no base_link output. Use `rviz2`, set Fixed Frame to `kinect_front_depth_optical_frame`, add MarkerArray `/phase67/front/markers`; for Back use its optical frame and `/phase67/back/markers`. Or use rqt_image_view for the two pose2d/overlay topics. These raw images and markers are not registered across cameras.

## Measured calibration

Keep cameras and UR3 fixed. Establish actual base_link origin and axis directions using the URDF/TF and surveyed physical references; do not assume controller base and ROS base_link are interchangeable. For each camera identify 5 training points and 2 independent check points distributed across the shared workspace, different heights and depths, on rigid nonreflective targets with known XYZ in base_link. Pixel coordinates must be on 512x424 registered color/depth image; use a saved registered image in an image viewer with pixel coordinates (not native 1920x1080 RGB). Make a marked flat area around each point; ROI must not cross object edges. With target stationary, the existing depth_center_check can read its approximate camera XYZ, e.g.:

```bash
python3 ~/ur3_hri_ws/tools/front_kinect_phase5_measurement/depth_center_check.py --prefix /kinect_front --seconds 6 --u 258 --v 206 --radius 2
```

Replace u/v with actual target pixel. Use /kinect_back for Back. If necessary repeat original image/ROI capture and average multiple measurements. Copy actual camera XYZ and separately measured base_link XYZ in meters into calibration_points_template.csv (one row per physical point and camera); never fill with guessed measurements. No need for same point ID to be visible in both cameras if both sets have reliable base coordinates. Solve:

```bash
python3 solve_calibration.py calibration_points_filled.csv --out calibration.json
```

Default preliminary research rejection thresholds: fit RMSE 50 mm, independent maximum 80 mm. These are discretionary demo thresholds, not guaranteed accuracy or safe clearances. Solver rejects collinear points, requires 4 or more fit points and at least 1 check point per camera; prefer 5+2. Check more targets later. If solve fails, do not use old/stale calibration.

Stop raw viewer and restart with actual calibration:

```bash
python3 phase67_view_fuse.py --calibration calibration.json
```

Outputs `/phase67/front/human_skeleton_base`, `/phase67/back/human_skeleton_base`, `/phase67/human_skeleton_base`, `/phase67/fused/markers`, `/phase67/status`. It applies R p + t directly to points; does NOT publish TF to avoid overwriting existing robot TF tree. RViz Fixed Frame base_link, MarkerArray `/phase67/fused/markers`. Calibrated raw camera marker topics still use original optical frame. Existing robot model can be displayed with its own TF/robot_description if available.

Pairs are closest available host timestamps <=150 ms, 250 ms buffer wait, samples older than 1 s rejected at processing, each sample consumed once. Header is older source stamp, not publish timestamp. Can publish single-camera fallbacks explicitly marked; mode and paired flag are distinct. Reuses Phase 1 fusion: average if within 0.20 m; prefer Front for conflicts, fill missing joints from other camera. Conflict count reported. Assumes one person; no cross-camera person association. Host stamps are not hardware sync. All thresholds are research defaults, not safe limits. No claims of calibrated fusion until actual measurement.

Python syntax checked and Kabsch tested on synthetic data. ROS runtime not available in authoring environment. Real dual throughput and calibration must be verified on target machine.
