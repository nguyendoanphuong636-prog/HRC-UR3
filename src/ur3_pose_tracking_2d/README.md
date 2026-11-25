# Phase 3 — UR3 MediaPipe Pose tracking from one Kinect v2 (2D ONLY)

**Research prototype. Not a robot safety device. No UR3 motion commands.**

Uses `/kinect_front/color/image_raw` (`bgra8`) published by the already working `kinect_v2_ros2_driver`. Detects one human with MediaPipe Pose Landmarker Lite, emits normalized 2D landmarks with original ROS image stamp, optional annotated image, and a stale/no-person heartbeat. **NO RGB-depth registration is performed** and **NO metric 3D points are published**.

## 1. Prepare venv (Ubuntu 24.04 / ROS 2 Jazzy / Python 3.12)

```bash
source /opt/ros/jazzy/setup.bash
sudo apt install -y python3-venv
python3 -m venv --system-site-packages ~/ur3_hri_ws/.venv-mediapipe
touch ~/ur3_hri_ws/.venv-mediapipe/COLCON_IGNORE
source ~/ur3_hri_ws/.venv-mediapipe/bin/activate
python -m pip install --upgrade pip
python -m pip install 'mediapipe==0.10.32'
python - <<'PY'
import sys, mediapipe, rclpy, cv2, numpy
print(sys.version)
print('mediapipe', mediapipe.__version__)
print('opencv', cv2.__version__, 'numpy', numpy.__version__)
print('rclpy OK')
PY
```

Do not `sudo pip install` or use `--break-system-packages`. If import fails, do not blindly upgrade NumPy/OpenCV in the system environment; show the traceback.

## 2. Download Google Pose Landmarker Lite model

```bash
mkdir -p ~/ur3_hri_ws/models
curl -fL 'https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_lite/float16/1/pose_landmarker_lite.task' \
  -o ~/ur3_hri_ws/models/pose_landmarker_lite.task
ls -lh ~/ur3_hri_ws/models/pose_landmarker_lite.task
```

Source: https://developers.google.com/edge/mediapipe/solutions/vision/pose_landmarker

## 3. Copy package and build

```bash
# Unzip ur3_pose_tracking_2d_phase3.zip downloaded from ChatGPT
mkdir -p ~/ur3_hri_ws/src
unzip -o ~/Downloads/ur3_pose_tracking_2d_phase3.zip -d /tmp/pose_phase3
cp -a /tmp/pose_phase3/ur3_pose_tracking_2d ~/ur3_hri_ws/src/
cd ~/ur3_hri_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ur3_pose_tracking_2d --symlink-install
source install/setup.bash
```

## 4. Run

Terminal A: **existing** Kinect ROS 2 driver (do not run Protonect concurrently)

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_hri_ws/install/setup.bash
export LD_LIBRARY_PATH="$HOME/freenect2/lib:${LD_LIBRARY_PATH:-}"
ros2 launch kinect_v2_ros2_driver dual_kinect.launch.py
```

Terminal B: use **venv interpreter explicitly** to make dependency resolution deterministic

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_hri_ws/install/setup.bash
source ~/ur3_hri_ws/.venv-mediapipe/bin/activate
python -m ur3_pose_tracking_2d.pose_node --ros-args \
  -p image_topic:=/kinect_front/color/image_raw \
  -p output_prefix:=/kinect_front/pose2d \
  -p model_path:=$HOME/ur3_hri_ws/models/pose_landmarker_lite.task \
  -p max_width:=640 -p max_fps:=6.0
```

Terminal C: verify small output (image transport is BEST_EFFORT; JSON output is default reliable):

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_hri_ws/install/setup.bash
ros2 topic echo /kinect_front/pose2d/status
ros2 topic echo /kinect_front/pose2d/landmarks_json --once
```

Optional overlay viewer (only works when node has a subscriber):

```bash
sudo apt install -y ros-jazzy-rqt-image-view
ros2 run rqt_image_view rqt_image_view
# Select: /kinect_front/pose2d/overlay
```

## Outputs

- `/kinect_front/pose2d/landmarks_json` — std_msgs/String JSON with original timestamp, frame_id, `detected`, `landmarks` (9 selected MediaPipe IDs: 0, 11–16, 23–24), normalized `x_norm`, `y_norm`, visibility and presence.
- `/kinect_front/pose2d/status` — std_msgs/String JSON heartbeat: `OBSERVING_PERSON_2D`, `NO_POSE_DETECTED_UNCERTAIN`, or `STALE_NO_FRAMES`.
- `/kinect_front/pose2d/overlay` — sensor_msgs/Image (`bgr8`, downscaled), only publishes when subscribed.

**This is an intentionally standalone 2D diagnostic format**. Do NOT remap this output to `/human_skeleton_*`, `/base_link` or any 3D-fusion topic. MediaPipe world coordinates are not automatically camera-calibrated metric Kinect coordinates. Next phase: RGB-depth registration + extrinsic calibration + 3D fusion.

If the source has high latency, reduce `max_width` to 480 and `max_fps` to 4; this affects compute cost, not the Kinect USB throughput. Missing pose may mean occlusion or detection failure, not an empty work area.

## Stop safely

Press Ctrl+C in the pose process first. UR3 must remain disabled from the research pipeline. No robot controllers are used by this package.
