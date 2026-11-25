# Kinect v2 ROS 2 read-only prototype (Phase 2)

## Purpose
Two Kinect v2 RGB + native-depth publishers using libfreenect2 v0.2.0 and ROS 2 Jazzy. No UR3 commands. Not a safety-rated tracking system.

## Requirements
- ROS 2 Jazzy, Ubuntu 24.04, colcon, `ros-jazzy-sensor-msgs`
- libfreenect2 built and installed under `$HOME/freenect2` (the previous installation instructions).
- Both cameras free (close Protonect first), valid USB permissions and sufficient USBFS memory.

## Build
```bash
source /opt/ros/jazzy/setup.bash
sudo apt install -y pkg-config ros-jazzy-sensor-msgs
mkdir -p ~/ur3_hri_ws/src
cp -r kinect_v2_ros2_driver ~/ur3_hri_ws/src/
export PKG_CONFIG_PATH="$HOME/freenect2/lib/pkgconfig:$HOME/freenect2/lib64/pkgconfig:${PKG_CONFIG_PATH:-}"
export LD_LIBRARY_PATH="$HOME/freenect2/lib:$HOME/freenect2/lib64:${LD_LIBRARY_PATH:-}"
cd ~/ur3_hri_ws
colcon build --packages-select kinect_v2_ros2_driver --symlink-install --cmake-clean-cache
source install/setup.bash
ros2 launch kinect_v2_ros2_driver dual_kinect.launch.py
```

## Validation in another terminal
```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_hri_ws/install/setup.bash
ros2 topic list | grep kinect
ros2 topic hz /kinect_front/depth/image_raw
ros2 topic hz /kinect_back/depth/image_raw
ros2 topic info /kinect_front/color/image_raw
ros2 topic echo /kinect_front/depth/camera_info --once
```

## Topic contract
`/kinect_front/{color,depth}/image_raw` and `/kinect_back/{color,depth}/image_raw` (sensor_msgs/Image), plus `.../camera_info` (sensor_msgs/CameraInfo). Native color BGRA8 1920x1080, native depth 32FC1 512x424 in **meters**, no color/depth spatial registration. ROS timestamps are **host receipt times**, not synchronized device exposure timestamps. Front/back labels are arbitrary until physical serial-to-position identification.

## Caveats
- This is a first integration prototype, **not tested on the user's Ubuntu machine**. Compile-time or driver-specific corrections may be necessary.
- The depth camera `CameraInfo` contains factory intrinsics and standard distortion terms, but the RGB calibration representation from libfreenect2 is not equivalent to a simple ROS plumb_bob model. Color distortion is unspecified; do not treat RGB and depth pixels as aligned.
- `frame_id` names alone **do not establish TF**. No camera extrinsic transforms are published. Do not use this for robot human safety or camera fusion until calibrated.
- CPU packet pipeline is used for simplicity and may be slow. Measure rate, frame loss, jitter, and compute load before choosing GPU/OpenCL.
- The existing `ur3_human_fusion` expects skeleton PoseArray in `base_link`, **not** these raw images. Human 2D/3D pose tracking, depth-to-RGB registration, camera extrinsics, confidence fusion, and prediction are separate next stages.

## 2026-11-17 diagnostics patch

- **Fixed a segmentation-fault risk:** libfreenect2 takes ownership of any `PacketPipeline` passed to `openDevice`. The previous `std::unique_ptr<CpuPacketPipeline>` could cause a double deletion on shutdown. The library now owns the pipeline exclusively.
- **Changed processing backend:** the launch file requests `packet_pipeline=opengl`. The previous CPU depth processing took ~214 ms/frame (about 4.6 Hz) on the target laptop. OpenGL is a hypothesis to test; performance and success on the user's hardware are NOT yet confirmed.
- **Fixed shutdown wait:** `hasNewFrame()` is polled every 10 ms. On tinythread builds, the timeout overload of `waitForNewFrame()` is ignored. A worker may hang during shutdown in the prior implementation when a camera stalls.
- Fixed `pkg-config` package name from `libfreenect2` to `freenect2`, and provided the `config/` directory.

### Upgrade existing ROS 2 workspace

Stop Protonect and all Kinect nodes before replacing the source. Unzip this archive, then copy the contained `kinect_v2_ros2_driver` folder into `~/ur3_hri_ws/src/`, replacing the prior version. Source the Jazzy setup, set `PKG_CONFIG_PATH` and `LD_LIBRARY_PATH`, then build with `--cmake-clean-cache`. The command can be found in the assistant's reply.

### Check OpenGL environment

Before trying the ROS2 launch, test the **same backend** with Protonect if possible:
```
cd ~/libfreenect2/build
./bin/Protonect gl -noviewer -frames 100 501294742442
./bin/Protonect gl -noviewer -frames 100 000441552747
```

For a controlled fallback change launch parameters from `opengl` to `cpu` (CPU was observed at only ~4 fps). Do not assume USB packet loss will disappear simply by using OpenGL. This prototype does not perform safety-rated tracking and must not control the UR3.
