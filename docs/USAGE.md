# Hướng dẫn chạy

Các lệnh dựa trên executable và tham số trong mã nguồn, chưa được chạy thử trên phần cứng. Mỗi node chạy ở terminal riêng, source /opt/ros/jazzy/setup.bash và ~/ur3_hri_ws/install/setup.bash trước. Không kết nối điều khiển robot thật bằng prototype này.

## 1. Kinect front/back
Thay SERIAL_FRONT và SERIAL_BACK bằng serial thật; mỗi camera chạy một tiến trình. Registration phải bật cho pose 3D.
```bash
ros2 run kinect_v2_ros2_driver kinect_publisher --ros-args -r __ns:=/kinect_front -p serial:=SERIAL_FRONT -p frame_prefix:=kinect_front -p enable_registration:=true
```
```bash
ros2 run kinect_v2_ros2_driver kinect_publisher --ros-args -r __ns:=/kinect_back -p serial:=SERIAL_BACK -p frame_prefix:=kinect_back -p enable_registration:=true
```
Nếu OpenGL không được hỗ trợ, thêm -p packet_pipeline:=cpu. Kiểm tra topic trước khi tiếp tục.

## 2. Pose 2D trên RGB đã registration
Trong mỗi terminal, activate venv MediaPipe và source workspace. Đặt SIDE lần lượt là front hoặc back.
```bash
cd ~/ur3_hri_ws
source .venv-mediapipe/bin/activate
SIDE=front
python -m ur3_pose_tracking_2d.pose_node --ros-args -r __node:=pose_tracker_2d_${SIDE} -p image_topic:=/kinect_${SIDE}/color_registered/image_raw -p output_prefix:=/kinect_${SIDE}/pose2d -p model_path:="$PWD/models/pose_landmarker_lite.task"
```
Mặc định node dùng RGB raw; phải đổi sang color_registered để node 3D chấp nhận đúng frame và 512×424.

## 3. Pose 3D
Chạy hai terminal, SIDE=front và SIDE=back.
```bash
SIDE=front
ros2 run ur3_pose_tracking_3d pose3d_node --ros-args -r __node:=pose_tracker_3d_${SIDE} -p prefix:=/kinect_${SIDE}
```
```bash
ros2 topic echo /kinect_front/pose3d/status
ros2 topic echo /kinect_back/pose3d/status
```

## 4. Fusion kiểm thử độc lập
Chỉ chạy khi không có publisher dữ liệu thật trên cùng topic base. Dữ liệu giả lập không phải kết quả Kinect.
```bash
ros2 run ur3_human_fusion synthetic_skeleton
```
Terminal khác:
```bash
ros2 run ur3_human_fusion fusion_node
```
Terminal khác:
```bash
ros2 topic echo /human_skeleton_fusion_status
ros2 topic echo /human_skeleton_base
```

## 5. Fusion dữ liệu thật và RViz
Dừng synthetic_skeleton. Cần hiệu chuẩn ngoại tại từng camera, TF hợp lệ tại timestamp và node biến đổi PoseArray camera sang base_link, giữ thứ tự joint và timestamp. Không đổi frame_id để giả lập phép biến đổi. Chỉ chạy fusion dữ liệu thật khi hai topic human_skeleton_base của camera đã được xác minh.
Mở rviz2, chọn Fixed Frame = base_link nếu TF robot có sẵn, thêm RobotModel và PoseArray trỏ tới /human_skeleton_base. PoseArray không tự tạo đường nối skeleton; muốn hiển thị xương cần Marker/MarkerArray riêng. Kiểm tra TF bằng ros2 run tf2_ros tf2_echo base_link kinect_front_depth_optical_frame (tương tự back). Chưa xác minh launch robot/Gazebo hoặc node chuyển đổi trong workspace; không đưa lệnh launch suy đoán.
