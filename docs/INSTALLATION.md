# Cài đặt

## Điều kiện
ROS 2 Jazzy (Ubuntu 24.04 là nền tảng Ubuntu tiêu chuẩn của Jazzy), Python tương thích với rclpy, colcon, rosdep, C++17, pkg-config, libfreenect2 và Kinect v2 với USB 3.0/nguồn phù hợp. Driver mặc định dùng OpenGL; libfreenect2 cần hỗ trợ OpenGL hoặc chọn cpu. Cài libfreenect2 theo https://github.com/OpenKinect/libfreenect2 và cấu hình quyền USB theo hướng dẫn upstream.

## Workspace mới
```bash
git clone https://github.com/nguyendoanphuong636-prog/HRC-UR3.git ~/ur3_hri_ws
cd ~/ur3_hri_ws
source /opt/ros/jazzy/setup.bash
pkg-config --modversion freenect2
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```
Nếu đã có workspace, không clone đè. Dùng git pull khi cây làm việc sạch.

## MediaPipe
```bash
cd ~/ur3_hri_ws
python3 -m venv --system-site-packages .venv-mediapipe
source .venv-mediapipe/bin/activate
python -m pip install -r requirements.txt
python -c 'import rclpy, cv2, numpy, mediapipe; print("imports OK")'
test -f models/pose_landmarker_lite.task
```
Cài python3-venv nếu thiếu. requirements.txt chỉ ghi thư viện trực tiếp; chưa khóa phiên bản vì chưa có pip freeze từ máy nghiên cứu. Không ghi đè venv đang hoạt động. ROS console script có thể dùng interpreter lúc build; chạy node 2D bằng python -m trong venv như hướng dẫn USAGE để chắc chắn dùng đúng Python. Kiểm tra tương thích ABI NumPy/OpenCV và rclpy.

## Kiểm tra
```bash
colcon list
ros2 pkg executables kinect_v2_ros2_driver
ros2 pkg executables ur3_pose_tracking_2d
ros2 pkg executables ur3_pose_tracking_3d
ros2 pkg executables ur3_human_fusion
```
UR3 description, Gazebo, MoveIt và cấu hình RViz chưa được xác minh; cần ghi lại workspace/phụ thuộc bên ngoài trước khi tái lập toàn bộ hệ thống.
