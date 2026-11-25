# ROS 2 topics và TF

Dùng P=/kinect_front hoặc /kinect_back. Bảng giả định namespace và prefix được cấu hình như USAGE.

| Topic | Message | Ý nghĩa |
| --- | --- | --- |
| P/color/image_raw | sensor_msgs/msg/Image | RGB gốc 1920×1080, bgra8/rgba8 |
| P/color/camera_info | sensor_msgs/msg/CameraInfo | Nội tại RGB |
| P/depth/image_raw | sensor_msgs/msg/Image | Depth gốc 512×424, 32FC1, mét |
| P/depth/camera_info | sensor_msgs/msg/CameraInfo | Nội tại depth |
| P/color_registered/image_raw | sensor_msgs/msg/Image | RGB registered 512×424 |
| P/depth_rect/image_raw | sensor_msgs/msg/Image | Depth đã hiệu chỉnh, 32FC1, mét |
| P/depth_rect/camera_info | sensor_msgs/msg/CameraInfo | Nội tại depth rect |
| P/pose2d/landmarks_json | std_msgs/msg/String | research_pose2d_v1, tọa độ chuẩn hóa |
| P/pose2d/overlay | sensor_msgs/msg/Image | Ảnh chú thích, bgr8, khi có subscriber |
| P/pose2d/status | std_msgs/msg/String | Trạng thái 2D |
| P/human_skeleton_depth | geometry_msgs/msg/PoseArray | XYZ camera optical frame |
| P/pose3d/status | std_msgs/msg/String | Trạng thái 3D |
| P/human_skeleton_base | geometry_msgs/msg/PoseArray | Đầu vào fusion, cần biến đổi ngoại tại |
| /human_skeleton_base | geometry_msgs/msg/PoseArray | Kết quả fusion |
| /human_skeleton_fusion_status | std_msgs/msg/String | Trạng thái fusion |

## Frame và timestamp
Driver đặt kinect_front_color_optical_frame và kinect_front_depth_optical_frame (tương tự back). RGB registered và depth rect dùng depth optical frame. Timestamp là thời điểm nhận trên host, không phải đồng bộ phần cứng giữa hai Kinect. Node 3D yêu cầu timestamp 2D trùng chính xác depth, frame đúng và kích thước 512×424. Depth rect dùng RELIABLE, camera_info và registered RGB dùng SensorDataQoS (best effort). Fusion subscriber best effort.

## Schema
Thứ tự 9 điểm: 0 (mũi), 11/12 (vai), 13/14 (khuỷu), 15/16 (cổ tay), 23/24 (hông), theo MediaPipe. Fusion còn chấp nhận schema 51 điểm (9 body + 42 hand), nhưng node 2D/3D hiện chỉ xuất 9 điểm. Node 3D đánh dấu điểm thiếu bằng NaN và orientation.w=0; điểm hợp lệ w=1. Đây là quy ước nghiên cứu, không phải orientation đo được. Fusion xuất NaN với w=1 cho điểm thiếu, nên consumer phải kiểm tra tọa độ hữu hạn.
TF cần có phép biến đổi từ từng depth optical frame sang base_link đã hiệu chuẩn; chưa xác minh thông số ngoại tại.
