# HRC-UR3

Nghiên cứu nhận thức vị trí người phục vụ tương tác/cộng tác người–robot UR3 trên ROS 2 Jazzy, sử dụng hai Kinect v2 và MediaPipe.

## Mục tiêu
- Thu nhận RGB và depth từ Kinect trước/sau.
- Nhận diện các điểm cơ thể 2D và tái tạo skeleton 3D.
- Hiệu chuẩn camera với robot, chuyển skeleton sang base_link và hợp nhất hai góc nhìn.
- Hiển thị người và robot trong RViz; hướng tới nghiên cứu tránh va chạm trong mô phỏng Gazebo.

## Các package
| Package | Chức năng |
| --- | --- |
| kinect_v2_ros2_driver | Driver C++ dùng libfreenect2, RGB/depth và registration |
| ur3_pose_tracking_2d | MediaPipe Tasks Pose Landmarker, JSON landmarks và overlay |
| ur3_pose_tracking_3d | Tái tạo XYZ trong depth optical frame |
| ur3_human_fusion | Hợp nhất PoseArray cùng hệ tọa độ và kiểm thử bằng dữ liệu giả lập |

## Luồng xử lý
Kinect front/back → RGB registered + depth → Pose 2D → Pose 3D camera frame → hiệu chuẩn & biến đổi base_link (cần xác minh triển khai) → fusion → RViz (cần cấu hình robot/TF).

## Tài liệu
- [Cài đặt](docs/INSTALLATION.md)
- [Chạy hệ thống](docs/USAGE.md)
- [ROS 2 topics và TF](docs/ROS2_TOPICS.md)
- [Kiến trúc](docs/ARCHITECTURE.md)
- [Kết quả và kế hoạch kiểm thử](docs/RESULTS.md)

## Phạm vi
Bốn package hiện tại là prototype nghiên cứu chỉ đọc dữ liệu. Không phải hệ thống an toàn được chứng nhận, không cung cấp protective stop. Chưa xác minh mô hình UR3/Gazebo/MoveIt, hiệu chuẩn ngoại tại và cấu hình RViz trong repository này. Không sử dụng để bảo vệ con người khi robot thật vận hành.

## Công việc tiếp theo
- Xác nhận phiên bản thư viện và môi trường chạy thực tế.
- Hiệu chuẩn hai camera với base_link, kiểm tra sai số và đồng bộ.
- Tích hợp RViz, mô hình UR3 và môi trường Gazebo.
- Đánh giá độ chính xác, độ trễ, che khuất, mất dữ liệu; nghiên cứu tránh va chạm trong mô phỏng.
