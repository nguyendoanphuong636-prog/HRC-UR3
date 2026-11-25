# Kết quả và kiểm thử

## Đã xác nhận qua đọc mã nguồn
- Driver RGB/depth, registration và nội tại camera.
- MediaPipe 2D, tái tạo 3D trong camera frame.
- Fusion PoseArray cùng frame với kiểm tra timestamp, dữ liệu thiếu và nguồn synthetic.

Đây là kiểm tra tĩnh mã nguồn, không phải xác nhận phần cứng hay hiệu năng thực nghiệm.

## Kết quả cần bổ sung từ máy nghiên cứu
| Thử nghiệm | Dữ liệu cần ghi | Trạng thái |
| --- | --- | --- |
| Kinect front/back | FPS, QoS, ảnh RGB/depth | Chưa đính kèm minh chứng |
| Pose 2D | Overlay, tỷ lệ phát hiện | Chưa đính kèm minh chứng |
| Pose 3D | XYZ, số joint hợp lệ, sai số ground truth | Chưa định lượng |
| Hiệu chuẩn base_link | Ma trận ngoại tại, residual, ngày đo | Chưa xác minh |
| Fusion | Chế độ BOTH/FILL_FROM_BACK, độ lệch thời gian | Chưa chạy lại |
| RViz/Gazebo | Ảnh robot và người, cấu hình chạy | Chưa đính kèm |

## Hình ảnh
Lưu ảnh được phép công bố vào docs/images/ (ví dụ pose2d-front.png, pose3d-rviz.png, ur3-gazebo.png). Khi có file thật, thêm liên kết Markdown và chú thích điều kiện đo. Không công bố ảnh nhận dạng người nếu chưa có quyền. Không dùng ảnh giả lập thay cho kết quả thực nghiệm.

## Đánh giá tiếp theo
Đo sai số XYZ bằng mốc chuẩn, độ trễ end-to-end, tỷ lệ joint hợp lệ, ảnh hưởng che khuất và mất camera. Kiểm thử dữ liệu stale/unsynced và NaN. Chỉ đánh giá tránh va chạm trong mô phỏng ở giai đoạn prototype.
