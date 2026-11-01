# Kiểm chứng — 28/10/2026

Đã thử **đúng luồng microphone thật → Whisper local → phân tích tên vùng → Nav2 → Gazebo**. Không publish văn bản thay tiếng nói, không teleop hoặc dùng Nav2 Goal bằng chuột trong lượt thử này. Node hướng dẫn chỉ bật service thu âm và theo dõi trạng thái.

## Kết quả giọng nói thật

| Câu Whisper nhận được | Tuyến/kết quả |
|---|---|
| Đi đến khu vực 1. | Đến A, ROUTE_COMPLETED |
| Đi đến khu vực hai. | Đến B, ROUTE_COMPLETED |
| Đi đến khu vực ba. | Đến C, ROUTE_COMPLETED |
| Di chuyển từ khu vực một đến khu vực hai. | Robot đang ở C; đi qua A trước rồi đến B, ROUTE_COMPLETED |
| Đi đến nhà ăn. | Tên chưa có trong catalog; REJECTED, **0 goal mới** |

Một/hai/ba và 1/2/3 là alias khai báo của A/B/C. Lượt đầu Whisper nhầm A thành “ám”; lượt đầu đến B nghe “hai” thành “hành”. Hai câu sai đều bị từ chối, robot không nhận tuyến mới. Sau khi thêm alias số và chỉnh ngữ cảnh tiếng Việt dùng chung, các mục thử trên hoàn thành. Không sửa transcript nhận được thành câu mong muốn.

Tổng cộng **7 lần thu**, gồm hai lần cần nói lại. Đây là một lượt demo trên một người dùng, không phải tỷ lệ nhận dạng đúng trên tập kiểm thử rộng. Ngữ cảnh Whisper dùng chung có cả ví dụ tên chưa tồn tại; không đặt đầu ra theo từng câu đang được yêu cầu. Ngưỡng log-probability vẫn -0,75, không hạ ngưỡng để ép chấp nhận.

Bản ghi đầy đủ, giữ cả lần sai:

- [Kết quả theo từng lần thu](validation/live_microphone_2026-10-28/results.json)
- [Sự kiện ASR, transcript và Nav2](validation/live_microphone_2026-10-28/events.jsonl)
- [Log ASR sau chỉnh ngữ cảnh](validation/live_microphone_2026-10-28/speech_context.log)
- [Ảnh Gazebo và RViz](validation/live_microphone_2026-10-28/images/)

Microphone đã tắt sau lượt thử; robot không có action đang chạy. Phiên thử dùng domain 80, camera Gazebo bám robot và Ogre2. Camera RGB mô phỏng giảm xuống 2 Hz riêng cho phiên này để giảm tải; LiDAR/Nav2 vẫn dùng cấu hình mô phỏng hiện có. Không tạo video demo trong lượt thử này.

## Bản đồ và đích đã kiểm tra

Dùng lại `depot_02.yaml`/PGM 602 × 302 của bài trước, độ phân giải 0,05 m/ô. Ba vùng chức năng được phân bằng geodesic trên phần trống liên thông lớn nhất, sau khi chặn unknown và loại vùng sát vật cản/biên dưới 0,30 m. Đây không phải nhận diện phòng kiến trúc.

Bộ đích của lượt nói thật đã được cập nhật thành mặc định:

| Vùng | x | y | yaw | Khoảng hở tĩnh |
|---|---:|---:|---:|---:|
| A | -3.992 | 0.016 | 0.0 | 3,836 m |
| B | 4.808 | -3.484 | 0.0 | 0,708 m |
| C | 3.208 | 3.166 | 0.0 | 0,708 m |

Các đích demo được chọn trong đúng nhãn để hành trình ngắn hơn; `auto_goal` giữ tọa độ tự động ban đầu. [Catalog của lượt thử](validation/live_microphone_2026-10-28/regions.yaml) và [ảnh vùng/đích](config/generated/regions_overlay.png) cho phép đối chiếu. Chuyển pixel dùng tâm ô, đảo hàng ảnh và áp dụng góc quay origin. Catalog được kiểm tra fingerprint map, clearance, vùng liên thông, pixel/world và nhãn goal khi load.

Kết quả action SUCCEEDED và quan sát demo không phải phép đo ground truth về sai số hoặc va chạm. Không có phép đo tỷ lệ nhận dạng trên nhiều người, môi trường âm thanh hoặc loại microphone.

## Build và kiểm thử tự động

- Build package và đăng ký executable thành công.
- **80 kiểm thử đạt:** 9 về occupancy map/catalog, 51 parser tiếng Việt, 20 giao thức action/readiness/hủy/timeout và khởi tạo ROS thật.
- Các trường hợp gồm: origin xoay; unknown bị chặn; goal an toàn và đúng nhãn; map cũ bị thay; alias số còn đúng sau sinh vùng lại; từ A đến B; A → C → B; tên lạ và alias mơ hồ; phủ định; busy; waypoint lỗi không đi tiếp; hủy trong khi chờ chấp nhận; không nhận tuyến mới trước khi action cũ kết thúc.
- Kiểm tra cú pháp Python/Bash đạt. Khởi tạo ROS cần socket loopback, nên test constructor được chạy ngoài sandbox chặn DDS.
- Whisper small nạp thật bằng CPU/int8. WAV im lặng bị từ chối trước publish; suy luận trực tiếp với VAD trên im lặng cho 0 segment ở lượt kiểm tra trước.

## Ghi chú hiển thị và dữ liệu cũ

GUI Ogre1 trong world depot từng làm cảnh trắng và lỗi render; mặc định bài này đã đổi sang Ogre2. [Ảnh xác minh vật liệu](validation/gazebo_ogre2_fixed.png). `gazebo_gui.launch.py` có thể kết nối lại GUI khi server còn chạy.

RViz đã hiển thị map, nhãn vùng và đường Nav2. Có thể có log shader ở lần tải map đầu dù ảnh map xuất hiện sau đó; tương tự hành vi ghi tại [RViz issue 463](https://github.com/ros2/rviz/issues/463).

[Smoke test văn bản trước đây](validation/navigation_text_results.json) dùng bộ đích tự động cũ và chỉ chứng minh phần Nav2; không dùng nó thay cho bằng chứng giọng nói thật ở bảng trên. Mã nguồn SLAM, YOLO và cử chỉ nằm trong package riêng.
