# TurtleBot 4 — Localization, Navigation qua 6 goal và nhận diện YOLO

**Phân vùng bản đồ và điều hướng tiếng Việt:** xem [README giọng nói](src/turtlebot4_voice_nav/README.md) để chạy theo từng terminal: Gazebo/RViz → undock → `ready=true` → Whisper → bật nghe từng câu; có hướng dẫn mở lại GUI Gazebo khi crash và báo cáo pdfLaTeX.

**Bài tập 6 — điều khiển bằng cử chỉ tay:** xem [README bài 6](src/turtlebot4_gesture_control/README.md) để chạy webcam MediaPipe, sân mô phỏng trống, node hành vi và [mở RViz xem quỹ đạo](src/turtlebot4_gesture_control/README.md#5-terminal-d--mở-rviz-và-xem-quỹ-đạo-robot).

Sử dụng bản đồ đã quét ở bài trước để định vị TurtleBot 4 trong Gazebo, chạy **node ROS 2 điều hướng tuần tự qua 6 goal**, đồng thời nhận diện vật thể từ camera bằng **YOLO11n**.

- Môi trường: **Ubuntu 24.04 · ROS 2 Jazzy · Gazebo Harmonic · Python 3.12**.
- Workspace: `~/tb4_ws`; package: `turtlebot4_slam_nav`.
- World: `depot`, đã tích hợp sẵn hai mô hình người, tự xuất hiện mỗi lần chạy.
- Bản đồ: `src/turtlebot4_slam_nav/maps/depot_02.yaml` và `depot_02.pgm`.
- Bộ goal: `src/turtlebot4_slam_nav/config/goals.yaml`.
- Bài này dùng lại map, **không chạy SLAM**. Hướng dẫn bài mapping trước ở [README_LAB04.md](README_LAB04.md).

## 1. Chuẩn bị terminal

Mở 4 terminal:

| Terminal | Công việc |
|---|---|
| A | Gazebo, AMCL, Nav2 và RViz |
| B | Chạy node điều hướng qua 6 goal |
| C | Chạy YOLO và hiển thị camera |
| D | Kiểm tra topic, localization và undock |

**Chạy khối sau trong mỗi terminal mới:**

```bash
cd ~/tb4_ws
export TB4_LAB_DOMAIN_ID=74
source src/turtlebot4_slam_nav/scripts/env.sh
export GZ_IP=127.0.0.1
```

Tất cả terminal phải dùng cùng domain. `env.sh` nạp ROS/workspace và cấu hình Gazebo partition tương ứng; không sửa `.bashrc`.

### Build sau khi thay đổi mã nguồn

```bash
/usr/bin/python3 -m colcon build --symlink-install \
  --packages-select turtlebot4_slam_nav
source install/local_setup.bash
ros2 pkg executables turtlebot4_slam_nav
mkdir -p log/lab05_run
```

Cần thấy các executable `goal_recorder`, `navigate_goals` và `yolo_detector`.

### Chuẩn bị YOLO — thực hiện một lần

Nếu đã có `.venv-yolo` và `models/yolo11n.pt` thì bỏ qua bước này.

```bash
bash src/turtlebot4_slam_nav/scripts/setup_yolo.sh
```

Script tạo môi trường Python riêng và chuẩn bị weights. Khi chạy detector, dùng `.venv-yolo/bin/python` như mục 4 để lấy đúng thư viện YOLO và `cv_bridge`.

## 2. Terminal A — khởi chạy Gazebo, map, AMCL và Nav2

Dừng launch cũ bằng `Ctrl+C` trước khi mở phiên mới.

```bash
ros2 launch turtlebot4_slam_nav navigation.launch.py \
  world:=depot gui_render_engine:=ogre2 \
  map:="$HOME/tb4_ws/src/turtlebot4_slam_nav/maps/depot_02.yaml"
```

Giữ terminal A mở, chờ robot và bản đồ xuất hiện. Gazebo phải ở trạng thái **Play**.

- Launch này bật cả AMCL, Nav2 và RViz.
- Với `depot_02.yaml` và spawn mặc định, **AMCL tự khởi tạo pose `(0, 0, 0)` trong hệ map**; không cần chọn lại `2D Pose Estimate` mỗi lần.
- Hai người đã được thêm trong `worlds/depot_objects.sdf`, không cần chạy lệnh spawn riêng.
- Người xuất hiện trong Gazebo/camera; ảnh bản đồ tĩnh cũ không tự cập nhật hình người.

Nếu GUI Gazebo gây tải cao hoặc crash, dừng phiên cũ rồi chạy chỉ server, vẫn giữ RViz:

```bash
ros2 launch turtlebot4_slam_nav navigation.launch.py \
  world:=depot gazebo_gui:=false \
  map:="$HOME/tb4_ws/src/turtlebot4_slam_nav/maps/depot_02.yaml"
```

Nếu đổi bản đồ hoặc vị trí spawn, phải xác định lại initial pose theo hệ map. Có thể đặt thủ công bằng `2D Pose Estimate`, hoặc truyền `auto_initial_pose:=true` cùng các tham số `initial_pose_x`, `initial_pose_y`, `initial_pose_yaw` đã kiểm chứng.

## 3. Terminal D — kiểm tra robot, localization và undock

Chạy các lệnh kiểm tra liên tục lần lượt; dùng `Ctrl+C` để dừng trước lệnh kế tiếp.

### LiDAR `/scan`

```bash
ros2 topic info /scan -v
ros2 topic echo /scan sensor_msgs/msg/LaserScan \
  --once --field header --qos-reliability best_effort
ros2 topic hz /scan
```

Cần có publisher, timestamp/frame LiDAR và dữ liệu được cập nhật.

### Odometry `/odom`

```bash
ros2 topic info /odom -v
ros2 topic echo /odom nav_msgs/msg/Odometry --once --field pose.pose
ros2 topic hz /odom
```

Pose phải thay đổi khi robot di chuyển. Odometry không phải pose toàn cục đã hiệu chỉnh trên map.

### Lệnh vận tốc `/cmd_vel`

```bash
ros2 topic info /cmd_vel -v
ros2 topic type /cmd_vel
```

Trong cấu hình này, `/cmd_vel` dùng `geometry_msgs/msg/TwistStamped`. Khi node navigation đang điều khiển robot, có thể quan sát một message:

```bash
ros2 topic echo /cmd_vel geometry_msgs/msg/TwistStamped \
  --once --qos-reliability best_effort
```

Nếu robot đang đứng yên và chưa có lệnh, `echo --once` có thể phải chờ. Không gửi lệnh vận tốc thủ công trong chuỗi navigation.

### Kiểm tra AMCL và Nav2

```bash
ros2 lifecycle get /amcl
ros2 lifecycle get /bt_navigator
ros2 topic echo /amcl_pose \
  --once --qos-durability transient_local
ros2 run tf2_ros tf2_echo map base_link \
  --ros-args -p use_sim_time:=true
```

Hai lifecycle node cần ở trạng thái `active`. Trong RViz, đặt **Fixed Frame = map**, kiểm tra vị trí/hướng robot và điểm LiDAR khớp với tường trên bản đồ. Dừng lệnh `tf2_echo` bằng `Ctrl+C` sau khi kiểm tra.

### Cho robot rời dock trước hành trình

```bash
ros2 topic echo /dock_status irobot_create_msgs/msg/DockStatus --once
```

Nếu `is_docked: true`, chạy:

```bash
ros2 action send_goal /undock \
  irobot_create_msgs/action/Undock '{}' --feedback
```

Chờ action `SUCCEEDED` và kiểm tra lại `is_docked: false`. Nếu đã undocked thì không cần gửi lại action.

## 4. Terminal C — chạy camera YOLO

Khởi chạy YOLO **trước node điều hướng**, giữ cửa sổ camera mở trong suốt hành trình:

```bash
mkdir -p log/lab05_run
export YOLO_CONFIG_DIR="$HOME/tb4_ws/log/ultralytics"

./.venv-yolo/bin/python -m turtlebot4_slam_nav.yolo_detector --ros-args \
  -p use_sim_time:=true \
  -p model_path:="$HOME/tb4_ws/models/yolo11n.pt" \
  -p image_topic:=/oakd/rgb/preview/image_raw \
  -p confidence:=0.25 -p image_size:=416 \
  -p max_fps:=2.0 -p cpu_threads:=2 \
  -p device:=cpu -p show_window:=true \
  2>&1 | tee log/lab05_run/yolo.log
```

Camera hiển thị bounding box, tên lớp và confidence khi phát hiện được đối tượng. Không có detection ở một số góc nhìn là tình huống cần đối chiếu ảnh thực tế, không tự kết luận node bị lỗi.

| Topic đầu ra | Nội dung |
|---|---|
| `/yolo/annotated_image` | Ảnh đã vẽ box, nhãn và confidence |
| `/yolo/detections` | JSON chứa timestamp, frame, thời gian xử lý và các detection |

Terminal D có thể theo dõi:

```bash
ros2 topic echo /yolo/detections std_msgs/msg/String
ros2 topic hz /yolo/annotated_image
```

Để xem ảnh trên RViz: **Add → Image → `/yolo/annotated_image`**, chọn Reliability **Best Effort**.

YOLO phục vụ nhận diện/hiển thị. Tránh vật cản vẫn do Nav2 và dữ liệu cảm biến đảm nhiệm; chương trình chưa dùng nhãn YOLO để điều khiển robot.

## 5. Terminal B — chạy tự động qua 6 goal

Bộ tọa độ hiện tại trong hệ `map`:

| Goal | x (m) | y (m) | yaw (rad) |
|---|---:|---:|---:|
| 1 | -2.731 | -2.663 | 0.000 |
| 2 | -2.935 | 2.572 | 0.000 |
| 3 | -6.649 | -0.159 | 0.000 |
| 4 | 4.208 | 3.912 | 0.000 |
| 5 | 8.017 | 1.366 | 0.000 |
| 6 | -6.360 | 3.465 | 0.000 |

Bảng làm tròn để dễ đọc; chương trình dùng tọa độ đầy đủ trong YAML.

```bash
mkdir -p log/lab05_run

ros2 run turtlebot4_slam_nav navigate_goals --ros-args \
  -p use_sim_time:=true \
  -p goals_file:="$HOME/tb4_ws/src/turtlebot4_slam_nav/config/goals.yaml" \
  -p results_file:="$HOME/tb4_ws/log/lab05_run/results.json" \
  -p pause_at_goal:=8.0 \
  2>&1 | tee log/lab05_run/navigation.log
```

Node gửi một action `/navigate_to_pose` tại mỗi thời điểm, chờ `SUCCEEDED` rồi mới gửi goal tiếp theo. Nếu goal lỗi, bị từ chối hoặc timeout, chương trình dừng chuỗi và lưu trạng thái.

**Sau khi node bắt đầu: không teleop, không dùng Nav2 Goal bằng chuột và không đặt lại initial pose.** RViz dùng để quan sát robot, đường đi và costmap.

### Kiểm tra kết quả

```bash
cat log/lab05_run/results.json
```

Lần chạy hoàn tất khi trạng thái chung là `SUCCEEDED` và cả 6 mục trong `goals` đều `SUCCEEDED`.

Để lưu nhiều lần thực nghiệm, đổi đường dẫn `results_file` và file log trước lần chạy tiếp theo. Các lệnh `tee` ở trên ghi lại các file của lần chạy hiện tại trong `log/lab05_run/`.

## 6. Ghi bộ goal mới — chỉ thực hiện trước hành trình

Nếu muốn dùng 6 goal hiện có, bỏ qua mục này. Để khảo sát bộ mới, ghi sang file khác:

```bash
ros2 run turtlebot4_slam_nav goal_recorder --ros-args \
  -p use_sim_time:=true \
  -p output_file:="$HOME/tb4_ws/log/goals_new.yaml" \
  -p yaw:=0.0
```

Trên RViz chọn **Publish Point**, Fixed Frame là `map`, click các điểm trong vùng có thể di chuyển. Publish Point chỉ có x/y; recorder dùng yaw từ tham số. Có thể đổi yaw cho điểm tiếp theo:

```bash
ros2 param set /goal_recorder yaw 1.57079632679
```

Nếu muốn ghi pose AMCL hiện tại, giữ recorder mở và gọi:

```bash
ros2 service call /record_current_pose std_srvs/srv/Trigger '{}'
```

Ghi xong thì `Ctrl+C` recorder. Khi chạy node điều hướng, truyền `goals_file` sang đúng file mới. Không chạy recorder hoặc khảo sát bằng tay trong hành trình tự động.

## 7. Cấu trúc mã nguồn chính

| Tệp / thư mục trong package | Chức năng |
|---|---|
| `launch/navigation.launch.py` | Mở mô phỏng, map, AMCL, Nav2 và RViz |
| `turtlebot4_slam_nav/launch_helpers.py` | Ghép các launch, chọn world và initial pose |
| `turtlebot4_slam_nav/goal_recorder.py` | Ghi tọa độ trước khi chạy |
| `turtlebot4_slam_nav/navigate_goals.py` | Gửi goal tuần tự, theo dõi và lưu kết quả |
| `turtlebot4_slam_nav/yolo_detector.py` | Camera → cv_bridge → YOLO → ảnh/JSON |
| `turtlebot4_slam_nav/goal_utils.py` | Đọc, kiểm tra và lưu goal YAML |
| `turtlebot4_slam_nav/map_utils.py` | Kiểm tra bản đồ trước khi nạp |
| `config/goals.yaml` | Sáu tọa độ mục tiêu |
| `maps/depot_02.yaml`, `maps/depot_02.pgm` | Bản đồ từ bài trước |
| `worlds/depot_objects.sdf` | World depot có sẵn hai người |
| `models/tb4_demo_person/` | Model người cùng mesh/texture |
| `scripts/env.sh` | Nạp ROS/workspace và cấu hình liên lạc |
| `scripts/setup_yolo.sh` | Chuẩn bị môi trường inference |
| `setup.py`, `package.xml` | Đóng gói node và khai báo phụ thuộc |
x
## 9. Dừng chương trình

1. `Ctrl+C` terminal B; node yêu cầu hủy action nếu robot còn đang thực hiện goal.
2. `Ctrl+C` terminal C để dừng YOLO.
3. Dừng các lệnh kiểm tra còn chạy ở D.
4. `Ctrl+C` terminal A và chờ launch thoát.

Đóng cửa sổ Gazebo không đồng nghĩa đã dừng simulator server. Chỉ khởi chạy phiên tiếp theo sau khi dừng launch cũ.

## 10. Xử lý nhanh các tình huống thường gặp

| Hiện tượng | Kiểm tra / cách xử lý |
|---|---|
| Không thấy node/topic | Nạp cùng `env.sh`, domain và `GZ_IP` trong mọi terminal; kiểm tra A còn chạy |
| AMCL báo chưa có initial pose | Kiểm tra đúng `depot_02.yaml`, spawn mặc định và `auto_initial_pose`; nếu đã đổi map/spawn thì đặt pose tương ứng |
| RViz báo queue đầy | Kiểm tra clock, Play, pose AMCL, TF và QoS LaserScan; xem trạng thái dữ liệu mới sau khởi tạo |
| Không có ảnh camera | Kiểm tra `/oakd/rgb/preview/image_raw`, robot standard, Play và QoS sensor |
| Không import được ultralytics | Chạy detector bằng `.venv-yolo/bin/python` và kiểm tra bước setup |
| Gazebo trắng/đen hoặc GUI crash | Dùng `gui_render_engine:=ogre2`; nếu cần chạy `gazebo_gui:=false` với RViz |
| Robot không rời dock | Kiểm tra `/dock_status`, thực hiện `/undock` trước chuỗi goal |
| YAML có điểm sai | Ghi/sửa trước hành trình; điểm thuộc frame map, nằm trong vùng trống và đủ khoảng hở |
| YOLO nhận sai thùng/cột | Đối chiếu ảnh và ghi nhận sai số; confidence hoặc số lần lặp lại không tự chứng minh nhãn đúng |

## Tài liệu tham khảo

- [TurtleBot 4 Navigator](https://turtlebot.github.io/turtlebot4-user-manual/tutorials/turtlebot4_navigator.html).
- [Nav2 Simple Commander API — Jazzy](https://docs.nav2.org/jazzy/configuration_and_development/simple_commander_api/simple_commander_api/).
- [Ultralytics Predict](https://docs.ultralytics.com/modes/predict/).

Đề cho phép dùng Simple Commander hoặc action trực tiếp; chương trình của bài này sử dụng ROS 2 ActionClient `/navigate_to_pose`.
