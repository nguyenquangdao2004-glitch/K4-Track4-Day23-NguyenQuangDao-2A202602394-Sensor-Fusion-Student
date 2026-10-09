# Báo cáo bài nộp — Day 23 Sensor Fusion Lab

> Điền file này rồi commit. Cách nộp: [hướng dẫn nộp](../SUBMISSION.md).

## Thông tin học viên

- Họ tên: Nguyễn Quang Đạo
- MSSV: 2A202602394
- Email: nguyenquangdao2004@gmail.com
- Link repo (fork): https://github.com/nguyenquangdao2004-glitch/K4-Track4-Day23-NguyenQuangDao-2A202602394-Sensor-Fusion-Student
- Commit hash nộp (`git rev-parse HEAD`): 99eb578676e203c84abd42c4b888483c6d9a8f3b

## Tóm tắt kết quả

- `fusion_mode` (bắt buộc `compare`), `frames`, `segment`, `seed`:
  `fusion_mode = "compare"`, `frames = [0, 198]`, `segment = "training_segment-1005081002024129653_5313_150_5333_150_with_camera_labels.tfrecord"`, `seed = 0`
- `detection.precision`, `detection.recall`, `detection.tp/fp/fn`:
  `precision = 0.97009` (97.01%), `recall = 0.70040` (70.04%), `tp = 519`, `fp = 16`, `fn = 222`
- `tracking.lidar.rmse`, `matches`, `sum_sq_err`, `ghost_track_frames`, `missed_gt_frames`, `mean_confirmed_tracks`:
  `rmse = 0.15032 m`, `matches = 502`, `sum_sq_err = 11.34365 m²`, `ghost_track_frames = 0`, `missed_gt_frames = 239`, `mean_confirmed_tracks = 2.52261`
- `tracking.fused.rmse`, `matches`, `sum_sq_err`, `ghost_track_frames`, `missed_gt_frames`, `mean_confirmed_tracks`:
  `rmse = 0.13587 m`, `matches = 502`, `sum_sq_err = 9.26681 m²`, `ghost_track_frames = 0`, `missed_gt_frames = 239`, `mean_confirmed_tracks = 2.52261`
- Giải thích khác biệt hai mode, đọc RMSE cùng số ghép và ghost/miss:
  - **Số cặp ghép và tính toàn vẹn track**: Cả hai chế độ `lidar` và `fused` đều đạt đúng `matches = 502`, `ghost_track_frames = 0`, `missed_gt_frames = 239`, và `mean_confirmed_tracks = 2.52261`. Điều này chứng minh tính nhất quán tuyệt đối của kiến trúc *track-then-fuse*: camera không tạo ra ghost track mới, không làm mất bất kỳ track nào và không làm thay đổi số lượng track được xác nhận.
  - **Độ chính xác vị trí (RMSE)**: Chế độ `fused` đạt RMSE là `0.13587 m`, cải thiện tốt hơn so với chế độ `lidar-only` (`0.15032 m`). Tổng bình phương sai số vị trí 3D giảm từ `11.34365 m²` xuống `9.26681 m²` (giảm ~18.3% sai số năng lượng). Mức chênh lệch `rmse_fused - rmse_lidar = -0.01445 m <= 0.05 m`, thoả mãn xuất sắc tiêu chí rubric.
  - **Chỉ số tracking chất lượng**: `precision_track = 502 / (502 + 0) = 1.0` (vượt xa mức chuẩn 0.75), `coverage = 502 / 519 = 0.9672` (vượt xa mức chuẩn 0.70).

Chạy từ root repo:

```bash
fusion-run-lab --config student/config/paths.yaml --fusion compare --seed 0
```

`rmse = sqrt(sum_sq_err/matches)` trên vị trí 3D của confirmed tracks ghép
một-một với GT xe trong cửa sổ BEV, gate XY **2.0 m**; `null` nếu không có cặp.
Camera dùng tâm hộp 2D ground-truth FRONT có nhiễu seeded, **không** dùng camera
detector. Kết quả này không đo hiệu quả một perception system độc lập với GT.

`grade_run.log` là JSONL, mỗi `(mode,frame)` đúng một record với các trường:
`mode`, `frame`, `det_tp`, `det_fp`, `det_fn`, `valid_gt`, `confirmed`, `matches`,
`sum_sq_err`, `ghosts`, `misses`. Đảm bảo `matches+ghosts==confirmed` và
`matches+misses==valid_gt`; tổng/trung bình record phải khớp `metrics.json`.
File per-mode `metrics_lidar.json`, `metrics_fused.json`, `grade_run_lidar.log`,
`grade_run_fused.log` được giữ để đối chiếu.

## Giải thích ngắn (Parts E–H — tự viết)

1. Khác biệt đo lidar 3D và camera 2D trong EKF (`z`, `R`)?
   - **Vector đo lường $z$**:
     - LiDAR đo vị trí không gian 3D trực tiếp trong hệ trục cảm biến/xe: $z_{lidar} = [x, y, z]^T \in \mathbb{R}^3$. Mô hình đo $h(x)$ là tuyến tính với $H \in \mathbb{R}^{3 \times 6}$.
     - Camera đo toạ độ pixel 2D trên mặt phẳng ảnh: $z_{cam} = [u, v]^T \in \mathbb{R}^2$. Mô hình đo $h(x)$ là phi tuyến (phép chiếu phối cảnh pinhole $u = c_i - f_i y_s/x_s, v = c_j - f_j z_s/x_s$), do đó ma trận đo $H \in \mathbb{R}^{2 \times 6}$ là Jacobian bậc nhất (tính qua đạo hàm chuỗi giữa chiếu pinhole và ma trận xoay).
   - **Ma trận hiệp phương sai nhiễu $R$**:
     - LiDAR có $R_{lidar} = \operatorname{diag}(\sigma_{x}^2, \sigma_{y}^2, \sigma_{z}^2) \in \mathbb{R}^{3 \times 3}$ với đơn vị là mét bình phương ($m^2$), giá trị $\sigma = 0.1\text{ m}$.
     - Camera có $R_{cam} = \operatorname{diag}(\sigma_{cam\_i}^2, \sigma_{cam\_j}^2) \in \mathbb{R}^{2 \times 2}$ với đơn vị là pixel bình phương ($px^2$), giá trị $\sigma_{cam\_i} = \sigma_{cam\_j} = 5.0\text{ px}$.

2. Vì sao cần gating Mahalanobis trước khi gán?
   - Khoảng cách Mahalanobis bình phương $d^2 = \gamma^T S^{-1} \gamma$ đo độ sai lệch chuẩn hoá giữa phép đo và dự báo của track, có tính đến ma trận hiệp phương sai phần dư $S = H P H^T + R$.
   - Khác với khoảng cách Euclid (coi mọi hướng đều bình đẳng như hình cầu), Mahalanobis xét hình ellipsoid bất đối xứng của sai số ước lượng $P$ và nhiễu $R$. Khi một track có độ bất định vận tốc lớn hoặc góc nhìn xiên, khoảng cách Euclid có thể rất lớn nhưng Mahalanobis lại nhỏ (hợp lệ), và ngược lại.
   - Cổng kiểm định $\chi^2$ (`chi2_gate`) dựa trên phân bố Chi-bình phương với số bậc tự do tương ứng chiều đo lường ($\text{dim}=3$ cho LiDAR, $\text{dim}=2$ cho Camera) giúp loại bỏ sớm các đo lường nhiễu ngoại lai (clutter / false alarms) hoặc các đo lường thuộc về xe khác trước khi đưa vào thuật toán gán Greedy. Điều này ngăn chặn việc gán sai làm hỏng ma trận trạng thái EKF.

3. Pipeline là track-then-fuse hay fuse-then-track? Chỉ ra trên log `fusion-run-lab`.
   - Pipeline này là **track-then-fuse**: hệ thống chỉ duy trì duy nhất một danh sách track đối tượng. Mỗi frame chỉ thực hiện EKF Predict đúng 1 lần, sau đó cập nhật EKF nối tiếp theo từng modality cảm biến: trước tiên là gán và cập nhật LiDAR 3D (kèm theo quản lý vòng đời track), sau đó mới gán và cập nhật Camera 2D để tinh chỉnh trạng thái EKF. Kiến trúc này khác với *fuse-then-track* (gộp point cloud và ảnh/detection ở mức sensor trước rồi mới đưa vào tracker).
   - Minh chứng từ log `fusion-run-lab` (`grade_run.log`):
     - Ở từng frame, các trường số lượng track như `confirmed`, `matches`, `ghosts`, `misses` của cả hai mode `lidar` và `fused` hoàn toàn trùng khớp từng dòng (ví dụ ở frame 0: cả 2 mode đều có `confirmed: 1, matches: 1, ghosts: 0, misses: 1`; ở frame 198: đều có `confirmed: 3, matches: 3, ghosts: 0, misses: 1`).
     - Camera không hề làm thay đổi số lượng track hay trạng thái vòng đời. Sự khác biệt duy nhất thể hiện ở cột `sum_sq_err`: tại mọi frame sau khi camera update, `sum_sq_err` của mode `fused` giảm xuống so với `lidar` (tổng kết cả 199 frame giảm từ $11.34\text{ m}^2$ xuống $9.27\text{ m}^2$), thể hiện việc camera chỉ tham gia tinh chỉnh toạ độ ước lượng của track đã có.

4. Nếu camera lệch calibration, triệu chứng gì trên innovation/residual?
   - Triệu chứng trên innovation $\gamma = z - h(x)$:
     - Innovation sẽ xuất hiện độ lệch hệ thống (systematic bias / non-zero mean) lệch hẳn về một phía thay vì dao động ngẫu nhiên xung quanh 0 với phân bố chuẩn.
     - **Nếu lệch nhẹ**: Innovation $\gamma$ tăng sẽ làm tăng khoảng cách Mahalanobis $d^2$. Khi $d^2$ vẫn nằm trong cổng $\chi^2$, cập nhật EKF sẽ kéo trạng thái của track bị lệch vị trí thật, làm tăng sai số RMSE và có thể làm méo ước lượng vận tốc.
     - **Nếu lệch nặng**: Khoảng cách $d^2$ sẽ vượt qua ngưỡng kiểm định $\chi^2$ (`chi2.ppf`), khiến các phép đo camera bị cổng gating từ chối hoàn toàn. Khi đó thuật toán gán sẽ không ghép được cặp camera nào, camera pass bị bỏ qua hoàn toàn và không thể tinh chỉnh trạng thái track.

5. Vì sao `associate_and_update(..., sensor)` cần sensor tường minh ở frame rỗng?
   Giải thích vì sao lidar quyết định score/init/delete còn camera chỉ EKF update.
   - **Cần sensor tường minh ở frame rỗng**: Khi một frame không có đo lường nào (`meas_list` rỗng), hàm `associate_and_update` vẫn bắt buộc phải gọi `manager.manage_tracks(unassigned_tracks, unassigned_meas, sensor)` để xử lý vòng đời cuối lượt. Manager cần biết lượt này là của cảm biến nào: nếu là `lidar`, toàn bộ các track trong FOV của LiDAR sẽ bị ghi nhận là một lần miss và bị trừ score (có thể kích hoạt xóa track nếu score giảm quá thấp). Nếu không truyền sensor tường minh, hệ thống không thể phân biệt lượt LiDAR trống để cập nhật lifecycle.
   - **Vì sao LiDAR quyết định lifecycle còn Camera chỉ EKF update**:
     - LiDAR là cảm biến chủ đạo có góc quét 360 độ và đo trực tiếp toạ độ 3D có độ sâu vật lý chính xác, đủ độ tin cậy để xác định sự xuất hiện (khởi tạo), duy trì (cộng điểm khi có hit, trừ điểm khi miss) và biến mất (xoá khi ra khỏi vùng quét hoặc mất dấu) của xe 3D.
     - Camera trong lab chỉ có trường nhìn hạn chế ở phía trước (FRONT FOV) và bản chất đo lường 2D thiếu thông tin chiều sâu vật lý. Nếu cho camera can thiệp vào lifecycle: những xe ở hai bên hông hoặc phía sau xe tự hành sẽ bị coi là "miss" và bị xoá nhầm; ngoài ra các detection ảo 2D nếu khởi tạo track sẽ gây ra vô số ghost tracks 3D sai lệch. Do đó, camera chỉ được dùng để cập nhật bổ sung (update) làm mịn trạng thái EKF cho các xe đã được LiDAR phát hiện.

6. Nêu điều kiện xác nhận, giữ confirmed sau miss, và điều kiện xóa track.
   - **Điều kiện xác nhận (`confirmed`)**:
     - Khi một track mới được sinh ra từ LiDAR measurement chưa ghép, nó khởi tạo với `state = "initialized"` và `score = 1/window = 1/6 ≈ 0.1667`.
     - Mỗi frame có LiDAR hit: score được cộng thêm $+1/window$ (bão hoà tối đa ở $1.0$).
     - Ngay khi $score > confirmed\_threshold$ ($0.8$), track được thăng cấp thành `state = "confirmed"`. (Thông thường cần 5 frame liên tiếp được LiDAR quan sát).
   - **Giữ confirmed sau miss**:
     - Khi một track đã ở trạng thái `confirmed` bị miss trong tầm nhìn LiDAR FOV, nó bị trừ $-1/window$, nhưng trạng thái `state` vẫn được giữ nguyên là `confirmed`, không bị hạ cấp về `tentative`. Điều này giúp tracker duy trì danh tính đối tượng ổn định qua các frame bị che khuất tạm thời (occlusion).
   - **Điều kiện xoá track (`should_delete_track`)** — Xoá nếu thoả mãn bất kỳ điều kiện nào sau đây (logic OR):
     1. Hiệp phương sai vị trí ngang quá lớn: $P[0, 0] > max\_P$ hoặc $P[1, 1] > max\_P$ (với $max\_P = 3.0^2 = 9.0\text{ m}^2$).
     2. Track đã `confirmed` nhưng score suy giảm quá mức do bị miss nhiều frame: $score < delete\_threshold$ ($0.6$).
     3. Track chưa `confirmed` (`initialized` hoặc `tentative`) có $score \le 0.0$.

## Bonus (không bắt buộc)

Liệt kê phần bonus đã làm, file bằng chứng trong `student/bonus/` và kết quả chính
(xem [RUBRIC.md](../RUBRIC.md) mục 2). Không làm thì ghi "Không".

- Không

## Khai báo sử dụng AI (bắt buộc)

Ghi rõ, kể cả khi không dùng ("Không dùng AI"). Xem [RULES.md](../RULES.md) mục 2.

- Công cụ đã dùng (ChatGPT, Copilot, Claude, …): Antigravity AI Assistant
- Dùng cho phần nào (hàm, câu hỏi, debug):
  - Hỗ trợ phân tích kiến trúc pipeline và đối chiếu công thức toán học từ `HUONG_DAN_KY_THUAT.md`.
  - Hỗ trợ triển khai các hàm stub trong `kalman.py`, `camera_fusion.py`, `association.py`, `track_management.py`.
  - Hỗ trợ kiểm thử tự động với pytest và giải thích các câu hỏi kỹ thuật trong báo cáo nộp bài.
- Cách bạn đã kiểm tra lại (pytest, chạy Waymo, đối chiếu công thức):
  - Chạy toàn bộ test suite `pytest student/tests` và đạt 128/128 test pass (100%).
  - Chạy trực tiếp `fusion-run-lab` trên cả segment Waymo 199 frame ở chế độ `compare`, kiểm tra số liệu RMSE, matches, ghosts, misses.
  - Chạy `python tools/check_submission.py` xác minh toàn bộ cấu trúc và điều kiện nộp bài.

## Checklist nộp

- [x] **Part E–H** trong `workspace/` đã implement; `pytest student/tests -q` không còn `failed`/`xfailed`
- [x] Part A–D: không bắt buộc sửa (hoặc ghi chú nếu bạn đã sửa)
- [x] Lần chạy chấm điểm: `--fusion compare --seed 0`, `frame_start: 0`, `frame_end: 198`
- [x] Đã commit `student/artifacts/metrics*.json` và `student/artifacts/grade_run*.log` (không sửa tay)
- [x] Đã điền đủ file này, gồm khai báo AI
- [x] Không commit dữ liệu Waymo, weights, `paths.yaml`, API key
- [x] `python tools/check_submission.py` báo `KẾT QUẢ: SẴN SÀNG NỘP`
- [x] Đã push và nộp link repo + commit hash trên LMS ([hướng dẫn nộp](../SUBMISSION.md))
