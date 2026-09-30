# MICROMOUSE ROBOT - HƯỚNG DẪN ĐIỀU KHIỂN & LỘ TRÌNH DÒ ĐƯỜNG (SEARCH RUN)

Dự án Micromouse sử dụng cấu hình **2 Cảm biến ToF VL53L0X đặt góc chéo 45° ở mũi xe**, **Cảm biến góc MPU6050**, **Động cơ GA12-N20 300 RPM kèm Encoder**, giao tiếp & telemetry qua **Web BLE Dashboard**.

---

## I. Cấu Trúc Phần Cứng & Nguyên Lý Cảm Biến 45°

```
                     VÁCH TƯỜNG TRƯỚC
        +-----------------------------------------+
        |         . (dL)          . (dR)          |
        |          \             /                |
        |           \           /   Khoảng cách   |
        |            \         /    tới tường     |
        |             \       /     trước: 50mm   |
VÁCH    |              \  45°/                    | VÁCH
HÔNG    |               [ Cảm biến 45° ]          | HÔNG
(84mm)  |                      |                  | (84mm)
        |                      | Thân xe          |
        |                      |                  |
        |               [ Trục Bánh Xe ]          |
        +-----------------------------------------+
                         Tâm ô (+)
```

### 1. Nguyên lý hoạt động của 2 cảm biến 45°
* **Chạy thẳng (Straight PID):** Hai tia 45° đập vào 2 vách hông. Tính sai số $e = d_L - d_R - \text{centerOffset}$ để PID tự động giữ xe ở chính giữa ô.
* **Gặp ngã rẽ bên hông (Wall Drop):** Cảm biến bên muốn rẽ quét qua mép cột, giá trị nhảy vọt ($> 220\text{mm}$). Xe chuyển sang giữ thẳng bằng **Gyro + Encoder** chạy bù thêm một đoạn $D_{\text{offset}}$ để đưa trục bánh xe vào đúng tâm ô rồi mới rẽ.
* **Tiến sát tường trước (Front Wall):** Khi cách tường trước $< 60\text{mm}$, cả 2 tia 45° đều đập vào tường trước khiến $d_L, d_R$ đồng thời tụt xuống thấp ($< 80\text{mm}$). Đây là tín hiệu phanh dừng chính xác tại tâm ô.
* **Cơ chế ghi nhớ vách hông (Wall Memory):** Khi xe vừa tiến vào ô (cách tường trước $10-15\text{cm}$), 2 cảm biến 45° còn nhìn thấy vách hông $\rightarrow$ ghi nhớ trạng thái vách trái/phải. Khi xe chạm tường trước, dựa vào ký ức này để quyết định: **Góc chữ L Trái (90°)**, **Góc chữ L Phải (90°)**, hay **Đường cụt (Quay đầu 180°)**.

---

## II. Lộ Trình 4 Bước Hoàn Thiện Dò Đường & Giải Mê Cung

```
                       ┌──────────────────────────────┐
                       │   BƯỚC 4: THUẬT TOÁN        │
                       │   Flood Fill (Loang nước)    │
                       │   & Ma trận bản đồ 16x16     │
                       └──────────────┬───────────────┘
                                      │
                       ┌──────────────┴───────────────┐
                       │   BƯỚC 3: MÁY TRẠNG THÁI     │
                       │   Hàm điều khiển từng ô:     │
                       │   moveForward(1), turnLeft().│
                       └──────────────┬───────────────┘
                                      │
                       ┌──────────────┴───────────────┐
                       │   BƯỚC 2: NHẬN DIỆN VÁCH Ô   │
                       │   Phát hiện: Tường trước,    │
                       │   Tường trái, Tường phải     │
                       └──────────────┬───────────────┘
                                      │
        ┌─────────────────────────────┴─────────────────────────────┐
        │   BƯỚC 1: CĂN CHỈNH VẬT LÝ CƠ BẢN (Quan trọng nhất!)      │
        │   1. Chạy thẳng 1 ô (180mm) dừng đúng tâm.               │
        │   2. Quay 90° và 180° vuông góc tuyệt đối.                │
        │   3. PID bám tâm đường không bị lắc đuôi cá.             │
        └───────────────────────────────────────────────────────────┘
```

---

### BƯỚC 1: Căn Chỉnh Chuyển Động Vật Lý Cơ Bản (Ưu Tiên Số 1)

Trước khi viết thuật toán, cần đảm bảo 3 bài test sau hoạt động hoàn hảo trên Web Dashboard:

1. **Test Chạy Thẳng 1 Ô (`stepCell(1)`):**
   * Đặt xe ở tâm ô 1, gửi lệnh tiến 1 ô ($180\text{ mm}$).
   * **Mục tiêu:** Xe tăng tốc mượt $\rightarrow$ giữ thẳng tâm đường $\rightarrow$ phanh dứt điểm (Active Braking) để trục bánh xe dừng chính xác tại tâm ô 2.
   * **Căn chỉnh:** Tinh chỉnh `pulsesPerCell` trong `RobotNav.h` (mặc định $\approx 1000$ xung).

2. **Test Xoay 90° & 180° Tại Chỗ (`turnLeft`, `turnRight`):**
   * Bấm quay trái 90°, quay phải 90°, quay đầu 180°.
   * **Mục tiêu:** Xoay quanh tâm trục bánh xe. Sau khi dừng, thân xe song song tuyệt đối với vách tường, không lệch xéo $5^\circ - 10^\circ$.
   * **Căn chỉnh:** Tinh chỉnh `leftCompensation` và `rightCompensation` trong `RobotNav.cpp`.

3. **Test PID Bám Tâm Đường (`startPID()`):**
   * Đặt xe hơi lệch trái hoặc lệch phải một góc nhỏ.
   * **Mục tiêu:** Xe tự bẻ lái nhẹ nhàng về lại trung tâm hẻm, không bị hiện tượng đánh võng / lắc đuôi cá.
   * **Căn chỉnh:** `wallPID (Kp, Ki, Kd)` và `wallDeadband`.

---

### BƯỚC 2: Nhận Diện & Đọc Vách Của Từng Ô (Sense Walls)

Xây dựng hàm quét vách tại mỗi ô mà xe đi qua:
```cpp
struct CellWalls {
    bool north; // Tường trước
    bool east;  // Tường phải
    bool south; // Tường sau
    bool west;  // Tường trái
};
```
* **Tường trước:** Nhận diện khi $(\text{smoothDL} \le 80\text{mm} \text{ hoặc } \text{smoothDR} \le 80\text{mm})$.
* **Tường hông (Trái/Phải):** Ghi nhớ giá trị $\text{smoothDL}, \text{smoothDR} < \text{wallThreshold}$ ($200\text{mm}$) lúc xe đang lao vào ô.

---

### BƯỚC 3: Đóng Gói Các Lệnh Chuyển Động Nguyên Tử (Atomic Motion)

Cung cấp các hàm cấp cao để tầng thuật toán chỉ việc gọi:
1. `moveOneCell()`: Chạy bám tường đúng 1 ô và dừng tại tâm ô tiếp theo.
2. `turnLeftAndStep()`: Xoay trái 90° tại tâm ô $\rightarrow$ chạy tiếp 1 ô.
3. `turnRightAndStep()`: Xoay phải 90° tại tâm ô $\rightarrow$ chạy tiếp 1 ô.
4. `turnAroundAndStep()`: Quay đầu 180° tại tâm ô $\rightarrow$ chạy tiếp 1 ô.

---

### BƯỚC 4: Thuật Toán Giải Mê Cung (Flood Fill Algorithm)

1. **Khởi tạo dữ liệu:**
   * Ma trận bản đồ tường: `uint8_t maze[16][16]` (lưu các vách đã phát hiện).
   * Ma trận trọng số khoảng cách: `uint8_t dist[16][16]` (4 ô đích trung tâm $(7,7), (7,8), (8,7), (8,8) = 0$, các ô khác khởi tạo theo khoảng cách Manhattan tăng dần).
2. **Vòng lặp Dò đường (Search Loop):**
   * Xe đến ô mới $\rightarrow$ Đọc vách ô $\rightarrow$ Cập nhật vào `maze[][]`.
   * Chạy hàm Loang nước `floodFill()` cập nhật lại mảng `dist[][]`.
   * Chọn ô láng giềng có `dist` nhỏ nhất $\rightarrow$ Thực hiện lệnh di chuyển ở Bước 3.
   * Lặp lại cho đến khi chạm tâm đích.
3. **Quay về điểm xuất phát:** Sau khi chạm đích, cập nhật ô xuất phát $(0,0) = 0$ để xe tự giải đường quay về vạch xuất phát, sẵn sàng cho vòng chạy **Speed Run**.

---

## III. Bảng Tra Cứu Thông Số Hiệu Chuẩn Thực Tế (Cheat Sheet)

| Thông số | Ý nghĩa | Giá trị tham khảo | Vị trí cấu hình |
| :--- | :--- | :--- | :--- |
| `pulsesPerCell` | Số xung Encoder cho 1 ô ($180\text{mm}$) | `1000` xung | [RobotNav.h](file:///c:/Users/Windows/Desktop/latex/mm-main/mm-main/lib/Navigation/RobotNav.h) |
| `front45Threshold` | Ngưỡng 45° nhận diện tường trước | `80 - 100` mm | [RobotNav.h](file:///c:/Users/Windows/Desktop/latex/mm-main/mm-main/lib/Navigation/RobotNav.h) |
| `wallThreshold` | Ngưỡng phân biệt có tường hông vs cửa mở | `200 - 230` mm | [RobotNav.cpp](file:///c:/Users/Windows/Desktop/latex/mm-main/mm-main/lib/Navigation/RobotNav.cpp) |
| `targetLeftDist` | Khoảng cách ToF trái lý tưởng ở tâm ô | `150 - 170` mm | [RobotNav.cpp](file:///c:/Users/Windows/Desktop/latex/mm-main/mm-main/lib/Navigation/RobotNav.cpp) |
| `targetRightDist`| Khoảng cách ToF phải lý tưởng ở tâm ô | `140 - 160` mm | [RobotNav.cpp](file:///c:/Users/Windows/Desktop/latex/mm-main/mm-main/lib/Navigation/RobotNav.cpp) |
| `baseForwardSpeed` | Tốc độ PWM chạy thẳng cơ bản | `70 - 85` (PWM) | [RobotNav.cpp](file:///c:/Users/Windows/Desktop/latex/mm-main/mm-main/lib/Navigation/RobotNav.cpp) |
| `turnSpeed` | Tốc độ PWM khi quay tại chỗ | `75 - 85` (PWM) | [RobotNav.cpp](file:///c:/Users/Windows/Desktop/latex/mm-main/mm-main/lib/Navigation/RobotNav.cpp) |
| `leftCompensation` | Bù góc trôi quán tính quay trái MPU6050 | `18.0` độ | [RobotNav.cpp](file:///c:/Users/Windows/Desktop/latex/mm-main/mm-main/lib/Navigation/RobotNav.cpp) |
| `rightCompensation`| Bù góc trôi quán tính quay phải MPU6050| `18.0` độ | [RobotNav.cpp](file:///c:/Users/Windows/Desktop/latex/mm-main/mm-main/lib/Navigation/RobotNav.cpp) |
