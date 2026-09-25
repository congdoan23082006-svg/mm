# TỔNG HỢP TIẾN ĐỘ DỰ ÁN MICROMOUSE

Dưới đây là tóm tắt toàn bộ những "bệnh" chúng ta đã bắt và chữa thành công, cùng với hướng đi tiếp theo cho chiếc xe của bạn.

---

## PHẦN 1: CÁC LỖI ĐÃ KHẮC PHỤC

### 1. Lỗi "Đi PID bị đâm vào tường" & "Rẽ trái thành rẽ phải"
*   **Nguyên nhân:** Động cơ bên trái và bên phải bị cắm dây (hoặc cấu hình) ngược nhau. Khi hàm PID muốn bẻ trái để né tường thì nó lại vô tình kích tốc độ cho bánh phải, làm xe cắm đầu thêm vào tường.
*   **Cách giải quyết:** Chúng ta đã tráo lại thứ tự chân PWM của `M1` và `M2` trong file `RobotConfig.h`. Bây giờ hệ thống đã nhận diện chuẩn xác M1 = Trái, M2 = Phải.

### 2. Lỗi "Đi lượn sóng" & "Lắc bần bật"
*   **Nguyên nhân:** Hệ số vi phân `Kd` quá lớn kết hợp với chu kỳ vòng lặp quá nhanh (`dt = 0.01s`), tạo ra những cú sốc điện lên tới 40-80 PWM chỉ vì xe nhích chệch 1mm. Cộng thêm việc sai số bị nhân ảo `1.5f`.
*   **Cách giải quyết:** Xóa bỏ bộ nhân `1.5f` ảo. Hạ cực mạnh `Kp` (xuống `0.2`) và ép `Kd` về `0` để xe đi tà tà ổn định trước. Bóp giới hạn bẻ lái (`PID_Out`) từ `120` xuống còn `60` để xe không bao giờ giật tay lái quá gắt.

### 3. Lỗi "Rẽ quay góc nhưng chỉ dùng 1 bánh"
*   **Nguyên nhân:** Logic Code thì xuất điện ra 2 bánh, nhưng vận tốc PWM cung cấp (`turnSpeed = 50`) là quá yếu. Bánh xe nào hộp số bị rít (ma sát cao hơn) sẽ bị đứng khựng lại, không đủ sức để quay.
*   **Cách giải quyết:** Nâng `turnSpeed` lên `80` và mức giảm tốc lúc hãm phanh `MIN_TURN_SPEED` lên `60`. Điện khỏe hơn sẽ giúp cả 2 bánh quay ngược chiều mượt mà.

### 4. Logic "Reset Gyro sau khi rẽ"
*   **Xác nhận:** Chức năng này của bạn code rất chuẩn. Hàm `rotateToAngle()` xoay xong sẽ tự gọi `resetYaw()` về 0. Hàm `startPID()` gọi ngay sau đó sẽ chốt cứng gốc 0 độ làm mục tiêu đi thẳng. Không cần sửa gì thêm.

---

## PHẦN 2: CHIẾN THUẬT ĐI LIỀN MẠCH & THUẬT TOÁN (A* / FLOOD FILL)

Chiếc xe của bạn là xe xịn, có module đếm xung **Encoder** (đã setup trong thư mục `lib/ENCODER`). Chúng ta sẽ tận dụng nó để làm thuật toán đi mê cung trơn tru theo các nguyên tắc sau:

### Kỹ thuật 1: Đo quãng đường (Odometry)
*   **Cách làm:** Không cần tính toán công thức lằng nhằng, dùng tay đẩy xe đúng **1 ô (18 cm)** và đọc giá trị hiển thị trên Serial Monitor (Ví dụ ra được **300 xung**).
*   **Chốt thông số:** `1 Ô = 300 XUNG`.

### Kỹ thuật 2: Đi thẳng liền mạch (Không phanh giữa đường)
Thay vì đi 1 ô, dừng lại, rồi lại đi tiếp. Xe sẽ dùng PID chạy một lèo xuyên qua hành lang dài.
```cpp
if (enc1A_count >= 300) {
    // 1. Tịnh tiến tọa độ (X, Y) lên 1 đơn vị trên bản đồ mê cung
    cellsTraversed++; 
    
    // 2. Trừ bớt 300 xung để bắt đầu đếm ô tiếp theo (Không dùng enc = 0 để tránh lọt xung)
    enc1A_count -= 300; 

    // 3. Hỏi não bộ (Thuật toán A*): "Ở tọa độ này có cần rẽ không?"
    if (can_re_tai_day) {
        stopPID();      // Có thì phanh cái két lại
        turn(90.0);     // Xoay đầu
        startPID();     // Lại phi tiếp
    }
    // Nếu não bão không cần rẽ -> Xe lướt qua ngã tư với tốc độ bàn thờ, không hề khựng lại!
}
```

---
**Lời khuyên:** Nhiệm vụ ưu tiên hàng đầu của bạn lúc này là **vác xe ra sa hình chạy thử**. Hãy nạp code và test xem xe đã hết bị "rắn bò" chưa, và test xem lúc rẽ 2 bánh đã quay tít chưa nhé! Mọi thứ về thuật toán chạy mê cung chỉ có thể viết được khi xe đi thẳng mượt mà.
