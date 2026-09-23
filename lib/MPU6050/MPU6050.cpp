#include "MPU6050.h"
#include <math.h>

// Các địa chỉ thanh ghi phần cứng MPU6050
#define MPU6050_REG_PWR_MGMT_1   0x6B
#define MPU6050_REG_GYRO_CONFIG  0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_ACCEL_XOUT_H 0x3B

MPU6050::MPU6050(uint8_t addr) {
    _addr = addr;
    _gyroZOffset = 0.0;
    _roll = 0.0;
    _pitch = 0.0;
    _yaw = 0.0;
    _yawSetpoint = 0.0;
    _lastTime = 0;
}

bool MPU6050::begin() {
    // 1. Kiểm tra thiết bị phản hồi
    Wire.beginTransmission(_addr);
    if (Wire.endTransmission() != 0) {
        return false; // Không tìm thấy MPU6050 ở địa chỉ _addr
    }

    // 2. Đánh thức MPU6050 (Power Management 1 = 0)
    Wire.beginTransmission(_addr);
    Wire.write(MPU6050_REG_PWR_MGMT_1);
    Wire.write(0x00);
    if (Wire.endTransmission() != 0) {
        return false;
    }
    delay(10); // Đợi chip khởi động lại dao động

    // 3. Cấu hình dải đo Gyro: +/- 2000 deg/s (Hệ số nhạy: 16.4 LSB/(deg/s)) để đo chính xác tốc độ quay cao của xe
    Wire.beginTransmission(_addr);
    Wire.write(MPU6050_REG_GYRO_CONFIG);
    Wire.write(0x18); 
    if (Wire.endTransmission() != 0) {
        return false;
    }

    // 4. Cấu hình dải đo Accel: +/- 8g
    Wire.beginTransmission(_addr);
    Wire.write(MPU6050_REG_ACCEL_CONFIG);
    Wire.write(0x10); 
    if (Wire.endTransmission() != 0) {
        return false;
    }

    _lastTime = micros();
    return true;
}

bool MPU6050::readRawData(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz) {
    Wire.beginTransmission(_addr);
    Wire.write(MPU6050_REG_ACCEL_XOUT_H);
    uint8_t err = Wire.endTransmission(false);
    if (err != 0) {
        return false; // I2C NACK hoặc bus bận, tránh gọi requestFrom gây Error 259
    }

    uint8_t bytesReceived = Wire.requestFrom(_addr, (uint8_t)14);
    if (bytesReceived != 14) {
        return false; // Không nhận đủ byte dữ liệu
    }

    *ax = (int16_t)((Wire.read() << 8) | Wire.read());
    *ay = (int16_t)((Wire.read() << 8) | Wire.read());
    *az = (int16_t)((Wire.read() << 8) | Wire.read());
    Wire.read(); Wire.read(); // Bỏ qua giá trị nhiệt độ (Temperature)
    *gx = (int16_t)((Wire.read() << 8) | Wire.read());
    *gy = (int16_t)((Wire.read() << 8) | Wire.read());
    *gz = (int16_t)((Wire.read() << 8) | Wire.read());
    return true;
}

void MPU6050::calibrate(int samples) {
    float sumZ = 0;
    int validSamples = 0;
    int16_t ax, ay, az, gx, gy, gz;
    
    // Lưu ý: Khi hiệu chuẩn, robot phải đặt đứng yên hoàn toàn
    for (int i = 0; i < samples; i++) {
        if (readRawData(&ax, &ay, &az, &gx, &gy, &gz)) {
            sumZ += gz;
            validSamples++;
        }
        delay(3);
    }
    
    // Tính toán độ lệch tĩnh (offset) cho trục Z với dải 2000 deg/s (16.4 LSB/(deg/s))
    if (validSamples > 0) {
        _gyroZOffset = (sumZ / validSamples) / 16.4f;
    }
    _roll = 0.0;
    _pitch = 0.0;
    _yaw = 0.0;
    _yawSetpoint = 0.0;
    _lastTime = micros();
}

void MPU6050::update() {
    int16_t ax, ay, az, gx, gy, gz;
    if (!readRawData(&ax, &ay, &az, &gx, &gy, &gz)) {
        return; // Đọc I2C thất bại -> bỏ qua vòng này, không tính toán sai góc
    }

    unsigned long now = micros();
    float dt = (now - _lastTime) / 1000000.0; // Đổi từ micro giây sang giây
    if (_lastTime == 0 || dt > 1.0) {
        dt = 0.01;
    }
    _lastTime = now;

    // Góc nghiêng lấy từ gia tốc kế, tính theo độ.
    _roll = atan2((float)ay, (float)az) * 180.0 / PI;
    _pitch = atan2(-(float)ax, sqrt((float)ay * ay + (float)az * az)) * 180.0 / PI;

    // Đổi giá trị thô sang đơn vị độ/giây (deg/s) với hệ số 16.4 (cho dải 2000 deg/s)
    // Cảm biến đặt úp nên đảo chiều quay quanh trục Z.
    float gyroZRate = -((gz / 16.4f) - _gyroZOffset);

    // Lọc nhiễu tĩnh nhỏ (Deadzone) tránh bị trôi góc khi xe đứng yên
    if (abs(gyroZRate) < 0.6) {
        gyroZRate = 0;
    }

    // Tích phân vận tốc góc theo thời gian để ra góc Yaw
    _yaw += gyroZRate * dt;
}

bool MPU6050::rotateToAngle(float targetAngle, uint8_t m1In1, uint8_t m1In2,
                            uint8_t m2In1, uint8_t m2In2,
                            float compensation, uint8_t turnSpeed,
                            unsigned long timeoutMs) {
    if (targetAngle == 0.0f) {
        return true;
    }

    update();
    float startAngle = _yaw;
    float requestedAngle = fabs(targetAngle);
    float effectiveAngle = requestedAngle > compensation
                               ? requestedAngle - compensation
                               : 0.0f;

    Serial.print(">> BAT DAU QUAY: Goc hien tai = ");
    Serial.print(startAngle, 2);
    Serial.print(" | Can quay = ");
    Serial.print(effectiveAngle, 2);
    Serial.print(" do | Toc do PWM = ");
    Serial.println(turnSpeed);

    // Điều khiển động cơ quay theo tốc độ PWM:
    // targetAngle > 0: Quay trái (Motor trái lùi, Motor phải tiến)
    // targetAngle < 0: Quay phải (Motor trái tiến, Motor phải lùi)
    if (targetAngle > 0.0f) {
        analogWrite(m1In1, 0);
        analogWrite(m1In2, turnSpeed);
        analogWrite(m2In1, turnSpeed);
        analogWrite(m2In2, 0);
    } else {
        analogWrite(m1In1, turnSpeed);
        analogWrite(m1In2, 0);
        analogWrite(m2In1, 0);
        analogWrite(m2In2, turnSpeed);
    }

    unsigned long turnStart = millis();
    bool reachedTarget = false;

    // Dùng độ lệch góc tuyệt đối để tránh lỗi ngược dấu làm quay tròn tít mù
    while (millis() - turnStart <= timeoutMs) {
        update();
        float turnedAngle = fabs(_yaw - startAngle);

        if (turnedAngle >= effectiveAngle) {
            reachedTarget = true;
            break;
        }
        delay(1);
    }

    // Phanh ngắn mạch (Active Braking) để triệt tiêu trớn quán tính ngay lập tức
    digitalWrite(m1In1, HIGH);
    digitalWrite(m1In2, HIGH);
    digitalWrite(m2In1, HIGH);
    digitalWrite(m2In2, HIGH);
    delay(40);

    // Nhả motor về trạng thái thả tự do
    digitalWrite(m1In1, LOW);
    digitalWrite(m1In2, LOW);
    digitalWrite(m2In1, LOW);
    digitalWrite(m2In2, LOW);

    update();
    Serial.print(">> KET QUA: Goc da quay duoc = ");
    Serial.print(fabs(_yaw - startAngle), 2);
    Serial.print(" do | Goc hien tai = ");
    Serial.print(_yaw, 2);
    Serial.println(reachedTarget ? " (THANH CONG)" : " (TIMEOUT - KET BANH HOAC LOI DOC GYRO)");

    return reachedTarget;
}

float MPU6050::getRoll() const {
    return _roll;
}

float MPU6050::getPitch() const {
    return _pitch;
}

float MPU6050::getYaw() const {
    return _yaw;
}

float MPU6050::getRelativeYaw() const {
    return _yaw - _yawSetpoint;
}

void MPU6050::setYawSetpoint() {
    _yawSetpoint = _yaw;
}

void MPU6050::resetYaw() {
    _yaw = 0.0;
    _yawSetpoint = 0.0;
}