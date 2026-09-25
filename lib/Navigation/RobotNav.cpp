#include "RobotNav.h"

RobotNav robotNav;

RobotNav::RobotNav()
    : mpu6050(MPU6050_ADDR),
      wallPID(0.18f, 0.0f, 0.0f, -20.0f,
              20.0f), // Kp 0.18, ép Kd = 0.0 triệt tiêu sốc bẻ lái ToF, kẹp output [-20, 20]
      gyroPID(0.6f, 0.0f, 0.0f, -60.0f, 60.0f) {
  leftReady = false;
  frontReady = false;
  rightReady = false;
  mpuReady = false;
  autoTestMode = false;
  pidRunActive = false;

  leftCompensation = 18.0f;
  rightCompensation = 18.0f;
  turnSpeed = 80;
  baseForwardSpeed = 75;
  currentLeftSpeed = 0;
  currentRightSpeed = 0;
  _targetYaw = 0.0f;
  _lastPIDLoopTime = 0;

  // Giá trị thực tế đo được tại tâm ô (Theo kết quả đo mới nhất trên sa hình)
  targetLeftDist = 170.0f;
  targetRightDist = 149.0f;
  centerOffset = 21.0f; // 170.0f - 149.0f = 21.0f
  wallThreshold = 230; // Khoảng cách < 230mm là có tường, > 230mm là cửa trống
  frontStopDist = 100; // Phanh dừng khi cách tường trước <= 100mm

  // Cấu hình đồng bộ bánh bằng Encoder (Cascaded Inner Loop)
  encoderSyncActive = true;
  encKp = 0.35f;
  invertEncLeft = false;
  invertEncRight = false;
  _lastEncLeft = 0;
  _lastEncRight = 0;
  _startEncLeft = 0;
  _startEncRight = 0;
  _smoothError = 0.0f;
}

long RobotNav::getLeftEncoder() const {
  return invertEncLeft ? -enc1A_count : enc1A_count;
}

long RobotNav::getRightEncoder() const {
  return invertEncRight ? -enc2A_count : enc2A_count;
}

void RobotNav::resetEnc() {
  resetEncoders();
  _lastEncLeft = 0;
  _lastEncRight = 0;
  _startEncLeft = 0;
  _startEncRight = 0;
}

void RobotNav::init() {
  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);
  stopMotors();

  // Khởi tạo phần cứng Encoder đọc xung bánh xe
  setupEncoders();
  resetEnc();
  Serial.println("ENCODERS INITIALIZED");

  Wire.begin(SDA_PIN, SCL_PIN);
  delay(100);

  pinMode(XSHUT_LEFT, OUTPUT);
  pinMode(XSHUT_FRONT, OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);
  digitalWrite(XSHUT_LEFT, LOW);
  digitalWrite(XSHUT_FRONT, LOW);
  digitalWrite(XSHUT_RIGHT, LOW);
  delay(100);

  leftReady = initVL53(sensorLeft, XSHUT_LEFT, ADDRESS_LEFT, "LEFT");
  frontReady = initVL53(sensorFront, XSHUT_FRONT, ADDRESS_FRONT, "FRONT");
  rightReady = initVL53(sensorRight, XSHUT_RIGHT, ADDRESS_RIGHT, "RIGHT");

  Wire.beginTransmission(MPU6050_ADDR);
  if (Wire.endTransmission() == 0) {
    Serial.println("MPU6050 FOUND @ 0x68");
    mpuReady = mpu6050.begin();
    if (mpuReady) {
      mpu6050.calibrate();
      Serial.println("MPU6050 READY - Z AXIS ONLY");
      bleManager.println(">> MPU6050 & SENSORS READY!");
    } else {
      Serial.println("MPU6050 INIT FAIL");
    }
  } else {
    Serial.println("MPU6050 NOT FOUND @ 0x68");
  }
}

bool RobotNav::initVL53(VL53L0X &sensor, uint8_t xshutPin, uint8_t address,
                        const char *name) {
  digitalWrite(xshutPin, HIGH);
  delay(50);
  sensor.setTimeout(500);

  if (!sensor.init()) {
    Serial.print(name);
    Serial.println(" VL53 FAIL");
    return false;
  }

  sensor.setAddress(address);
  sensor.startContinuous();
  Serial.print(name);
  Serial.print(" VL53 OK @ 0x");
  Serial.println(address, HEX);
  return true;
}

void RobotNav::stopMotors() {
  currentLeftSpeed = 0;
  currentRightSpeed = 0;
  analogWrite(M1_IN1, 0);
  analogWrite(M1_IN2, 0);
  analogWrite(M2_IN1, 0);
  analogWrite(M2_IN2, 0);
  digitalWrite(M1_IN1, LOW);
  digitalWrite(M1_IN2, LOW);
  digitalWrite(M2_IN1, LOW);
  digitalWrite(M2_IN2, LOW);
}

void RobotNav::brakeMotors() {
  analogWrite(M1_IN1, 0);
  analogWrite(M1_IN2, 0);
  analogWrite(M2_IN1, 0);
  analogWrite(M2_IN2, 0);
  digitalWrite(M1_IN1, HIGH);
  digitalWrite(M1_IN2, HIGH);
  digitalWrite(M2_IN1, HIGH);
  digitalWrite(M2_IN2, HIGH);
  delay(25);
  stopMotors();
}

void RobotNav::turnRight(float angle) {
  if (!mpuReady) {
    String msg = "MPU6050 chua san sang, khong the quay!";
    Serial.println(msg);
    bleManager.println(msg);
    return;
  }
  bool completed = mpu6050.rotateToAngle(-angle, M1_IN1, M1_IN2, M2_IN1, M2_IN2,
                                         rightCompensation, turnSpeed);
  String res =
      completed ? ">> DA RE PHAI XONG" : ">> CANH BAO: RE PHAI TIMEOUT";
  Serial.println(res);
  bleManager.println(res);
}

void RobotNav::turnLeft(float angle) {
  if (!mpuReady) {
    String msg = "MPU6050 chua san sang, khong the quay!";
    Serial.println(msg);
    bleManager.println(msg);
    return;
  }

  bool completed = mpu6050.rotateToAngle(angle, M1_IN1, M1_IN2, M2_IN1, M2_IN2,
                                         leftCompensation, turnSpeed);
  String res =
      completed ? ">> DA RE TRAI XONG" : ">> CANH BAO: RE TRAI TIMEOUT";
  Serial.println(res);
  bleManager.println(res);
}

void RobotNav::runTurnTest() {
  if (!mpuReady)
    return;

  bleManager.println("==========================================");
  bleManager.println(">> AUTO TEST: CHUAN BI QUAY TRAI 90 DO...");
  delay(800);
  turnLeft(90.0f);
  stopMotors();
  delay(1500);

  bleManager.println(">> AUTO TEST: CHUAN BI QUAY PHAI 90 DO VE HUONG CU...");
  delay(800);
  turnRight(90.0f);
  stopMotors();
  delay(1500);
  bleManager.println(">> AUTO TEST: HOAN TAT 1 CHU KY.");
}

void RobotNav::startPID() {
  autoTestMode = false;
  if (mpuReady) {
    mpu6050.update();
    _targetYaw = mpu6050.getYaw();
  }
  wallPID.reset();
  gyroPID.reset();
  _startEncLeft = getLeftEncoder();
  _startEncRight = getRightEncoder();
  _lastEncLeft = _startEncLeft;
  _lastEncRight = _startEncRight;
  _smoothError = 0.0f;
  _lastPIDLoopTime = micros();
  pidRunActive = true;
}

void RobotNav::stopPID() {
  pidRunActive = false;
  autoTestMode = false;
  stopMotors();
}

void RobotNav::updatePIDLoop() {
  if (!pidRunActive)
    return;

  unsigned long now = micros();
  float dt = (now - _lastPIDLoopTime) / 1000000.0f;
  if (_lastPIDLoopTime == 0 || dt > 0.5f || dt <= 0.0f)
    dt = 0.01f;
  _lastPIDLoopTime = now;

  // Đọc raw data
  uint16_t raw_dL = leftReady ? sensorLeft.readRangeContinuousMillimeters() : 999;
  uint16_t raw_dF = frontReady ? sensorFront.readRangeContinuousMillimeters() : 999;
  uint16_t raw_dR = rightReady ? sensorRight.readRangeContinuousMillimeters() : 999;

  // Khởi tạo bộ lọc EMA (Exponential Moving Average)
  static float smooth_dL = targetLeftDist;
  static float smooth_dR = targetRightDist;
  
  if (raw_dL < 800) {
    smooth_dL = (0.4f * raw_dL) + (0.6f * smooth_dL);
  } else {
    smooth_dL = (0.15f * 999.0f) + (0.85f * smooth_dL); // Chuyển tiếp mượt khi mất tường/lóa, không giật vọt
  }

  if (raw_dR < 800) {
    smooth_dR = (0.4f * raw_dR) + (0.6f * smooth_dR);
  } else {
    smooth_dR = (0.15f * 999.0f) + (0.85f * smooth_dR);
  }

  uint16_t dL = (uint16_t)smooth_dL;
  uint16_t dR = (uint16_t)smooth_dR;
  uint16_t dF = raw_dF; // Phía trước cần phản ứng nhanh để phanh, không nên lọc

  // 1. Phanh dừng an toàn khi gặp vách tường trước (ở tâm là 110mm, tới <= 65mm
  // thì dừng)
  if (frontReady && dF > 20 && dF <= frontStopDist) {
    stopMotors();
    pidRunActive = false;
    String msg =
        ">> [PID] PHANH DUNG: GAP VAC TUONG TRUOC (" + String(dF) + " mm)";
    Serial.println(msg);
    bleManager.println(msg);
    return;
  }

  bool hasLeftWall = (leftReady && dL > 20 && dL < wallThreshold);
  bool hasRightWall = (rightReady && dR > 20 && dR < wallThreshold);

  // 1. Tính độ chênh lệch xung tức thời giữa 2 bánh trong chu kỳ này
  long curL = getLeftEncoder();
  long curR = getRightEncoder();
  long deltaL = curL - _lastEncLeft;
  long deltaR = curR - _lastEncRight;
  _lastEncLeft = curL;
  _lastEncRight = curR;

  // Sai số Encoder: nếu bánh trái quay nhiều hơn bánh phải (đầu xe lệch phải) -> enc_error > 0
  float enc_error = (float)(deltaL - deltaR);

  // 2. Tính Sai số Thô (Raw Error) theo từng trường hợp tường (Hybrid Error)
  float raw_error = 0.0f;
  const float ENC_WEIGHT = 0.35f; // Trọng số kìm quán tính của Encoder

  if (hasLeftWall && hasRightWall) {
    // Trường hợp 1: Có cả 2 tường (Cân bằng tâm ô + Giữ thẳng bằng Encoder)
    float wall_error = ((float)dL - (float)dR) - centerOffset;
    raw_error = (0.7f * wall_error) + (ENC_WEIGHT * enc_error);

  } else if (hasLeftWall) {
    // Trường hợp 2: Chỉ có tường trái (Bám tường trái + Giữ quỹ đạo bằng Encoder)
    float wall_error = ((float)dL - targetLeftDist);
    raw_error = (0.7f * wall_error) + (ENC_WEIGHT * enc_error);

  } else if (hasRightWall) {
    // Trường hợp 3: Chỉ có tường phải (Bám tường phải + Giữ quỹ đạo bằng Encoder)
    float wall_error = (targetRightDist - (float)dR);
    raw_error = (0.7f * wall_error) + (ENC_WEIGHT * enc_error);

  } else {
    // Trường hợp 4: Không có tường (Ngã tư) -> Phối hợp Encoder & Gyro giữ thẳng tuyệt đối
    mpu6050.update();
    float gyro_error = _targetYaw - mpu6050.getYaw();
    raw_error = (0.6f * gyro_error) + (0.4f * enc_error);
  }

  // 3. Lọc mượt sai số qua Low-Pass Filter (chống gai nhọn khe nứt tường)
  const float ALPHA = 0.3f; // 30% giá trị mới, 70% giữ quán tính mượt mà
  _smoothError = (ALPHA * raw_error) + ((1.0f - ALPHA) * _smoothError);

  // 4. Kẹp trần sai số tối đa và tính tín hiệu điều khiển PID
  const float MAX_WALL_ERROR = 20.0f;
  float final_error = constrain(_smoothError, -MAX_WALL_ERROR, MAX_WALL_ERROR);
  float pidOut = wallPID.computeError(final_error, dt);

  // 5. Xuất PWM cho 2 bánh
  int leftSpeed = baseForwardSpeed - (int)pidOut;
  int rightSpeed = baseForwardSpeed + (int)pidOut;

  leftSpeed = constrain(leftSpeed, 50, 255);
  rightSpeed = constrain(rightSpeed, 50, 255);

  currentLeftSpeed = leftSpeed;
  currentRightSpeed = rightSpeed;

  analogWrite(M1_IN1, leftSpeed);
  analogWrite(M1_IN2, 0);
  analogWrite(M2_IN1, rightSpeed);
  analogWrite(M2_IN2, 0);
}

void RobotNav::update() {
  if (pidRunActive) {
    updatePIDLoop();
  }

  if (mpuReady) {
    mpu6050.update();
  }

  if (autoTestMode && mpuReady) {
    runTurnTest();
  }
}
