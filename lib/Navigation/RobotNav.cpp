#include "RobotNav.h"

RobotNav robotNav;

RobotNav::RobotNav()
    : mpu6050(MPU6050_ADDR),
      wallPID(0.25f, 0.0f, 0.8f, -40.0f,
              40.0f), // Kp 0.25, THÊM Kd = 0.8 làm phanh giảm xóc chống quá đà
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
}

void RobotNav::init() {
  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);
  stopMotors();

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
  
  if (raw_dL < 800) smooth_dL = (0.5f * raw_dL) + (0.5f * smooth_dL); else smooth_dL = 999;
  if (raw_dR < 800) smooth_dR = (0.5f * raw_dR) + (0.5f * smooth_dR); else smooth_dR = 999;

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

  float error = 0.0f;
  float pidOut = 0.0f;

  if (hasLeftWall && hasRightWall) {
    // Cả 2 bên đều có tường: sai số lệch tâm ô chuẩn
    error = ((float)dL - (float)dR) - centerOffset;
    if (abs(error) < 5.0f) error = 0.0f; // Khử nhiễu: Sai lệch dưới 5mm coi như xe đang đi thẳng
    pidOut = wallPID.computeError(error, dt);
  } else if (hasLeftWall) {
    // Chỉ có tường trái: bám tường trái ở cự ly chuẩn
    error = ((float)dL - targetLeftDist);
    if (abs(error) < 5.0f) error = 0.0f;
    pidOut = wallPID.computeError(error, dt);
  } else if (hasRightWall) {
    // Chỉ có tường phải: bám tường phải ở cự ly chuẩn
    error = (targetRightDist - (float)dR);
    if (abs(error) < 5.0f) error = 0.0f;
    pidOut = wallPID.computeError(error, dt);
  } else {
    // Không có tường hai bên (ngã tư): dùng Gyro MPU6050 giữ góc thẳng
    mpu6050.update();
    float currentYaw = mpu6050.getYaw();
    error = _targetYaw - currentYaw;
    pidOut = gyroPID.computeError(error, dt);
  }

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
