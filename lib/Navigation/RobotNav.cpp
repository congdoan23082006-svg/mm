#include "RobotNav.h"

RobotNav robotNav;

RobotNav::RobotNav()
    : mpu6050(MPU6050_ADDR),
      wallPID(1.2f, 0.005f, 0.25f, -120.0f, 120.0f),
      gyroPID(2.0f, 0.01f, 0.4f, -120.0f, 120.0f) {
    leftReady = false;
    frontReady = false;
    rightReady = false;
    mpuReady = false;
    autoTestMode = false;
    pidRunActive = false;

    leftCompensation = 18.0f;
    rightCompensation = 8.0f;
    turnSpeed = 90;
    baseForwardSpeed = 95;
    _targetYaw = 0.0f;
    _lastPIDLoopTime = 0;
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

bool RobotNav::initVL53(VL53L0X &sensor, uint8_t xshutPin, uint8_t address, const char *name) {
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
    digitalWrite(M1_IN1, LOW);
    digitalWrite(M1_IN2, LOW);
    digitalWrite(M2_IN1, LOW);
    digitalWrite(M2_IN2, LOW);
}

void RobotNav::brakeMotors() {
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
    bool completed = mpu6050.rotateToAngle(-angle, M1_IN1, M1_IN2, M2_IN1, M2_IN2, rightCompensation, turnSpeed);
    String res = completed ? ">> DA RE PHAI XONG" : ">> CANH BAO: RE PHAI TIMEOUT";
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

    bool completed = mpu6050.rotateToAngle(angle, M1_IN1, M1_IN2, M2_IN1, M2_IN2, leftCompensation, turnSpeed);
    String res = completed ? ">> DA RE TRAI XONG" : ">> CANH BAO: RE TRAI TIMEOUT";
    Serial.println(res);
    bleManager.println(res);
}

void RobotNav::runTurnTest() {
    if (!mpuReady) return;

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

void RobotNav::updatePIDLoop() {
    if (!pidRunActive) return;

    unsigned long now = micros();
    float dt = (now - _lastPIDLoopTime) / 1000000.0f;
    if (_lastPIDLoopTime == 0 || dt > 0.5f) dt = 0.01f;
    _lastPIDLoopTime = now;

    uint16_t dL = leftReady ? sensorLeft.readRangeContinuousMillimeters() : 999;
    uint16_t dF = frontReady ? sensorFront.readRangeContinuousMillimeters() : 999;
    uint16_t dR = rightReady ? sensorRight.readRangeContinuousMillimeters() : 999;

    if (dF < 60) {
        stopMotors();
        pidRunActive = false;
        String msg = ">> [PID] PHANH DUNG: GAP VAC TUONG TRUOC (" + String(dF) + " mm)";
        Serial.println(msg);
        bleManager.println(msg);
        return;
    }

    constexpr uint16_t WALL_THRESHOLD = 180;
    constexpr float TARGET_WALL_DIST = 60.0f;

    bool hasLeftWall = (dL < WALL_THRESHOLD);
    bool hasRightWall = (dR < WALL_THRESHOLD);

    float error = 0.0f;
    float pidOut = 0.0f;

    if (hasLeftWall && hasRightWall) {
        error = (float)dL - (float)dR;
        pidOut = wallPID.computeError(error, dt);
    } else if (hasLeftWall) {
        error = ((float)dL - TARGET_WALL_DIST) * 1.5f;
        pidOut = wallPID.computeError(error, dt);
    } else if (hasRightWall) {
        error = (TARGET_WALL_DIST - (float)dR) * 1.5f;
        pidOut = wallPID.computeError(error, dt);
    } else {
        mpu6050.update();
        float currentYaw = mpu6050.getYaw();
        error = _targetYaw - currentYaw;
        pidOut = gyroPID.computeError(error, dt);
    }

    int leftSpeed = baseForwardSpeed - (int)pidOut;
    int rightSpeed = baseForwardSpeed + (int)pidOut;

    leftSpeed = constrain(leftSpeed, 0, 255);
    rightSpeed = constrain(rightSpeed, 0, 255);

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
