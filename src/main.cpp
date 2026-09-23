#include <Arduino.h>
#include <MPU6050.h>
#include <VL53L0X.h>
#include <Wire.h>
#include "BLEManager.h"
#include "PIDController.h"

#define SDA_PIN 19
#define SCL_PIN 18
#define XSHUT_LEFT 20
#define XSHUT_FRONT 9
#define XSHUT_RIGHT 8
#define M1_IN1 6
#define M1_IN2 7
#define M2_IN1 14
#define M2_IN2 15

VL53L0X sensorLeft;
VL53L0X sensorFront;
VL53L0X sensorRight;
MPU6050 mpu6050(0x68);

// Các bộ điều khiển PID
PIDController wallPID(1.2f, 0.005f, 0.25f, -120.0f, 120.0f);
PIDController gyroPID(2.0f, 0.01f, 0.4f, -120.0f, 120.0f);

bool leftReady = false;
bool frontReady = false;
bool rightReady = false;
bool mpuReady = false;

bool autoTestMode = false;
bool pidRunActive = false;
float targetYaw = 0.0f;
uint8_t baseForwardSpeed = 95;

// Thông số bù quán tính quay góc & tốc độ
float leftCompensation = 18.0f;
float rightCompensation = 8.0f;
uint8_t turnSpeed = 90;

unsigned long lastTelemetryTime = 0;
unsigned long lastPIDLoopTime = 0;

void stopMotors() {
    digitalWrite(M1_IN1, LOW);
    digitalWrite(M1_IN2, LOW);
    digitalWrite(M2_IN1, LOW);
    digitalWrite(M2_IN2, LOW);
}

void brakeMotors() {
    digitalWrite(M1_IN1, HIGH);
    digitalWrite(M1_IN2, HIGH);
    digitalWrite(M2_IN1, HIGH);
    digitalWrite(M2_IN2, HIGH);
    delay(25);
    stopMotors();
}

/**
 * Rẽ phải tại chỗ góc tùy chọn bằng MPU6050
 */
void turnRight(float angle = 90.0f, float compensation = rightCompensation, uint8_t speed = turnSpeed) {
    if (!mpuReady) {
        String msg = "MPU6050 chua san sang, khong the quay!";
        Serial.println(msg);
        bleManager.println(msg);
        return;
    }
    bool completed = mpu6050.rotateToAngle(-angle, M1_IN1, M1_IN2, M2_IN1, M2_IN2, compensation, speed);
    String res = completed ? ">> DA RE PHAI XONG" : ">> CANH BAO: RE PHAI TIMEOUT";
    Serial.println(res);
    bleManager.println(res);
}

/**
 * Rẽ trái tại chỗ góc tùy chọn bằng MPU6050
 */
void turnLeft(float angle = 90.0f, float compensation = leftCompensation, uint8_t speed = turnSpeed) {
    if (!mpuReady) {
        String msg = "MPU6050 chua san sang, khong the quay!";
        Serial.println(msg);
        bleManager.println(msg);
        return;
    }

    bool completed = mpu6050.rotateToAngle(angle, M1_IN1, M1_IN2, M2_IN1, M2_IN2, compensation, speed);
    String res = completed ? ">> DA RE TRAI XONG" : ">> CANH BAO: RE TRAI TIMEOUT";
    Serial.println(res);
    bleManager.println(res);
}

void runTurnTest() {
    if (!mpuReady) return;

    bleManager.println("==========================================");
    bleManager.println(">> AUTO TEST: CHUAN BI QUAY TRAI 90 DO...");
    delay(800);
    turnLeft(90.0f, leftCompensation, turnSpeed);
    stopMotors();
    delay(1500);

    bleManager.println(">> AUTO TEST: CHUAN BI QUAY PHAI 90 DO VE HUONG CU...");
    delay(800);
    turnRight(90.0f, rightCompensation, turnSpeed);
    stopMotors();
    delay(1500);
    bleManager.println(">> AUTO TEST: HOAN TAT 1 CHU KY.");
}

/**
 * Thuật toán PID Bám Tường & Giữ Hướng Thẳng
 */
void updatePIDLoop() {
    if (!pidRunActive) return;

    unsigned long now = micros();
    float dt = (now - lastPIDLoopTime) / 1000000.0f;
    if (lastPIDLoopTime == 0 || dt > 0.5f) dt = 0.01f;
    lastPIDLoopTime = now;

    uint16_t dL = leftReady ? sensorLeft.readRangeContinuousMillimeters() : 999;
    uint16_t dF = frontReady ? sensorFront.readRangeContinuousMillimeters() : 999;
    uint16_t dR = rightReady ? sensorRight.readRangeContinuousMillimeters() : 999;

    // Phanh dừng khẩn cấp nếu gặp tường trước sát nút (< 60mm)
    if (dF < 60) {
        stopMotors();
        pidRunActive = false;
        String msg = ">> [PID] PHANH DUNG: GAP VAC TUONG TRUOC (" + String(dF) + " mm)";
        Serial.println(msg);
        bleManager.println(msg);
        return;
    }

    constexpr uint16_t WALL_THRESHOLD = 180; // Ngưỡng nhận diện tường
    constexpr float TARGET_WALL_DIST = 60.0f; // Khoảng cách tường mục tiêu (mm)

    bool hasLeftWall = (dL < WALL_THRESHOLD);
    bool hasRightWall = (dR < WALL_THRESHOLD);

    float error = 0.0f;
    float pidOut = 0.0f;

    if (hasLeftWall && hasRightWall) {
        // Có cả 2 tường: Giữ xe đúng vị trí chính giữa (Chênh lệch = 0)
        error = (float)dL - (float)dR;
        pidOut = wallPID.computeError(error, dt);
    } else if (hasLeftWall) {
        // Chỉ có tường trái: Giữ khoảng cách dL = TARGET_WALL_DIST
        error = ((float)dL - TARGET_WALL_DIST) * 1.5f;
        pidOut = wallPID.computeError(error, dt);
    } else if (hasRightWall) {
        // Chỉ có tường phải: Giữ khoảng cách dR = TARGET_WALL_DIST
        error = (TARGET_WALL_DIST - (float)dR) * 1.5f;
        pidOut = wallPID.computeError(error, dt);
    } else {
        // Không có tường hai bên: Chuyển sang PID giữ góc Yaw bằng Gyro
        mpu6050.update();
        float currentYaw = mpu6050.getYaw();
        error = targetYaw - currentYaw;
        pidOut = gyroPID.computeError(error, dt);
    }

    // Tính tốc độ PWM 2 động cơ M1 (Trái) và M2 (Phải)
    int leftSpeed = baseForwardSpeed - (int)pidOut;
    int rightSpeed = baseForwardSpeed + (int)pidOut;

    leftSpeed = constrain(leftSpeed, 0, 255);
    rightSpeed = constrain(rightSpeed, 0, 255);

    // Xuất xung PWM điều khiển động cơ tiến
    analogWrite(M1_IN1, leftSpeed);
    analogWrite(M1_IN2, 0);
    analogWrite(M2_IN1, rightSpeed);
    analogWrite(M2_IN2, 0);
}

void printStatus() {
    uint16_t dLeft = leftReady ? sensorLeft.readRangeContinuousMillimeters() : 0;
    uint16_t dFront = frontReady ? sensorFront.readRangeContinuousMillimeters() : 0;
    uint16_t dRight = rightReady ? sensorRight.readRangeContinuousMillimeters() : 0;
    
    mpu6050.update();
    float yaw = mpu6050.getYaw();

    String statusMsg = "--- STATS ---\n";
    statusMsg += "Yaw: " + String(yaw, 2) + " deg\n";
    statusMsg += "TOF (L/F/R): " + String(dLeft) + " / " + String(dFront) + " / " + String(dRight) + " mm\n";
    statusMsg += "PARAMS -> LEFT_COMP: " + String(leftCompensation, 1) + " | RIGHT_COMP: " + String(rightCompensation, 1) + " | SPEED: " + String(turnSpeed) + "\n";
    statusMsg += "PID GAINS -> Kp: " + String(wallPID.getKp(), 2) + " | Ki: " + String(wallPID.getKi(), 3) + " | Kd: " + String(wallPID.getKd(), 2) + "\n";
    statusMsg += "PID ACTIVE: " + String(pidRunActive ? "YES" : "NO");

    Serial.println(statusMsg);
    bleManager.println(statusMsg);
}

void processBLECommands() {
    if (!bleManager.hasCommand()) return;

    String cmd = bleManager.readCommand();
    cmd.toUpperCase();

    if (cmd == "TL" || cmd == "TEST_LEFT") {
        pidRunActive = false;
        bleManager.println(">> [BLE] LENH: RE TRAI 90 DO");
        turnLeft(90.0f, leftCompensation, turnSpeed);
        stopMotors();
    } else if (cmd == "TR" || cmd == "TEST_RIGHT") {
        pidRunActive = false;
        bleManager.println(">> [BLE] LENH: RE PHAI 90 DO");
        turnRight(90.0f, rightCompensation, turnSpeed);
        stopMotors();
    } else if (cmd == "PID_ON" || cmd == "FORWARD" || cmd == "FWD") {
        autoTestMode = false;
        mpu6050.update();
        targetYaw = mpu6050.getYaw();
        wallPID.reset();
        gyroPID.reset();
        pidRunActive = true;
        lastPIDLoopTime = micros();
        bleManager.println(">> [BLE] KICH HOAT PID BAM TUONG & GIU HUONG TIEN THANG!");
    } else if (cmd == "PID_OFF" || cmd == "STOP" || cmd == "ST") {
        pidRunActive = false;
        autoTestMode = false;
        stopMotors();
        bleManager.println(">> [BLE] DA DUNG XE (PID OFF)");
    } else if (cmd.startsWith("SET_LC=")) {
        float val = cmd.substring(7).toFloat();
        if (val >= 0.0f && val <= 45.0f) {
            leftCompensation = val;
            bleManager.println(">> [BLE] CAP NHAT LEFT_COMPENSATION = " + String(leftCompensation, 1));
        }
    } else if (cmd.startsWith("SET_RC=")) {
        float val = cmd.substring(7).toFloat();
        if (val >= 0.0f && val <= 45.0f) {
            rightCompensation = val;
            bleManager.println(">> [BLE] CAP NHAT RIGHT_COMPENSATION = " + String(rightCompensation, 1));
        }
    } else if (cmd.startsWith("SET_SPD=")) {
        int val = cmd.substring(8).toInt();
        if (val >= 40 && val <= 255) {
            turnSpeed = (uint8_t)val;
            baseForwardSpeed = (uint8_t)val;
            bleManager.println(">> [BLE] CAP NHAT SPEED = " + String(turnSpeed));
        }
    } else if (cmd.startsWith("SET_KP=")) {
        float val = cmd.substring(7).toFloat();
        wallPID.setGains(val, wallPID.getKi(), wallPID.getKd());
        gyroPID.setGains(val * 1.5f, gyroPID.getKi(), gyroPID.getKd());
        bleManager.println(">> [BLE] CAP NHAT PID Kp = " + String(val, 2));
    } else if (cmd.startsWith("SET_KI=")) {
        float val = cmd.substring(7).toFloat();
        wallPID.setGains(wallPID.getKp(), val, wallPID.getKd());
        gyroPID.setGains(gyroPID.getKp(), val, gyroPID.getKd());
        bleManager.println(">> [BLE] CAP NHAT PID Ki = " + String(val, 3));
    } else if (cmd.startsWith("SET_KD=")) {
        float val = cmd.substring(7).toFloat();
        wallPID.setGains(wallPID.getKp(), wallPID.getKi(), val);
        gyroPID.setGains(gyroPID.getKp(), gyroPID.getKi(), val * 1.5f);
        bleManager.println(">> [BLE] CAP NHAT PID Kd = " + String(val, 2));
    } else if (cmd == "AUTO_ON") {
        pidRunActive = false;
        autoTestMode = true;
        bleManager.println(">> [BLE] KICH HOAT CHEDO AUTO TEST RE TRAI/PHAI");
    } else if (cmd == "AUTO_OFF") {
        autoTestMode = false;
        pidRunActive = false;
        stopMotors();
        bleManager.println(">> [BLE] TAT CHEDO AUTO TEST");
    } else if (cmd == "STATUS" || cmd == "GET") {
        printStatus();
    }
}

bool initVL53(VL53L0X &sensor, uint8_t xshutPin, uint8_t address, const char *name) {
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

void setup() {
    Serial.begin(115200);
    delay(500);

    pinMode(M1_IN1, OUTPUT);
    pinMode(M1_IN2, OUTPUT);
    pinMode(M2_IN1, OUTPUT);
    pinMode(M2_IN2, OUTPUT);
    stopMotors();
    Serial.println("MOTORS STOPPED");

    Wire.begin(SDA_PIN, SCL_PIN);
    delay(100);

    // 1. Khởi tạo BLE Telemetry Server
    bleManager.begin("MM-Robot-BLE");

    // 2. Khởi tạo Cảm biến khoảng cách VL53L0X
    pinMode(XSHUT_LEFT, OUTPUT);
    pinMode(XSHUT_FRONT, OUTPUT);
    pinMode(XSHUT_RIGHT, OUTPUT);
    digitalWrite(XSHUT_LEFT, LOW);
    digitalWrite(XSHUT_FRONT, LOW);
    digitalWrite(XSHUT_RIGHT, LOW);
    delay(100);

    leftReady = initVL53(sensorLeft, XSHUT_LEFT, 0x30, "LEFT");
    frontReady = initVL53(sensorFront, XSHUT_FRONT, 0x31, "FRONT");
    rightReady = initVL53(sensorRight, XSHUT_RIGHT, 0x32, "RIGHT");

    // 3. Khởi tạo Gyro MPU6050
    Wire.beginTransmission(0x68);
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

void loop() {
    // Xử lý các lệnh đến từ điện thoại qua BLE
    processBLECommands();

    // Vòng lặp cập nhật PID bám tường
    if (pidRunActive) {
        updatePIDLoop();
    }

    // Cập nhật giá trị MPU6050 liên tục
    if (mpuReady) {
        mpu6050.update();
    }

    // Đẩy Telemetry định kỳ mỗi 200ms khi có BLE kết nối
    if (bleManager.isConnected() && millis() - lastTelemetryTime >= 200) {
        lastTelemetryTime = millis();
        float yaw = mpu6050.getYaw();
        uint16_t dLeft = leftReady ? sensorLeft.readRangeContinuousMillimeters() : 0;
        uint16_t dFront = frontReady ? sensorFront.readRangeContinuousMillimeters() : 0;
        uint16_t dRight = rightReady ? sensorRight.readRangeContinuousMillimeters() : 0;
        
        String jsonMsg = "{\"type\":\"telemetry\",\"yaw\":" + String(yaw, 1) +
                         ",\"l\":" + String(dLeft) +
                         ",\"f\":" + String(dFront) +
                         ",\"r\":" + String(dRight) +
                         ",\"lc\":" + String(leftCompensation, 1) +
                         ",\"rc\":" + String(rightCompensation, 1) +
                         ",\"spd\":" + String(turnSpeed) +
                         ",\"kp\":" + String(wallPID.getKp(), 2) +
                         ",\"ki\":" + String(wallPID.getKi(), 3) +
                         ",\"kd\":" + String(wallPID.getKd(), 2) +
                         ",\"pid\":" + String(pidRunActive ? "true" : "false") +
                         ",\"auto\":" + String(autoTestMode ? "true" : "false") + "}";
        bleManager.println(jsonMsg);
    }

    // Chế độ Auto Test nếu bật
    if (autoTestMode && mpuReady) {
        runTurnTest();
    } else if (!autoTestMode && !pidRunActive) {
        delay(10);
    }
}
