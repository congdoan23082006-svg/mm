#include <Arduino.h>
#include <VL53L0X.h>
#include <Wire.h>



// =======================
// I2C
// =======================
#define SDA_PIN 19
#define SCL_PIN 18

// =======================
// VL53L0X XSHUT
// =======================
#define XSHUT_LEFT 20
#define XSHUT_FRONT 9
#define XSHUT_RIGHT 8

VL53L0X sensorLeft;
VL53L0X sensorFront;
VL53L0X sensorRight;

// =======================
// Motor driver
// =======================
#define M1_IN1 6
#define M1_IN2 7

#define M2_IN1 14
#define M2_IN2 15

// =======================
// Encoder
// =======================
#define ENC1_A 1
#define ENC1_B 2

#define ENC2_A 3
#define ENC2_B 4

volatile long enc1A_count = 0;
volatile long enc1B_count = 0;

volatile long enc2A_count = 0;
volatile long enc2B_count = 0;

// =======================
// Analog
// =======================
#define ANALOG_PIN 0

// =======================
// Motor direction timing
// =======================
bool motorDir = false;
unsigned long lastToggle = 0;

// ======================================================
// Encoder Interrupts
// ======================================================

void IRAM_ATTR enc1A_ISR() { enc1A_count++; }

void IRAM_ATTR enc1B_ISR() { enc1B_count++; }

void IRAM_ATTR enc2A_ISR() { enc2A_count++; }

void IRAM_ATTR enc2B_ISR() { enc2B_count++; }

// ======================================================
// Motor control
// ======================================================

void motorForward() {
    digitalWrite(M1_IN1, HIGH);
    digitalWrite(M1_IN2, LOW);

    digitalWrite(M2_IN1, HIGH);
    digitalWrite(M2_IN2, LOW);
}

void motorBackward() {
    digitalWrite(M1_IN1, LOW);
    digitalWrite(M1_IN2, HIGH);

    digitalWrite(M2_IN1, LOW);
    digitalWrite(M2_IN2, HIGH);
}

// ======================================================
// Setup VL53L0X
// ======================================================

void setupVL53() {
    pinMode(XSHUT_LEFT, OUTPUT);
    pinMode(XSHUT_FRONT, OUTPUT);
    pinMode(XSHUT_RIGHT, OUTPUT);

    // Tắt tất cả
    digitalWrite(XSHUT_LEFT, LOW);
    digitalWrite(XSHUT_FRONT, LOW);
    digitalWrite(XSHUT_RIGHT, LOW);

    delay(100);

    // LEFT
    digitalWrite(XSHUT_LEFT, HIGH);
    delay(50);

    sensorLeft.setTimeout(500);

    if (!sensorLeft.init()) {
        Serial.println("LEFT VL53 FAIL");
    }

    sensorLeft.setAddress(0x30);

    // FRONT
    digitalWrite(XSHUT_FRONT, HIGH);
    delay(50);

    sensorFront.setTimeout(500);

    if (!sensorFront.init()) {
        Serial.println("FRONT VL53 FAIL");
    }

    sensorFront.setAddress(0x31);

    // RIGHT
    digitalWrite(XSHUT_RIGHT, HIGH);
    delay(50);

    sensorRight.setTimeout(500);

    if (!sensorRight.init()) {
        Serial.println("RIGHT VL53 FAIL");
    }

    sensorRight.setAddress(0x32);

    sensorLeft.startContinuous();
    sensorFront.startContinuous();
    sensorRight.startContinuous();
}

// ======================================================
// Setup
// ======================================================

void setup() {
    Serial.begin(115200);

    Wire.begin(SDA_PIN, SCL_PIN);

    // Motor
    pinMode(M1_IN1, OUTPUT);
    pinMode(M1_IN2, OUTPUT);

    pinMode(M2_IN1, OUTPUT);
    pinMode(M2_IN2, OUTPUT);

    // Encoder
    pinMode(ENC1_A, INPUT_PULLUP);
    pinMode(ENC1_B, INPUT_PULLUP);

    pinMode(ENC2_A, INPUT_PULLUP);
    pinMode(ENC2_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENC1_A), enc1A_ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC1_B), enc1B_ISR, RISING);

    attachInterrupt(digitalPinToInterrupt(ENC2_A), enc2A_ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC2_B), enc2B_ISR, RISING);

    // VL53
    setupVL53();

    Serial.println("SYSTEM START");
}

// ======================================================
// Loop
// ======================================================

void loop() {
    // =========================
    // Đổi chiều mỗi 3 giây
    // =========================
    if (millis() - lastToggle > 3000) {
        lastToggle = millis();

        motorDir = !motorDir;

        if (motorDir) {
            motorForward();
        } else {
            motorBackward();
        }
    }

    // =========================
    // VL53L0X
    // =========================
    uint16_t leftDist = sensorLeft.readRangeContinuousMillimeters();
    uint16_t frontDist = sensorFront.readRangeContinuousMillimeters();
    uint16_t rightDist = sensorRight.readRangeContinuousMillimeters();

    // =========================
    // Analog
    // =========================
    int analogValue = analogRead(ANALOG_PIN);

    // =========================
    // Serial print
    // =========================
    Serial.print("VL53 LEFT: ");
    Serial.print(leftDist);

    Serial.print(" mm | FRONT: ");
    Serial.print(frontDist);

    Serial.print(" mm | RIGHT: ");
    Serial.print(rightDist);

    Serial.print(" mm | ENC1_A: ");
    Serial.print(enc1A_count);

    Serial.print(" | ENC1_B: ");
    Serial.print(enc1B_count);

    Serial.print(" | ENC2_A: ");
    Serial.print(enc2A_count);

    Serial.print(" | ENC2_B: ");
    Serial.print(enc2B_count);

    Serial.print(" | ADC0: ");
    Serial.println(analogValue);

    delay(100);
}