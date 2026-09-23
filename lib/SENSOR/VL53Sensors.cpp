
#include "VL53Sensors.h"

// ================= CONSTRUCTOR =================

VL53Sensors::VL53Sensors()
{
    leftReady = false;
    frontReady = false;
    rightReady = false;
}

// ================= BEGIN =================

void VL53Sensors::begin()
{
    Wire.begin(SDA_PIN, SCL_PIN);

    pinMode(XSHUT_LEFT, OUTPUT);
    pinMode(XSHUT_FRONT, OUTPUT);
    pinMode(XSHUT_RIGHT, OUTPUT);

    setupSensors();
}

// ================= SETUP SENSORS =================

void VL53Sensors::setupSensors()
{
    // Tắt cả 3 cảm biến
    digitalWrite(XSHUT_LEFT, LOW);
    digitalWrite(XSHUT_FRONT, LOW);
    digitalWrite(XSHUT_RIGHT, LOW);

    delay(100);

    // =================================================
    // LEFT
    // =================================================

    digitalWrite(XSHUT_LEFT, HIGH);
    delay(100);

    sensorLeft.setTimeout(500);

    leftReady = sensorLeft.init();

    if (leftReady)
    {
        sensorLeft.setAddress(ADDRESS_LEFT);
        sensorLeft.startContinuous();
    }

    // =================================================
    // FRONT
    // =================================================

    digitalWrite(XSHUT_FRONT, HIGH);
    delay(100);

    sensorFront.setTimeout(500);

    frontReady = sensorFront.init();

    if (frontReady)
    {
        sensorFront.setAddress(ADDRESS_FRONT);
        sensorFront.startContinuous();
    }

    // =================================================
    // RIGHT
    // =================================================

    digitalWrite(XSHUT_RIGHT, HIGH);
    delay(100);

    sensorRight.setTimeout(500);

    rightReady = sensorRight.init();

    if (rightReady)
    {
        sensorRight.setAddress(ADDRESS_RIGHT);
        sensorRight.startContinuous();
    }

    // =================================================
    // DEBUG
    // =================================================

    Serial.print("VL53 LEFT: ");
    Serial.println(leftReady ? "OK" : "FAIL");

    Serial.print("VL53 FRONT: ");
    Serial.println(frontReady ? "OK" : "FAIL");

    Serial.print("VL53 RIGHT: ");
    Serial.println(rightReady ? "OK" : "FAIL");
}

// ================= READ LEFT =================

uint16_t VL53Sensors::readLeft()
{
    if (!leftReady)
    {
        return 0;
    }

    return sensorLeft.readRangeContinuousMillimeters();
}

// ================= READ FRONT =================

uint16_t VL53Sensors::readFront()
{
    if (!frontReady)
    {
        return 0;
    }

    return sensorFront.readRangeContinuousMillimeters();
}

// ================= READ RIGHT =================

uint16_t VL53Sensors::readRight()
{
    if (!rightReady)
    {
        return 0;
    }

    return sensorRight.readRangeContinuousMillimeters();
}

// ================= STATUS =================

bool VL53Sensors::leftOK()
{
    return leftReady;
}

bool VL53Sensors::frontOK()
{
    return frontReady;
}

bool VL53Sensors::rightOK()
{
    return rightReady;
}

