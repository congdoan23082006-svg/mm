
#ifndef VL53SENSORS_H
#define VL53SENSORS_H

#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>

// ================= PIN =================
#define SDA_PIN        19
#define SCL_PIN        18

#define XSHUT_LEFT     20
#define XSHUT_FRONT    9
#define XSHUT_RIGHT    8

// ================= ADDRESS =================
#define ADDRESS_LEFT   0x30
#define ADDRESS_FRONT  0x31
#define ADDRESS_RIGHT  0x32

// ================= CLASS =================
class VL53Sensors {
public:
    VL53Sensors();

    void begin();

    uint16_t readLeft();
    uint16_t readFront();
    uint16_t readRight();

    bool leftOK();
    bool frontOK();
    bool rightOK();

private:
    VL53L0X sensorLeft;
    VL53L0X sensorFront;
    VL53L0X sensorRight;

    bool leftReady;
    bool frontReady;
    bool rightReady;

    void setupSensors();
};

#endif

