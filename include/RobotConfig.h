#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include <Arduino.h>

// ================= I2C PINOUT =================
#define SDA_PIN 19
#define SCL_PIN 18

// ================= VL53L0X XSHUT PINS =================
#define XSHUT_LEFT  20
#define XSHUT_FRONT 9
#define XSHUT_RIGHT 8

// ================= VL53L0X I2C ADDRESSES =================
#define ADDRESS_LEFT  0x30
#define ADDRESS_FRONT 0x31
#define ADDRESS_RIGHT 0x32

// ================= DRV8833 MOTOR DRIVER PINS =================
#define M1_IN1 6   // Động cơ trái
#define M1_IN2 7
#define M2_IN1 14  // Động cơ phải
#define M2_IN2 15

// ================= MPU6050 I2C ADDRESS =================
#define MPU6050_ADDR 0x68

#endif
