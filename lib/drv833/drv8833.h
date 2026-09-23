#ifndef DRV8833_H
#define DRV8833_H

#include <Arduino.h>

// Định nghĩa chân kết nối DRV8833
#define M1_IN1 7
#define M1_IN2 6
#define M2_IN1 15
#define M2_IN2 14

void setupMotors();
void motorForward();
void motorBackward();
void motorStop();

#endif