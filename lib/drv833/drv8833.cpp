#include "drv8833.h"

void setupMotors() {
    pinMode(M1_IN1, OUTPUT);
    pinMode(M1_IN2, OUTPUT);
    pinMode(M2_IN1, OUTPUT);
    pinMode(M2_IN2, OUTPUT);

    motorStop();
}

void motorForward() {
    // Cùng logic với motorForward() trong main.cpp.
    digitalWrite(M1_IN1, HIGH);
    digitalWrite(M1_IN2, LOW);

    digitalWrite(M2_IN1, HIGH);
    digitalWrite(M2_IN2, LOW);
}

void motorBackward() {
    // Cùng logic với motorBackward() trong main.cpp.
    digitalWrite(M1_IN1, LOW);
    digitalWrite(M1_IN2, HIGH);

    digitalWrite(M2_IN1, LOW);
    digitalWrite(M2_IN2, HIGH);
}

void motorStop() {
    // DRV8833 Dừng thả trôi (Coast / Fast Decay): Cả 2 chân LOW
    digitalWrite(M1_IN1, LOW);
    digitalWrite(M1_IN2, LOW);
    digitalWrite(M2_IN1, LOW);
    digitalWrite(M2_IN2, LOW);
}