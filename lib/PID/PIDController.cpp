#include "PIDController.h"

PIDController::PIDController(float kp, float ki, float kd, float minOut, float maxOut) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
    _minOutput = minOut;
    _maxOutput = maxOut;
    _integral = 0.0f;
    _prevError = 0.0f;
}

void PIDController::setGains(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

void PIDController::setOutputLimits(float minOut, float maxOut) {
    _minOutput = minOut;
    _maxOutput = maxOut;
}

void PIDController::reset() {
    _integral = 0.0f;
    _prevError = 0.0f;
}

float PIDController::compute(float setpoint, float input, float dt) {
    float error = setpoint - input;
    return computeError(error, dt);
}

float PIDController::computeError(float error, float dt) {
    if (dt <= 0.0f) dt = 0.01f;

    // 1. Thành phần Proportional (P)
    float pTerm = _kp * error;

    // 2. Thành phần Integral (I) với Anti-Windup
    _integral += error * dt;
    float iTerm = _ki * _integral;

    // Giới hạn chống bão hòa tích phân (Anti-Windup Clamping)
    if (iTerm > _maxOutput) {
        iTerm = _maxOutput;
        _integral = _maxOutput / (_ki > 0.0f ? _ki : 1.0f);
    } else if (iTerm < _minOutput) {
        iTerm = _minOutput;
        _integral = _minOutput / (_ki > 0.0f ? _ki : 1.0f);
    }

    // 3. Thành phần Derivative (D)
    float derivative = (error - _prevError) / dt;
    float dTerm = _kd * derivative;

    _prevError = error;

    // 4. Tổng tín hiệu điều khiển PID
    float output = pTerm + iTerm + dTerm;

    // Giới hạn biên đầu ra PID
    if (output > _maxOutput) output = _maxOutput;
    if (output < _minOutput) output = _minOutput;

    return output;
}
