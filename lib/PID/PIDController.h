#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>

class PIDController {
public:
    PIDController(float kp = 0.0f, float ki = 0.0f, float kd = 0.0f, float minOut = -255.0f, float maxOut = 255.0f);

    void setGains(float kp, float ki, float kd);
    void setOutputLimits(float minOut, float maxOut);
    void reset();

    float compute(float setpoint, float input, float dt);
    float computeError(float error, float dt);

    float getKp() const { return _kp; }
    float getKi() const { return _ki; }
    float getKd() const { return _kd; }
    float getPrevError() const { return _prevError; }
    float getIntegral() const { return _integral; }

private:
    float _kp;
    float _ki;
    float _kd;

    float _integral;
    float _prevError;

    float _minOutput;
    float _maxOutput;
};

#endif
