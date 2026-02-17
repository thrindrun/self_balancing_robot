#include "pid.h"
#include <Arduino.h>

pid::pid(float p, float i, float d, float minVal, float maxVal) {
    kp = p; ki = i; kd = d;
    minOut = minVal; maxOut = maxVal;
    integral = 0;
    lastError = 0;
}

float pid::compute(float setpoint, float measuredValue, float dt) {
    float error = setpoint - measuredValue;
    float P = kp*error;
    integral += error*dt;
    float I = ki*integral;
    float D = kd*(error-lastError)/dt;
    float output = P+I+D;
    lastError = error;
    return constrain(output,minOut,maxOut);
}

void pid::reset() {
    integral = 0;
    lastError = 0;
}