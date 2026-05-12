#include "smc2.h"
#include <Arduino.h>
#include <math.h>

smc::smc(float _k1, float _k2, float _k3, float _k4, float _lambda1, float _lambda2, float _min, float _max) {
    k1 = _k1; k2 = _k2; k3 = _k3; k4 = _k4;
    lambda1 = _lambda1; lambda2 = _lambda2;
    minOut = _min; maxOut = _max;
}

float smc::compute(float x, float x_dot, float theta, float theta_dot) {
    float s2 = theta_dot + lambda2*theta;
    float s1 = x_dot + lambda1*x;
    float u = -k1*s1 - k2*s2 - k3*tanh(s1) - k4*tanh(s2);
    if (u > maxOut) u = maxOut;
    else if (u < minOut) u = minOut;

    return u;
}

void smc::setK1(float val) { k1 = val; }
void smc::setK2(float val) { k2 = val; }
void smc::setK3(float val) { k3 = val; }
void smc::setK4(float val) { k4 = val; }
void smc::setLambda1(float val) { lambda1 = val; }
void smc::setLambda2(float val) { lambda2 = val; }