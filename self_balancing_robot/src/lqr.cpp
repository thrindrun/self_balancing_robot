#include "lqr.h"
#include <Arduino.h>

lqr::lqr(float _k1, float _k2, float _k3, float _k4, float _min, float _max) {
    k1 = _k1; k2 = _k2; k3 = _k3; k4 = _k4;
    minOut = _min; maxOut = _max;
}

float lqr::compute(float x, float x_dot, float theta, float theta_dot) {
    // u = -kx
    float output = -(k1*(x-theta)+k2*(x_dot-theta_dot)+k3*theta+k4*theta_dot);
    if (output > maxOut) output = maxOut;
    else if (output < minOut) output = minOut;
    return output;
}

void lqr::setK1(float _k1) {k1 = _k1;}
void lqr::setK2(float _k2) {k2 = _k2;}
void lqr::setK3(float _k3) {k3 = _k3;}
void lqr::setK4(float _k4) {k4 = _k4;}