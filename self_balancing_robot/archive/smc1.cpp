#include "smc1.h"
#include <Arduino.h>

smc::smc(float _c1, float _c2, float _c3, float _c4, float _eta, float _phi, float _min, float _max) {
    c1 = _c1; c2 = _c2; c3 = _c3; c4 = _c4;
    eta = _eta; phi = _phi;
    minOut = _min; maxOut = _max;
}

float smc::saturation(float s, float phi) {
    if (s > phi) return 1.0;
    else if (s < -phi) return -1.0;
    return s/phi;
}

float smc::compute(float x, float x_dot, float theta, float theta_dot) {
    float s = c1*x+c2*x_dot+c3*theta+c4*theta_dot;
    // u = -eta * sat(s/phi);
    float output = -eta * saturation(s, phi);
    if (output > maxOut) output = maxOut;
    else if (output < minOut) output = minOut;
    return output;
}

void smc::setC1(float val) { c1 = val; }
void smc::setC2(float val) { c2 = val; }
void smc::setC3(float val) { c3 = val; }
void smc::setC4(float val) { c4 = val; }
void smc::setEta(float val) { eta = val; }
void smc::setPhi(float val) { phi = val; }