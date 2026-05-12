#include "smc3.h"
#include <Arduino.h>
#include <math.h>

smc::smc(float _k1, float _k2, float _lambda1, float _lambda2, float _eta, float _phi, float _min, float _max) {
    k1 = _k1; k2 = _k2;
    lambda1 = _lambda1; lambda2 = _lambda2;
    eta = _eta; phi = _phi;
    minOut = _min; maxOut = _max;
}

float smc::saturation(float s, float phi) {
    if (s > phi) return 1.0;
    else if (s < -phi) return -1.0;
    return s/phi;
}

float smc::compute(float x, float x_dot, float theta, float theta_dot) {
    float s2 = theta_dot + lambda2*theta;
    float s1 = x_dot + lambda1*x;
    float S = k1*s1 + k2*s2;
    float output = -eta * saturation(S, phi);
    if (output > maxOut) output = maxOut;
    else if (output < minOut) output = minOut;
    return output;
}

void smc::setK1(float val) { k1 = val; }
void smc::setK2(float val) { k2 = val; }
void smc::setEta(float val) { eta = val; }
void smc::setPhi(float val) { phi = val; }
void smc::setLambda1(float val) { lambda1 = val; }
void smc::setLambda2(float val) { lambda2 = val; }

// eta 800
// phi 2-5
