#include "smc_all.h"
#include <Arduino.h>
#include <math.h>

SMC::SMC(float _min, float _max) {
    minOut = _min; maxOut = _max;
    cls = {0,0,0,0,0,0};
    hyb = {0,0,0,0,0,0};
    hie = {0,0,0,0,0,0};
}

float SMC::saturation(float s, float phi) {
    if (s > phi) return 1.0;
    else if (s < -phi) return -1.0;
    return s/phi;
}

void SMC::setParamsCls(float _c1, float _c2, float _c3, float _c4, float _eta, float _phi) {
    cls.c1 = _c1; cls.c2 = _c2; cls.c3 = _c3; cls.c4 = _c4;
    cls.eta = _eta; cls.phi = _phi;
}

void SMC::setParamsHyb(float _k1, float _k2, float _k3, float _k4, float _lambda1, float _lambda2) {
    hyb.k1 = _k1; hyb.k2 = _k2; hyb.k3 = _k3; hyb.k4 = _k4;
    hyb.lambda1 = _lambda1; hyb.lambda2 = _lambda2;
}

void SMC::setParamsHie(float _k1, float _k2, float _lambda1, float _lambda2, float _eta, float _phi) {
    hie.k1 = _k1; hie.k2 = _k2; hie.lambda1 = _lambda1; hie.lambda2 = _lambda2;
    hie.eta = _eta; hie.phi = _phi;
}

float SMC::computeCls(float x, float x_dot, float theta, float theta_dot) {
    float s = cls.c1*x + cls.c2*x_dot + cls.c3*theta + cls.c4*theta_dot;
    float output = -cls.eta * saturation(s, cls.phi);
    if (output > maxOut) output = maxOut;
    else if (output < minOut) output = minOut;
    return output;
}

float SMC::computeHyb(float x, float x_dot, float theta, float theta_dot) {
    float s1 = x_dot + hyb.lambda1*x;
    float s2 = theta_dot + hyb.lambda2*theta;
    float u = -hyb.k1*s1 - hyb.k2*s2 - hyb.k3*tanh(s1) - hyb.k4*tanh(s2);
    if (u > maxOut) u = maxOut;
    else if (u < minOut) u = minOut;
    return u;
}

float SMC::computeHie(float x, float x_dot, float theta, float theta_dot) {
    float s1 = x_dot + hie.lambda1*x;
    float s2 = theta_dot + hie.lambda2*theta;
    float S = hie.k1*s1 + hie.k2*s2;
    float output = -hie.eta * saturation(S, hie.phi);
    if (output > maxOut) output = maxOut;
    else if (output < minOut) output = minOut;
    return output;
}

void SMC::setClsC1(float val) { cls.c1 = val; }
void SMC::setClsC2(float val) { cls.c2 = val; }
void SMC::setClsC3(float val) { cls.c3 = val; }
void SMC::setClsC4(float val) { cls.c4 = val; }
void SMC::setClsEta(float val) { cls.eta = val; }
void SMC::setClsPhi(float val) { cls.phi = val; }

void SMC::setHybK1(float val) { hyb.k1 = val; }
void SMC::setHybK2(float val) { hyb.k2 = val; }
void SMC::setHybK3(float val) { hyb.k3 = val; }
void SMC::setHybK4(float val) { hyb.k4 = val; }
void SMC::setHybLambda1(float val) { hyb.lambda1 = val; }
void SMC::setHybLambda2(float val) { hyb.lambda2 = val; }

void SMC::setHieK1(float val) { hie.k1 = val; }
void SMC::setHieK2(float val) { hie.k2 = val; }
void SMC::setHieLambda1(float val) { hie.lambda1 = val; }
void SMC::setHieLambda2(float val) { hie.lambda2 = val; }
void SMC::setHieEta(float val) { hie.eta = val; }
void SMC::setHiePhi(float val) { hie.phi = val; }