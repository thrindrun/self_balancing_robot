#ifndef SMC_H
#define SMC_H

#include <Arduino.h>

class SMC {
    private:
        float minOut, maxOut;
        float saturation(float s, float phi);
        struct {float c1,c2,c3,c4,eta,phi;} cls; // classic
        struct {float k1,k2,k3,k4,lambda1,lambda2;} hyb; //hibrid
        struct {float k1,k2,lambda1,lambda2,eta,phi;} hie; //hiearchical

    public:
        SMC (float _min, float _max);
        float computeCls(float x, float x_dot, float theta, float theta_dot);
        void setParamsCls(float _c1, float _c2, float _c3, float _c4, float _eta, float _phi);
        float computeHyb(float x, float x_dot, float theta, float theta_dot);
        void setParamsHyb(float _k1, float _k2, float _k3, float _k4, float _lambda1, float _lambda2);
        float computeHie(float x, float x_dot, float theta, float theta_dot);
        void setParamsHie(float _k1, float _k2, float _lambda1, float _lambda2, float _eta, float _phi);

        void setClsC1(float val);
        void setClsC2(float val);
        void setClsC3(float val);
        void setClsC4(float val);
        void setClsEta(float val);
        void setClsPhi(float val);

        void setHybK1(float val);
        void setHybK2(float val);
        void setHybK3(float val);
        void setHybK4(float val);
        void setHybLambda1(float val);
        void setHybLambda2(float val);

        void setHieK1(float val);
        void setHieK2(float val);
        void setHieLambda1(float val);
        void setHieLambda2(float val);
        void setHieEta(float val);
        void setHiePhi(float val);
};

#endif