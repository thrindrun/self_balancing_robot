#ifndef SMC_H
#define SMC_H

class smc {
    private:
        float k1,k2,lambda1,lambda2,eta,phi;
        float minOut, maxOut;
        float saturation(float s, float phi);

    public:
        smc(float _k1, float _k2, float _lambda1, float _lambda2, float _eta, float _phi, float _min, float _max);
        float compute(float x, float x_dot, float theta, float theta_dot);
        void setK1(float val);
        void setK2(float val);
        void setEta(float val);
        void setPhi(float val);
        void setLambda1(float val);
        void setLambda2(float val);
};

#endif