#ifndef SMC_H
#define SMC_H

class smc {
    private:
        float k1,k2,k3,k4,lambda1,lambda2;
        float minOut, maxOut;

    public:
        smc(float _k1, float _k2, float _k3, float _k4, float _lambda1, float _lambda2, float _min, float _max);
        float compute(float x, float x_dot, float theta, float theta_dot);
        void setK1(float val);
        void setK2(float val);
        void setK3(float val);
        void setK4(float val);
        void setLambda1(float val);
        void setLambda2(float val);
};

#endif