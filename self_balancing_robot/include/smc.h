#ifndef SMC.h
#define SMC.h

class smc {
    private:
        float c1,c2,c3,c4;
        float eta; // reaching gain
        float phi; // chattering fix
        float minOut, maxOut;
        float saturation(float s, float phi);

    public:
        smc(float _c1, float _c2, float _c3, float _c4, float _eta, float _phi, float _min, float _max);
        float compute(float x, float x_dot, float theta, float theta_dot);
};

#endif