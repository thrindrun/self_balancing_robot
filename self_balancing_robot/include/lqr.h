#ifndef LQR_H 
#define LQR_H 

class lqr {
    private:
        float k1,k2,k3,k4;
        float minOut,maxOut;
    
    public:
        lqr(float _k1, float _k2, float _k3, float _k4, float _min, float _max);
        float compute(float x, float x_dot, float theta, float theta_dot);
};

#endif