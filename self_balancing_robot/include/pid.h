#ifndef PID.h
#define PID.h

class pid {
    private:
        float kp,ki,kd;
        float integral,lastError;
        float minOut,maxOut;

    public:
        pid(float p, float i, float d, float minVal, float maxVal); // constructer
        float compute(float setpoint, float measuredValue, float dt); // calculation
        void reset(); // reset the integral
};

#endif