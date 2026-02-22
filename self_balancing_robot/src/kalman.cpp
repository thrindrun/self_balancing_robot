#include "kalman.h"

Kalman::Kalman() {
    // Initialize tuning parameters
    Q_angle = 0.001;
    Q_bias = 0.003;
    R_measure = 0.03;

    // Initialize state
    angle = 0.0;
    bias = 0.0;

    // Initialize covariance matrix
    P[0][0] = 0.0;
    P[0][1] = 0.0;
    P[1][0] = 0.0;
    P[1][1] = 0.0;
}

// The main filter function
double Kalman::getAngle(double newAngle, double newRate, double dt) {
    // Prediction step
    // 1. Update state estimate
    angle += dt * (newRate - bias);

    // 2. Update state covariance
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // Correction step
    // 1. Calculate innovation
    y = newAngle - angle;

    // 2. Calculate innovation covariance
    S = P[0][0] + R_measure;

    // 3. Calculate Kalman gain
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    // 4. Update state estimate
    angle += K[0] * y;
    bias += K[1] * y;

    // 5. Update state covariance
    double P00_temp = P[0][0];
    double P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    return angle;
}

void Kalman::setQangle(double newQ_angle) {
    Q_angle = newQ_angle;
}

void Kalman::setQbias(double newQ_bias) {
    Q_bias = newQ_bias;
}

void Kalman::setRmeasure(double newR_measure) {
    R_measure = newR_measure;
}
