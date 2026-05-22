#ifndef KALMAN_H
#define KALMAN_H

class Kalman {
public:
    Kalman();
    double getAngle(double newAngle, double newRate, double dt);

    // Setters for tuning parameters
    void setQangle(double newQ_angle);
    void setQbias(double newQ_bias);
    void setRmeasure(double newR_measure);

private:
    // Process noise covariance
    double Q_angle; // Process noise variance for the accelerometer
    double Q_bias;  // Process noise variance for the gyro bias
    double R_measure; // Measurement noise variance - this is actually the variance of the measurement noise

    double angle; // The angle in degrees
    double bias;  // The gyro bias in degrees/sec
    double P[2][2]; // Error covariance matrix

    double K[2]; // Kalman gain
    double y; // Angle difference
    double S; // Estimate error
};

#endif // KALMAN_H
