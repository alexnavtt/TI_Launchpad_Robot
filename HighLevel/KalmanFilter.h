#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

class KalmanFilter{
public:
    KalmanFilter();
    void init(float x, float y, float theta);
    void updateMagnetometer();
    void updateOdometry(float vx, float vy, float omega);
    void updateDistanceSensor();
    void predict();

private:
    float x_;       // robot position - x
    float y_;       // robot position - y
    float theta_;   // robot orientation
    float vx_;
    float vy_;
    float omega_;
    float dt_;

    float var_x_;
    float var_y_;
    float var_t_;
    float var_vx_;
    float var_vy_;
    float var_omega_;
};

#endif
