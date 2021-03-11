#include "HighLevel/KalmanFilter.h"

KalmanFilter::KalmanFilter(){}

void KalmanFilter::init(float x, float y, float theta){
    x_ = x;
    y_ = y;
    theta_ = theta;
}

void KalmanFilter::updateOdometry(float vx, float vy, float omega){
    float d_vx = vx - vx_;
    float d_vy = vy - vy_;
    float d_th = omega - omega_;
}

void KalmanFilter::predict(){

}
