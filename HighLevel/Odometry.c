#define ARM_MATH_CM4

#include "msp.h"
#include "stdio.h"
#include "LowLevel/T32.h"
#include "HighLevel/Odometry.h"
#include "third_party/CMSIS/Include/arm_math.h"

#define _2PI 6.2831853

static float x_;
static float y_;
static float theta_;

// Update the robot location by 1 time step
void Odom_Step(float vx, float omega){
    static float t_last = 0;

    // Measure time interval
    float t_now = T32_Now();
    float dt = t_now - t_last;

    // Update robot position
    x_ += vx*dt*arm_cos_f32(theta_);
    y_ += vx*dt*arm_sin_f32(theta_);
    theta_ += omega*dt;

    // Constraint theta
    while (theta_ > PI){
        theta_ -= _2PI;
    }

    while (theta_ < -PI){
        theta_ += _2PI;
    }

    // Update time
    t_last = t_now;
}

// Initialize the static variable inside Odom_Step
void Odom_Init(){
    Odom_Step(0, 0);
}

// Create a new estimate of the robot location
void Odom_Update(float x, float y, float theta){
    x_ = x;
    y_ = y;
    theta_ = theta;
    Odom_Step(0, 0);
}

// Retrieve the robot location in the odom frame
void Odom_Get(float* x, float* y, float* theta){
    *x = x_;
    *y = y_;
    *theta = theta_;
}

void Odom_Show(){
    printf("x: %.2f, y: %.2f, theta: %.2f\n", x_, y_, theta_);
}
