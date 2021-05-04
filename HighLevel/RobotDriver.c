#include "msp.h"
#include <math.h>
#include <stdio.h>
#include "RobotUtil.h"

#include "HighLevel/MotorPID.h"
#include "HighLevel/Odometry.h"
#include "HighLevel/RobotDriver.h"
#include "HighLevel/RobotKinematics.h"

#include "Peripherals/Magnetometer.h"

#define PI_OVER_12 0.2617993878
#define RAD2DEG    57.2958
#define DEG2RAD    0.0174533

bool Driver_GoToAngle(float setpt){
    // Get the current robot angle
    float theta = Mag_GetAngle();
//    float theta = Odom_Theta();

    // Calculate the mininum angle error
    float error = Util_AngleDiff(setpt, theta);

    // Debug
//    printf("Theta: %.2f, Setpt: %.2f, Error: %.2f\n", theta, setpt, error);
//    PID_setPoint(0,0);
//    return false;

    // If error is small enough, we have arrived
    if (fabs(error) < 0.05) {
        PID_setPoint(0,0);
        return true;
    }

    // Get the control action with P control
    float desired_omega = error * 3;
    if      (desired_omega > 1)  desired_omega = 1;
    else if (desired_omega < -1) desired_omega = -1;

    // Calculate what each motor should do
    Driver_SetVelocity(0, desired_omega);

    return false;
}

// NEEDS DEBUGGING. SOMETHING'S WRONG
bool Driver_GoTo(float x, float y){
    // Get the current location of the robot
    float x_error = x - Odom_X();   // mm
    float y_error = y - Odom_Y();   // mm
    float error_sq_norm = x_error*x_error + y_error*y_error;

    // Find the angle between the start and end points
    float theta_desired = atan2(y_error, x_error);
    float theta_error = Util_AngleDiff(Mag_GetAngle(), theta_desired);

    // --- Control the robot --- //

    // Threshold for considering robot as arrived
    if (error_sq_norm < 2500){  // 50x50 mm^2
        PID_setPoint(0, 0);
        return true;
    }

    // Go to the desired angle first
    else if (fabs(theta_error) > PI_OVER_12){
        Driver_GoToAngle(theta_desired);
    }

    // Then drive towards the point once the angle is right
    else{
        float desired_omega = theta_error;
        float desired_vel   = error_sq_norm * 0.25;
        float leftVel, rightVel;
        inverseKinematics(desired_vel, desired_omega, &leftVel, &rightVel);
        PID_setPoint(leftVel, rightVel);
    }

    return false;
}

void Driver_SetVelocity(float v, float omega){
    float leftVel, rightVel;
    inverseKinematics(v, omega, &leftVel, &rightVel);
    PID_setPoint(leftVel, rightVel);
}

