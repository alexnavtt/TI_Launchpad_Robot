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
#define PI_OVER_6  0.5235987756
#define RAD2DEG    57.2958
#define DEG2RAD    0.0174533

bool Driver_GoToAngle(float setpt){
    // Get the current robot angle
    float theta = Mag_GetAngle();

    // Calculate the mininum angle error
    float error = Util_AngleDiff(setpt, theta);

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

bool Driver_GoToAngleOdom(float setpt){
    // Get the current robot angle
    float theta = Odom_Theta();

    // Calculate the mininum angle error
    float error = Util_AngleDiff(setpt, theta);

    // If error is small enough, we have arrived
    if (fabs(error) < 0.05) {
        PID_setPoint(0,0);
        return true;
    }

    // Get the control action with P control
    float desired_omega = error * 3;
    if      (desired_omega > 1)  desired_omega = 1;
    else if (desired_omega < -1) desired_omega = -1;

    if (fabs(desired_omega) < 0.3){
        desired_omega = 0.3 * Util_Sign(desired_omega);
    }

    // Calculate what each motor should do
    Driver_SetVelocity(0, desired_omega);

    return false;
}


bool Driver_GoTo(float x, float y){
    // Get the current location of the robot
    float x_error = x - Odom_X();   // mm
    float y_error = y - Odom_Y();   // mm
    float error_sq_norm = x_error*x_error + y_error*y_error;

    // Threshold for considering robot as arrived
    if (error_sq_norm < 150*150){  // 15 cm bounding box
        PID_setPoint(0, 0);
        return true;
    }

    // Find the angle between the start and end points
    float theta_desired = atan2(y_error, x_error);
    float theta_error = Util_AngleDiff(theta_desired, Mag_GetAngle());
//    float theta_error = Util_AngleDiff(theta_desired, Odom_Theta());

    // --- Control the robot --- //

    // Go to the desired angle first
    if (fabs(theta_error) > PI_OVER_6){
        Driver_GoToAngle(theta_desired);
//        Driver_GoToAngleOdom(theta_desired);
    }

    // Then drive towards the point once the angle is about right
    else{
//        Odom_Show();
        // Limit the size of the error
        if (error_sq_norm > 1000) error_sq_norm = 1000;
        if (error_sq_norm < 500)  error_sq_norm = 500;

        float desired_omega = theta_error * 1;
        float desired_vel   = error_sq_norm * 0.1;
//        printf("Vel: %.2f, Omega: %.2f\n", desired_vel, desired_omega);
        Driver_SetVelocity(desired_vel, desired_omega);
    }

    return false;
}

void Driver_SetVelocity(float v, float omega){
    float leftVel, rightVel;
    inverseKinematics(v, omega, &leftVel, &rightVel);
    PID_setPoint(leftVel, rightVel);
}

