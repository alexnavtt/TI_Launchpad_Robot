#include "msp.h"
#include <math.h>

#include "HighLevel/MotorPID.h"
#include "HighLevel/Odometry.h"
#include "HighLevel/RobotDriver.h"
#include "HighLevel/RobotKinematics.h"

#include "Peripherals/Magnetometer.h"

#define PI_OVER_12 0.2617993878
#define RAD2DEG    57.2958
#define DEG2RAD    0.0174533

static inline int16_t modulo(int16_t a, int16_t b){
    return a - (a/b) * b;
}

static float angleDiff(float theta_now, float theta_desired){
    int16_t error = RAD2DEG*(theta_desired - theta_now);
    error = modulo(error + 180, 360) - 180;
    return DEG2RAD*error;
}

bool Driver_GoToAngle(float angle){
    // Get the current robot angle
    float theta = Mag_GetAngle();

    // Calculate the mininum angle error
    float error = angleDiff(theta, angle);

    // Get the control action with P control
    float desired_omega = error * 2;
    if      (desired_omega > 1)  desired_omega = 1;
    else if (desired_omega < -1) desired_omega = -1;
    if (fabs(error) < 0.05) {
        PID_setPoint(0,0);
        return true;
    }

    // Calculate what each motor should do
    float leftVel, rightVel;
    inverseKinematics(0, desired_omega, &leftVel, &rightVel);

    // Send to the Motor PID controller
    PID_setPoint(leftVel, rightVel);

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
    float theta_error = angleDiff(Mag_GetAngle(), theta_desired);

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
}
