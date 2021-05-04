// Low Level
#include "RobotUtil.h"
#include "onboardLED.h"
// High Level
#include "HighLevel/Odometry.h"
#include "HighLevel/MotorPID.h"
#include "HighLevel/RobotDriver.h"
// Peripherals
#include "Peripherals/Sonar.h"
#include "Peripherals/Magnetometer.h"
// State Machine Definitions
#include "StateMachine/StateMachineFunctions.h"

void calibrate(){
    redLED();
    // Used to tell when a full rotation has been completed
    static float last_theta;

    switch(rCalEvent){
    case CAL_INIT:
        // Start spinning the robot at 1 rad/s
        Driver_SetVelocity(0, 1);

        // Start recording angles
        last_theta = 0;
        Odom_Update(0, 0, 0);

        // Record that everything is normal
        rCalEvent = CAL_NOTHING;
        break;

    case CAL_NOTHING:
        // Check to see if we are done spinning
        if (Odom_Theta() - last_theta < 0){
            rCalEvent = CAL_FINISHED;
            PID_setPoint(0,0);
        }
        last_theta = Odom_Theta();
        Mag_Read();
        Mag_UpdateOffset();
        break;
    }
}

void orientate(){
    blueLED();
    static bool init = true;
    static float min_dist   = 1.0f * 1e6;
    static float min_angle  = 0;
    static float last_theta = 0;

    // First time entering function
    if (init){
        init = false;
        Odom_Update(0, 0, 0);
        rCalEvent = CAL_NOTHING;

        // Rotate the robot at 1 rad/s while orienting
        Driver_SetVelocity(0, 1);
        return;
    }

    // If we found a new minimum distance, record it and its angle
    float new_angle = Mag_GetAngle();
    float new_dist = Sonar_Read(0);
    if (new_dist < min_dist){
        min_dist = new_dist;
        min_angle = new_angle;
    }

    // If we've done a whole rotation, move on
    float theta = Odom_Theta();
//    printf("New angle: %.2f\t Last angle: %.2f\n", theta, last_theta);
    if (theta < last_theta){
        // Set south as angle of zero
        Mag_SetOffset(Util_Angle(min_angle));
        rCalEvent = CAL_FINISHED;
    }

    // Record the last angle
    last_theta = theta;
}
