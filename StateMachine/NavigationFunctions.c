// Low Level
#include "Motor.h"
#include "RobotUtil.h"
#include "onboardLED.h"
// High Level
#include "HighLevel/Odometry.h"
#include "HighLevel/MotorPID.h"
#include "HighLevel/RobotDriver.h"
#include "HighLevel/RobotKinematics.h"
// Peripherals
#include "Peripherals/Sonar.h"
#include "Peripherals/Magnetometer.h"
// State Machine Definitions
#include "StateMachine/StateMachineFunctions.h"

// Use the sonar sensors and magnetometer to determine where we are on the board
void getUnlost(){
    purpleLED();
    // Align ourselves with the play area
    if (Driver_GoToAngle(M_PI)){
        float theta = Mag_GetAngle();
        float x = (Sonar_Read(0) + 1000*ROBOT_RADIUS_F32) * cos(theta) * -1;
        float y = (Sonar_Read(0) + 1000*ROBOT_RADIUS_F32) * cos(theta) * -1;
        Odom_Update(x, y, theta);
        rNavEvent = NAV_IN_RANGE;
    }else{
        rNavEvent = NAV_NOTHING;
    }
}

void approachBackboard(){
    blueLED();
    // If we're in range of the backboard, move to the checking phase
    if (Driver_GoTo(500, Odom_Y())){
        PID_setPoint(0, 0);
        rNavEvent = NAV_IN_RANGE;
//        Motor_Disable();
    }
}

// State
void goToNextBeacon(){
    yellowLED();
//    static unsigned int beacon = 0;     // First look for the left beacon
//    static bool beacon_checked = false;

    // Facing left
    Driver_GoTo(Odom_X(), 300);
    if (Sonar_Read(0) < 300){
        rNavEvent = NAV_IN_RANGE;
    }
}

void alignWithNet(){
    redLED();
    if (Driver_GoToAngle(M_PI)){
        rNavEvent = NAV_IN_RANGE;
    }
}

