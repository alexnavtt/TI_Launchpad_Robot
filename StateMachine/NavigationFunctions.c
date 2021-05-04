// Low Level
#include "RobotUtil.h"
#include "onboardLED.h"
// High Level
#include "HighLevel/Odometry.h"
#include "HighLevel/RobotDriver.h"
#include "HighLevel/RobotKinematics.h"
// Peripherals
#include "Peripherals/Sonar.h"
#include "Peripherals/Magnetometer.h"
// State Machine Definitions
#include "StateMachine/StateMachineFunctions.h"

void getUnlost(){
    purpleLED();
    // Align ourselves with the play area
    if (Driver_GoToAngle(M_PI)){
        float theta = Mag_GetAngle();
        float x = (Sonar_Read(1) + 1000*ROBOT_RADIUS_F32) * cos(theta);
        float y = (Sonar_Read(0) + 1000*ROBOT_RADIUS_F32) * cos(theta);
        Odom_Update(x, y, theta);
        rNavEvent = NAV_IN_RANGE;
    }else{
        rNavEvent = NAV_NOTHING;
    }
}

void approachBackboard(){
    if (Driver_GoToAngle(M_PI)){
        // Drive forward at 100 mm/s
        Driver_SetVelocity(100, 0);

        // If we're in range of the backboard, move to the checking phase
        if (Odom_Y() < 200){
            rNavEvent = NAV_IN_RANGE;
        }
    }else{
        rNavEvent = NAV_ERROR;
    }
}

void goToNextBeacon(){
    static unsigned int beacon = 0; // First look for the left beacon
    static bool beacon_checked = false;

    // Facing left
    if (Driver_GoToAngle(3*M_PI_2)){

    }
}
