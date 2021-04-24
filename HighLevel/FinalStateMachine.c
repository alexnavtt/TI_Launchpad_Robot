#include "msp.h"
#include "Motor.h"
#include "BumpInt.h"
#include "RobotUtil.h"

#include "LowLevel/T32.h"
#include "HighLevel/MotorPID.h"
#include "HighLevel/Odometry.h"
#include "HighLevel/RobotDriver.h"
#include "HighLevel/RobotKinematics.h"
#include "HighLevel/FinalStateMachine.h"

#include "Peripherals/Sonar.h"
#include "Peripherals/Magnetometer.h"

void dummy(){}

enum StateMachineStage{
    CALIBRATION,
    NAVIGATION,
    SHOOTING
};

enum CalibrationState{
    INIT,
    REVERSE,
    CALIBRATE,
    ORIENTATE,
    COMPLETE
};

enum CalibrationEvent{
    CAL_NOTHING,
    CAL_BUMP,
    CAL_FINISHED,
    CAL_INIT
};

enum NavigationState{
    LOST,
    SEARCHING,
    HOMING,
    ARRIVED,
    NAV_REVERSE
};

enum NavigationEvent{
    NAV_NOTHING,
    NAV_BUMP,
    NAV_IN_RANGE,
    NAV_LOCATED
};

enum ShootingState{
    AIMING,
    FIRING,
    RELOADING,
    VERIFYING
};

enum ShootingEvent{
    SHOOT_NOTHING,
    SHOOTING_FIRE,
    SHOOTING_ALIGNED,
    SHOOT_BUMP
};

// Forward declarations
const RobotState CalibrationStateMachine[];
const RobotState NavigationStateMachine[];
const RobotState ShootingStateMachine[];

static enum StateMachineStage rStage;
static enum CalibrationEvent rCalEvent;
static enum NavigationEvent  rNavEvent;
static enum ShootingEvent  rShootEvent;
static const RobotState* rState;
static bool is_bumped = false;

/* ================================================== */
/* ----- These are functions used at all stages ----- */
/* ================================================== */

static void bump(){
    Motor_Disable();
    rCalEvent   = CAL_BUMP;
    rNavEvent   = NAV_BUMP;
    rShootEvent = SHOOT_BUMP;
    is_bumped = true;
}

static void reverse(){
    static float start_time = 0;

    // Drive backwards and erase the bump state
    PID_setPoint(-3,-3);
    Motor_Enable();
    rCalEvent   = CAL_NOTHING;
    rNavEvent   = NAV_NOTHING;
    rShootEvent = SHOOT_NOTHING;

    if (is_bumped){
        // Record bump time
        start_time = T32_Now();
        is_bumped = false;
    }else if (T32_Now() - start_time > 1.0){
        // If sufficient time has passed, set things back to normal
        PID_setPoint(0, 0);
        switch(rStage){
        case CALIBRATION:
            rState    = &CalibrationStateMachine[INIT];
            rCalEvent = CAL_NOTHING;
            break;
        case NAVIGATION:
            rState    = &NavigationStateMachine[LOST];
            rNavEvent = NAV_NOTHING;
            break;
        case SHOOTING:
            // Since shooting was interrupted, we go back to navigation
            rStage    = NAVIGATION;
            rState    = &NavigationStateMachine[HOMING];
            rNavEvent = NAV_NOTHING;
        }
    }

}

static void advance(){
    switch (rStage){
    case CALIBRATION:
        rStage = NAVIGATION;
        break;
    case NAVIGATION:
        rStage = SHOOTING;
        break;
    case SHOOTING:
        rStage = NAVIGATION;
        break;
    }
}

/* ============================================================= */
/* ----- These are functions used in the calibration stage ----- */
/* ============================================================= */

static void calibrate(){
    // Used to tell when a full rotation has been completed
    static float last_theta;

    switch(rCalEvent){
    case CAL_INIT:
        // Start spinning the robot
        ;float leftVel, rightVel;
        inverseKinematics(0, 1, &leftVel, &rightVel);
        PID_setPoint(leftVel, rightVel);

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
        break;
    }
}

static void orientate(){
    static bool init = true;
    static float min_dist = (float)1e6;
    static float min_angle = 0;
    static float start_angle = 0;
    static float last_angle  = 0;

    // First time entering function
    if (init){
        init = false;
        start_angle = Mag_GetAngle();

        // Rotate the robot while orienting
        float leftVel, rightVel;
        inverseKinematics(0, 1, &leftVel, &rightVel);
        PID_setPoint(leftVel, rightVel);
    }

    // If we found a new minimum distance, record it and its angle
    float new_angle = Mag_GetAngle();
    float new_dist = Sonar_Read(0);
    if (new_dist < min_dist){
        min_dist = new_dist;
        min_angle = new_angle;
    }

    // If we've done a whole rotation, move on
    if (Util_Angle(new_angle - start_angle) < Util_Angle(start_angle - last_angle)){
        Mag_SetOffset(Util_Angle(min_angle + M_PI));
        rCalEvent = CAL_FINISHED;
    }

    // Record the last angle
    last_angle = new_angle;
}

// Events are NOTHING, BUMP, FINISHED, INIT
const RobotState CalibrationStateMachine[] = {
    {INIT,      {CALIBRATE, REVERSE,  CALIBRATE, INIT }, &calibrate},
    {REVERSE,   {REVERSE,   REVERSE,  INIT,      INIT }, &reverse},
    {CALIBRATE, {CALIBRATE, REVERSE,  ORIENTATE, INIT }, &calibrate},
    {ORIENTATE, {ORIENTATE, REVERSE,  COMPLETE,  INIT }, &orientate},
    {COMPLETE,  {COMPLETE,  COMPLETE, COMPLETE,  INIT }, &advance}
};

/* ============================================================ */
/* ----- These are functions used in the navigation stage ----- */
/* ============================================================ */

static void getUnlost(){
    // Align ourselves with the play area
    if (Driver_goToAngle(0)){
        float x = Sonar_Read(1);
        float y = Sonar_Read(0);
        Odom_Update(x, y, Mag_GetAngle());
    }

}

// Events are NOTHING, BUMP, IN_RANGE, LOCATED
const RobotState NavigationStateMachine[] = {
    {LOST,         {LOST,        NAV_REVERSE, LOST,        HOMING,    }, &getUnlost},
    {SEARCHING,    {SEARCHING,   NAV_REVERSE, SEARCHING,   HOMING,    }, &dummy},
    {HOMING,       {HOMING,      NAV_REVERSE, HOMING,      HOMING,    }, &dummy},
    {ARRIVED,      {ARRIVED,     NAV_REVERSE, ARRIVED,     ARRIVED,   }, &advance},
    {NAV_REVERSE,  {NAV_REVERSE, NAV_REVERSE, NAV_REVERSE, NAV_REVERSE}, &dummy}
};

/* ----- These are functions used in the shooting stage ----- */

// Events are NOTHING, FIRE, ALIGNED, BUMP
const RobotState ShootingStateMachine[] = {
    {AIMING,    {AIMING, RELOADING, FIRING, AIMING}, &dummy},
    {FIRING,    {FIRING, VERIFYING, FIRING, AIMING}, &dummy},
    {RELOADING, {RELOADING, RELOADING, RELOADING, RELOADING}, &dummy},
    {VERIFYING, {VERIFYING, RELOADING, VERIFYING, VERIFYING}, &dummy}
};

/* ======================================================================== */
/* ----- These are the functions available in other translation units ----- */
/* ======================================================================== */

void State_Init(){
    BumpInt_Init(&bump);
    rStage = CALIBRATION;
    rCalEvent   = CAL_INIT;
    rNavEvent   = NAV_NOTHING;
    rShootEvent = SHOOT_NOTHING;
}

void State_Next(){
    switch(rStage){
    case CALIBRATION:
        rState = &CalibrationStateMachine[rState->next_state_by_event[rCalEvent]];
        break;
    case NAVIGATION:
        rState = &NavigationStateMachine[rState->next_state_by_event[rNavEvent]];
        break;
    case SHOOTING:
        rState = &ShootingStateMachine[rState->next_state_by_event[rShootEvent]];
        break;
    }

    rState->action();
}

