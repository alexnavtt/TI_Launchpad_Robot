#include <stdio.h>

#include "msp.h"
#include "Motor.h"
#include "BumpInt.h"
#include "RobotUtil.h"
#include "onboardLED.h"

#include "LowLevel/T32.h"
#include "HighLevel/MotorPID.h"
#include "HighLevel/Odometry.h"
#include "HighLevel/RobotDriver.h"
#include "HighLevel/RobotKinematics.h"

#include "Peripherals/Sonar.h"
#include "Peripherals/Magnetometer.h"

#include "StateMachine/FinalStateMachine.h"
#include "StateMachine/StateMachineFunctions.h"

// Debug
void dummy(){
    PID_setPoint(0, 0);
    whiteLED();
}

enum StateMachineStage{
    CALIBRATION,
    NAVIGATION,
    SHOOTING
};

// Forward declarations
const RobotState CalibrationStateMachine[];
const RobotState NavigationStateMachine[];
const RobotState ShootingStateMachine[];

static enum StateMachineStage rStage;
enum CalibrationEvent rCalEvent = CAL_INIT;
enum NavigationEvent  rNavEvent = NAV_NOTHING;
enum ShootingEvent  rShootEvent = SHOOT_NOTHING;
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
            break;
        case NAVIGATION:
            rState    = &NavigationStateMachine[LOST];
            break;
        case SHOOTING:
            // Since shooting was interrupted, we go back to navigation
            rStage    = NAVIGATION;
            rState    = &NavigationStateMachine[HOMING];
        }
    }

}

// Advance to the next stage of the plan
static void advance(){
    yellowLED();
    Driver_GoToAngle(M_PI);
    return;

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

// CALIBRATION
// Function definitions found in CalibrationFunctions.c
// Events are NOTHING, BUMP, FINISHED, INIT
const RobotState CalibrationStateMachine[] = {
    {INIT,      {CALIBRATE, REVERSE,  DUMMY_CAL, INIT }, &calibrate},
    {REVERSE,   {REVERSE,   REVERSE,  INIT,      INIT }, &reverse},
    {CALIBRATE, {CALIBRATE, REVERSE,  ORIENTATE, INIT }, &calibrate},
    {ORIENTATE, {ORIENTATE, REVERSE,  COMPLETE,  INIT }, &orientate},
    {COMPLETE,  {COMPLETE,  COMPLETE, COMPLETE,  INIT }, &advance}
};


// NAVIGATION
// Function definitions found in NavigationFunctions.c
// Events are NOTHING, BUMP, IN_RANGE, ERROR
const RobotState NavigationStateMachine[] = {
    {LOST,         {LOST,        NAV_REVERSE, APPROACHING, LOST       }, &getUnlost},
    {APPROACHING,  {APPROACHING, NAV_REVERSE, SEARCHING,   LOST       }, &approachBackboard},
    {SEARCHING,    {SEARCHING,   NAV_REVERSE, HOMING,      LOST,      }, &dummy},
    {CHECKING,     {CHECKING,    NAV_REVERSE, CHECKING,    SEARCHING  }, &dummy},
    {HOMING,       {HOMING,      NAV_REVERSE, HOMING,      HOMING,    }, &dummy},
    {ARRIVED,      {ARRIVED,     NAV_REVERSE, ARRIVED,     ARRIVED,   }, &advance},
    {NAV_REVERSE,  {NAV_REVERSE, NAV_REVERSE, NAV_REVERSE, NAV_REVERSE}, &dummy}
};


// SHOOTING
// Function definitions found in ShootingFunctions.c
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
    rState = &CalibrationStateMachine[INIT];
}

void State_Next(){
    uint8_t last_state = rState->state;

    switch(rStage){
    case CALIBRATION:
        rState = &CalibrationStateMachine[rState->next_state_by_event[rCalEvent]];
        if (rState->state != last_state) rCalEvent = CAL_NOTHING;
        break;
    case NAVIGATION:
        rState = &NavigationStateMachine[rState->next_state_by_event[rNavEvent]];
        if (rState->state != last_state) rNavEvent = NAV_NOTHING;
        break;
    case SHOOTING:
        rState = &ShootingStateMachine[rState->next_state_by_event[rShootEvent]];
        if (rState->state != last_state) rShootEvent = SHOOT_NOTHING;
        break;
    }

//    static const char* modes[] = {"CAL", "NAV", "SHOOT"};
//    static const char* states[] = {"INIT", "REVERSE", "CALIBRATE", "ORIENTATE", "COMPLETE"};
//    printf("Stage: %s\tState: %s\n", modes[rStage], states[rState->state]);
    rState->action();
}

