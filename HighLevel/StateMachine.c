#include <stdio.h>
#include <math.h>
#include "msp.h"
#include "Motor.h"
#include "Clock.h"
#include "BumpInt.h"
#include "Reflectance.h"
#include "HighLevel/Odometry.h"
#include "HighLevel/MotorPID.h"
#include "HighLevel/RobotDriver.h"
#include "HighLevel/StateMachine.h"

static const RobotState* rState;
static enum RobotStateEvent rEvent;

#ifdef ROOMBA
void bump(){
    uint8_t reading = ~Bump_Read();
    uint8_t left_bump  = reading & 0b00111000;
    uint8_t right_bump = reading & 0b00000111;
    if (reading == 0) return;
    else if (left_bump && right_bump) rEvent = BUMP_BOTH;
    else if (left_bump)  rEvent = BUMP_LEFT;
    else if (right_bump) rEvent = BUMP_RIGHT;
    if (rState->state == LEFT_TURN || rState->state == RIGHT_TURN){
        Motor_Disable();
    }
}

void driveForward(){
//    setMotorSpeeds(300, 300);
    PID_setPoint(3, 3);
}

void reverse(){
//    setMotorSpeeds(-300, -300);
    PID_setPoint(-3, -3);
    Clock_Delay1ms(1200);
}

void turnRight(){
    rEvent = NOTHING;
//    setMotorSpeeds(300, -300);
    PID_setPoint(3, -3);
    Clock_Delay1ms(1200);
}

void turnLeft(){
    rEvent = NOTHING;
//    setMotorSpeeds(-300, 300);
    PID_setPoint(-3, 3);
    Clock_Delay1ms(1200);
}

void Stop(){
//    setMotorSpeeds(0, 0);
    PID_setPoint(0, 0);
    Clock_Delay1ms(1000);
}

// Next state in the order {NO_BUMP, LEFT_BUMP, RIGHT_BUMP, BOTH_BUMP}
const RobotState StateMachine[] = {
    {SHUTDOWN,   {SHUTDOWN,   SHUTDOWN,   SHUTDOWN,  SHUTDOWN  }, &Motor_Disable},
    {FREE,       {FREE,       HALT,       HALT,      HALT      }, &driveForward},
    {REVERSE,    {RIGHT_TURN, RIGHT_TURN, LEFT_TURN, RIGHT_TURN}, &reverse},
    {RIGHT_TURN, {FREE,       SHUTDOWN,   SHUTDOWN,  SHUTDOWN  }, &turnRight},
    {LEFT_TURN,  {FREE,       SHUTDOWN,   SHUTDOWN,  SHUTDOWN  }, &turnLeft},
    {HALT,       {REVERSE,    REVERSE,    REVERSE,   REVERSE   }, &Stop}
};

void Init_Required(){
    rState = &StateMachine[1];
}

void readState(){}

#elif defined LINE_FOLLOWER

// Bumper callback
void bump(){
    Motor_Stop();
    rEvent = BUMP;
}

void reverse(){
    setMotorSpeeds(-150, -150);
    Clock_Delay1ms(200);
    int32_t pos = Reflectance_GetPos();
    if (pos == 500) return;
    if (pos < 50 && pos > -50){
        rEvent = FOUND_TARGET;
    }else{
        rEvent = FOUND_LINE;
    }
}

void follow(){
    int32_t pos = Reflectance_GetPos();

    if (pos == 500) {
        rEvent = LOST_LINE;
        return;
    }

    if (pos < -200){
        setMotorSpeeds(75, 150);
        rEvent = LOST_TARGET;
    }else
    if (pos < -50){
        setMotorSpeeds(100, 150);
        rEvent = LOST_TARGET;
    }else
    if (pos < 50){
        setMotorSpeeds(150, 150);
        rEvent = FOUND_TARGET;
    }else
    if (pos < 200){
        setMotorSpeeds(150, 100);
        rEvent = LOST_TARGET;
    }else{
        setMotorSpeeds(150, 75);
        rEvent = LOST_TARGET;
    }

    Clock_Delay1ms(50);
}

// State machine definition
const RobotState StateMachine[] = {
    {LOST,       {LOST,       HALTED, LOST,       LOST,   ON_TARGET, OFF_TARGET}, &reverse},
    {ON_TARGET,  {ON_TARGET,  HALTED, OFF_TARGET, LOST,   ON_TARGET, ON_TARGET},  &follow},
    {OFF_TARGET, {OFF_TARGET, HALTED, OFF_TARGET, LOST,   ON_TARGET, OFF_TARGET}, &follow},
    {HALTED,     {HALTED,     HALTED, HALTED,     HALTED, HALTED,    HALTED},     &Motor_Stop}
};

void Init_Required(){
    Reflectance_Init();
    Reflectance_Update();
    rState = &StateMachine[1];
}

void readState(){}

#elif defined SQUARE

//static float x, y, theta, theta_setpoint;
static uint8_t setpoint_index = 0;
static float x_setpoint[4] = {0, 0, 200, 200};
static float y_setpoint[4] = {0, 200, 200, 0};

static void bump(){
    Motor_Disable();
    rEvent = BUMP;
}

static void driveForward(){
//    PID_setPoint(3, 3);
    if (Driver_GoTo(x_setpoint[setpoint_index], y_setpoint[setpoint_index])){
        rEvent = CORNER_REACHED;
    }
}

static void rotate(){
//    PID_setPoint(3, -3);
//    if (Driver_GoToAngle(theta_setpoint)){
        rEvent = ANGLE_REACHED;
//    }
}

static void updateSetpoint(){
    if (rState->state == CORNER_ENTER){
        setpoint_index++;
//        theta_setpoint -= M_PI_2;
//        if (theta_setpoint < -M_PI){
//            theta_setpoint += 2*M_PI;
//        }
    }
}

static void pause(){
    updateSetpoint();
    PID_setPoint(0, 0);
    Clock_Delay1ms(250);
    Odom_Update(0, 0, 0);
}

// NOTHING, BUMP, CORNER_REACHED, ANGLE_REACHED
const RobotState StateMachine[] = {
    {DRIVING,      {DRIVING,      HALTED, CORNER_ENTER, DRIVING    }, &driveForward},
    {ROTATING,     {ROTATING,     HALTED, ROTATING,     CORNER_EXIT}, &rotate},
    {CORNER_ENTER, {ROTATING,     HALTED, ROTATING,     ROTATING   }, &pause},
    {CORNER_EXIT,  {DRIVING,      HALTED, DRIVING,      DRIVING    }, &pause},
    {HALTED,       {HALTED,       HALTED, HALTED,       HALTED     }, &bump}
};

void readState(){
//    Odom_Get(&x, &y, &theta);
}

void Init_Required(){
    Odom_Init();
    Odom_Update(0, 0, 0);
    readState();
//    theta_setpoint = 0;
    rState = &StateMachine[1];  // start off by going to angle 0
    rEvent = NOTHING;
}


#elif defined COMPETITION_BUILD

void bump(){

}

const RobotState StateMachine[] = {

};

#endif

void State_Init(){
    BumpInt_Init(&bump);
    Init_Required();
    rState->action();
}

void State_Next(){
    readState();
//    enum RobotStateIndex prev_state = rState->state;
    rState = &StateMachine[rState->next_state_by_event[rEvent]];
//    if(rState->state != prev_state) rState->action();
    rState->action();
//    Reflectance_Show();
}

