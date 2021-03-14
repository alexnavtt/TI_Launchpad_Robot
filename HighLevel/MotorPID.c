#include "msp.h"
#include <stdio.h>
#include "Motor.h"
#include "LowLevel/T32.h"
#include "HighLevel/Encoders.h"
#include "HighLevel/MotorPID.h"

static uint16_t Kp[2];
static uint16_t Ki[2];
static uint16_t Kff[2];
static float max_val[2];
static float setpt[2];

void PID_Init(){
    // Initialize static variables
    setpt[0] = 0.0f;
    setpt[1] = 0.0f;
    PID_Control(L_MOTOR);
    PID_Control(R_MOTOR);
}

void PID_setGains(enum MotorIndex motor, uint16_t newKp, uint16_t newKi, uint16_t newKff){
    Kp[motor]  = newKp;
    Ki[motor]  = newKi;
    Kff[motor] = newKff;
}

void PID_setMaxVal(enum MotorIndex motor, float val){
    max_val[motor] = val;
}

void PID_setPoint(float left_setpoint, float right_setpoint){
    setpt[0] = left_setpoint;
    setpt[1] = right_setpoint;
}

void PID_Control(enum MotorIndex motor){
    static float sum[2] = {0, 0};
    static float lastTime[2] = {0, 0};

    // Measure the time interval
    float thisTime  = T32_Now();
    float dt        = thisTime - lastTime[motor];
    lastTime[motor] = thisTime;

    // Get the motor speed (rad/s)
    float curr;
    switch(motor){
        case L_MOTOR: curr = Encoder_leftWheelVel(); break;
        case R_MOTOR: curr = Encoder_rightWheelVel(); break;
    }

    // Calculate the error terms
    float error = setpt[motor] - curr;
    sum[motor] += error * dt;

    // Calculate control action for PI control
    float control_action =  Kp[motor]*error + Ki[motor]*sum[motor] + Kff[motor]*setpt[motor];

    // Limit control action
    if (control_action >  max_val[motor]) control_action =  max_val[motor];
    if (control_action < -max_val[motor]) control_action = -max_val[motor];

    switch(motor){
    case L_MOTOR:
        leftMotor((int16_t)control_action);
        break;
    case R_MOTOR:
        rightMotor((int16_t)control_action);
        break;
    }
}

void PID_update(){
    PID_Control(L_MOTOR);
    PID_Control(R_MOTOR);
}
