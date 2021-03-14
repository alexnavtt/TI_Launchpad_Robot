// Standard includes
#include <stdio.h>

// Board includes
#include "msp.h"
#include "Tachometer.h"

// Project includes
#include <LowLevel/T32.h>
#include <HighLevel/Encoders.h>

// Constants
#define PI 3.14159265358979
static const uint32_t microRadPerTick = 1e6*PI/180;   // micro radian change of an encoder tick (1 degree per tick)

// Return values
static volatile int32_t leftVel, rightVel;   // rotation speeds of each wheel (micro_rad/s)

// Other
static volatile int32_t leftSteps1, rightSteps1;             // encoder tick count at previous time step
static volatile int32_t leftSteps2, rightSteps2;             // encoder tick count at current time step
static volatile uint32_t lastTime, thisTime;                 // time stamp of previous and current time steps
static volatile uint16_t leftTachPeriod, rightTachPeriod;    // tachometer interval between consecutive ticks
static volatile enum TachDirection leftDir, rightDir;        // tachometer wheel directions

// Read the tachometer and update the wheel rotation speeds
void Encoder_Update(){
    lastTime = thisTime;
    thisTime = T32_Count();
    uint32_t one_over_dt = T32_Freq()/(thisTime - lastTime);  // i.e. 1/(timer_period * timer_count) (Unit: 1/s)

    // Get the tick count recorded by the tachometer
    leftSteps2  = leftSteps1;
    rightSteps2 = rightSteps1;
    Tachometer_Get(&leftTachPeriod, &leftDir, &leftSteps1, &rightTachPeriod, &rightDir, &rightSteps1);

    // Calculate the rotational speeds of each wheel
    int32_t leftTicks  = leftSteps1 - leftSteps2;
    int32_t rightTicks = rightSteps1 - rightSteps2;
    leftVel  = microRadPerTick * leftTicks  * one_over_dt;     // micro_rad/s
    rightVel = microRadPerTick * rightTicks * one_over_dt;     // micro_rad/s
}

// Initialize the tachometer
void Encoder_Init(){
    Tachometer_Init();
}

// Retrieve the angular velocity of the wheels (micro radians per second)
void Encoder_WheelAngularVel(uint32_t* left_wheel_angular_vel, uint32_t* right_wheel_angular_vel){
    *right_wheel_angular_vel = rightVel;
    *left_wheel_angular_vel  = leftVel;
}

// Retrieve floating point rad/s measurements
float Encoder_leftWheelVel() {return  leftVel*1.0e-6;}
float Encoder_rightWheelVel(){return rightVel*1.0e-6;}

void Encoder_Show(){
    printf("\tLeft: %.2f rad/s\t\tRight: %.2f rad/s\n", Encoder_leftWheelVel(), Encoder_rightWheelVel());
}
