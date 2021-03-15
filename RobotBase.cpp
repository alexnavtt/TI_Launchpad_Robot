// C includes
extern "C"{

// Standard Includes
#include <stdint.h>
#include <stdio.h>

// Board Includes
#include "msp.h"
#include "core_cm4.h"

// Basic Robot Includes
#include "PWM.h"
#include "Clock.h"           // Clock
#include "Motor.h"           // Motors
#include "TExaS.h"           // Debug
#include "CortexM.h"         // Interrupt functions are defined here
#include "BumpInt.h"         // Bumpers with interrupts
#include "pushButton.h"      // Launchpad buttons
#include "onboardLED.h"      // Launchpad LEDs
#include "Reflectance.h"     // Line Sensor

// Project Specific Includes
#include "LowLevel/T32.h"
#include "LowLevel/Timer.h"
#include "HighLevel/Encoders.h"
#include "HighLevel/MotorPID.h"
#include "HighLevel/Odometry.h"
#include "HighLevel/StateMachine.h"
#include "HighLevel/RobotKinematics.h"

}

// C++ includes
#include "HighLevel/KalmanFilter.h"
//#include "ti/devices/msp432p4xx/driverlib/driverlib.h"


/* ----- TIMERS -----*/
//   SysTick is being used for Reflectance readings
//   TimerA0 is being used for motor PWM (channels 3 & 4 output mode)
//   TimerA1 is being used for 10Hz and 50Hz loop
//   TimerA2 is unused
//   TimerA3 is being used for drive encoders (channels 0 & 1 input capture)
//   Timer32 (Module 1) is being used as an all purpose timer for getting current system time
//   Timer32 (Module 2) is unused

/* ----- INTERRUPTS -----*/
//  Bumpers     - NVIC Interrupt 38 (GPIO Port 4) - Priority 1
//  Push Button - NVIC Interrupt 35 (GPIO Port 1) - Priority 3
//  Reflectance - NVIC Interrupt -1 (SysTick)     - Priority 2
//  loop50Hz    - NVIC Interrupt 10 (TimerA1_0)   - Priority 2
//  loop10Hz    - NVIC Interrupt 11 (TimerA1_N)   - Priority 2
//  Encoders    - NVIC Interrupt 14 (TimerA3_0)   - Priority 0
//  Encoders    - NVIC Interrupt 15 (TimerA3_N)   - Priority 0

/* ----- GPIO -----*/
// P1 - 0: LED - Red
//      1: Button
//      4: Button
//      6: Magnetometer
//      7: Magnetometer

// P2 - 0: LED - Multicoloured
//      1: LED - Multicoloured
//      2: LED - Multicoloured
//      6: Drive Motor Speed
//      7: Drive Motor Speed

// P3 - 6: Drive Motor Enable
//      7: Drive Motor Enable

// P4 - 0: Bumper
//      1: Bumper
//      2: Bumper
//      3: Bumper
//      4: Bumper
//      5: Bumper

// P5 - 3: Line Sensor
//      4: Drive Motor Direction
//      5: Drive Motor Direction

// P7 - 0: Line Sensor
//      1: Line Sensor
//      2: Line Sensor
//      3: Line Sensor
//      4: Line Sensor
//      5: Line Sensor
//      6: Line Sensor
//      7: Line Sensor

// P9 - 2: Line Sensor

//P10 - 4: Tachometer
//      5: Tachometer

void configurePID(){
    PID_Init();
    PID_setGains(L_MOTOR, 0, 70, 40);
    PID_setGains(R_MOTOR, 0, 70, 40);
    PID_setMaxVal(L_MOTOR, 300);
    PID_setMaxVal(R_MOTOR, 300);
}

void initAll(){
    Clock_Init48MHz();              // Init Clock to 48MHz and SMCLK to 12MHz
    onboardLEDInit();               // Init onboard LED attached to P2.0-2 and red LED on P1.0
    pushButtonInit();               // Init onboard buttons with interrupts
    Motor_Init(5000);               // Enable motors with a 5kHz PWM signal
    Reflectance_Init();             // Init line sensor
    BumpInt_Init(&Motor_Disable);   // Enable bumpers to stop robot
    Encoder_Init();                 // Enable tachometers
    T32_1_Init();                   // Start 32-bit clock (overflow once every 6 hours)
}

void toggle(){
    TOGGLE_RED_LED;
}

void softMotorStop(){
    setMotorSpeeds(0,0);
}

// This loop will run every 100ms
void loop10Hz(){
    Reflectance_Update();
    Encoder_Update();
    PID_update();
}

// This loopp will run every 20ms
void loop50Hz(){
    static float vx, omega;
    forwardKinematics(Encoder_leftWheelVel(), Encoder_rightWheelVel(), &vx, &omega);
    Odom_Step(vx, omega);
}

void startLoops(){
    Timer_runAtRate(50, &loop50Hz, TIMER_A1);       // Start loop at 50Hz
    Timer_runSecondary(5, &loop10Hz, TIMER_A1);     // Start loop at 1/5 of 50Hz (i.e. 10Hz)
}

void showVel(){
    static float vx;
    static float omega;
    forwardKinematics(Encoder_leftWheelVel(), Encoder_rightWheelVel(), &vx, &omega);
    printf("\tvx: %.2f\t\tomega: %.2f\n", vx, omega);
}

void main(void){
    // Initialize
    DisableInterrupts();
    initAll();
    NVIC_SetPriority(SysTick_IRQn, 2);

    // Setup
    attachToLeftButton(&showVel);
    attachToRightButton(&Odom_Show);
    configurePID();
    EnableInterrupts();
    KalmanFilter Kalman;

    // Loop main
    State_Init();
    startLoops();

    float leftVel, rightVel;
    inverseKinematics(0, 1.571, &leftVel, &rightVel);
    PID_setPoint(leftVel,rightVel);

    float x, y, theta1, theta2;
    while(1){
//        Odom_Get(&x, &y, &theta1);
//        if (theta1 < theta2) TOGGLE_RED_LED;
//        Clock_Delay1ms(10);
//        Odom_Get(&x, &y, &theta2);
//        if (theta2 < theta1) TOGGLE_RED_LED;
//        Clock_Delay1ms(10);
        // Advance state machine
        State_Next();
//        WaitForInterrupt();
    }
}
