
// C includes
extern "C"{

// Standard Includes
#include <stdint.h>
#include <stdio.h>
#include <math.h>

// Board Includes
#include "msp.h"
#include "core_cm4.h"

// Basic Robot Includes
#include "PWM.h"
#include "Clock.h"           // Clock
#include "Motor.h"           // Motors
#include "TExaS.h"           // Debug
#include "SysTick.h"         // SysTick
#include "CortexM.h"         // Interrupt functions are defined here
#include "BumpInt.h"         // Bumpers with interrupts
#include "pushButton.h"      // Launchpad buttons
#include "onboardLED.h"      // Launchpad LEDs
#include "Reflectance.h"     // Line Sensor

// Project Specific Includes
#include "LowLevel/T32.h"
#include "LowLevel/myI2C.h"
#include "LowLevel/Timer.h"
#include "HighLevel/Encoders.h"
#include "HighLevel/MotorPID.h"
#include "HighLevel/Odometry.h"
#include "HighLevel/StateMachine.h"
#include "HighLevel/RobotKinematics.h"

#include "Peripherals/TOF.h"
#include "Peripherals/Sonar.h"
#include "Peripherals/Magnetometer.h"

}

// C++ includes
#include "HighLevel/KalmanFilter.h"


/* ----- TIMERS -----*/
//   SysTick is being used for Reflectance readings
//   TimerA0 is being used for motor PWM (channels 3 & 4 output mode)
//   TimerA1 is being used for 10Hz and 50Hz loop
//   TimerA2 is being used for Sonar distance sensors
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
//      6: Magnetometer I2C
//      7: Magnetometer I2C

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

// P5 - 0: Sonar drive
//      1: Sonar drive
//      3: Line Sensor
//      4: Drive Motor Direction
//      5: Drive Motor Direction
//      6: Sonar Input Capture
//      7: Sonar Input Capture

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
    PID_setGains(L_MOTOR, 30, 60, 40);
    PID_setGains(R_MOTOR, 30, 60, 40);
    PID_setMaxVal(L_MOTOR, 300);
    PID_setMaxVal(R_MOTOR, 300);
    PID_setMinVal(L_MOTOR, 50);
    PID_setMinVal(R_MOTOR, 50);
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
    Sonar_Init();                   // Configure the sonar distance sensor
    Mag_Init();                     // Init magnetometer (slow function)
    Odom_Update(0,0,0);             // Set odom to start at (x, y, theta) = (0, 0, 0)
//    configurePID();                 // Set PID gains and limits
}

void toggle(){
    TOGGLE_RED_LED;
}

void softMotorStop(){
    setMotorSpeeds(0,0);
}

static void printAngle(){
    printf("Mag Angle: %.2f\n", Mag_GetAngle());
}

// This loop will run every 100ms
void loop10Hz(){
    Reflectance_Update();
    Encoder_Update();
    PID_update();
    Sonar_Update();
}

// This loop will run every 20ms
void loop50Hz(){
    static float vx, omega;
    forwardKinematics(Encoder_leftWheelVel(), Encoder_rightWheelVel(), &vx, &omega);
    Odom_Step(vx, omega);
}

void startLoops(){
    Timer_runAtRate(50, &loop50Hz, TIMER_A1);       // Start loop at 50Hz
    Timer_runSecondary(5, &loop10Hz, TIMER_A1);     // Start loop at 1/5 of 50Hz (i.e. 10Hz)
}

void calibrate(){
    // Rotate the robot at 1 rad/s
    float leftVel, rightVel;
    inverseKinematics(0, 1, &leftVel, &rightVel);
    PID_setPoint(leftVel, rightVel);

    // Calibrate the magnetometer
    float last_angle = Odom_Theta();
    while (true){
        Mag_Read();
        SysTick_Wait1ms(20);
        if (Odom_Theta() - last_angle < 0) break;
        last_angle = Odom_Theta();
    }
    PID_setPoint(0,0);
}

void main(void){
    // Initialize
    DisableInterrupts();
    initAll();

    // Setup
    attachToRightButton(&Odom_Show);
    attachToLeftButton(&printAngle);
    Clock_Delay1ms(10);
    EnableInterrupts();

    // Start updating sensors
    startLoops();

    // Calibrate the Magnetometer
    //    calibrate();

    // Loop main
//    State_Init();

    float T1 = T32_Now();
    float delay = 0.1; // delay time in seconds
    while(1){
        // Advance state machine
        if (T32_Now() - T1 > delay){
//            State_Next();
            printf("Dist: %.2f, %.2f\n", Sonar_Read(0), Sonar_Read(1));
            T1 = T32_Now();
        }
    }
}
