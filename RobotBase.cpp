
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

// Project Specific Includes
#include "LowLevel/T32.h"
#include "LowLevel/myI2C.h"
#include "LowLevel/Timer.h"
#include "HighLevel/Encoders.h"
#include "HighLevel/MotorPID.h"
#include "HighLevel/Odometry.h"
#include "HighLevel/RobotKinematics.h"
#include "HighLevel/FinalStateMachine.h"

// Peripheral Sensor Includes
#include "Peripherals/Sonar.h"
#include "Peripherals/Launcher.h"
#include "Peripherals/MiniServo.h"
#include "Peripherals/Magnetometer.h"
}

// C++ includes
#include "HighLevel/KalmanFilter.h"


/* ----- TIMERS -----*/
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
//      4: Launch Motor PWM
//      5: Launch Motor PWM
//      6: Drive Motor Speed
//      7: Drive Motor Speed

// P3 - 6: Drive Motor Enable
//      7: Drive Motor Enable

// P4 - 0: Bumper
//      2: Bumper
//      3: Bumper
//      4: Debug
//      5: Bumper
//      6: Bumper
//      7: Bumper

// P5 - 0: Sonar drive
//      1: Sonar drive
//      4: Drive Motor Direction
//      5: Drive Motor Direction
//      6: Sonar Input Capture
//      7: Sonar Input Capture

// P6 - 3: Launcher Direction

// P7 - 1: Launcher Direction
//      2: Launcher Direction
//      3: Launcher Direction

// P9 - 2: Line Sensor

//P10 - 4: Tachometer
//      5: Tachometer



// Init Debug (Scope P4.4)
void useScope(){
    P4->SELC &= ~0x10;
    P4->DIR  &= ~0x10;
    TExaS_Init(SCOPE);
}

// Init Debug (LogicAnalyzer P4.4)
void useLogic(){
    P4->SELC &= ~0x10;
    P4->DIR  &= ~0x10;
    TExaS_Init(LOGICANALYZER_P4);
}

void configurePID(){
    PID_Init();
    PID_setGains(L_MOTOR, 30, 60, 40);
    PID_setGains(R_MOTOR, 30, 60, 40);
    PID_setMaxVal(L_MOTOR, 300);
    PID_setMaxVal(R_MOTOR, 300);
    PID_setMinVal(L_MOTOR, 50);
    PID_setMinVal(R_MOTOR, 50);
}

void baseRobotInit(){
    SysTick_Init();
    Clock_Init48MHz();              // Init Clock to 48MHz and SMCLK to 12MHz
    pushButtonInit();               // Init onboard buttons with interrupts
    Motor_Init(5000);               // Enable motors with a 5kHz PWM signal
    BumpInt_Init(&Motor_Disable);   // Enable bumpers to stop robot
    Encoder_Init();                 // Enable tachometers
    T32_1_Init();                   // Start 32-bit clock (overflow once every 6 hours)
}

void highLevelInit(){
    Odom_Update(0,0,0);             // Set odom to start at (x, y, theta) = (0, 0, 0)
    configurePID();                 // Set PID gains and limits
}

void peripheralInit(){
    Sonar_Init();                   // Configure the sonar distance sensor
//    Mag_Init();                     // Init magnetometer (slow function)
    Servo_Init();
    Launcher_Init();
}

void toggle(){
    TOGGLE_RED_LED;
}

void softMotorStop(){
    setMotorSpeeds(0,0);
}

// This loop will run every 100ms
void loop10Hz(){
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

int main(void){
    // Initialize
    DisableInterrupts();

    baseRobotInit();
    highLevelInit();
    peripheralInit();
    Clock_Delay1ms(10);

    // Setup
    EnableInterrupts();
    attachToLeftButton(&Servo_FullCW);
    attachToRightButton(&Servo_FullACW);

    // LogicAnalyzer/Scope
//    useLogic();
//    useScope();

    // Testing Components
    SysTick_Wait1ms(2000);
    Launcher_Fire();

    // Timing variables
    float TState, T10, T50 = T10 = TState = T32_Now();
    float delay = 1.0f; // delay time in seconds

    // Loop main
//    State_Init();
    while(true){
        float now = T32_Now();

        // Advance state machine
        if (now - TState > delay){
//            State_Next();
            printf("Dist0: %.2f, Dist1 %.2f\n", Sonar_Read(0), Sonar_Read(1));
            TState = now;
        }

        // 50Hz Loop
        if (now - T50 > 0.02){
            loop50Hz();
            T50 = now;
        }

        // 10Hz Loop
        if (now - T10 > 0.1){
            loop10Hz();
            T10 = now;
        }

//        WaitForInterrupt();
    }
}
