#include "msp.h"
#include "SysTick.h"
#include "Peripherals/Launcher.h"

// ----- Connections -----
//
// ENA to P2.4 (Dark Orange)
// IN1 to P7.3 (Green)
// IN2 to P6.3 (Gray)
// IN3 to P7.2 (Yellow)
// IN4 to P7.1 (White/Gray)
// ENB to P2.5 (Light Orange)
// Motor Power (Red) to L298N Motor Power 1
// Motor Power (Black) to L298N Motor Power 2
// L298N GND to Launchpad GND
// L298N GND to Battery GND
// L298N +12V to Battery +9V

#define LEFT_PWM_PIN   0x20
#define LEFT_PWM_INDEX 2

#define RIGHT_PWM_PIN   0x10
#define RIGHT_PWM_INDEX 1

void Launcher_Init(){
    // Configure left PWM (assumes Motor_Init has been called)
    P2->SEL0 |=  LEFT_PWM_PIN;                  // Timer0A functions
    P2->SEL1 &= ~LEFT_PWM_PIN;                  // Timer0A functions
    P2->DIR  |=  LEFT_PWM_PIN;                  // Configure as Timer output module
    TIMER_A0->CCTL[LEFT_PWM_INDEX] = 0x0040;    // Toggle/Reset mode
    TIMER_A0->CCR[LEFT_PWM_INDEX]  = 0;         // set duty cycle to 0 on startup

    // Configure right PWM
    P2->SEL0 |=  RIGHT_PWM_PIN;                 // Timer0A functions
    P2->SEL1 &= ~RIGHT_PWM_PIN;                 // Timer0A functions
    P2->DIR  |=  RIGHT_PWM_PIN;                 // Configure as Timer output module
    TIMER_A0->CCTL[RIGHT_PWM_INDEX] = 0x0040;   // Toggle/Reset mode
    TIMER_A0->CCR[RIGHT_PWM_INDEX]  = 0;        // set duty cycle to 0 on startup

    // Configure GPIO pins
    P6->SELC &= ~0x08;
    P6->DIR  |=  0x08;
    P6->OUT  &= ~0x08;

    P7->SELC &= ~0x0E;
    P7->DIR  |=  0x0E;
    P7->OUT  &= ~0x0E;
}

static void leftMotorOn(uint8_t duty){
    P7->OUT = (P7->OUT & ~0x02) | 0x04;
    TIMER_A0->CCR[LEFT_PWM_INDEX] = (duty/100.0f) * TIMER_A0->CCR[0];
}

static void rightMotorOn(uint8_t duty){
    P6->OUT |=  0x08;
    P7->OUT &= ~0x08;
    TIMER_A0->CCR[RIGHT_PWM_INDEX] = (duty/100.0f) * TIMER_A0->CCR[0];
}

void Launcher_Off(){
    P7->OUT &= ~0x0E;
    P6->OUT &= ~0X08;
}

void Launcher_Fire(){
    leftMotorOn(100);
    rightMotorOn(100);

    SysTick_Wait1ms(2000);

    Launcher_Off();
}
