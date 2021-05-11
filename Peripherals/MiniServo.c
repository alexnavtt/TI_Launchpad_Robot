#include "msp.h"

#define MIN_TICK_COUNT 750
#define MAX_TICK_COUNT 2250
#define TICKS_PER_MICROSECOND 2

void Servo_Init(){
    /* Configure TimerA1 as PWM */

    // Halt the timer
    TIMER_A1->CTL &= ~0x0002;

    // Clear TimerA1 settings
    TIMER_A1->CTL |= 0x0004;

    // Configure the clock on TimerA0
    // Bits 15-10  0 : Reserved
    // Bits 9-8 = 10 : Set source clock to SMCLK (System Module Clock). This is 12MHz if we use Clock_Init48MHz()
    // Bits 7-6 = 00 : Divide input clock into 1 (i.e. no change)
    // Bits 5-4 = 11 : Set to up-down mode (clock will count up to TimerA0->CCR[0] and then back down to zero on a loop)
    // Bit  3   =  0 : Reserved
    // Bit  2   =  0 : Set this bit to clear all settings
    // Bit  1   =  0 : TimerA interrupt disabled
    // Bit  0   =  0 : TimerA interrupt flag
    TIMER_A1->CTL |= 0x0230;

    // Divide by 6 (Now 12/6 = 2MHz => 0.5us per tick)
    TIMER_A1->EX0 = 0x05;

    // Period of 2*20,000*0.5us = 20ms (the required period for the Servo according to the datasheet)
    TIMER_A1->CCR[0] = 20000;

    // Set up Pin 7.7 as PWM output
    P7->SEL0 |=  0x80;
    P7->SEL1 &= ~0x80;
    P7->DIR  |=  0x80;
    TIMER_A1->CCTL[1] = 0x0040;             // Toggle/Reset mode
    TIMER_A1->CCR[1]  = MIN_TICK_COUNT;     // Set servo to minimum angle

    // Enable TimerA1
    TIMER_A1->CTL |= 0x0002;
}

void Servo_SetToAngle(int angle){
    uint32_t required_micros = (angle/180.0f) * (MAX_TICK_COUNT - MIN_TICK_COUNT) + MIN_TICK_COUNT;
    uint16_t required_ticks = required_micros * TICKS_PER_MICROSECOND;
    TIMER_A1->CCR[1] = required_ticks;
}


void Servo_FullCW(){
    TIMER_A1->CCR[1] = MIN_TICK_COUNT;
}


void Servo_FullACW(){
    TIMER_A1->CCR[1] = MAX_TICK_COUNT;
}

void Servo_Center(){
    TIMER_A1->CCR[1] = (MAX_TICK_COUNT + MIN_TICK_COUNT)/2;
}


