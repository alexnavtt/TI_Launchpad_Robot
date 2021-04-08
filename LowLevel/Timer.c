#include "msp.h"
#include "stdio.h"
#include "onboardLED.h"
#include "LowLevel/Timer.h"

static void dummyFunc(){}
static void (*Timer0Task0)() = &dummyFunc;
static void (*Timer1Task0)() = &dummyFunc;
static void (*Timer2Task0)() = &dummyFunc;
static void (*Timer0TaskN)() = &dummyFunc;
static void (*Timer1TaskN)() = &dummyFunc;
static void (*Timer2TaskN)() = &dummyFunc;
static volatile uint16_t T0_overflow_count;
static volatile uint16_t T1_overflow_count;
static volatile uint16_t T2_overflow_count;
static uint16_t T0_overflow_des;
static uint16_t T1_overflow_des;
static uint16_t T2_overflow_des;

// Calculate the required clock divider
void Timer_getDividersForFrequency(uint32_t freq, uint16_t* period, uint8_t* CTL_mask, uint8_t* EX0_mask){
    // Input clock frequency (SMCLK)
    uint32_t ClockFreq = 12000000;

    uint32_t required_ticks = ClockFreq/freq;           // the number of clock ticks to achieve the desired frequency
    uint8_t dividerCTL_MASK = 0x00;                     // used for writing to TIMER_A->CTL
    uint8_t dividerEX0      = 0x01;                     // divider set in TIMER_A->EX0
    while (required_ticks > 0xFFFF && dividerCTL_MASK != 3){
        required_ticks  = required_ticks >> 1;   // halve ticks
        dividerCTL_MASK++;                       // increment mask
    }
    while (required_ticks > 0xFFFF && dividerEX0 != 8){
        required_ticks = (required_ticks * dividerEX0)/(dividerEX0 + 1);    // increase divider by 1
        dividerEX0 += 1;                                                    // increase divider by 1
    }

    // Set return values
    *period = required_ticks;
    *CTL_mask = (dividerCTL_MASK << 6);
    *EX0_mask = dividerEX0 - 1;
}

void Timer_runAtRate(uint32_t freq, void (*task)(), Timer_A_Type* Timer){
    // TimerA3 is in use and we cannot go below 3Hz
    if (freq < 3 || Timer == TIMER_A3){
        purpleLED(); return;
    }

    // Clear previous timer settings
    Timer->CTL |= 0x0004;

    // Get the dividers for the input timer, and the total tick count
    uint8_t EX0_mask, CTL_mask;
    uint16_t period;
    Timer_getDividersForFrequency(freq, &period, &CTL_mask, &EX0_mask);

    // Configure the clock
    Timer->CTL |= (0x0002            // enable interrupt
               |   0x0010            // up mode
               |   0x0200            // source SMCLK
               |   CTL_mask);        // set divider

    // Configure secondary divider
    Timer->EX0 = EX0_mask;

    // Enable compare interupt when counter reaches Timer->CCR[0]
    Timer->CCTL[0] = 0x0010;

    // Set the interrupt tick count
    Timer->CCR[0] = period;

    // Clear the interrupt flag and assign the task
    Timer->CCTL[0] &= ~0x0001;
    if      (Timer == TIMER_A0){
        Timer0Task0 = task;
        NVIC->ISER[0] |= 0x00000100;                                // enable interrupt
        NVIC->IP[2]    = (NVIC->IP[2] & 0xFFFFFF00) | 0x00000040;   // set priority 2
    }
    else if (Timer == TIMER_A1){
        Timer1Task0 = task;
        NVIC->ISER[0] |= 0x00000400;                                // enable interrupt
        NVIC->IP[2]    = (NVIC->IP[2] & 0xFF00FFFF) | 0x00400000;   // set priority 2
    }
    else if (Timer == TIMER_A2){
        Timer2Task0 = task;
        NVIC->ISER[0] |= 0x00001000;                                // enable interrupt
        NVIC->IP[3]    = (NVIC->IP[3] & 0xFFFFFF00) | 0x00000040;   // set priority 2
    }
}

void Timer_runSecondary(uint16_t divider, void(*task)(), Timer_A_Type* Timer){
    if      (Timer == TIMER_A0){
        Timer0TaskN = task;
        T0_overflow_des = divider - 1;
        NVIC->ISER[0] |= 0x00000200;                                // enable overflow interrupt
        NVIC->IP[2]    = (NVIC->IP[2] & 0xFFFF00FF) | 0x00004000;   // set priority 2
    }
    else if (Timer == TIMER_A1){
        Timer1TaskN = task;
        T1_overflow_des = divider - 1;
        NVIC->ISER[0] |= 0x00000800;                                // enable overflow interrupt
        NVIC->IP[2]    = (NVIC->IP[2] & 0x00FFFFFF) | 0x40000000;   // set priority 2
    }
    else if (Timer == TIMER_A2){
        Timer2TaskN = task;
        T2_overflow_des = divider - 1;
        NVIC->ISER[0] |= 0x00002000;                                // enable overflow interrupt
        NVIC->IP[3]    = (NVIC->IP[3] & 0xFFFF00FF) | 0x00004000;   // set priority 2
    }
}


void TA0_0_IRQHandler(){
    TIMER_A0->CCTL[0] &= ~0x0001;
    Timer0Task0();
}

void TA1_0_IRQHandler(){
    TIMER_A1->CCTL[0] &= ~0x0001;
    Timer1Task0();
}

void TA2_0_IRQHandler(){
    TIMER_A2->CCTL[0] &= ~0x0001;
    Timer2Task0();
}

void TA0_N_IRQHandler(){
    TIMER_A0->IV;
    if (T0_overflow_count != T0_overflow_des){
        T0_overflow_count++;
        return;
    }else{
        T0_overflow_count = 0;
    }
    Timer0TaskN();
}

void TA1_N_IRQHandler(){
    TIMER_A1->IV;
    if (T1_overflow_count != T1_overflow_des){
        T1_overflow_count++;
        return;
    }else{
        T1_overflow_count = 0;
        Timer1TaskN();
    }
}

//void TA2_N_IRQHandler(){
//    TIMER_A2->IV;
//    if (T2_overflow_count != T2_overflow_des){
//        T2_overflow_count++;
//        return;
//    }else{
//        T2_overflow_count = 0;
//    }
//    Timer2TaskN();
//}
