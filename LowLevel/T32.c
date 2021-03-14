#include "msp.h"
#include "LowLevel/T32.h"

#ifdef __cplusplus
extern "C"{
#endif

static float T32_PERIOD;
static uint32_t T32_FREQ;

void T32_1_Init(){
//    TIMER32_1->CONTROL = 0x0000008E;
    TIMER32_1->CONTROL =   (0x00000002    // 32 bit operation
                          | 0x00000004    // divide input by 16 (3 MHz)
                          | 0x00000080);  // enable timer
    T32_PERIOD = 16.0f/48000000;
    T32_FREQ   = 48000000/16;
}

// Get the tick frequency of the Timer
inline uint32_t T32_Freq(){
    return T32_FREQ;
}

// Get the period of a single Timer tick
inline float T32_Period(){
    return T32_PERIOD;
}

// Get the number of ticks since the timer was initialized
inline uint32_t T32_Count(){
    return 0xFFFFFFFF - TIMER32_1->VALUE;
}

// Get the time passed in seconds since the timer was initialized
inline float T32_Now(){
    return T32_Count() * T32_PERIOD;
}

#ifdef __cplusplus
}
#endif
