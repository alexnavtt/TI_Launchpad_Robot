#ifndef ROBOT_TIMER_H
#define ROBOT_TIMER_H

#include "Clock.h"

// Calculate the required clock divider to tun at a certain frequency
void Timer_getDividersForFrequency(
        uint32_t freq,
        uint16_t* period,
        uint8_t* CTL_mask,
        uint8_t* EX0_mask
);

void Timer_runAtRate(uint32_t freq, void (*task)(), Timer_A_Type* Timer);

void Timer_runSecondary(uint16_t divider, void(*task)(), Timer_A_Type* Timer);

#endif
