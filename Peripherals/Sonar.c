#include "msp.h"
#include "SysTick.h"
#include "Peripherals/Sonar.h"

#define DRIVE_PINS 0x03  // Pins 5.0(Sonar2) & 5.1(Sonar1)
#define ECHO_PINS  0xC0  // Pins 5.6(Sonar1) & 5.7(Sonar2)

static float timer_tick = 8*8.333e-2;  // microSeconds per timer tick
#define MICROS(dur) ((dur) * timer_tick)

volatile static bool data_ready[2] = {true, true};
volatile static uint16_t duration[2] = {0, 0};

void Sonar_Init(){
    // Configure drive pin as GPIO
    P5->SELC &= ~DRIVE_PINS; // GPIO
    P5->DIR  |=  DRIVE_PINS; // Out
    P5->OUT  &= ~DRIVE_PINS; // Drive pins off by default

    // Configure trigger pin as input capture
    P5->SEL0 |=  ECHO_PINS;
    P5->SEL1 &= ~ECHO_PINS;
    P5->DIR  &= ~ECHO_PINS;

    // Prepare TimerA2 for configuration
    TIMER_A2->CTL &= ~0x0030;   // Halt the timer
    TIMER_A2->CTL |=  0x0004;   // Clear settings

    // Divide input clock by 8 (prevents overflow error) for long distances
    TIMER_A2->EX0 = 0x07;

    // Configure P5.6 as input capture
    TIMER_A2->CCTL[1] = 0xC910;   // Capture on both edges, CCI1A, interrupts

    // Configure P5.7 as input capture
    TIMER_A2->CCTL[2] = 0xC910;   // Capture on both edges, CCI2A, interrupts

    // Enable interrupt with priority 2
    NVIC->IP[3]    = (NVIC->IP[3] & 0xFFFF00FF) | 0x00004000;
    NVIC->ISER[0] |= 0x00002000;

    // Enable TimerA2 continuous mode
    TIMER_A2->CTL = 0x0220;
}

void Sonar_Update(){
    if (!(data_ready[0] && data_ready[1])) return;    // already in the middle of a measurement

    // Trigger the soundwave
    P5->OUT |=  DRIVE_PINS;
    SysTick_Wait1us(10);
    P5->OUT &= ~DRIVE_PINS;
}

float Sonar_Read(uint8_t index){
    return MICROS(duration[index])/58.0f;
}

void TA2_N_IRQHandler(void){
    static uint16_t start_time[2];

    // Forward delcarations
    uint8_t index;      // which module is triggered
    bool rising;        // rising edge or falling edge
    uint16_t edge_time; // interrupt timestamp

    // Determine which Sonar module is generating the interrupt
    uint8_t interrupt_state = TIMER_A2->IV; // Note: This clears the highest priority interrupt flag
    switch(interrupt_state){
    case 0x02:
    case 0x06:
        // Sonar 1 interrupt
        index = 0;
        rising = TIMER_A2->CCTL[1] & CCI;  // read the capture pin 5.6
        edge_time = TIMER_A2->CCR[1];  // record the timestamp
        break;
    case 0x04:
        // Sonar 2 interrupt
        index = 1;
        rising = TIMER_A2->CCTL[2] & CCI;  // read the capture pin 5.6
        edge_time = TIMER_A2->CCR[2];
        break;
    default:
        // Unknown interrupt, exit handler
        return;
    }

    // Input signal starting
    if (rising){
        start_time[index] = edge_time;
        data_ready[index] = false;
    }

    // Input signal ending
    else {
        if (MICROS(edge_time - start_time[index]) > 10){
            duration[index] = edge_time - start_time[index];
        }
        data_ready[index] = true;
    }
}
