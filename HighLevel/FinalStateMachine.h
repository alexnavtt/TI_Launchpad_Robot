#ifndef FINAL_STATE_MACHINE_H_
#define FINAL_STATE_MACHINE_H_

#include <stdbool.h>
#include <stdint.h>

typedef struct{
    const uint8_t state;
    const uint8_t next_state_by_event[4];
    void (*action)();
} RobotState;

void State_Init();
void State_Next();

#endif
