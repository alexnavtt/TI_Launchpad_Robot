#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C"{
#endif

#define ROOMBA
//#define LINE_FOLLOWER
//#define SQUARE

#ifdef ROOMBA
enum RobotStateIndex{
    SHUTDOWN,
    FREE,
    REVERSE,
    RIGHT_TURN,
    LEFT_TURN,
    HALT
};

enum RobotStateEvent{
    NOTHING,
    BUMP_LEFT,
    BUMP_RIGHT,
    BUMP_BOTH,
};

#define EVENT_COUNT 4

#elif defined LINE_FOLLOWER

enum RobotStateIndex{
    LOST,           // Robot does not know where line is
    ON_TARGET,      // Robot is over the center of the line
    OFF_TARGET,     // Robot is over the line but not the center
    HALTED          // Robot is stopped
};

enum RobotStateEvent{
    NOTHING,
    BUMP,
    LOST_TARGET,
    LOST_LINE,
    FOUND_TARGET,
    FOUND_LINE
};

#define EVENT_COUNT 6

#elif defined SQUARE

enum RobotStateIndex{
    DRIVING,
    ROTATING,
    CORNER_ENTER,
    CORNER_EXIT,
    HALTED,
};

enum RobotStateEvent{
    NOTHING,
    BUMP,
    CORNER_REACHED,
    ANGLE_REACHED
};

#define EVENT_COUNT 4

#endif

typedef struct{
    const uint8_t state;
    const uint8_t next_state_by_event[EVENT_COUNT];
    void (*action)();
} RobotState;

void State_Init();
void State_Next();

#ifdef __cplusplus
}
#endif

#endif
