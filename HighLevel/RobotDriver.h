#ifndef ROBOT_DRIVER_H_
#define ROBOT_DRIVER_H_

#include <stdbool.h>

bool Driver_GoToAngle(float theta);
bool Driver_GoTo(float x, float y);
void Driver_SetVelocity(float v, float omega);
bool Driver_GoToAngleOdom(float theta);

#endif
