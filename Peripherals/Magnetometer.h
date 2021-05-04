#ifndef ROBOT_BASE_MAGNETOMETER_H
#define ROBOT_BASE_MAGNETOMETER_H

#include <stdint.h>

void  Mag_Init();
void  Mag_Read();
float Mag_GetAngle();
void  Mag_ReadRegister(uint8_t);
void  Mag_SetOffset(float offset);
void  Mag_UpdateOffset();

#endif
