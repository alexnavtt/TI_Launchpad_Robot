#include <stdbool.h>

#ifndef SONAR_SENSOR_H
#define SONAR_SENSOR_H

void Sonar_Init();
void Sonar_Update();
float Sonar_Read(uint8_t index);

#endif
