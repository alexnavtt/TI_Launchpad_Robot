#ifndef ROBOT_UTIL_H_
#define ROBOT_UTIL_H_

#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#define RAD2DEG    57.2958
#define DEG2RAD    0.0174533

static inline int16_t Util_Modulo(int16_t a, int16_t b){
    return a - (a/b) * b;
}

static inline int16_t Util_Sign(float a){
    if (a == 0) return 0;
    else if (a > 0) return 1;
    else return -1;
}

static inline float Util_Angle(float a){
    while (a >= 2*M_PI) a -= 2*M_PI;
    while (a <  0     ) a += 2*M_PI;
    return a;
}

static inline float Util_AngleDiff(float a, float b){
    int16_t error = RAD2DEG*(a - b);
    error = (error + 540) % 360 - 180;
    return DEG2RAD*error;
}

static inline bool Util_AngleInRange(float theta, float target){
    float error = Util_AngleDiff(theta, target);
    return abs(error) < 0.05;
}

#endif
