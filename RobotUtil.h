#ifndef ROBOT_UTIL_H_
#define ROBOT_UTIL_H_

#include <math.h>

#define RAD2DEG    57.2958
#define DEG2RAD    0.0174533

static inline int16_t modulo(int16_t a, int16_t b){
    return a - (a/b) * b;
}

static inline float Util_AngleDiff(float theta_now, float theta_desired){
    int16_t error = RAD2DEG*(theta_desired - theta_now);
    error = modulo(error + 180, 360) - 180;
    return DEG2RAD*error;
}

static inline float Util_Angle(float a){
    while (a > 2*M_PI) a -= 2*M_PI;
    while (a < 0     ) a += 2*M_PI;
}

#endif
