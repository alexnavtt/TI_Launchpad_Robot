#ifndef MOTOR_PID_H
#define MOTOR_PID_H

#include <stdint.h>

enum MotorIndex{
    L_MOTOR,
    R_MOTOR
};

extern float PID_setpt[2];

void PID_Init();


/**
 * Set PID gains for a specific motor
 *
 * @param motor     the index of the motor to change
 * @param Kp        Proportional gain
 * @param Kd        Derivative gain
 * @param Ki        Integral gain
 * @return none
 * @brief  Set PID gains
 */
void PID_setGains(enum MotorIndex motor, uint16_t Kp, uint16_t Ki, uint16_t Kff);
void PID_setMaxVal(enum MotorIndex motor, float val);
void PID_setMaxIntVal(enum MotorIndex motor, float val);
void PID_setPoint(float left_setpt, float right_setpt);
void PID_Control(enum MotorIndex motor);
void PID_update(void);


#endif
