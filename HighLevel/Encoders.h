#ifndef ROBOT_ENCODERS_H
#define ROBOT_ENCODERS_H

#ifdef __cplusplus
extern "C"{
#endif

void Encoder_Init();
void Encoder_Update();
float Encoder_leftWheelVel();
float Encoder_rightWheelVel();
void Encoder_Show();

#ifdef __cplusplus
}
#endif

#endif
