#ifndef ROBOT_ODOMETRY_H
#define ROBOT_ODOMETRY_H

void Odom_Init();
void Odom_Step(float vx, float omega);
void Odom_Get(float* x, float* y, float* theta);
void Odom_Update(float x, float y, float theta);
void Odom_Show();
float Odom_Theta();
float Odom_X();
float Odom_Y();

#endif
