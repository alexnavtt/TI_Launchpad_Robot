#ifndef ROBOT_KINEMATICS_H
#define ROBOT_KINEMATICS_H

// --- Kinematic Equations ---
/*
 * d : Robot diameter
 * R : Turning radius
 * vL: Left wheel linear velocity
 * vR: Right wheel linear velocity
 * w : Robot angular velocity (omega)
 * vx: Robot linear velocity (robot CoM speed)
 *
 * SAME DIRECTION
 * Left  wheel: vL = w*(R - d/2)
 * Right wheel: vR = w*(R + d/2)
 *
 * Subtract EQ1 from EQ2:
 *        -> vR - vL = w*(d)
 *        -> w = (vR - vL)/d
 *
 * DIFFERENT DIRECTION
 * Left Wheel:  vL = vx - w*R/2
 * Right Wheel: vR = vx + w*R/2
 *
 * Subtract EQ1 from EQ2
 *        -> vR - vL = w*(d)
 *        -> w = (vR - vL)/d
 *
 * THEREFORE: Equation is the same in both cases
 */

// Measurements in mm
#define ROBOT_DIAMETER 150
#define ROBOT_RADIUS    75
#define WHEEL_RADIUS    35
#define ROBOT_RADIUS_F32 0.075f
#define WHEEL_RADIUS_F32 0.035f

/*
 * @param leftWheelVel      angular velocity of the left wheel (rad/s)
 * @param rightWheelVel     angular velocity of the right wheel (rad/s)
 * @param vx                output: linear speed of the robot (mm/s)
 * @param omega             output: angular velocity of the robot (rad/s)
 */
static inline void forwardKinematics(float leftWheelOmega, float rightWheelOmega, float* vx, float* omega){
    // Linear speed of the wheels
    float rightWheelVel = rightWheelOmega * WHEEL_RADIUS;
    float leftWheelVel  = leftWheelOmega * WHEEL_RADIUS;

    *omega = (rightWheelVel - leftWheelVel)/ROBOT_DIAMETER;
    *vx    = (rightWheelVel + leftWheelVel)*0.5;
}

/*
 * @param driveSpeed  input: linear speed of the robot (mm/s)
 * @param omega       input: angular velocity of the robot (rad/s)
 * @param leftSpeed  output: rotation speed of the left wheel  (rad/s)
 * @param rightSpeed output: rotation speed of the right wheel (rad/s)
 */
static inline void inverseKinematics(float driveSpeed, float omega, float* leftSpeed, float* rightSpeed){
    *leftSpeed  = (driveSpeed*0.001 - omega*ROBOT_RADIUS_F32)/WHEEL_RADIUS_F32;
    *rightSpeed = (driveSpeed*0.001 + omega*ROBOT_RADIUS_F32)/WHEEL_RADIUS_F32;
}

#endif
