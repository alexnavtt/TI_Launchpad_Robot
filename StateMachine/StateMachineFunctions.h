#ifndef STATE_MACHINE_FUNCTIONS_H_
#define STATE_MACHINE_FUNCTIONS_H_

  /////////////////
 // Calibration //
/////////////////

// Rotate the robot in order to calibrate the magnetometer
void calibrate();

// Rotate the robot in order to determine which direction is forwards
void orientate();

enum CalibrationState{
    INIT,
    REVERSE,
    CALIBRATE,
    ORIENTATE,
    COMPLETE,
    DUMMY_CAL
};

enum CalibrationEvent{
    CAL_NOTHING,
    CAL_BUMP,
    CAL_FINISHED,
    CAL_INIT
};

  ////////////////
 // Navigation //
////////////////

// Use the Sonar sensors to approximate the robot location in the playing field
void getUnlost();

// Move towards the backboard in order to search for a beacon
void approachBackboard();

// Move along the backboard to find the correct beacon
void goToNextBeacon();

enum NavigationState{
    LOST,
    APPROACHING,
    SEARCHING,
    CHECKING,
    HOMING,
    ARRIVED,
    NAV_REVERSE,
    DUMMY_NAV
};

enum NavigationEvent{
    NAV_NOTHING,
    NAV_BUMP,
    NAV_IN_RANGE,
    NAV_ERROR
};


  //////////////
 // Shooting //
//////////////

enum ShootingState{
    AIMING,
    FIRING,
    RELOADING,
    VERIFYING,
    DUMMY_SHOOT
};

enum ShootingEvent{
    SHOOT_NOTHING,
    SHOOTING_FIRE,
    SHOOTING_ALIGNED,
    SHOOT_BUMP
};

// Global Variables: Be very careful with these
extern enum CalibrationEvent rCalEvent;
extern enum NavigationEvent rNavEvent;
extern enum ShootingEvent rShootEvent;

#endif
