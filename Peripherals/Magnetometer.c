#include "msp.h"
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "string.h"
#include "Clock.h"
#include "LowLevel/myI2C.h"
#include "Peripherals/Magnetometer.h"

// HMC5883L Characteristics
#define MAGNETOMETER_ADDRESS 0x1E
#define CONTROL_REGISTER_A   0x00
#define CONTROL_REGISTER_B   0x01
#define MODE_REGISTER        0x02
#define DATA_ADDRESS_START   0x03
#define WRITE_MODE           0x3C
#define READ_MODE            0x3D

static uint8_t RxData[6];
struct Readings{
    int16_t x;
    int16_t y;
    int16_t z;
} Readings;

static int16_t max_x = -10000;
static int16_t max_y = -10000;
static int16_t min_x =  10000;
static int16_t min_y =  10000;
static int16_t x_offset;
static int16_t y_offset;
static int16_t x_range;
static int16_t y_range;


enum DirectionIndex{
    X_INDEX,
    Y_INDEX
};

static void writeRegister(uint8_t addr, uint8_t val){
    uint8_t TxData[3] = {WRITE_MODE, addr, val};
    I2C_SendMultiByte(MODULE_0, MAGNETOMETER_ADDRESS, TxData, 3);
}

void Mag_Init(){
    // Briefly power off the device on startup
    P5->OUT  &= ~0x02;
    Clock_Delay1ms(10);
    P5->OUT  |=  0x02;

    // Setup I2C
    I2C_Config config;
    config.mode       = MASTER;
    config.interrupts = NO_INTERRUPTS;
    config.rate       = I2C_400KBPS;
    I2C_Configure(MODULE_0, config);

    // Wait for device startup time
    Clock_Delay1ms(50);

    // Clear RxData buffer
    memset(RxData, 0x00, 6);

    // Configure Device
    writeRegister(CONTROL_REGISTER_A, 0x78);
    writeRegister(CONTROL_REGISTER_B, 0x20);
    writeRegister(MODE_REGISTER,      0x00);
}

static void updateOffset(){
    if (Readings.x > max_x) max_x = Readings.x;
    if (Readings.x < min_x) min_x = Readings.x;
    if (Readings.y > max_y) max_y = Readings.y;
    if (Readings.y < min_y) min_y = Readings.y;

    x_range = 0.5*(max_x - min_x);
    y_range = 0.5*(max_y - min_y);
    x_offset = 0.5*(max_x + min_x);
    y_offset = 0.5*(max_y + min_y);
}

void Mag_Read(){
    uint8_t TxData[1] = {DATA_ADDRESS_START};
    I2C_SendAndReceiveWithRestart(MODULE_0, MAGNETOMETER_ADDRESS, TxData, 1, RxData, 6);

    Readings.x = (((uint16_t)RxData[0]) << 8) + RxData[1];
    Readings.y = (((uint16_t)RxData[4]) << 8) + RxData[5];

    updateOffset();
}

float Mag_GetAngle(){
    Mag_Read();
    return atan2(-(Readings.y - y_offset)/(float)y_range, (Readings.x - x_offset)/(float)x_range);
}

void Mag_ReadRegister(uint8_t addr){
    uint8_t Tx[1] = {addr};
    uint8_t Rx[1] = {0};
    I2C_SendAndReceiveWithRestart(MODULE_0, MAGNETOMETER_ADDRESS, Tx, 1, Rx, 1);

//    printf("Value at address 0x%02x is 0x%02x\n", addr, Rx[0]);
}
