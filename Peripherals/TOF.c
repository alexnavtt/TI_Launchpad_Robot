#include <LowLevel/myI2C.h>
#include <stdio.h>
#include "Clock.h"
#include "Peripherals/TOF.h"
#include "ti/devices/msp432p4xx/driverlib/i2c.h"
//#include "ti/devices/msp432p4xx/driverlib/driverlib.h"

#define TOF_ADDRESS 0x29

static void writeRegister(uint8_t addr, uint8_t data){
    uint8_t TxData[2] = {addr, data};
    bool success = I2C_SendMultiByte(MODULE_1, TOF_ADDRESS, TxData, 2);
}

static uint8_t readRegister(uint8_t addr){
    uint8_t data;
    byte TxData[2] = {addr, TOF_ADDRESS << 1};
    bool send_success =    I2C_SendOneByte(MODULE_1, TOF_ADDRESS, addr);
    bool receive_success = I2C_ReceiveOneByte(MODULE_1, TOF_ADDRESS, &data);
    return data;
}

void TOF_Init(){
    // Setup I2C
    I2C_Config config;
    config.mode       = MASTER;
    config.interrupts = NO_INTERRUPTS;
    config.rate       = I2C_400KBPS;
    I2C_Configure(MODULE_1, config);

//    writeRegister(0x00, 0x01);
}

bool TOF_verifyID(){
//    writeRegister(0x00, 0x01);
    uint8_t ID = readRegister(0xC0);
    printf("ID: 0x%02x\n", ID);
    return ID == 0xEE;
//    return false;
}

void TOF_Test(){
    writeRegister(0x00, 0x00);
//    printf("%s\n", I2C_CheckForSlave(MODULE_0, 0x00) ? "Slave Found" : "Slave Absent");
}
