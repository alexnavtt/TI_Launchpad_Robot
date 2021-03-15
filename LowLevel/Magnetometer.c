#include "msp.h"
#include "string.h"
#include "LowLevel/Magnetometer.h"
#include "ti/devices/msp432p4xx/driverlib/driverlib.h"

// Address for magnetometer
#define MAGNETOMETER_ADDRESS 0x0D

// I2C Commands
#define GENERATE_START_TX EUSCI_B0->CTLW0 = (EUSCI_B0->CTLW0 & ~0x0004) | 0x0012
#define GENERATE_START_RX EUSCI_B0->CTLW0 = (EUSCI_B0->CTLW0 & ~0x0014) | 0x0002
#define GENERATE_STOP  EUSCI_B0->CTLW0 |= 0x0004

static uint8_t RxData[4];

static inline void waitForAck(){
    while (!EUSCI_B0->IFG & 0x0002);
}

void Mag_Init(){
    // Configure pins 1.6 & 1.7 in I2C mode
    P1->SEL0 |=  0xC0;
    P1->SEL1 &= ~0xC0;

    // Clear RxData buffer
    memset(RxData, 0x00, 4);

    // Disable I2C module for setup
    EUSCI_B0->CTLW0 |= 0x0001;

    // --- Configure the I2C module (Register 0) --- //
    // Bit     15 =  0: own address is 7-bit
    // Bit     14 =  0: slave address is 7-bit
    // Bit     13 =  0: single master environment
    // Bit     12 =  0: reserved
    // >>> Byte = 0
    // Bit     11 =  1: master mode
    // Bits  10-9 = 11: I2C mode
    // Bit      8 =  1: synchronous mode
    // >>> Byte = F
    // Bits   7-6 = 11: clock source SMCLK (12 MHz)
    // Bit      5 =  0: ACK transmit bit
    // Bit      4 =  0: transmitter/receiver bit
    // >>> Byte = C
    // Bit      3 =  0: NACK transmit bit
    // Bit      2 =  0: STOP transmit bit
    // Bit      1 =  0: START transmit bit
    // Bit      0 =  1: disable I2C module
    // >>> Byte = 1
    EUSCI_B0->CTLW0 = 0x0FC1;

    // --- Configure the I2C module (Register 1) --- //
    // Bits 15-12 = 00: reserved
    // >>> Byte = 0
    // Bits  11-9 = 00: reserved
    // Bit      8 =  0: reserved for slave mode
    // >>> Byte = 0
    // Bits   7-6 = 00: disable timout
    // Bit      5 =  0: send a NACK at the end of receive
    // Bit      4 =  0: auto acknowledge slave address
    // >>> Byte = 0
    // Bits   3-2 = 00: no automatic STOP
    // Bits   1-0 = 00: 50ns deglitch time
    EUSCI_B0->CTLW1 = 0x0000;

    // --- Set the I2C Clock prescaler --- //
    EUSCI_B0->BRW = 30; // 12MHz/30 = 400 kb/s

    // Enable I2C and disable interrupts
    EUSCI_B0->CTLW0 &= ~0x0001;
    EUSCI_B0->IE = 0;

    // Wait for bus to be free (should be immediate)
    while(EUSCI_B0->STATW & UCBBUSY);

    // Set slave address
    EUSCI_B0->I2CSA = MAGNETOMETER_ADDRESS;

    // Generate a START condition (and clear any stop condition)
    GENERATE_START_TX;
    waitForAck();

    // Send the address of the control register for the magnetometer
    EUSCI_B0->TXBUF = 0x09;
    waitForAck();

    // Send the value of the control register for the magnetometer
    // Bits 7-6 = 01: continuous mode
    // Bits 5-4 = 00: 2 Gauss range
    // Bits 3-2 = 10: 100 Hz operation
    // Bits 1-0 = 01: 256 over-sample ratio
    EUSCI_B0->TXBUF = 0x49;
    waitForAck();

    // Generate the STOP signal and clear interrupt flag
    GENERATE_STOP;
    EUSCI_B0->IFG   &= ~0x0002;
}

void Mag_Read(){
    // Wait for ready condition
    while(EUSCI_B0->STATW & UCBBUSY);

    // Generate a transmit START condition
    GENERATE_START_TX;
    waitForAck();

    // Send the address of the data out register
    EUSCI_B0->TXBUF = 0x00;
    waitForAck();

    // Start the data receive protocol
    GENERATE_START_RX;
    for (uint8_t i = 0; i < 4; i++){
        while(!EUSCI_B0->IFG & 0x0001);
        RxData[i] = EUSCI_B0->RXBUF;
    }

    // Terminate communication
    GENERATE_STOP;
}

static void Mag_readX(){

}

static void Mag_readY(){

}
