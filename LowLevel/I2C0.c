#include "msp.h"
#include "Clock.h"
#include "LowLevel/I2C0.h"

// I2C Commands
#define GENERATE_START_TX       EUSCI_B0->CTLW0 = (EUSCI_B0->CTLW0 & ~UCTXSTP) | (UCTR + UCTXSTT)
#define GENERATE_START_RX       EUSCI_B0->CTLW0 = (EUSCI_B0->CTLW0 & ~(UCTXSTP + UCTR)) | UCTXSTT
#define GENERATE_STOP           EUSCI_B0->CTLW0 |= UCTXSTP
#define BUS_BUSY               (EUSCI_B0->STATW & UCBBUSY)
#define TX_SENT                (EUSCI_B0->IFG & UCTXIFG0)
#define RX_RECEIVED            (EUSCI_B0->IFG & UCRXIFG0)
#define TX_FAIL                (EUSCI_B0->IFG & UCNACKIFG)
#define ENTER_CONFIG_MODE       EUSCI_B0->CTLW0 |= UCSWRST
#define EXIT_CONFIG_MODE        EUSCI_B0->CTLW0 &= ~UCSWRST
#define CLEAR_INTERRUPTS        EUSCI_B0->IFG = 0
#define USE_AUTO_STOP           EUSCI_B0->CTLW1 |= UCASTP1
#define DISABLE_AUTO_STOP       EUSCI_B0->CTLW1 &= ~UCASTP1
#define SET_DATA_COUNT(N)       EUSCI_B0->TBCNT = N

void I2C_0_Configure(const I2C_Config config){
    // Configure pins 1.6 (SDA) & 1.7 (SCL) in I2C mode
    P1->SEL0 |=  0xC0;
    P1->SEL1 &= ~0xC0;

    ENTER_CONFIG_MODE;

    // --- Configure the I2C module (Register 0) --- //
    // Bit     15 =  0: own address is 7-bit
    // Bit     14 =  0: slave address is 7-bit
    // Bit     13 =  0: single master environment
    // Bit     12 =  0: reserved
    // >>> Byte = 0
    // Bit     11 =  ?: master mode
    // Bits  10-9 = 11: I2C mode
    // Bit      8 =  1: synchronous mode
    // >>> Byte = F
    // Bits   7-6 = 11: clock source SMCLK (12 MHz)
    // Bit      5 =  X: ACK transmit bit
    // Bit      4 =  X: transmitter/receiver bit
    // >>> Byte = C
    // Bit      3 =  X: NACK transmit bit (slave mode)
    // Bit      2 =  X: STOP transmit bit
    // Bit      1 =  X: START transmit bit
    // Bit      0 =  1: disable I2C module
    // >>> Byte = 1
    EUSCI_B0->CTLW0 = 0x07C1 | (config.mode << 11);

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
    EUSCI_B0->BRW = config.rate; // 12MHz/30 = 400 kHz

    // Manage interrupts
    CLEAR_INTERRUPTS;
    EUSCI_B0->IE  = config.interrupts * (
                    0x0001 |    // Receive interrupt
                    0x0002 |    // Transmit interrupt
                    0x0020 |    // NACK interrupt
                    0x0040 );   // Byte counter interrupt

    EXIT_CONFIG_MODE;

    // Wait for bus to be free (should be immediate)
    for (uint32_t i = 0; i < 1e5; i++){
        if (!BUS_BUSY) break;
    }
}

static bool waitForAckTx(){
    // Wait for at most 1ms
    for (uint32_t i = 0; i < 48000; i++){
       if (TX_SENT) return true;
    }
    GENERATE_STOP;
    CLEAR_INTERRUPTS;
    return false;
}

static bool waitForAckRx(){
    // Wait for at most 10ms
    for (uint32_t i = 0; i < 480000; i++){
        if (RX_RECEIVED) return true;
    }
    GENERATE_STOP;
    CLEAR_INTERRUPTS;
    return false;
}

static bool waitForBus(){
    for (uint32_t i = 0; i < 480000; i++){
        if (!BUS_BUSY) return true;
    }
    return false;
}

bool I2C_0_SendOneByte(byte addr, byte data){
    EUSCI_B0->I2CSA = addr;

    while(BUS_BUSY);

    GENERATE_START_TX;
    if (!waitForAckTx()) return false;

    EUSCI_B0->TXBUF = data;
    if (!waitForAckTx()) return false;

    GENERATE_STOP;
    return true;
}

bool I2C_0_SendMultiByte(byte addr, byte* data, unsigned int count){
    while(BUS_BUSY);

    EUSCI_B0->I2CSA = addr;

    bool success = true;

    GENERATE_START_TX;
    for (unsigned int i = 0; i < count; i++){
        EUSCI_B0->TXBUF = data[i];
        if (!waitForAckTx()) success = false;
    }
    GENERATE_STOP;

    return success;
}

bool I2C_0_ReceiveOneByte(byte addr, byte* data){
    EUSCI_B0->I2CSA = addr;

    while(BUS_BUSY);

    GENERATE_START_RX | UCTXSTP;
    if (!waitForAckRx()) return false;

    *data = EUSCI_B0->RXBUF;
    return true;
}

bool I2C_0_ReceiveMultiByte(byte addr, byte data[], unsigned int count){
    bool success = true;

    while(BUS_BUSY);

    EUSCI_B0->I2CSA = addr;

    ENTER_CONFIG_MODE;
    EUSCI_B0->TBCNT = count;
    EUSCI_B0->CTLW1 |= UCASTP1;
    EXIT_CONFIG_MODE;

    GENERATE_START_TX;
    for (unsigned int i = 0; i < count; i++){
        if (!waitForAckRx()) {
            success = false;
            break;
        }
        data[i] = EUSCI_B0->RXBUF;
    }

    ENTER_CONFIG_MODE;
    EUSCI_B0->CTLW1 &= ~UCASTP1;
    EXIT_CONFIG_MODE;

    return success;
}

bool I2C_0_SendAndReceiveWithRestart(byte addr, byte* TxData, unsigned int TxCount, byte* RxData, unsigned int RxCount){
    bool success = true;

//    while(BUS_BUSY);
    if (!waitForBus()) return false;

    EUSCI_B0->I2CSA = addr;

    ENTER_CONFIG_MODE;
    SET_DATA_COUNT(TxCount + RxCount);
    USE_AUTO_STOP;
    EXIT_CONFIG_MODE;

    GENERATE_START_TX;
    if (!waitForAckTx()) success = false;

    for (unsigned int i = 0; i < TxCount; i++){
        EUSCI_B0->TXBUF = TxData[i];
        if (!waitForAckTx())
            success = false;
    }

    GENERATE_START_RX;
    for (unsigned int i = 0; i < RxCount; i++){
        if (!waitForAckRx())
            success = false;
        RxData[i] = EUSCI_B0->RXBUF;
    }

    Clock_Delay1us(100);
    ENTER_CONFIG_MODE;
    DISABLE_AUTO_STOP;
    EXIT_CONFIG_MODE;

    return success;
}

