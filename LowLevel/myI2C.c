#include <stdio.h>

#include "msp.h"
#include "Clock.h"
#include "LowLevel/myI2C.h"

static EUSCI_B_Type* I2C[4] = {
    EUSCI_B0,
    EUSCI_B1,
    EUSCI_B2,
    EUSCI_B3
};

static bool I2C_Configured[4] = {false, false, false, false};

static inline void generateStartTx(enum I2C_Module N){
    I2C[N]->CTLW0 = (I2C[N]->CTLW0 & ~UCTXSTP) | (UCTR + UCTXSTT);
}

static inline void generateStartRx(enum I2C_Module N){
    I2C[N]->CTLW0 = (I2C[N]->CTLW0 & ~(UCTXSTP + UCTR)) | UCTXSTT;
}

static inline void generateStop(enum I2C_Module N){
    I2C[N]->CTLW0 |= UCTXSTP;
}

static inline bool busBusy(enum I2C_Module N){
    return I2C[N]->STATW & UCBBUSY;
}

static inline void sendByteTx(enum I2C_Module N, byte data){
    I2C[N]->TXBUF = data;
}

static inline bool txSent(enum I2C_Module N){
    return I2C[N]->IFG & UCTXIFG0;
}

static inline bool rxReceived(enum I2C_Module N){
    return I2C[N]->IFG & UCRXIFG0;
}

static inline uint8_t readBuffer(enum I2C_Module N){
    return I2C[N]->RXBUF;
}

static inline void enterConfigMode(enum I2C_Module N){
    I2C[N]->CTLW0 |= UCSWRST;
}

static inline void exitConfigMode(enum I2C_Module N){
    I2C[N]->CTLW0 &= ~UCSWRST;
}

static inline void clearInterrupts(enum I2C_Module N){
    I2C[N]->IFG = 0x00;
}

static inline void useAutoStop(enum I2C_Module N){
    I2C[N]->CTLW1 |= UCASTP1;
}

static inline void disableAutoStop(enum I2C_Module N){
    I2C[N]->CTLW1 &= ~UCASTP1;
}

static inline void setDataCount(enum I2C_Module N, uint32_t cnt){
    I2C[N]->TBCNT = cnt;
}

static inline void setSlaveAddress(enum I2C_Module N, uint8_t addr){
    I2C[N]->I2CSA = addr & 0x7F;
}

void I2C_Configure(enum I2C_Module N, const I2C_Config config){
    // Configure pins 1.6 (SDA) & 1.7 (SCL) in I2C mode
    switch (N){
    case MODULE_0:
        // Pins 1.6(SDA) and 1.7(SCL)
        P1->SEL0 |=  0xC0;
        P1->SEL1 &= ~0xC0;
        break;

    case MODULE_1:
        // Pins 6.4(SDA) and 6.5(SCL)
        P6->SEL0 |=  0x30;
        P6->SEL1 &= ~0x30;
        break;

    case MODULE_2:
        // Pins 3.6(SDA) and 3.7(SCL)
        P3->SEL0 |=  0xC0;
        P3->SEL1 &= ~0xC0;
        break;

    case MODULE_3:
        // Pins 10.2(SDA) and 10.3(SCL)
        P10->SEL0 |=  0x0C;
        P10->SEL1 &= ~0x0C;
    }

    EUSCI_B_Type* Module = I2C[N];

    enterConfigMode(N);

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
    Module->CTLW0 = 0x07C1 | (config.mode << 11);

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
    Module->CTLW1 = 0x0000;

    // --- Set the I2C Clock prescaler --- //
    Module->BRW = config.rate;

    // Manage interrupts
    clearInterrupts(N);
    Module->IE  = config.interrupts * (
                    0x0001 |    // Receive interrupt
                    0x0002 |    // Transmit interrupt
                    0x0020 |    // NACK interrupt
                    0x0040 );   // Byte counter interrupt

    exitConfigMode(N);

    // Wait for bus to be free (should be immediate)
    for (uint32_t i = 0; i < 1e5; i++){
        if (!busBusy(N)) break;
    }

    I2C_Configured[N] = true;
}

// Receive acknowledgement from the slave device after a Tx Start
static bool waitAfterStartTx(enum I2C_Module N){
    EUSCI_B_Type* Module = I2C[N];
    uint32_t count = 480000;
    while((Module->CTLW0 & UCTXSTT) || (!txSent(N))){
        count--;
        if (!count) return false;
    };
    return true;
}

static bool waitForAckTx(enum I2C_Module N){
    // Wait for at most 1ms
    for (uint32_t i = 0; i < 48000; i++){
       if (txSent(N)) return true;
    }
    generateStop(N);
    clearInterrupts(N);
    return false;
}

static bool waitForAckRx(enum I2C_Module N){
    // Wait for at most 100ms
    for (uint32_t i = 0; i < 4800000; i++){
        if (rxReceived(N)) return true;
    }
    generateStop(N);
    clearInterrupts(N);
    return false;
}

static bool waitForBus(enum I2C_Module N){
    for (uint32_t i = 0; i < 480000; i++){
        if (!busBusy(N)) return true;
    }
    disableAutoStop(N);
    generateStop(N);
    return false;
//    while(busBusy(N));
//    return true;
}

bool I2C_CheckForSlave(enum I2C_Module N, byte addr){
    if (!I2C_Configured[N]) return false;
    bool success;

    setSlaveAddress(N, addr);
    generateStartTx(N);
    waitAfterStartTx(N);

    sendByteTx(N, 0x00);
    success = waitForAckTx(N);

    generateStop(N);
    return success;
}

void I2C_ScanForDevices(enum I2C_Module N){
    if (!I2C_Configured[N]) printf("Module %d not configured, cannot scan for devices\n", N);
    uint8_t count = 0;
    for (uint8_t addr = 0x00; addr < 0x80; addr++){
        if (I2C_CheckForSlave(N, addr)){
            printf("Device found at address 0x%02x\n", addr);
            count++;
        }
    }
    printf("Scan completed. %zd device%s found\n", count, count > 1 ? "s" : "");
}

bool I2C_SendOneByte(enum I2C_Module N, byte addr, byte data){
    setSlaveAddress(N, addr);

    if (!waitForBus(N)) return false;

    generateStartTx(N);
    if (!waitForAckTx(N)) return false;

    sendByteTx(N, data);
    if (!waitForAckTx(N)) return false;

    generateStop(N);
    return true;
}

bool I2C_SendMultiByte(enum I2C_Module N, byte addr, byte* data, unsigned int count){
    if (!waitForBus(N)) return false;

    setSlaveAddress(N, addr);

    bool success = true;

    generateStartTx(N);
    for (unsigned int i = 0; i < count; i++){
        sendByteTx(N, data[i]);
        if (!waitForAckTx(N)) success = false;
    }
    generateStop(N);

    return success;
}

bool I2C_ReceiveOneByte(enum I2C_Module N, byte addr, byte* data){
    setSlaveAddress(N, addr);

    if (!waitForBus(N)) return false;

    generateStartRx(N);
    generateStop(N);
    if (!waitForAckRx(N)) return false;

    *data = readBuffer(N);
    return true;
}

bool I2C_ReceiveMultiByte(enum I2C_Module N, byte addr, byte data[], unsigned int count){
    bool success = true;

    if (!waitForBus(N)) return false;

    setSlaveAddress(N, addr);

    enterConfigMode(N);
    setDataCount(N, count);
    useAutoStop(N);
    exitConfigMode(N);

    generateStartRx(N);
    for (unsigned int i = 0; i < count; i++){
        if (!waitForAckRx(N)) {
            success = false;
        }
        data[i] = readBuffer(N);
    }

    enterConfigMode(N);
    disableAutoStop(N);
    exitConfigMode(N);

    if (!success) generateStop(N);

    return success;
}

bool I2C_SendAndReceiveWithRestart(enum I2C_Module N, byte addr, byte* TxData, unsigned int TxCount, byte* RxData, unsigned int RxCount){
    bool success = true;

    if (!waitForBus(N)) return false;

    setSlaveAddress(N, addr);

    enterConfigMode(N);
    setDataCount(N, TxCount + RxCount);
    useAutoStop(N);
    exitConfigMode(N);

    generateStartTx(N);
    if (!waitForAckTx(N)) success = false;

    for (unsigned int i = 0; i < TxCount; i++){
        sendByteTx(N, TxData[i]);
        if (!waitForAckTx(N))
            success = false;
    }

    generateStartRx(N);
    for (unsigned int i = 0; i < RxCount; i++){
        if (!waitForAckRx(N))
            success = false;
        RxData[i] = readBuffer(N);
    }

    Clock_Delay1us(100);
    enterConfigMode(N);
    disableAutoStop(N);
    exitConfigMode(N);

    return success;
}

