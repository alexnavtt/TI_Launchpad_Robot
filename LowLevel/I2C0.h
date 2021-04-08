#ifndef I2C_0_H_
#define I2C_0_H_

#include <stdbool.h>
#include <stdint.h>

typedef uint8_t byte;

enum I2C_Mode{
    SLAVE,
    MASTER
};

enum I2C_Interrupts{
    NO_INTERRUPTS,
    USE_INTERRUPTS
};

enum I2C_Rate{
    I2C_400KBPS = 30,
    I2C_120KBPS = 100
};

typedef struct I2C_Config{
    enum I2C_Mode mode;
    enum I2C_Interrupts interrupts;
    enum I2C_Rate rate;
} I2C_Config;

void I2C_0_Configure(const I2C_Config config);
bool I2C_0_SendOneByte(byte addr, byte data);
bool I2C_0_SendMultiByte(byte addr, byte data[], unsigned int count);
bool I2C_0_ReceiveOneByte(byte addr, byte* data);
bool I2C_0_ReceiveMultiByte(byte addr, byte data[], unsigned int count);
bool I2C_0_SendAndReceiveWithRestart(byte addr, byte* TxData, unsigned int TxCount, byte* RxData, unsigned int RxCount);

#endif
