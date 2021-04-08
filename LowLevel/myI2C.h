#ifndef I2C_1_H_
#define I2C_1_H_

#include <stdbool.h>
#include <stdint.h>

typedef uint8_t byte;

enum I2C_Module{
    MODULE_0,
    MODULE_1,
    MODULE_2,
    MODULE_3
};

#ifndef I2C_0_H_
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
#endif

void I2C_ScanForDevices(enum I2C_Module);
bool I2C_CheckForSlave(enum I2C_Module, byte addr);
void I2C_Configure(enum I2C_Module, const I2C_Config config);
bool I2C_SendOneByte(enum I2C_Module, byte addr, byte data);
bool I2C_SendMultiByte(enum I2C_Module, byte addr, byte data[], unsigned int count);
bool I2C_ReceiveOneByte(enum I2C_Module, byte addr, byte* data);
bool I2C_ReceiveMultiByte(enum I2C_Module, byte addr, byte data[], unsigned int count);
bool I2C_SendAndReceiveWithRestart(enum I2C_Module, byte addr, byte* TxData, unsigned int TxCount, byte* RxData, unsigned int RxCount);

#endif
