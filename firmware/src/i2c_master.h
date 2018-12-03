#ifndef _I2C_MASTER_H
#define _I2C_MASTER_H

#include <stdint.h>
#include <stdbool.h>


uint8_t StartTransfer(uint8_t restart,int I2CBus);
uint8_t TransmitOneByte(uint8_t data,int I2CBus);
uint8_t StopTransfer(int I2CBus);
uint8_t WriteI2CData(uint8_t *I2CData,int NumByte,int I2CBus);
uint8_t ReadI2CData(uint8_t *I2CData,int NumByte,int I2CBus);
uint8_t ReadMultipleI2CData(uint8_t *I2CData,int NumByteToWrite,int NumByteToRead,int I2CBus);

#endif /* _I2C_MASTER_H */