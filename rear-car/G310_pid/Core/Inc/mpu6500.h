#ifndef __MPU6500_
#define __MPU6500_
#include "main.h"
typedef struct {
    int16_t accX, accY, accZ;
    int16_t gyroX, gyroY, gyroZ;
} MPU6500_Data;
void MPU6500_WriteReg(uint8_t reg, uint8_t data);
uint8_t MPU6500_ReadReg(uint8_t reg);
void MPU6500_ReadRegs(uint8_t reg, uint8_t *buf, uint16_t len);
uint8_t MPU6500_Init(void);
void MPU6500_Get_Data(MPU6500_Data *data);
#endif
