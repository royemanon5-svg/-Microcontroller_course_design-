#include "mpu6500.h"
#include "I2C.h"
#define MPU6500_ADDR (0x68 << 1)
void MPU6500_WriteReg(uint8_t reg, uint8_t data) {
    I2C_Start();
    I2C_SendByte(MPU6500_ADDR); // 发送设备写地址
    I2C_SendByte(reg);          // 发送寄存器地址
    I2C_SendByte(data);         // 发送数据
    I2C_Stop();
}
uint8_t MPU6500_ReadReg(uint8_t reg) {
    uint8_t data = 0;
    I2C_Start();
    I2C_SendByte(MPU6500_ADDR);
    I2C_SendByte(reg);
    
    I2C_Start();                // 重启信号
    I2C_SendByte(MPU6500_ADDR | 0x01); // 发送设备读地址
    data = I2C_ReadByte(0);     // 0 表示读取后发送 NACK
    I2C_Stop();
    return data;
}
// 新增：多字节读取
void MPU6500_ReadRegs(uint8_t reg, uint8_t *buf, uint16_t len) {
    I2C_Start();
    I2C_SendByte(MPU6500_ADDR);
    I2C_SendByte(reg);
    
    I2C_Start();
    I2C_SendByte(MPU6500_ADDR | 0x01);
    for(uint16_t i=0; i<len; i++) {
        // 最后一字节发送 NACK(0)，其余发送 ACK(1)
        buf[i] = I2C_ReadByte(i < (len - 1)); 
    }
    I2C_Stop();
}

uint8_t MPU6500_Init(void) {
    uint8_t id;
    HAL_Delay(100);

    id = MPU6500_ReadReg(0x75);   // ✅ 单字节读取
    if (id != 0x70) return 1;

    MPU6500_WriteReg(0x6B, 0x00);
    HAL_Delay(50);
    MPU6500_WriteReg(0x1B, 0x18);
    MPU6500_WriteReg(0x1C, 0x10); // ✅ 修正拼写
    MPU6500_WriteReg(0x1A, 0x03);

    return 0;
}

void MPU6500_Get_Data(MPU6500_Data *data) {
    uint8_t buf[14];
    MPU6500_ReadRegs(0x3B, buf, 14); // ✅ 用多字节读取

    // ✅ 强制转换为int16_t
    data->accX  = (int16_t)((buf[0]  << 8) | buf[1]);
    data->accY  = (int16_t)((buf[2]  << 8) | buf[3]);
    data->accZ  = (int16_t)((buf[4]  << 8) | buf[5]);
    data->gyroX = (int16_t)((buf[8]  << 8) | buf[9]);
    data->gyroY = (int16_t)((buf[10] << 8) | buf[11]);
    data->gyroZ = (int16_t)((buf[12] << 8) | buf[13]);
}
