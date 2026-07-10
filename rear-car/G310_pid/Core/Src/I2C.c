#include "I2C.h"

// 简单的延时，确保时序稳定
void I2C_Delay(void) {
    for(volatile int i=0; i<5; i++); 
}

// 起始信号：SCL高时，SDA由高变低
void I2C_Start(void) {
    SDA_H; SCL_H; I2C_Delay();
    SDA_L; I2C_Delay();
    SCL_L; I2C_Delay();
}

// 停止信号：SCL高时，SDA由低变高
void I2C_Stop(void) {
    SDA_L; SCL_L; I2C_Delay();
    SCL_H; I2C_Delay();
    SDA_H; I2C_Delay();
}

// 发送一个字节并返回应答位
uint8_t I2C_SendByte(uint8_t byte) {
    for (int i = 0; i < 8; i++) {
        if (byte & 0x80) SDA_H; else SDA_L;
        byte <<= 1;
        I2C_Delay();
        SCL_H; I2C_Delay();
        SCL_L; I2C_Delay();
    }
    // 读取 ACK
    SDA_H; SCL_H; I2C_Delay();
    uint8_t ack = SDA_READ;
    SCL_L; I2C_Delay();
    return ack;
}

// 读取一个字节
uint8_t I2C_ReadByte(uint8_t ack) {
    uint8_t byte = 0;
    SDA_H; // 释放总线
    for (int i = 0; i < 8; i++) {
        byte <<= 1;
        SCL_H; I2C_Delay();
        if (SDA_READ) byte |= 0x01;
        SCL_L; I2C_Delay();
    }
    // 发送 ACK 或 NACK
    if (ack) SDA_L; else SDA_H;
    SCL_H; I2C_Delay();
    SCL_L; I2C_Delay();
    SDA_H;
    return byte;
}
