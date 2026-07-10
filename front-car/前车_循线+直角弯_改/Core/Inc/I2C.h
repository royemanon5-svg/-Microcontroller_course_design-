#ifndef __I2C_H
#define __I2C_H

#include "stm32f1xx_hal.h"

// 引脚操作定义
#define SCL_H         HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET)
#define SCL_L         HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET)
#define SDA_H         HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET)
#define SDA_L         HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET)
#define SDA_READ      HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11)

// 基础函数声明
void I2C_Start(void);
void I2C_Stop(void);
uint8_t I2C_SendByte(uint8_t byte);
uint8_t I2C_ReadByte(uint8_t ack);

#endif
