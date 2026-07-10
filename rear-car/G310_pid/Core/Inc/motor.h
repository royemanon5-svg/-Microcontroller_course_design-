#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"
/* 左电机 (Motor A) 引脚定义 */
#define MOTOR_A_PORT      GPIOA
#define MOTOR_AIN1_PIN    GPIO_PIN_2
#define MOTOR_AIN2_PIN    GPIO_PIN_3

/* 右电机 (Motor B) 引脚定义 */
#define MOTOR_B_PORT      GPIOB
#define MOTOR_BIN1_PIN    GPIO_PIN_0
#define MOTOR_BIN2_PIN    GPIO_PIN_1

/* 函数声明 */
void Motor_Init(void);
void Motor_SetPWM(int16_t PWML, int16_t PWMR);
//读取编码器的值
void Get_Motor_Speed(int16_t *leftSpeed, int16_t *rightSpeed);

#endif
