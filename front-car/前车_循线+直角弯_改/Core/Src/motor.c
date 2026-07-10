#include "motor.h"

// 声明 CubeMX 在 main.c 中定义的 TIM2 句柄
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
/**
  * @brief 电机初始化：启动 PWM 信号，初始状态停止
  */

void Motor_Init(void)
{
    // 1. 开启 PWM 通道
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // 对应 PA0
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // 对应 PA1

    // 2. 初始状态引脚全部拉低（停止）
    HAL_GPIO_WritePin(MOTOR_A_PORT, MOTOR_AIN1_PIN | MOTOR_AIN2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_B_PORT, MOTOR_BIN1_PIN | MOTOR_BIN2_PIN, GPIO_PIN_RESET);
}

/**
  * @brief 电机驱动核心函数
  * @param PWML 左电机速度/方向 (-999 到 999)
  * @param PWMR 右电机速度/方向 (-999 到 999)
  */
void Get_Motor_Speed(int16_t *leftSpeed, int16_t *rightSpeed)
{
    // 强制转换为 short 类型以处理正负反转（计数器溢出）
    *leftSpeed  = (short)__HAL_TIM_GET_COUNTER(&htim3); 
    *rightSpeed = (short)__HAL_TIM_GET_COUNTER(&htim4);
    
    // 读取后清零，这样下次读到的就是这 10ms 内的脉冲增量
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
}
void Motor_SetPWM(int16_t PWML, int16_t PWMR)
{
    // 1. 限幅处理 (假设 ARR 设置为 999)
    if(PWML > 999)  PWML = 999;
    if(PWML < -999) PWML = -999;
    if(PWMR > 999)  PWMR = 999;
    if(PWMR < -999) PWMR = -999;

    // --- 左电机控制 (PA2, PA3) ---
    if (PWML > 0)
    {
        HAL_GPIO_WritePin(MOTOR_A_PORT, MOTOR_AIN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_A_PORT, MOTOR_AIN2_PIN, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, PWML);
    }
		/*else if(PWML == 0)
		{
			HAL_GPIO_WritePin(MOTOR_A_PORT, MOTOR_AIN1_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(MOTOR_A_PORT, MOTOR_AIN2_PIN, GPIO_PIN_RESET);
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
		
		}*/
    else
    {
        HAL_GPIO_WritePin(MOTOR_A_PORT, MOTOR_AIN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_A_PORT, MOTOR_AIN2_PIN, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, -PWML);
    }
		

    // --- 右电机控制 (PB0, PB1) ---
    if (PWMR > 0)
    {
        HAL_GPIO_WritePin(MOTOR_B_PORT, MOTOR_BIN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_B_PORT, MOTOR_BIN2_PIN, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, PWMR);
    }
		/*else if(PWMR == 0)
		{
			HAL_GPIO_WritePin(MOTOR_B_PORT, MOTOR_BIN1_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(MOTOR_B_PORT, MOTOR_BIN2_PIN, GPIO_PIN_RESET);
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
		
		}*/
    else
    {
        HAL_GPIO_WritePin(MOTOR_B_PORT, MOTOR_BIN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_B_PORT, MOTOR_BIN2_PIN, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, -PWMR);
    }
}
