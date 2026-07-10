#include "encoder.h"

// 定义全局变量记录脉冲数
volatile int32_t Encoder_Left_Count = 0;
volatile int32_t Encoder_Right_Count = 0;

/**
  * @brief 外部中断回调函数（由 HAL 库自动调用）
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // --- 左编码器处理 (PA2 触发中断) ---
    if (GPIO_Pin == GPIO_PIN_2)
    {
        // 读取 B 相 (PA3) 的电平来判断方向
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3))
        {
            Encoder_Left_Count++;
        }
        else
        {
            Encoder_Left_Count--;
        }
    }

    // --- 右编码器处理 (PA4 触发中断) ---
    else if (GPIO_Pin == GPIO_PIN_4)
    {
        // 读取 B 相 (PA5) 的电平来判断方向
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5))
        {
            Encoder_Right_Count++;
        }
        else
        {
            Encoder_Right_Count--;
        }
    }
}

/**
  * @brief 获取速度并清零（单位：每采样周期的脉冲数）
  */
int16_t Get_Encoder_Speed(uint8_t side)
{
    int16_t speed;
    if (side == 0) // 左
    {
        speed = Encoder_Left_Count;
        Encoder_Left_Count = 0;
    }
    else // 右
    {
        speed = Encoder_Right_Count;
        Encoder_Right_Count = 0;
    }
    return speed;
}