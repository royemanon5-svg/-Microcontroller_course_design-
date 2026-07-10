#include "adc.h"

/* 全局变量定义 */
volatile uint16_t adc_values[8] = {0};

/**
  * @brief 74HC4051 通道选择
  * @param ch: 0~7 (Y0~Y7)
  *        S2 S1 S0 = ch 的二进制位
  */
static void MUX_SelectChannel(uint8_t ch)
{
  /* S0 = ch bit0 */
  HAL_GPIO_WritePin(MUX_S0_PORT, MUX_S0_PIN,
                    (ch & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  /* S1 = ch bit1 */
  HAL_GPIO_WritePin(MUX_S1_PORT, MUX_S1_PIN,
                    (ch & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  /* S2 = ch bit2 */
  HAL_GPIO_WritePin(MUX_S2_PORT, MUX_S2_PIN,
                    (ch & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
  * @brief 初始化 74HC4051 通道选择引脚 (S0/S1/S2)
  */
static void MX_4051_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* S0: PA5, push-pull output */
  GPIO_InitStruct.Pin = MUX_S0_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(MUX_S0_PORT, &GPIO_InitStruct);

  /* S1: PB12 */
  GPIO_InitStruct.Pin = MUX_S1_PIN;
  HAL_GPIO_Init(MUX_S1_PORT, &GPIO_InitStruct);

  /* S2: PB13 */
  GPIO_InitStruct.Pin = MUX_S2_PIN;
  HAL_GPIO_Init(MUX_S2_PORT, &GPIO_InitStruct);

  /* 初始选中通道 0 */
  MUX_SelectChannel(0);
}

/* ---------- 公开函数 ---------- */

/**
  * @brief 初始化 4051 控制引脚
  *        ADC1 由 CubeMX 生成的 MX_ADC1_Init() 在 main.c 中初始化
  */
void ADC_Init(void)
{
  MX_4051_GPIO_Init();
}

/**
  * @brief 轮询采集全部 8 路灰度值
  *        逐个切换 4051 通道，ADC 单次转换，结果存入 adc_values[8]
  *        调用一次耗时约 8 * (10us 稳定 + ~4us 转换) ≈ 120us
  */
void ADC_ReadAll(void)
{
  for (uint8_t ch = 0; ch < 8; ch++)
  {
    /* 切换 4051 通道 */
    MUX_SelectChannel(ch);

    /* 等待 4051 输出稳定（导通电阻 + 负载电容，10us 足够） */
    for (volatile int i = 0; i < 72; i++) { __NOP(); }  // ~10us @72MHz

    /* 启动 ADC 单次转换 */
    HAL_ADC_Start(&hadc1);

    /* 等待转换完成 */
    if (HAL_ADC_PollForConversion(&hadc1, 1) == HAL_OK)
    {
      adc_values[ch] = HAL_ADC_GetValue(&hadc1);
    }

    /* 停止 ADC */
    HAL_ADC_Stop(&hadc1);
  }
}
