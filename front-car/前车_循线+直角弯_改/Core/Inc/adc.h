#ifndef __ADC_H
#define __ADC_H

#include "main.h"

/* ADC阈值：大于此值判定为黑线，否则为白线 */
#define ADC_THRESHOLD  325

/* 74HC4051 通道选择引脚 */
#define MUX_S0_PORT  GPIOA
#define MUX_S0_PIN   GPIO_PIN_5     // PA5 = S0 (A)
#define MUX_S1_PORT  GPIOB
#define MUX_S1_PIN   GPIO_PIN_12    // PB12 = S1 (B) 原OUT1
#define MUX_S2_PORT  GPIOB
#define MUX_S2_PIN   GPIO_PIN_13    // PB13 = S2 (C) 原OUT2

/* ADC DMA 循环缓冲区，由 ADC_ReadAll() 刷新 */
extern volatile uint16_t adc_values[8];

/* ADC 句柄 */
extern ADC_HandleTypeDef hadc1;

/* 初始化 ADC1 和 4051 GPIO，并执行首次采集 */
void ADC_Init(void);

/* 轮询采集全部 8 路灰度值，存入 adc_values[8] */
void ADC_ReadAll(void);

#endif /* __ADC_H */
