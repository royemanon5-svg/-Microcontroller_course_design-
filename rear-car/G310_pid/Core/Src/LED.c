#include "LED.h"   
void LED_ON(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
}

void LED_OFF(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

void LED_Turn(void)
{
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);
}
