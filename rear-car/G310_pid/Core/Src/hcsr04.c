#include "hcsr04.h"

#define HCSR04_TIMEOUT_US 30000U

static HCSR04_Status last_status = HCSR04_STATUS_NO_ECHO;

static void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000U);

    while ((uint32_t)(DWT->CYCCNT - start) < ticks) {
    }
}

static uint32_t micros(void)
{
    return DWT->CYCCNT / (SystemCoreClock / 1000000U);
}

void HCSR04_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
    last_status = HCSR04_STATUS_NO_ECHO;
}

uint16_t HCSR04_ReadDistanceCm(void)
{
    uint32_t start_wait;
    uint32_t pulse_start;
    uint32_t pulse_width;

    if (HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin) == GPIO_PIN_SET) {
        last_status = HCSR04_STATUS_ECHO_STUCK_HIGH;
        return HCSR04_INVALID_DISTANCE_CM;
    }

    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
    delay_us(2);
    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
    delay_us(15);
    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);

    start_wait = micros();
    while (HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin) == GPIO_PIN_RESET) {
        if ((uint32_t)(micros() - start_wait) > HCSR04_TIMEOUT_US) {
            last_status = HCSR04_STATUS_NO_ECHO;
            return HCSR04_INVALID_DISTANCE_CM;
        }
    }

    pulse_start = micros();
    while (HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin) == GPIO_PIN_SET) {
        if ((uint32_t)(micros() - pulse_start) > HCSR04_TIMEOUT_US) {
            last_status = HCSR04_STATUS_ECHO_STUCK_HIGH;
            return HCSR04_INVALID_DISTANCE_CM;
        }
    }

    pulse_width = micros() - pulse_start;
    last_status = HCSR04_STATUS_OK;
    return (uint16_t)(pulse_width / 58U);
}

HCSR04_Status HCSR04_LastStatus(void)
{
    return last_status;
}
