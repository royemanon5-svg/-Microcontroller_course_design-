#include "soft_uart.h"
#include <stdio.h>
#include <stdarg.h>

/* PB3 = TX (output), PB4 = RX (input) */
#define TX_PIN   GPIO_PIN_3
#define TX_PORT  GPIOB
#define RX_PIN   GPIO_PIN_4
#define RX_PORT  GPIOB

static uint32_t bit_ticks; /* DWT cycles per bit */

/* ---------- DWT cycle-accurate delay ---------- */
static void Delay_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/* Wait until DWT->CYCCNT >= target (overflow-safe for intervals < 2^31) */
static void WaitUntil(uint32_t target)
{
    while ((int32_t)(DWT->CYCCNT - target) < 0);
}

/* ---------- Init ---------- */
void SoftUART_Init(uint32_t baudrate)
{
    bit_ticks = SystemCoreClock / baudrate;

    Delay_Init();

    /* Release PB3 (JTDO) and PB4 (JNTRST) — keep SWD on PA13/PA14 */
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_AFIO_REMAP_SWJ_NOJTAG();

    GPIO_InitTypeDef cfg = {0};

    /* TX — push-pull, idle high */
    cfg.Pin   = TX_PIN;
    cfg.Mode  = GPIO_MODE_OUTPUT_PP;
    cfg.Pull  = GPIO_NOPULL;
    cfg.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(TX_PORT, &cfg);
    TX_PORT->BSRR = TX_PIN;

    /* RX — input with pull-up */
    cfg.Pin  = RX_PIN;
    cfg.Mode = GPIO_MODE_INPUT;
    cfg.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(RX_PORT, &cfg);
}

/* ---------- TX (all timing relative to first edge — zero drift) ---------- */
void SoftUART_SendByte(uint8_t data)
{
    __disable_irq();
    uint32_t t = DWT->CYCCNT;

    /* start bit */
    TX_PORT->BRR = TX_PIN;
    t += bit_ticks;
    WaitUntil(t);

    /* 8 data bits, LSB first */
    for (int i = 0; i < 8; i++) {
        if (data & 0x01)
            TX_PORT->BSRR = TX_PIN;
        else
            TX_PORT->BRR = TX_PIN;
        data >>= 1;
        t += bit_ticks;
        WaitUntil(t);
    }

    /* stop bit */
    TX_PORT->BSRR = TX_PIN;
    t += bit_ticks;
    WaitUntil(t);

    __enable_irq();
}

void SoftUART_SendString(const char *str)
{
    while (*str) {
        SoftUART_SendByte((uint8_t)*str);
        str++;
    }
}

void SoftUART_Printf(const char *fmt, ...)
{
    static char buf[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    SoftUART_SendString(buf);
}

/* ---------- RX (all sampling from start edge — zero drift) ---------- */
uint8_t SoftUART_ReceiveByte(uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();

    /* wait for falling edge (start bit) */
    while (RX_PORT->IDR & RX_PIN) {
        if (timeout_ms && (HAL_GetTick() - t0) >= timeout_ms)
            return 0;
    }

    __disable_irq();
    uint32_t edge = DWT->CYCCNT; /* start-bit falling edge */

    /* verify start bit at its center (0.5 bit from edge) */
    WaitUntil(edge + bit_ticks / 2);
    if (RX_PORT->IDR & RX_PIN) {
        __enable_irq();
        return 0;
    }

    /* sample data bits at 1.5, 2.5 … 8.5 bit-times from edge */
    uint8_t data = 0;
    for (int i = 0; i < 8; i++) {
        WaitUntil(edge + bit_ticks + bit_ticks / 2 + i * bit_ticks);
        if (RX_PORT->IDR & RX_PIN)
            data |= (1 << i);
    }

    __enable_irq();
    return data;
}
