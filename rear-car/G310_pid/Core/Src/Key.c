#include "Key.h"
#include "main.h"

/* ==================== 双按键扫描实现 ==================== */

/* 按键配置 */
typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
    uint8_t       id;                 // 1=KEY1(PA11), 2=KEY2(PA12)
} KeyConfig;

static const KeyConfig key_cfg[] = {
    { key1_GPIO_Port, key1_Pin, 1 },       // KEY1 = PA11
    { key2_GPIO_Port, key2_Pin, 2 },       // KEY2 = PA12
};
#define KEY_COUNT (sizeof(key_cfg) / sizeof(key_cfg[0]))

/* 按键内部状态 */
typedef enum {
    KS_IDLE = 0,
    KS_DEBOUNCE_DOWN,
    KS_PRESSED,
    KS_LONG_PRESSED,
    KS_DEBOUNCE_UP,
} KeyState;

typedef struct {
    KeyState  state;
    uint32_t  press_start;
    uint32_t  release_time;
    uint8_t   event_pending;
    uint8_t   event_ready;
} KeyData;

static KeyData kd[KEY_COUNT];

/* 兼容全局变量 */
volatile uint8_t Task_Progress = 0;
volatile uint8_t key_switch_flag = 0;

/* 阈值 */
#define DEBOUNCE_MS       20
#define LONG_PRESS_MS     1000
#define DOUBLE_CLICK_MS   400

static uint8_t read_key_pin(uint8_t idx)
{
    // 上拉输入：按下 = 低电平 = GPIO_PIN_RESET
    return (HAL_GPIO_ReadPin(key_cfg[idx].port, key_cfg[idx].pin) == GPIO_PIN_RESET) ? 1 : 0;
}

void Key_Scan(void)
{
    uint32_t now = HAL_GetTick();
    for (uint8_t i = 0; i < KEY_COUNT; i++)
    {
        uint8_t pressed = read_key_pin(i);
        switch (kd[i].state)
        {
            case KS_IDLE:
                if (pressed)
                {
                    kd[i].state = KS_DEBOUNCE_DOWN;
                    kd[i].press_start = now;
                }
                break;

            case KS_DEBOUNCE_DOWN:
                if (pressed)
                {
                    if ((now - kd[i].press_start) >= DEBOUNCE_MS)
                        kd[i].state = KS_PRESSED;
                }
                else
                {
                    kd[i].state = KS_IDLE;
                }
                break;

            case KS_PRESSED:
                if (pressed)
                {
                    if ((now - kd[i].press_start) >= LONG_PRESS_MS)
                    {
                        kd[i].state = KS_LONG_PRESSED;
                        kd[i].event_pending = KEY_EVENT_LONG;
                        kd[i].event_ready = 1;
                        /* 兼容旧接口：长按也设置 key_switch_flag */
                        if (key_cfg[i].id == 1)
                            key_switch_flag = 1;
                    }
                }
                else
                {
                    uint32_t hold = now - kd[i].press_start;
                    if (hold < LONG_PRESS_MS && hold >= DEBOUNCE_MS)
                    {
                        if ((now - kd[i].release_time) <= DOUBLE_CLICK_MS)
                        {
                            kd[i].event_pending = KEY_EVENT_DOUBLE;
                            kd[i].event_ready = 1;
                            kd[i].release_time = 0;
                        }
                        else
                        {
                            kd[i].release_time = now;
                            kd[i].event_pending = KEY_EVENT_SHORT;
                            kd[i].event_ready = 1;
                            /* 兼容旧接口 */
                            if (key_cfg[i].id == 1)
                                key_switch_flag = 1;
                        }
                    }
                    kd[i].state = KS_IDLE;
                }
                break;

            case KS_LONG_PRESSED:
                if (!pressed)
                {
                    kd[i].state = KS_IDLE;
                    kd[i].release_time = now;
                }
                break;

            default:
                kd[i].state = KS_IDLE;
                break;
        }
    }
}

KeyEvent Key1_GetEvent(void)
{
    for (uint8_t i = 0; i < KEY_COUNT; i++)
    {
        if (key_cfg[i].id == 1 && kd[i].event_ready)
        {
            kd[i].event_ready = 0;
            return (KeyEvent)kd[i].event_pending;
        }
    }
    return KEY_EVENT_NONE;
}

KeyEvent Key2_GetEvent(void)
{
    for (uint8_t i = 0; i < KEY_COUNT; i++)
    {
        if (key_cfg[i].id == 2 && kd[i].event_ready)
        {
            kd[i].event_ready = 0;
            return (KeyEvent)kd[i].event_pending;
        }
    }
    return KEY_EVENT_NONE;
}
