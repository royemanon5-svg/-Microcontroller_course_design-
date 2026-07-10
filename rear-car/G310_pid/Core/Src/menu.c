#include "main.h"
#include "menu.h"
#include "Key.h"
#include "OLED.h"
#include "pid.h"
#include <stdio.h>

/* ==================== 全局变量 ==================== */

volatile uint8_t g_target_laps  = LAPS_DEFAULT;
volatile uint8_t g_current_lap  = 0;
volatile uint8_t g_running      = 0;

static uint8_t lap_display_dirty = 1;

/* ==================== 菜单初始化 ==================== */

void Menu_Init(void)
{
    OLED_Clear();

    OLED_ShowString(1, 1, "=== LAP  MENU ===");
    OLED_ShowString(2, 1, "Set Laps: ");
    OLED_ShowNum(2, 11, g_target_laps, 1);
    OLED_ShowString(3, 1, "K1:+ K2:- Hold>");
    OLED_ShowString(4, 1, "K2 Long= START");

    g_running = 0;
    g_current_lap = 0;
    lap_display_dirty = 0;
}

/* ==================== 菜单主循环 ==================== */

void Menu_Process(void)
{
    KeyEvent e1 = Key1_GetEvent();
    KeyEvent e2 = Key2_GetEvent();

    /* ===== 运行中：仅检测急停 ===== */
    if (g_running)
    {
        if (e1 == KEY_EVENT_LONG)
        {
            // KEY1 长按 = 急停
            g_running = 0;
            OLED_Clear();
            OLED_ShowString(1, 1, "!!! STOPPED !!!");
            OLED_ShowString(2, 1, "Lap: ");
            OLED_ShowNum(2, 6, g_current_lap, 1);
            OLED_ShowString(2, 8, "/");
            OLED_ShowNum(2, 9, g_target_laps, 1);
            OLED_ShowString(4, 1, "RST to restart");
        }
        return;
    }

    /* KEY1 (PA11) 短按：圈数+1 */
    if (e1 == KEY_EVENT_SHORT)
    {
        if (g_target_laps < LAPS_MAX)
            g_target_laps++;
        lap_display_dirty = 1;
    }
    /* KEY1 长按：也可启动 */
    else if (e1 == KEY_EVENT_LONG && g_target_laps > 0)
    {
        g_running = 1;
        g_current_lap = 0;
        OLED_Clear();
        OLED_ShowString(1, 1, ">>> START! <<<");
        OLED_ShowString(2, 1, "Target Laps: ");
        OLED_ShowNum(2, 14, g_target_laps, 1);
        OLED_ShowString(4, 1, "   Go! Go! Go!");
        HAL_Delay(500);
        OLED_Clear();
        OLED_ShowString(1, 1, "Running... 0/");
        OLED_ShowNum(1, 14, g_target_laps, 1);
        OLED_ShowString(4, 1, "K1 Long = STOP");
        return;
    }

    /* KEY2 (PA12) 短按：圈数-1 */
    if (e2 == KEY_EVENT_SHORT)
    {
        if (g_target_laps > LAPS_MIN)
            g_target_laps--;
        lap_display_dirty = 1;
    }
    /* KEY2 长按：确认启动 */
    else if (e2 == KEY_EVENT_LONG && g_target_laps > 0)
    {
        g_running = 1;
        g_current_lap = 0;
        OLED_Clear();
        OLED_ShowString(1, 1, ">>> START! <<<");
        OLED_ShowString(2, 1, "Target Laps: ");
        OLED_ShowNum(2, 14, g_target_laps, 1);
        OLED_ShowString(4, 1, "   Go! Go! Go!");
        HAL_Delay(500);
        OLED_Clear();
        OLED_ShowString(1, 1, "Running... 0/");
        OLED_ShowNum(1, 14, g_target_laps, 1);
        OLED_ShowString(4, 1, "K1 Long = STOP");
        return;
    }

    /* 刷新圈数显示 */
    if (lap_display_dirty)
    {
        OLED_ShowNum(2, 11, g_target_laps, 1);
        lap_display_dirty = 0;
    }
}

/* ==================== 运行中 OLED 刷新 ==================== */

void Menu_UpdateRunInfo(void)
{
    if (!g_running)
        return;

    extern volatile int16_t last_speedL, last_speedR;
    char buf[17];

    /* 第2行：速度 */
    snprintf(buf, sizeof(buf), "L%+04d R%+04d", last_speedL, last_speedR);
    OLED_ShowString(2, 1, buf);

    /* 第3行：航向角 */
    float yaw = Yaw_GetAngle();
    snprintf(buf, sizeof(buf), "Yaw:%+06.1f", (double)yaw);
    OLED_ShowString(3, 1, buf);

    /* 第1行保留圈数显示（由 Menu_LapComplete 更新） */
}

/* ==================== 圈数完成 ==================== */

void Menu_LapComplete(void)
{
    if (!g_running)
        return;

    g_current_lap++;

    char buf[17];
    snprintf(buf, sizeof(buf), "Running... %d/%d", g_current_lap, g_target_laps);
    OLED_ShowString(1, 1, buf);

    if (g_current_lap >= g_target_laps)
    {
        g_running = 0;
        OLED_ShowString(1, 1, "!!! COMPLETE !!!");
        OLED_ShowString(2, 1, "All Laps Done! ");
        OLED_ShowString(3, 1, "Laps: ");
        OLED_ShowNum(3, 6, g_target_laps, 1);
        OLED_ShowString(4, 1, "RST to restart ");
    }
}
