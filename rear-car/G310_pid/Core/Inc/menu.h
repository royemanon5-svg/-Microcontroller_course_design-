#ifndef __MENU_H
#define __MENU_H

#include "stdint.h"

/* 菜单状态 */
typedef enum {
    MENU_SET_LAPS,       // 设置圈数界面
    MENU_RUNNING,        // 运行中
    MENU_STOPPED,        // 急停状态
} MenuState;

/* 圈数设置范围 */
#define LAPS_MIN     1
#define LAPS_MAX     9
#define LAPS_DEFAULT 1

/* 全局变量 */
extern volatile uint8_t g_target_laps;   // 目标圈数
extern volatile uint8_t g_current_lap;   // 当前已完成圈数
extern volatile uint8_t g_running;       // 运行标志: 0=菜单, 1=运行中

/* 函数声明 */
void Menu_Init(void);
void Menu_Process(void);
void Menu_UpdateRunInfo(void);
void Menu_LapComplete(void);            // 完成一圈时调用

#endif
