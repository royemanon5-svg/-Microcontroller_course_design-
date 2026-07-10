#ifndef __KEY_H
#define __KEY_H

#include "stdint.h"

/* 按键事件类型（与 menu.h 保持一致） */
typedef enum {
    KEY_EVENT_NONE = 0,
    KEY_EVENT_SHORT,     // 短按 (< 800ms)
    KEY_EVENT_LONG,      // 长按 (> 1000ms)
    KEY_EVENT_DOUBLE,    // 双击 (< 400ms 间隔)
} KeyEvent;

/* 旧的兼容标志（仍被部分代码引用） */
extern volatile uint8_t Task_Progress;
extern volatile uint8_t key_switch_flag;

/* 统一按键扫描（10ms周期调用，扫描 PA11 和 PA12） */
void Key_Scan(void);

/* 获取按键事件（读取后自动清除） */
KeyEvent Key1_GetEvent(void);  // KEY1 = PA11
KeyEvent Key2_GetEvent(void);  // KEY2 = PA12

#endif
