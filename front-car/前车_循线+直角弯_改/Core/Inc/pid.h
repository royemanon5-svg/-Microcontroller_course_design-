#ifndef __PID_H
#define __PID_H
// ==================== 8路模拟传感器宏定义 ====================
// ADC阈值和缓冲区由 adc.h 提供
#include "adc.h"

// 扫描顺序 = ADC通道号升序：IN4→IN5→IN10→IN11→IN12→IN13→IN14→IN15
// adc_values[0]=PA4/IN4=L4最左, ..., adc_values[7]=PC5/IN15=R4最右
#define SENSOR_L4  ((adc_values[0] < ADC_THRESHOLD) ? GPIO_PIN_SET : GPIO_PIN_RESET)  // PA4  IN4  最左
#define SENSOR_L3  ((adc_values[1] < ADC_THRESHOLD) ? GPIO_PIN_SET : GPIO_PIN_RESET)  // PA5  IN5
#define SENSOR_L2  ((adc_values[2] < ADC_THRESHOLD) ? GPIO_PIN_SET : GPIO_PIN_RESET)  // PC0  IN10
#define SENSOR_L1  ((adc_values[3] < ADC_THRESHOLD) ? GPIO_PIN_SET : GPIO_PIN_RESET)  // PC1  IN11
#define SENSOR_R1  ((adc_values[4] < ADC_THRESHOLD) ? GPIO_PIN_SET : GPIO_PIN_RESET)  // PC2  IN12
#define SENSOR_R2  ((adc_values[5] < ADC_THRESHOLD) ? GPIO_PIN_SET : GPIO_PIN_RESET)  // PC3  IN13
#define SENSOR_R3  ((adc_values[6] < ADC_THRESHOLD) ? GPIO_PIN_SET : GPIO_PIN_RESET)  // PC4  IN14
#define SENSOR_R4  ((adc_values[7] < ADC_THRESHOLD) ? GPIO_PIN_SET : GPIO_PIN_RESET)  // PC5  IN15 最右
// ===========================================================

/* ==================== 转弯参数 ==================== */
#define CORNER_FWD_TICKS  20
#define CORNER_FWD_PWM    180
#define TURN_STOP_TICKS   12
#define TURN_BRAKE_PWM    70
#define TURN_MIN_PWM      120
#define TURN_MAX_PWM      150
#define CORNER_LOCK_MS    300
#define CORNER_SUM_THRESH 4500

#define line_detected      (SENSOR_L4!=GPIO_PIN_RESET||SENSOR_L3!=GPIO_PIN_RESET||SENSOR_L2!=GPIO_PIN_RESET||SENSOR_L1!=GPIO_PIN_RESET||SENSOR_R1!=GPIO_PIN_RESET||SENSOR_R2!=GPIO_PIN_RESET||SENSOR_R3!=GPIO_PIN_RESET||SENSOR_R4!=GPIO_PIN_RESET)
#define center_on_line     (SENSOR_L1!=GPIO_PIN_RESET||SENSOR_R1!=GPIO_PIN_RESET)
// ===========================================================

typedef struct {
    float Kp;           // 比例系数
    float Ki;           // 积分系数
    float Kd;           // 微分系数
    float SetPoint;     // 目标位置（通常为0，即中心）
    float last_error;   // 上一次误差
    float integral;     // 积分累加
	  float last_derivative;
} tPID;
void Tracking_Reset(void);
void PID_Init(void);
int16_t GetWeightedPosition(void);
float Tracking_Task(int16_t* Speed);
extern tPID PID_SpeedL, PID_SpeedR;
extern float gyroZ_offset;
extern uint8_t angle_enabled;  // 不行就在pid.h里声明
float PID_Caculate(tPID *pid, float error);
void MPU6500_CalibrateGyro(void);
void Yaw_Update(void);
float Yaw_GetAngle(void);
void Yaw_Reset(void);
void AnglePID_Enable(void);
void AnglePID_Disable(void);
void AnglePID_Process(int16_t base_speed,
                      float target_angle,
                      int16_t *speed_left, int16_t *speed_right);
#endif
