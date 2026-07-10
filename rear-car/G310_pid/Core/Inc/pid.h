#ifndef __PID_H
#define __PID_H
// =========== 调参区 ===============
#define ANGLE_DERIVATIVE_LIMIT 55.0f
#define TURN_MIN_PWM        170  //150 转弯的最小速度
#define TURN_MAX_PWM        190 //180 转弯的最大速度
#define ANGLE_OUTPUT_LIMIT  250.0f //180.0f 输出限幅
#define MAX_OUT_CHANGE 40.0f
// 陀螺仪量程±2000deg/s，对应灵敏度16.4 LSB/(deg/s)
#define GYRO_SENSITIVITY  16.4f
// 角度PID
// 2 0.02 0.3 
#define angle_Kp 2.0f  //2.0
#define angle_Ki 0.0f //0.00
#define angle_Kd 2.5f //2.5
//速度PID
#define SpeedL_Kp 4.0f // 经验值，需根据电机特性调整 2.5
#define SpeedL_Ki 0.8f // 1.3
#define SpeedL_Kd 0.01 //0.005
    
#define SpeedR_Kp 6.5f // 5.0
#define SpeedR_Ki 0.8f // 1.3
#define SpeedR_Kd 0.011 //0.005
// ====================================================
typedef struct {
    float Kp;           // 比例系数
    float Ki;           // 积分系数
    float Kd;           // 微分系数
    float SetPoint;     // 目标位置（通常为0，即中心）
    float last_error;   // 上一次误差
    float integral;     // 积分累加
	  float last_derivative;
} tPID;
//============速度环声明===============
void SpeedPID_Init(void);
extern tPID PID_SpeedL, PID_SpeedR;
float PID_Caculate(tPID *pid, float error);
//============角度环声明===============
void MPU6500_CalibrateGyro(void);
void Yaw_Update(void);
void Yaw_Reset(void);
void AnglePID_Init(void);
void AnglePID_Enable(void);
void AnglePID_Disable(void);
void AnglePID_Process(int16_t base_speed,
                      float target_angle,
                      int16_t *speed_left, int16_t *speed_right);
float Yaw_GetAngle(void);
extern float gyroZ_offset;
extern float gyroZ_bias_dynamic;
extern uint8_t angle_enabled;  
#endif
