#include "main.h"
#include "PID.h"
#include <math.h>
#include <stdlib.h>
#include "mpu6500.h"
//================变量定义======================
uint16_t still_cnt = 0; //小车完全静止的持续计数
static float last_out = 0.0f; //上一次角度环 PID 的实际控制量
tPID PID_SpeedL, PID_SpeedR;
//================PID通用计算代码===============
float PID_Caculate(tPID *pid, float error)
{
    // 1. 积分死区
    if (fabs(error) > 1.0f) 
    {
        pid->integral += error;
    }
		else
		{
			pid->integral *= 0.8f;
		}
		// 积分限幅
        if (pid->integral > 25.0f)  pid->integral = 25.0f; //100
        if (pid->integral < -25.0f) pid->integral = -25.0f; //100
    // 2. 微分低通滤波
    float current_derivative = error - pid->last_error;
    pid->last_derivative = 0.7f * pid->last_derivative + 0.3f * current_derivative;
    // 3. 计算输出
    float output = (pid->Kp * error) + (pid->Ki * pid->integral) + (pid->Kd * pid->last_derivative);
    // 4. 输出限幅（注意：速度环和位置环的限幅可能不同，建议由调用者处理或在结构体加变量）
    pid->last_error = error;
    return output;
}
//================速度环===========================
void SpeedPID_Init(void)
{
	 // 速度环通常只用 P 和 I（D 项容易放大编码器噪声，通常设为 0）
    PID_SpeedL.Kp = SpeedL_Kp;
    PID_SpeedL.Ki = SpeedL_Ki; 
    PID_SpeedL.Kd = SpeedL_Kd; 
    
    PID_SpeedR.Kp = SpeedR_Kp; 
    PID_SpeedR.Ki = SpeedR_Ki; 
    PID_SpeedR.Kd = SpeedR_Kd; 
}
// ==================== 航向角积分 ====================
MPU6500_Data mpu_data;
static float yaw_angle = 0.0f;          // 当前航向角（度）
static uint32_t last_tick = 0;          // 上次更新时间戳
float gyroZ_offset = 0.0f;              // 零偏补偿值
float gyroZ_bias_dynamic = 0.0f;   //小车运行过程中，动态计算并更新的陀螺仪静止零偏值。
// 重置航向角（转弯前调用，把当前位置设为0）
void Yaw_Reset(void)
{
    yaw_angle = 0.0f;
    last_tick = HAL_GetTick();
}

void MPU6500_CalibrateGyro(void)
{
    int32_t sum = 0;
		HAL_Delay(1000);  // 等1秒确保静止
    for (int i = 0; i < 500; i++)
    {
        MPU6500_Get_Data(&mpu_data);
        sum += mpu_data.gyroZ;
        HAL_Delay(2);
    }
    gyroZ_offset = sum / 500.0f;  // 保存零偏
}
void Yaw_Update(void)
{ 	
		MPU6500_Get_Data(&mpu_data);
   float dt = 0.01f; // 固定10ms，不用HAL_GetTick
    
   float gyroZ_raw_dps = (mpu_data.gyroZ - gyroZ_offset) / GYRO_SENSITIVITY;
	
    // ---- 动态零漂估计 ----
    // 当两轮都几乎静止时，陀螺仪读数就是纯零漂
    // last_speedL/R 是你已有的全局编码器速度变量
    extern volatile int16_t last_speedL, last_speedR;
    if (abs(last_speedL) < 5 && abs(last_speedR) < 5)
    {
        still_cnt++;
        if (still_cnt > 30) // 静止超过300ms才更新，防止瞬间误判
        {
            // 低通滤波，缓慢跟踪真实零偏变化
            gyroZ_bias_dynamic = gyroZ_bias_dynamic * 0.98f 
                                + gyroZ_raw_dps * 0.02f;
        }
    }
    else
    {
        still_cnt = 0; // 一旦运动就停止更新
    }
    
    // ---- 用修正后的值积分 ----
    float gyroZ_corrected = gyroZ_raw_dps - gyroZ_bias_dynamic;
    
    // 死区：小于0.3DPS的抖动直接忽略（根据你的传感器噪声调整）
    if (gyroZ_corrected > -0.3f && gyroZ_corrected < 0.3f)
        gyroZ_corrected = 0.0f;
    
    yaw_angle += gyroZ_corrected * dt;
    
    if      (yaw_angle >  180.0f) yaw_angle -= 360.0f;
    else if (yaw_angle < -180.0f) yaw_angle += 360.0f;
		
}
// 获取当前航向角
float Yaw_GetAngle(void)
{
    return yaw_angle;
}
// ==================== 角度PID（相关代码）====================
static float   angle_integral = 0.0f; //积分项
static float   angle_last_error = 0.0f; //上一次微分项
static float   angle_last_derivative = 0.0f; //上一次经过低通滤波后的微分值
static float angle_output_limit = ANGLE_OUTPUT_LIMIT; //角度环限速
uint8_t angle_enabled = 0;
void AnglePID_Init(void)
{
    angle_integral      = 0.0f;
    angle_last_error    = 0.0f;
    angle_enabled       = 0;
}

void AnglePID_Enable(void)
{
    angle_enabled          = 1;
    angle_integral         = 0.0f;
    angle_last_error       = 0.0f;
    angle_last_derivative  = 0.0f; 
		last_out        = 0.0f;
}

void AnglePID_Disable(void)
{
    angle_enabled = 0;
}
//清空积分累计
void AnglePID_ResetIntegral(void)
{
    angle_integral         = 0.0f;
    angle_last_error       = 0.0f;
    angle_last_derivative  = 0.0f;
}
//过圆处理（角度制）
float AngleError(float target, float current)
{
    float diff = target - current;
    diff = fmodf(diff, 360.0f);
    if      (diff >  180.0f) diff -= 360.0f;
    else if (diff < -180.0f) diff += 360.0f;
    return diff;
}
//过圆处理（弧度制）
float AngleErrorRad(float target_rad, float current_rad)
{
    float diff = target_rad - current_rad;
    return atan2f(sinf(diff), cosf(diff));
}
void AnglePID_Process(int16_t base_speed,
                      float target_angle,
                      int16_t *speed_left, int16_t *speed_right)
{
		static float last_out = 0.0f;
		//开关检查
    if (!angle_enabled)
		{
			last_out = 0.0f;
			return;
		}
    float current_angle = Yaw_GetAngle();//获得当前的角度
    float error = target_angle - current_angle;// 目标 - 当前 = 角度误差
		// 角度过零处理（走近道）
    if      (error >  180.0f) error -= 360.0f;
    else if (error < -180.0f) error += 360.0f;
    if (fabs(error) > 1.0f)// 积分死区：误差大于1度才累加
    {
        angle_integral += error;
        if (angle_integral >  100.0f) angle_integral =  100.0f;// 积分限幅
        if (angle_integral < -100.0f) angle_integral = -100.0f;
    }

    static uint8_t last_error_valid = 0;
    float derivative = 0;

 if (!last_error_valid)
    {
      angle_last_error      = error;
      angle_last_derivative = 0;
      last_error_valid      = 1;
      derivative            = 0;
     }
  else
     {
       derivative = error - angle_last_error;
     }
		
    derivative            = 0.6f * angle_last_derivative + 0.4f * derivative;// 微分滤波
    angle_last_derivative = derivative;

    if (derivative >  ANGLE_DERIVATIVE_LIMIT) derivative =  ANGLE_DERIVATIVE_LIMIT;
    if (derivative < -ANGLE_DERIVATIVE_LIMIT) derivative = -ANGLE_DERIVATIVE_LIMIT;
		//PID公式
    float out = angle_Kp * error + angle_Ki * angle_integral + angle_Kd * derivative;
    float out_change = out - last_out;
		 // 2. 第一层：限制输出的变化率（斜率滤波）
    if (out_change >  MAX_OUT_CHANGE) out = last_out + MAX_OUT_CHANGE;
    if (out_change < -MAX_OUT_CHANGE) out = last_out - MAX_OUT_CHANGE;
    last_out = out;
		// 3.第二层：限制输出的最大绝对值
    if (out >  angle_output_limit) out =  angle_output_limit;
    if (out < -angle_output_limit) out = -angle_output_limit;
		 
    angle_last_error = error;
		//防止数据丢失
    *speed_left  = base_speed - (int16_t)(out + 0.5f);
    *speed_right = base_speed + (int16_t)(out + 0.5f);
		
    if (*speed_left  >  800) *speed_left  =  800;
    if (*speed_left  < -800) *speed_left  = -800;
    if (*speed_right >  800) *speed_right =  800;
    if (*speed_right < -800) *speed_right = -800;
}
