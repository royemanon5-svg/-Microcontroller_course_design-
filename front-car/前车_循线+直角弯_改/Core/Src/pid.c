#include "main.h"
#include "PID.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "mpu6500.h"
extern UART_HandleTypeDef huart1;
// ==================== 用户可调参数 ====================
#define ANGLE_DERIVATIVE_LIMIT 55.0f
#define MAX_SPEED           50
#define MIN_SPEED           -50
#define MAX_CORRECTION      20
#define TRACK_BASE_SPEED    20

extern float ErrorLInt, ErrorRInt;

#define KP                   0.5f
#define KI                   0.0f
#define KD                   0.5f

#define LOST_COUNT_EDGE      3
#define LOST_COUNT_MAX       3

// 8路传感器权重（对称）
#define WEIGHT_L4   -85
#define WEIGHT_L3   -55
#define WEIGHT_L2   -30
#define WEIGHT_L1   -10
#define WEIGHT_R1    10
#define WEIGHT_R2    30
#define WEIGHT_R3    55
#define WEIGHT_R4    85
// ====================================================
static uint8_t  lost_count = 0;
static int16_t  last_valid_pos = 0;
static float gyroZ_bias_dynamic = 0.0f;  // 动态零偏估计
static uint16_t still_cnt = 0;
void Tracking_Reset(void)
{
    lost_count = 0;
    last_valid_pos = 0;
    PID_Init();
}

tPID PID_Tracking;
tPID PID_SpeedL, PID_SpeedR;
extern uint8_t LineFlag;  //灯控标志（在主函数中定义）

//static float OutSpeed;   PID输出修正量


// PID初始化
void PID_Init(void)
{
    PID_Tracking.Kp         = KP;
    PID_Tracking.Ki         = KI;
    PID_Tracking.Kd         = KD;
    PID_Tracking.SetPoint   = 0.0f;
    PID_Tracking.last_error = 0.0f;
    PID_Tracking.integral   = 0.0f;
	
	  // 速度环通常只用 P 和 I（D 项容易放大编码器噪声，通常设为 0）
    PID_SpeedL.Kp = 2.5; // 经验值，需根据电机特性调整
    PID_SpeedL.Ki = 1.3;
    PID_SpeedL.Kd = 0.005;
    
    PID_SpeedR.Kp = 5.0;
    PID_SpeedR.Ki = 1.3;
    PID_SpeedR.Kd = 0.005;
}

int16_t GetWeightedPosition(void)
{
    uint8_t count = 0;
    int16_t sum   = 0;

    if (SENSOR_L4 != GPIO_PIN_RESET) { sum += WEIGHT_L4; count++; }
    if (SENSOR_L3 != GPIO_PIN_RESET) { sum += WEIGHT_L3; count++; }
    if (SENSOR_L2 != GPIO_PIN_RESET) { sum += WEIGHT_L2; count++; }
    if (SENSOR_L1 != GPIO_PIN_RESET) { sum += WEIGHT_L1; count++; }
    if (SENSOR_R1 != GPIO_PIN_RESET) { sum += WEIGHT_R1; count++; }
    if (SENSOR_R2 != GPIO_PIN_RESET) { sum += WEIGHT_R2; count++; }
    if (SENSOR_R3 != GPIO_PIN_RESET) { sum += WEIGHT_R3; count++; }
    if (SENSOR_R4 != GPIO_PIN_RESET) { sum += WEIGHT_R4; count++; }

    if (count == 0)
    {
        return last_valid_pos;
    }
    else
    {
        last_valid_pos = sum / count + 40 ;
        return last_valid_pos;
    }
}

float PID_Caculate(tPID *pid, float error)
{
    // 1. 保留你的积分死区逻辑
    if (fabs(error) > 1.0f) 
    {
        pid->integral += error;
    }
		else
		{
			pid->integral *= 0.8f;
		}
		
		// 积分限幅
        if (pid->integral > 100.0f)  pid->integral = 100.0f;
        if (pid->integral < -100.0f) pid->integral = -100.0f;

    // 2. 保留你的微分低通滤波逻辑
    float current_derivative = error - pid->last_error;
    pid->last_derivative = 0.7f * pid->last_derivative + 0.3f * current_derivative;

    // 3. 计算输出
    float output = (pid->Kp * error) + 
                   (pid->Ki * pid->integral) + 
                   (pid->Kd * pid->last_derivative);

    // 4. 输出限幅（注意：速度环和位置环的限幅可能不同，建议由调用者处理或在结构体加变量）
    // 这里暂时保持原样，但建议根据 pid 指针判断或使用通用限幅

    pid->last_error = error;
    return output;
}

float Tracking_Task(int16_t* Speed)
{
    int16_t position = GetWeightedPosition();

    uint8_t all_lost = (
        SENSOR_L4 == GPIO_PIN_RESET &&
        SENSOR_L3 == GPIO_PIN_RESET &&
        SENSOR_L2 == GPIO_PIN_RESET &&
        SENSOR_L1 == GPIO_PIN_RESET &&
        SENSOR_R1 == GPIO_PIN_RESET &&
        SENSOR_R2 == GPIO_PIN_RESET &&
        SENSOR_R3 == GPIO_PIN_RESET &&
        SENSOR_R4 == GPIO_PIN_RESET
    ) ? 1 : 0;

    if (all_lost)
    {
        lost_count++;
        if (lost_count > LOST_COUNT_EDGE && abs(last_valid_pos) >= 40)
        {
            Speed[0] = 0;
            Speed[1] = 0;
            LineFlag = 1;
					
            return 0;
        }
        if (lost_count > LOST_COUNT_MAX)
        {
            Speed[0] = 0;
            Speed[1] = 0;
            LineFlag  = 1;
					  PID_SpeedL.integral = 0;
            PID_SpeedR.integral = 0;
            PID_SpeedL.last_error = 0;
            PID_SpeedR.last_error = 0;
            return 0;
        }
    }
    else
    {
        lost_count     = 0;
        last_valid_pos = position;
    }

    float correction = PID_Caculate(&PID_Tracking, 0.0f - (float)position);

    int16_t left_speed  = TRACK_BASE_SPEED - (int16_t)(correction + 0.5f);
    int16_t right_speed = TRACK_BASE_SPEED + (int16_t)(correction + 0.5f);

    if (left_speed  >  MAX_SPEED) left_speed  =  MAX_SPEED;
    if (left_speed  <  MIN_SPEED) left_speed  =  MIN_SPEED;
    if (right_speed >  MAX_SPEED) right_speed =  MAX_SPEED;
    if (right_speed <  MIN_SPEED) right_speed =  MIN_SPEED;

    Speed[0] = left_speed;
    Speed[1] = right_speed;
	

    return (float)position;
}




// ==================== 航向角积分 ====================
MPU6500_Data mpu_data;
static float yaw_angle = 0.0f;          // 当前航向角（度）
static uint32_t last_tick = 0;          // 上次更新时间戳
float gyroZ_offset = 0.0f;  // 零偏补偿值

// 陀螺仪量程±2000deg/s，对应灵敏度16.4 LSB/(deg/s)
#define GYRO_SENSITIVITY  16.4f


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



// 调用一次更新一次航向角，放在定时器中断或主循环里定期调用
/*(void Yaw_Update(void)
{
    MPU6500_Get_Data(&mpu_data);

    uint32_t now = HAL_GetTick();
    float dt = (now - last_tick) / 1000.0f;
    last_tick = now;

    float gyroZ_dps = (mpu_data.gyroZ - gyroZ_offset) / GYRO_SENSITIVITY;
    
    // 加静止死区，小于0.5deg/s认为静止不积分
    if (fabs(gyroZ_dps) > 0.5f)
    {
        yaw_angle += gyroZ_dps * dt;
        if      (yaw_angle >  180.0f) yaw_angle -= 360.0f;
        else if (yaw_angle < -180.0f) yaw_angle += 360.0f;
    }
	
}*/
void Yaw_Update(void)
{
   MPU6500_Get_Data(&mpu_data);
    
    float dt = 0.01f; // 固定10ms，不用HAL_GetTick
    
    float gyroZ_raw_dps = (mpu_data.gyroZ - gyroZ_offset) / GYRO_SENSITIVITY;
    
    // ---- 动态零漂估计 ----
    // 当两轮都几乎静止时，陀螺仪读数就是纯零漂
    // last_speedL/R 是你已有的全局编码器速度变量
    extern volatile int16_t last_speedL, last_speedR;
    if ((abs(last_speedL) < 5) && (abs(last_speedR) < 5) &&
        (fabsf(gyroZ_raw_dps) < 1.5f))
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

// 重置航向角（转弯前调用，把当前位置设为0）
void Yaw_Reset(void)
{
    yaw_angle = 0.0f;
    last_tick = HAL_GetTick();
}



// ==================== 角度PID（不变）====================
 float   angle_Kp = 0.8f;
 float   angle_Ki = 0.0f;
 float   angle_Kd = 2.0f;
 float   angle_output_limit = 200.0f;
static float   angle_integral = 0.0f;
static float   angle_last_error = 0.0f;
static float   angle_last_out    = 0.0f;
uint8_t angle_enabled = 0;
static float   angle_last_derivative = 0.0f;

void AnglePID_Init(float Kp, float Ki, float Kd, float output_limit)
{
    angle_Kp            = Kp;
    angle_Ki            = Ki;
    angle_Kd            = Kd;
    angle_output_limit  = output_limit;
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
    angle_last_out         = 0.0f;
}

void AnglePID_Disable(void)
{
    angle_enabled = 0;
}
void AnglePID_Process(int16_t base_speed,
                      float target_angle,
                      int16_t *speed_left, int16_t *speed_right)
{
    if (!angle_enabled)return;

    float current_angle = Yaw_GetAngle();

    float error = target_angle - current_angle;
    if      (error >  180.0f) error -= 360.0f;
    else if (error < -180.0f) error += 360.0f;

    if (fabs(error) > 1.0f)
    {
        angle_integral += error;
        if (angle_integral >  100.0f) angle_integral =  100.0f;
        if (angle_integral < -100.0f) angle_integral = -100.0f;
    }

    static uint8_t last_error_valid = 0;
    float derivative = 0;

    if (fabs(error) > 90.0f)
    {
        angle_last_error      = error;
        angle_last_derivative = 0;
        last_error_valid      = 0;
        derivative            = 0;
    }
    else
    {
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
    }

    derivative            = 0.6f * angle_last_derivative + 0.4f * derivative;
    angle_last_derivative = derivative;

    if (derivative >  ANGLE_DERIVATIVE_LIMIT) derivative =  ANGLE_DERIVATIVE_LIMIT;
    if (derivative < -ANGLE_DERIVATIVE_LIMIT) derivative = -ANGLE_DERIVATIVE_LIMIT;

    float out = angle_Kp * error + angle_Ki * angle_integral + angle_Kd * derivative;

#define MAX_OUT_CHANGE 40.0f
    float out_change = out - angle_last_out;
    if (out_change >  MAX_OUT_CHANGE) out = angle_last_out + MAX_OUT_CHANGE;
    if (out_change < -MAX_OUT_CHANGE) out = angle_last_out - MAX_OUT_CHANGE;
    angle_last_out = out;

    if (out >  angle_output_limit) out =  angle_output_limit;
    if (out < -angle_output_limit) out = -angle_output_limit;

    angle_last_error = error;

    *speed_left  = base_speed - (int16_t)(out + 0.5f);
    *speed_right = base_speed + (int16_t)(out + 0.5f);
		
    if (*speed_left  >  800) *speed_left  =  800;
    if (*speed_left  < -800) *speed_left  = -800;
    if (*speed_right >  800) *speed_right =  800;
    if (*speed_right < -800) *speed_right = -800;
}
/*void AnglePID_Process(int16_t base_speed, float target_angle, int16_t *speed_left, int16_t *speed_right)
{
		extern UART_HandleTypeDef huart1;
    
    if (!angle_enabled) {
        *speed_left = base_speed;
        *speed_right = base_speed;
        return;
    }
    
    float current_angle = Yaw_GetAngle();
    
    float error = target_angle - current_angle;
    float out = angle_Kp * error; 

    if (out > angle_output_limit) out = angle_output_limit;
    if (out < -angle_output_limit) out = -angle_output_limit;

    *speed_left  = (int16_t)(base_speed - out);
    *speed_right = (int16_t)(base_speed + out);
}*/

void AnglePID_ResetIntegral(void)
{
    angle_integral         = 0.0f;
    angle_last_error       = 0.0f;
    angle_last_derivative  = 0.0f;
}

float AngleError(float target, float current)
{
    float diff = target - current;
    diff = fmodf(diff, 360.0f);
    if      (diff >  180.0f) diff -= 360.0f;
    else if (diff < -180.0f) diff += 360.0f;
    return diff;
}

float AngleErrorRad(float target_rad, float current_rad)
{
    float diff = target_rad - current_rad;
    return atan2f(sinf(diff), cosf(diff));
}

