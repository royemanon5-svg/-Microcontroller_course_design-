/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Rear car controller: NRF24L01 receive + HC-SR04 following
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include "hcsr04.h"
#include "I2C.h"
#include "motor.h"
#include "mpu6500.h"
#include "nrf24l01.h"
#include "OLED.h"
#include "pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    REAR_MODE_STOP = 0,
    REAR_MODE_BUFFERING,
    REAR_MODE_FOLLOWING
} RearFollowMode;

typedef struct {
    NRF24L01_Packet packet;
} TrackPoint;

typedef struct {
    NRF24L01_Packet radio;
    NRF24L01_Packet delayed_radio;
    uint16_t distance_cm;
    uint8_t distance_valid;
    uint8_t radio_ok;
    RearFollowMode mode;
    int16_t left_pwm;
    int16_t right_pwm;
} RearCarState;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FOLLOW_VALID_MIN_CM     3
#define FOLLOW_VALID_MAX_CM     80
#define FOLLOW_CONTROL_MIN_CM   10
//追赶范围的最大限制
#define FOLLOW_CONTROL_MAX_CM   30 //35
#define FOLLOW_START_DISTANCE_CM 20
#define FOLLOW_START_CONFIRM_N  3U
#define FOLLOW_DISTANCE_TARGET_CM 20
#define FOLLOW_DISTANCE_DEADBAND_CM 2
//间距控制Kp
#define FOLLOW_DISTANCE_KP      2.0f//8.0f
#define FOLLOW_DISTANCE_MAX_PWM 80
#define FOLLOW_DISTANCE_STABLE_N 2U
#define FOLLOW_DISTANCE_MAX_JUMP_CM 4U
#define FOLLOW_MAX_BASE_PWM     999
#define FOLLOW_MAX_TURN_PWM     999
#define FOLLOW_MAX_HEADING_PWM  120
#define FOLLOW_MIN_DRIVE_PWM    20
#define FOLLOW_TURN_DETECT_PWM  40
#define FOLLOW_PIVOT_TURN_PWM   60
#define FOLLOW_CORNER_APPROACH_MAX_PWM 120
#define FOLLOW_PREVIEW_DISTANCE_MM 5U
#define FOLLOW_HEADING_TARGET_PREVIEW_MM 20U
#define FOLLOW_STRAIGHT_STABLE_PACKETS 6U
#define FOLLOW_STRAIGHT_STABLE_YAW_DEG 2.0f
#define FOLLOW_EFFECTIVE_TRACK_MM 110.0f
#define FOLLOW_CURVATURE_BLEND_PERCENT 10
#define FOLLOW_CURVATURE_MIN_YAW_DEG 0.5f
#define FOLLOW_TURN_RISE_SLEW_PWM 25
#define FOLLOW_TURN_FALL_SLEW_PWM 80
#define FOLLOW_TURN_BRAKE_ANGLE_DEG 25.0f
//后车直角弯转的角度
#define FOLLOW_RIGHT_ANGLE_GYRO_TARGET_DEG 73.0f
#define FOLLOW_RIGHT_ANGLE_STOP_ERROR_DEG 3.0f
#define FOLLOW_RIGHT_ANGLE_SETTLE_N 5U
#define FRONT_YAW_SIGN          1.0f
#define REAR_YAW_SIGN           1.0f
#define FOLLOW_CONTROL_MS       10U
#define FOLLOW_DISTANCE_MS      50U
#define FOLLOW_OLED_MS          200U
#define TRACK_BUFFER_SIZE       256U
/* 22.5 mm wheel, 13 PPR, 20.409 gearbox and TIM encoder TI1 x2. */
#define FRONT_TICKS_TO_MM_NUM   141372U
#define FRONT_TICKS_TO_MM_DEN   530634U
#define REAR_TICKS_TO_MM_NUM    141372U
#define REAR_TICKS_TO_MM_DEN    530634U
/* Calibrate odometry here instead of changing the rear-car PWM. */
#define REAR_PATH_SCALE_NUM     1000U
#define REAR_PATH_SCALE_DEN     1000U
#define FOLLOW_PLAYBACK_DELAY_MM 200U //150
#define FOLLOW_RIGHT_ANGLE_EXTRA_MM 50U
#define FOLLOW_RIGHT_ANGLE_EXTRA_TICKS \
    ((FOLLOW_RIGHT_ANGLE_EXTRA_MM * FRONT_TICKS_TO_MM_DEN + \
      FRONT_TICKS_TO_MM_NUM / 2U) / FRONT_TICKS_TO_MM_NUM)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
volatile int16_t last_speedL = 0;
volatile int16_t last_speedR = 0;
static RearCarState car = {0};
static TrackPoint track_buffer[TRACK_BUFFER_SIZE];
static uint16_t track_head = 0U;
static uint16_t track_tail = 0U;
static uint16_t track_count = 0U;
static NRF24L01_Packet playback_anchor = {0};
static uint8_t playback_anchor_valid = 0U;
static uint8_t front_motion_seen = 0U;
static volatile uint8_t playback_active = 0U;
static uint8_t start_distance_count = 0U;
static volatile uint32_t rear_path_ticks = 0U;
static volatile uint32_t rear_odometer_ticks = 0U;
static volatile uint8_t rear_path_half_tick = 0U;
static volatile uint8_t rear_odometer_half_tick = 0U;
static uint64_t front_path_origin_um = 0U;
static uint8_t distance_stable_count = 0U;
static uint16_t last_raw_distance_cm = 0U;
static int16_t distance_trim_pwm = 0;
static int16_t last_turn_feedforward_pwm = 0;
static uint64_t playback_target_front_um = 0U;
static float rear_target_yaw = 0.0f;
static float front_yaw_reference = 0.0f;
static float rear_yaw_reference = 0.0f;
static uint8_t heading_reference_valid = 0U;
static uint8_t right_angle_approach_active = 0U;
static uint32_t right_angle_path_offset_ticks = 0U;
static uint8_t last_live_right_angle_flag = 0U;
static uint8_t right_angle_turn_active = 0U;
static uint8_t right_angle_finish_pending = 0U;
static uint8_t right_angle_settle_count = 0U;
static float right_angle_target_yaw = 0.0f;
static uint8_t last_curve_turning_command = 0U;
static uint8_t curve_exit_target_pending = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
static int16_t clamp_i16(int32_t value, int16_t min_value, int16_t max_value);
static uint32_t ticks_to_mm(uint32_t ticks, uint32_t numerator,
                            uint32_t denominator);
static uint64_t ticks_to_um(uint32_t ticks, uint32_t numerator,
                            uint32_t denominator);
static int16_t interpolate_i16(int16_t start, int16_t end,
                               uint64_t numerator, uint64_t denominator);
static int16_t interpolate_yaw_x10(int16_t start, int16_t end,
                                   uint64_t numerator, uint64_t denominator);
static float angle_error_deg(float target, float current);
static float normalize_angle_deg(float angle);
static uint8_t RearCar_IsDistanceValid(uint16_t distance_cm);
static void TrackBuffer_Reset(void);
static void TrackBuffer_Push(const NRF24L01_Packet *packet);
static uint8_t TrackBuffer_StartPlayback(void);
static uint8_t TrackBuffer_GetPlayback(NRF24L01_Packet *packet);
static uint8_t TrackBuffer_GetYawAt(uint64_t target_front_um,
                                    int16_t *yaw_x10);
static uint8_t TrackBuffer_GetStableStraightYaw(int16_t *yaw_x10);
static uint8_t TrackBuffer_FinishRightAngle(void);
static void RearCar_RecordPacket(const NRF24L01_Packet *packet);
static uint8_t RearCar_CanUseUltrasonic(uint8_t turning_command);
static void RearCar_UpdateDistanceTrim(uint8_t trusted);
static void RearCar_ControlTask(void);
static void RearCar_UpdateOLED(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static int16_t clamp_i16(int32_t value, int16_t min_value, int16_t max_value)
{
    if (value > max_value) {
        return max_value;
    }
    if (value < min_value) {
        return min_value;
    }
    return (int16_t)value;
}

static uint32_t ticks_to_mm(uint32_t ticks, uint32_t numerator,
                            uint32_t denominator)
{
    return (uint32_t)((((uint64_t)ticks * numerator) +
                       (denominator / 2U)) / denominator);
}

static uint64_t ticks_to_um(uint32_t ticks, uint32_t numerator,
                            uint32_t denominator)
{
    return (((uint64_t)ticks * numerator * 1000U) +
            (denominator / 2U)) / denominator;
}

static int16_t interpolate_i16(int16_t start, int16_t end,
                               uint64_t numerator, uint64_t denominator)
{
    int64_t delta;
    int64_t value;

    if ((denominator == 0U) || (numerator >= denominator)) {
        return end;
    }

    delta = (int64_t)end - start;
    value = (int64_t)start + (delta * (int64_t)numerator) /
                              (int64_t)denominator;
    return (int16_t)value;
}

static int16_t interpolate_yaw_x10(int16_t start, int16_t end,
                                   uint64_t numerator, uint64_t denominator)
{
    int32_t delta = (int32_t)end - start;
    int32_t value;

    while (delta > 1800) {
        delta -= 3600;
    }
    while (delta < -1800) {
        delta += 3600;
    }
    if ((denominator == 0U) || (numerator >= denominator)) {
        value = (int32_t)start + delta;
    } else {
        value = (int32_t)start +
                (int32_t)(((int64_t)delta * (int64_t)numerator) /
                          (int64_t)denominator);
    }

    while (value > 1800) {
        value -= 3600;
    }
    while (value < -1800) {
        value += 3600;
    }
    return (int16_t)value;
}

static float angle_error_deg(float target, float current)
{
    float diff = target - current;

    while (diff > 180.0f) {
        diff -= 360.0f;
    }
    while (diff < -180.0f) {
        diff += 360.0f;
    }

    return diff;
}

static float normalize_angle_deg(float angle)
{
    while (angle > 180.0f) {
        angle -= 360.0f;
    }
    while (angle < -180.0f) {
        angle += 360.0f;
    }

    return angle;
}

static uint8_t RearCar_IsDistanceValid(uint16_t distance_cm)
{
    if (distance_cm == HCSR04_INVALID_DISTANCE_CM) {
        return 0U;
    }
    if (distance_cm < FOLLOW_VALID_MIN_CM) {
        return 0U;
    }
    if (distance_cm > FOLLOW_VALID_MAX_CM) {
        return 0U;
    }

    return 1U;
}

static void TrackBuffer_Reset(void)
{
    track_head = 0U;
    track_tail = 0U;
    track_count = 0U;
    playback_anchor_valid = 0U;
    playback_active = 0U;
    rear_path_ticks = 0U;
    rear_path_half_tick = 0U;
    distance_trim_pwm = 0;
    last_turn_feedforward_pwm = 0;
    playback_target_front_um = 0U;
    right_angle_approach_active = 0U;
    right_angle_path_offset_ticks = 0U;
    last_live_right_angle_flag = 0U;
    right_angle_turn_active = 0U;
    right_angle_finish_pending = 0U;
    right_angle_settle_count = 0U;
    right_angle_target_yaw = 0.0f;
    last_curve_turning_command = 0U;
    curve_exit_target_pending = 0U;
}

static void TrackBuffer_Push(const NRF24L01_Packet *packet)
{
    track_buffer[track_head].packet = *packet;
    track_head = (uint16_t)((track_head + 1U) % TRACK_BUFFER_SIZE);

    if (track_count < TRACK_BUFFER_SIZE) {
        track_count++;
    } else {
        /* Keep the newest trajectory if recording lasts beyond buffer capacity. */
        track_tail = (uint16_t)((track_tail + 1U) % TRACK_BUFFER_SIZE);
    }
}

static uint8_t TrackBuffer_StartPlayback(void)
{
    if (track_count == 0U) {
        return 0U;
    }

    front_path_origin_um = ticks_to_um(track_buffer[track_tail].packet.path_ticks,
                                      FRONT_TICKS_TO_MM_NUM,
                                      FRONT_TICKS_TO_MM_DEN);
    rear_path_ticks = 0U;
    rear_path_half_tick = 0U;
    playback_anchor_valid = 0U;
    playback_active = 1U;
    return 1U;
}

static uint8_t TrackBuffer_GetPlayback(NRF24L01_Packet *packet)
{
    uint64_t rear_um;
    uint64_t scaled_rear_um;
    uint64_t playback_delay_um;
    uint64_t target_front_um;

    if ((playback_active == 0U) || (packet == 0)) {
        return 0U;
    }

    rear_um = ticks_to_um(rear_path_ticks,
                         REAR_TICKS_TO_MM_NUM,
                         REAR_TICKS_TO_MM_DEN);
    scaled_rear_um = ((rear_um * REAR_PATH_SCALE_NUM) +
                      (REAR_PATH_SCALE_DEN / 2U)) /
                     REAR_PATH_SCALE_DEN;
    playback_delay_um = (uint64_t)FOLLOW_PLAYBACK_DELAY_MM * 1000U;
    target_front_um = front_path_origin_um;
    if (scaled_rear_um > playback_delay_um) {
        target_front_um += scaled_rear_um - playback_delay_um;
    }
    playback_target_front_um = target_front_um;

    /* Execute each front command when the rear car reaches the same wheel path. */
    while (track_count > 0U) {
        TrackPoint *point = &track_buffer[track_tail];

        uint64_t point_um = ticks_to_um(point->packet.path_ticks,
                                       FRONT_TICKS_TO_MM_NUM,
                                       FRONT_TICKS_TO_MM_DEN);

        if (point_um > target_front_um) {
            break;
        }

        playback_anchor = point->packet;
        playback_anchor_valid = 1U;
        track_tail = (uint16_t)((track_tail + 1U) % TRACK_BUFFER_SIZE);
        track_count--;
    }

    if (playback_anchor_valid == 0U) {
        return 0U;
    }

    *packet = playback_anchor;
    if (track_count > 0U) {
        const NRF24L01_Packet *next = &track_buffer[track_tail].packet;
        uint64_t anchor_um = ticks_to_um(playback_anchor.path_ticks,
                                         FRONT_TICKS_TO_MM_NUM,
                                         FRONT_TICKS_TO_MM_DEN);
        uint64_t next_um = ticks_to_um(next->path_ticks,
                                       FRONT_TICKS_TO_MM_NUM,
                                       FRONT_TICKS_TO_MM_DEN);

        if ((next_um > anchor_um) && (target_front_um > anchor_um) &&
            !(((next->flags & FRONT_CAR_FLAG_RIGHT_ANGLE) != 0U) &&
              ((playback_anchor.flags &
                FRONT_CAR_FLAG_RIGHT_ANGLE) == 0U))) {
            uint64_t position = target_front_um - anchor_um;
            uint64_t span = next_um - anchor_um;

            if (position > span) {
                position = span;
            }
            packet->speed = interpolate_i16(playback_anchor.speed, next->speed,
                                            position, span);
            packet->turn = interpolate_i16(playback_anchor.turn, next->turn,
                                           position, span);
            packet->yaw = interpolate_yaw_x10(playback_anchor.yaw, next->yaw,
                                              position, span);
        }
    }

    return 1U;
}

static uint8_t TrackBuffer_GetYawAt(uint64_t target_front_um,
                                    int16_t *yaw_x10)
{
    NRF24L01_Packet lower;
    uint64_t lower_um;
    uint16_t index;
    uint16_t remaining;

    if ((yaw_x10 == 0) || (playback_anchor_valid == 0U)) {
        return 0U;
    }

    lower = playback_anchor;
    lower_um = ticks_to_um(lower.path_ticks,
                           FRONT_TICKS_TO_MM_NUM,
                           FRONT_TICKS_TO_MM_DEN);
    index = track_tail;
    remaining = track_count;

    while (remaining > 0U) {
        const NRF24L01_Packet *upper = &track_buffer[index].packet;
        uint64_t upper_um = ticks_to_um(upper->path_ticks,
                                        FRONT_TICKS_TO_MM_NUM,
                                        FRONT_TICKS_TO_MM_DEN);

        if (upper_um > target_front_um) {
            if ((upper_um > lower_um) && (target_front_um > lower_um)) {
                *yaw_x10 = interpolate_yaw_x10(
                    lower.yaw, upper->yaw,
                    target_front_um - lower_um,
                    upper_um - lower_um);
            } else {
                *yaw_x10 = lower.yaw;
            }
            return 1U;
        }

        lower = *upper;
        lower_um = upper_um;
        index = (uint16_t)((index + 1U) % TRACK_BUFFER_SIZE);
        remaining--;
    }

    *yaw_x10 = lower.yaw;
    return 1U;
}

static uint8_t TrackBuffer_GetStableStraightYaw(int16_t *yaw_x10)
{
    uint16_t index = track_tail;
    uint16_t remaining = track_count;
    uint8_t stable_count = 0U;
    float reference_yaw = 0.0f;
    float yaw_sum = 0.0f;

    if (yaw_x10 == 0) {
        return 0U;
    }

    while (remaining > 0U) {
        const NRF24L01_Packet *point = &track_buffer[index].packet;
        int16_t turn_abs = (point->turn < 0) ?
            (int16_t)(-(int32_t)point->turn) : point->turn;

        if (((point->flags & FRONT_CAR_FLAG_RIGHT_ANGLE) == 0U) &&
            (turn_abs < FOLLOW_TURN_DETECT_PWM)) {
            float point_yaw = FRONT_YAW_SIGN *
                              ((float)point->yaw / 10.0f);

            if (stable_count == 0U) {
                reference_yaw = point_yaw;
                yaw_sum = point_yaw;
                stable_count = 1U;
            } else {
                float yaw_delta = angle_error_deg(point_yaw,
                                                  reference_yaw);

                if (fabsf(yaw_delta) <= FOLLOW_STRAIGHT_STABLE_YAW_DEG) {
                    yaw_sum += reference_yaw + yaw_delta;
                    stable_count++;
                } else {
                    reference_yaw = point_yaw;
                    yaw_sum = point_yaw;
                    stable_count = 1U;
                }
            }

            if (stable_count >= FOLLOW_STRAIGHT_STABLE_PACKETS) {
                float average_yaw = normalize_angle_deg(
                    yaw_sum / (float)stable_count);
                *yaw_x10 = (int16_t)(average_yaw * 10.0f +
                    ((average_yaw >= 0.0f) ? 0.5f : -0.5f));
                return 1U;
            }
        } else {
            stable_count = 0U;
            yaw_sum = 0.0f;
        }

        index = (uint16_t)((index + 1U) % TRACK_BUFFER_SIZE);
        remaining--;
    }

    return 0U;
}

static uint8_t TrackBuffer_FinishRightAngle(void)
{
    if ((playback_anchor_valid != 0U) &&
        ((playback_anchor.flags & FRONT_CAR_FLAG_RIGHT_ANGLE) == 0U)) {
        return 1U;
    }

    while (track_count > 0U) {
        const NRF24L01_Packet *point = &track_buffer[track_tail].packet;

        if ((point->flags & FRONT_CAR_FLAG_RIGHT_ANGLE) == 0U) {
            playback_anchor = *point;
            playback_anchor_valid = 1U;
            track_tail = (uint16_t)((track_tail + 1U) % TRACK_BUFFER_SIZE);
            track_count--;
            return 1U;
        }

        track_tail = (uint16_t)((track_tail + 1U) % TRACK_BUFFER_SIZE);
        track_count--;
    }

    return 0U;
}

static void RearCar_RecordPacket(const NRF24L01_Packet *packet)
{
    int16_t speed_abs = (packet->speed < 0) ?
                        (int16_t)(-(int32_t)packet->speed) : packet->speed;
    int16_t turn_abs = (packet->turn < 0) ?
                       (int16_t)(-(int32_t)packet->turn) : packet->turn;
    NRF24L01_Packet buffered_packet;
    uint8_t right_angle_flag;

    car.radio = *packet;
    if (front_motion_seen == 0U) {
        if ((speed_abs >= FOLLOW_MIN_DRIVE_PWM) ||
            (turn_abs >= FOLLOW_PIVOT_TURN_PWM)) {
            TrackBuffer_Reset();
            front_motion_seen = 1U;
        }
    }

    if (front_motion_seen != 0U) {
        buffered_packet = *packet;
        right_angle_flag =
            ((packet->flags & FRONT_CAR_FLAG_RIGHT_ANGLE) != 0U) ? 1U : 0U;
        if ((right_angle_flag != 0U) &&
            (last_live_right_angle_flag == 0U)) {
            right_angle_path_offset_ticks +=
                FOLLOW_RIGHT_ANGLE_EXTRA_TICKS;
        }
        last_live_right_angle_flag = right_angle_flag;
        buffered_packet.path_ticks += right_angle_path_offset_ticks;
        TrackBuffer_Push(&buffered_packet);
    }

    if ((packet->flags & FRONT_CAR_FLAG_RIGHT_ANGLE) != 0U) {
        right_angle_approach_active = 1U;
    }
}

static uint8_t RearCar_CanUseUltrasonic(uint8_t turning_command)
{
    if ((turning_command != 0U) || (car.distance_valid == 0U) ||
        (distance_stable_count < FOLLOW_DISTANCE_STABLE_N) ||
        (car.distance_cm < FOLLOW_CONTROL_MIN_CM) ||
        (car.distance_cm > FOLLOW_CONTROL_MAX_CM)) {
        return 0U;
    }
    return 1U;
}

static void RearCar_UpdateDistanceTrim(uint8_t trusted)
{
    int16_t target_trim = 0;
    int16_t error_cm;
    int16_t delta;
    int16_t step;

    if (trusted != 0U) {
        error_cm = (int16_t)car.distance_cm - FOLLOW_DISTANCE_TARGET_CM;
        if ((error_cm >= -FOLLOW_DISTANCE_DEADBAND_CM) &&
            (error_cm <= FOLLOW_DISTANCE_DEADBAND_CM)) {
            error_cm = 0;
        }
        target_trim = clamp_i16((int32_t)((float)error_cm * FOLLOW_DISTANCE_KP),
                                -FOLLOW_DISTANCE_MAX_PWM,
                                FOLLOW_DISTANCE_MAX_PWM);
    }

    delta = (int16_t)(target_trim - distance_trim_pwm);
    step = (int16_t)(delta / 4);
    if ((step == 0) && (delta != 0)) {
        step = (delta > 0) ? 1 : -1;
    }
    distance_trim_pwm = (int16_t)(distance_trim_pwm + step);
}

static void RearCar_Stop(void)
{
    car.left_pwm = 0;
    car.right_pwm = 0;
    car.mode = REAR_MODE_STOP;
    Motor_SetPWM(0, 0);
    HeadingPID_Reset();
    last_turn_feedforward_pwm = 0;
    last_curve_turning_command = 0U;
    curve_exit_target_pending = 0U;
}

static void RearCar_ControlTask(void)
{
    float front_yaw;
    float preview_front_yaw;
    float preview_yaw_delta;
    float heading_preview_front_yaw;
    float stable_straight_front_yaw;
    float curvature_turn;
    float heading_error;
    float heading_pwm;
    float right_angle_error;
    float right_angle_error_abs;
    int16_t base_pwm;
    int16_t path_base_abs;
    int16_t raw_turn_pwm;
    int16_t curvature_turn_pwm;
    int16_t target_turn_pwm;
    int16_t turn_pwm;
    int16_t turn_slew_delta;
    int16_t turn_slew_limit;
    int16_t steering_pwm;
    int16_t steering_limit;
    int16_t base_abs;
    int16_t raw_turn_abs;
    int16_t right_angle_turn_pwm;
    int16_t preview_yaw_x10;
    int16_t heading_preview_yaw_x10;
    int16_t stable_straight_yaw_x10;
    uint8_t preview_valid;
    uint8_t heading_preview_valid;
    uint8_t stable_straight_valid;
    uint8_t turning_command;
    uint8_t pivot_command;
    int32_t left;
    int32_t right;
    uint32_t now = HAL_GetTick();
    const NRF24L01_Packet *delayed_packet = &car.delayed_radio;

    car.radio_ok = NRF24L01_IsConnected(now);
    if (!car.radio_ok) {
        heading_reference_valid = 0U;
        front_motion_seen = 0U;
        start_distance_count = 0U;
        TrackBuffer_Reset();
        RearCar_Stop();
        return;
    }

    if ((front_motion_seen == 0U) || (playback_active == 0U)) {
        if ((front_motion_seen != 0U) && car.distance_valid &&
            (distance_stable_count >= FOLLOW_DISTANCE_STABLE_N) &&
            (car.distance_cm >= FOLLOW_START_DISTANCE_CM)) {
            if (start_distance_count < FOLLOW_START_CONFIRM_N) {
                start_distance_count++;
            }
        } else {
            start_distance_count = 0U;
        }

        if ((front_motion_seen != 0U) &&
            (start_distance_count >= FOLLOW_START_CONFIRM_N)) {
            if (TrackBuffer_StartPlayback() == 0U) {
                RearCar_Stop();
                car.mode = REAR_MODE_BUFFERING;
                return;
            }
        } else {
            RearCar_Stop();
            car.mode = REAR_MODE_BUFFERING;
            return;
        }
    }

    (void)TrackBuffer_GetPlayback(&car.delayed_radio);
    if (playback_active == 0U) {
        RearCar_Stop();
        car.mode = REAR_MODE_BUFFERING;
        return;
    }

    /* After the 20 cm start gate, rear distance selects the front path point. */
    base_pwm = clamp_i16(delayed_packet->speed,
                         -FOLLOW_MAX_BASE_PWM,
                         FOLLOW_MAX_BASE_PWM);
    car.mode = REAR_MODE_FOLLOWING;

    front_yaw = FRONT_YAW_SIGN * ((float)delayed_packet->yaw / 10.0f);
    preview_yaw_x10 = delayed_packet->yaw;
    preview_valid = TrackBuffer_GetYawAt(
        playback_target_front_um +
        (uint64_t)FOLLOW_PREVIEW_DISTANCE_MM * 1000U,
        &preview_yaw_x10);
    preview_front_yaw = FRONT_YAW_SIGN * ((float)preview_yaw_x10 / 10.0f);
    preview_yaw_delta = angle_error_deg(preview_front_yaw, front_yaw);
    heading_preview_yaw_x10 = delayed_packet->yaw;
    heading_preview_valid = TrackBuffer_GetYawAt(
        playback_target_front_um +
        (uint64_t)FOLLOW_HEADING_TARGET_PREVIEW_MM * 1000U,
        &heading_preview_yaw_x10);
    heading_preview_front_yaw = FRONT_YAW_SIGN *
                                ((float)heading_preview_yaw_x10 / 10.0f);

    raw_turn_pwm = clamp_i16(delayed_packet->turn,
                             -FOLLOW_MAX_TURN_PWM,
                             FOLLOW_MAX_TURN_PWM);
    raw_turn_abs = (raw_turn_pwm < 0) ?
                   (int16_t)(-(int32_t)raw_turn_pwm) : raw_turn_pwm;

    if (right_angle_finish_pending != 0U) {
        if (TrackBuffer_FinishRightAngle() == 0U) {
            RearCar_Stop();
            return;
        }

        car.delayed_radio = playback_anchor;
        front_yaw_reference = FRONT_YAW_SIGN *
                              ((float)playback_anchor.yaw / 10.0f);
        rear_yaw_reference = right_angle_target_yaw;
        rear_target_yaw = right_angle_target_yaw;
        heading_reference_valid = 1U;
        right_angle_finish_pending = 0U;
        right_angle_approach_active = 0U;
        HeadingPID_Reset();
        last_turn_feedforward_pwm = 0;
        RearCar_Stop();
        return;
    }

    if (((delayed_packet->flags & FRONT_CAR_FLAG_RIGHT_ANGLE) != 0U) &&
        (right_angle_turn_active == 0U)) {
        right_angle_target_yaw = normalize_angle_deg(
            REAR_YAW_SIGN * Yaw_GetAngle() +
            ((raw_turn_pwm >= 0) ? FOLLOW_RIGHT_ANGLE_GYRO_TARGET_DEG :
                                   -FOLLOW_RIGHT_ANGLE_GYRO_TARGET_DEG));
        rear_target_yaw = right_angle_target_yaw;
        right_angle_turn_active = 1U;
        right_angle_settle_count = 0U;
        distance_trim_pwm = 0;
        last_turn_feedforward_pwm = 0;
        HeadingPID_Reset();
    }

    if (right_angle_turn_active != 0U) {
        right_angle_error = angle_error_deg(
            right_angle_target_yaw, REAR_YAW_SIGN * Yaw_GetAngle());
        right_angle_error_abs = fabsf(right_angle_error);
        right_angle_turn_pwm = 0;

        if (right_angle_error_abs <= FOLLOW_RIGHT_ANGLE_STOP_ERROR_DEG) {
            if (right_angle_settle_count < FOLLOW_RIGHT_ANGLE_SETTLE_N) {
                right_angle_settle_count++;
            }
        } else {
            right_angle_settle_count = 0U;
            right_angle_turn_pwm = (right_angle_error > 0.0f) ?
                                   TURN_MAX_PWM : -TURN_MAX_PWM;
            if (right_angle_error_abs <= FOLLOW_TURN_BRAKE_ANGLE_DEG) {
                right_angle_turn_pwm = (right_angle_error > 0.0f) ?
                                       TURN_MIN_PWM : -TURN_MIN_PWM;
            }
        }

        if (right_angle_settle_count >= FOLLOW_RIGHT_ANGLE_SETTLE_N) {
            right_angle_turn_active = 0U;
            right_angle_finish_pending = 1U;
            right_angle_settle_count = 0U;
            rear_target_yaw = right_angle_target_yaw;
            HeadingPID_Reset();
            last_turn_feedforward_pwm = 0;
            RearCar_Stop();
            return;
        }

        car.left_pwm = (int16_t)-right_angle_turn_pwm;
        car.right_pwm = right_angle_turn_pwm;
        car.mode = REAR_MODE_FOLLOWING;
        Motor_SetPWM(car.left_pwm, car.right_pwm);
        return;
    }

    if ((delayed_packet->flags & FRONT_CAR_FLAG_RIGHT_ANGLE) != 0U) {
        right_angle_approach_active = 0U;
    }
    turning_command = (((delayed_packet->flags &
                         FRONT_CAR_FLAG_RIGHT_ANGLE) != 0U) ||
                       (raw_turn_abs >= FOLLOW_TURN_DETECT_PWM) ||
                       ((preview_valid != 0U) &&
                        (fabsf(preview_yaw_delta) >=
                         FOLLOW_CURVATURE_MIN_YAW_DEG))) ? 1U : 0U;


    if (heading_reference_valid == 0U) {
        front_yaw_reference = front_yaw;
        rear_yaw_reference = REAR_YAW_SIGN * Yaw_GetAngle();
        rear_target_yaw = rear_yaw_reference;
        heading_reference_valid = 1U;
    }

    stable_straight_valid = 0U;
    if (turning_command != 0U) {
        curve_exit_target_pending = 0U;
        rear_target_yaw = normalize_angle_deg(rear_yaw_reference +
            angle_error_deg(front_yaw, front_yaw_reference));
    } else {
        if (last_curve_turning_command != 0U) {
            curve_exit_target_pending = 1U;
            HeadingPID_Reset();
        }

        if (curve_exit_target_pending != 0U) {
            stable_straight_valid = TrackBuffer_GetStableStraightYaw(
                &stable_straight_yaw_x10);
            if (stable_straight_valid != 0U) {
                stable_straight_front_yaw =
                    (float)stable_straight_yaw_x10 / 10.0f;
                rear_target_yaw = normalize_angle_deg(rear_yaw_reference +
                    angle_error_deg(stable_straight_front_yaw,
                                    front_yaw_reference));
                front_yaw_reference = stable_straight_front_yaw;
                rear_yaw_reference = rear_target_yaw;
                curve_exit_target_pending = 0U;
                HeadingPID_Reset();
            } else {
                rear_target_yaw = normalize_angle_deg(rear_yaw_reference +
                    angle_error_deg((heading_preview_valid != 0U) ?
                                    heading_preview_front_yaw : front_yaw,
                                    front_yaw_reference));
            }
        }
    }
    last_curve_turning_command = turning_command;
    heading_error = angle_error_deg(rear_target_yaw,
                                    REAR_YAW_SIGN * Yaw_GetAngle());

    path_base_abs = (base_pwm < 0) ? (int16_t)-base_pwm : base_pwm;
    pivot_command = (((delayed_packet->flags &
                       FRONT_CAR_FLAG_RIGHT_ANGLE) != 0U) ||
                     ((path_base_abs < FOLLOW_MIN_DRIVE_PWM) &&
                      (raw_turn_abs >= FOLLOW_PIVOT_TURN_PWM))) ? 1U : 0U;
    curvature_turn_pwm = raw_turn_pwm;
    if ((preview_valid != 0U) &&
        (path_base_abs >= FOLLOW_MIN_DRIVE_PWM) &&
        (fabsf(preview_yaw_delta) >= FOLLOW_CURVATURE_MIN_YAW_DEG)) {
        curvature_turn = (float)path_base_abs *
                         (FOLLOW_EFFECTIVE_TRACK_MM * 0.5f) *
                         (preview_yaw_delta * 0.0174532925f) /
                         (float)FOLLOW_PREVIEW_DISTANCE_MM;
        curvature_turn_pwm = clamp_i16((int32_t)curvature_turn,
                                       -FOLLOW_MAX_TURN_PWM,
                                       FOLLOW_MAX_TURN_PWM);

        /* Reject a gyro-derived curvature direction that conflicts with turn. */
        if ((raw_turn_abs >= FOLLOW_TURN_DETECT_PWM) &&
            (((raw_turn_pwm > 0) && (curvature_turn_pwm < 0)) ||
             ((raw_turn_pwm < 0) && (curvature_turn_pwm > 0)))) {
            curvature_turn_pwm = raw_turn_pwm;
        }
    }

    if (pivot_command != 0U) {
        /* Replay the front car's in-place corner state at full recorded PWM. */
        target_turn_pwm = raw_turn_pwm;
    } else if (turning_command != 0U) {
        target_turn_pwm = clamp_i16(
            ((int32_t)curvature_turn_pwm * FOLLOW_CURVATURE_BLEND_PERCENT +
             (int32_t)raw_turn_pwm *
             (100 - FOLLOW_CURVATURE_BLEND_PERCENT)) / 100,
            -FOLLOW_MAX_TURN_PWM,
            FOLLOW_MAX_TURN_PWM);
    } else {
        target_turn_pwm = 0;
    }

    /* Reduce feedforward before the current path point's yaw is reached. */
    if ((pivot_command == 0U) && (target_turn_pwm != 0) &&
        (fabsf(heading_error) < FOLLOW_TURN_BRAKE_ANGLE_DEG)) {
        target_turn_pwm = (int16_t)(
            (float)target_turn_pwm * fabsf(heading_error) /
            FOLLOW_TURN_BRAKE_ANGLE_DEG);
    }

    /* Cut feedforward immediately after reaching or crossing target yaw. */
    if ((pivot_command == 0U) &&
        (((target_turn_pwm > 0) && (heading_error <= 0.0f)) ||
         ((target_turn_pwm < 0) && (heading_error >= 0.0f)))) {
        target_turn_pwm = 0;
        last_turn_feedforward_pwm = 0;
    }

    if (pivot_command != 0U) {
        turn_pwm = target_turn_pwm;
    } else {
        turn_slew_delta = (int16_t)(target_turn_pwm -
                                    last_turn_feedforward_pwm);
        turn_slew_limit = ((target_turn_pwm == 0) ||
                           (((target_turn_pwm > 0) ==
                             (last_turn_feedforward_pwm > 0)) &&
                            (((target_turn_pwm < 0) ? -target_turn_pwm :
                              target_turn_pwm) <
                             ((last_turn_feedforward_pwm < 0) ?
                              -last_turn_feedforward_pwm :
                              last_turn_feedforward_pwm)))) ?
                          FOLLOW_TURN_FALL_SLEW_PWM :
                          FOLLOW_TURN_RISE_SLEW_PWM;
        turn_slew_delta = clamp_i16(turn_slew_delta,
                                    -turn_slew_limit,
                                    turn_slew_limit);
        turn_pwm = (int16_t)(last_turn_feedforward_pwm + turn_slew_delta);
    }
    last_turn_feedforward_pwm = turn_pwm;

    if (turning_command != 0U) {
        distance_trim_pwm = 0;
    } else {
        RearCar_UpdateDistanceTrim(RearCar_CanUseUltrasonic(0U));
    }
    base_pwm = clamp_i16((int32_t)base_pwm + distance_trim_pwm,
                         -FOLLOW_MAX_BASE_PWM,
                         FOLLOW_MAX_BASE_PWM);
    if ((delayed_packet->speed >= 0) && (base_pwm < 0)) {
        base_pwm = 0;
    }
    if (right_angle_approach_active != 0U) {
        base_pwm = clamp_i16(base_pwm,
                             -FOLLOW_CORNER_APPROACH_MAX_PWM,
                             FOLLOW_CORNER_APPROACH_MAX_PWM);
    }

    heading_pwm = HeadingPID_Update(rear_target_yaw,
                                    REAR_YAW_SIGN * Yaw_GetAngle(),
                                    (float)FOLLOW_CONTROL_MS / 1000.0f,
                                    (float)FOLLOW_MAX_HEADING_PWM);

    /* During pivot replay, heading feedback may assist but never oppose it. */
    if ((pivot_command != 0U) &&
        (((turn_pwm > 0) && (heading_pwm < 0.0f)) ||
         ((turn_pwm < 0) && (heading_pwm > 0.0f)))) {
        heading_pwm = 0.0f;
    }

    base_abs = (base_pwm < 0) ? (int16_t)-base_pwm : base_pwm;

    /* A stopped front car must not trigger gyro-only rotation. */
    if ((base_abs < FOLLOW_MIN_DRIVE_PWM) && (pivot_command == 0U)) {
        RearCar_Stop();
        return;
    }

    steering_pwm = clamp_i16((int32_t)turn_pwm + (int32_t)heading_pwm,
                             -FOLLOW_MAX_TURN_PWM,
                             FOLLOW_MAX_TURN_PWM);

    if (pivot_command == 0U) {
        /* During normal driving, keep both wheels in the commanded direction. */
        steering_limit = base_abs;
        steering_pwm = clamp_i16(steering_pwm, -steering_limit, steering_limit);
    }

    left = (int32_t)base_pwm - (int32_t)steering_pwm;
    right = (int32_t)base_pwm + (int32_t)steering_pwm;

    car.left_pwm = clamp_i16(left, -999, 999);
    car.right_pwm = clamp_i16(right, -999, 999);
    Motor_SetPWM(car.left_pwm, car.right_pwm);
}

static void RearCar_UpdateOLED(void)
{
    uint32_t distance_mm = (uint32_t)((((uint64_t)rear_odometer_ticks * 141372U) +
                                       265317U) / 530634U);
    int32_t yaw_x10 = (int32_t)(Yaw_GetAngle() * 10.0f);
    uint32_t yaw_abs_x10 = (yaw_x10 < 0) ?
                           (uint32_t)(-yaw_x10) : (uint32_t)yaw_x10;

    OLED_Clear();
    OLED_ShowString(1, 1, "D:");
    if (car.distance_valid) {
        OLED_ShowNum(1, 3, car.distance_cm, 3);
        OLED_ShowString(1, 6, "cm");
    } else {
        OLED_ShowString(1, 3, "---cm");
    }

    if (!car.radio_ok) {
        OLED_ShowString(1, 10, "RF:NO");
    } else if (car.mode == REAR_MODE_BUFFERING) {
        OLED_ShowString(1, 10, "RF:BF");
    } else {
        OLED_ShowString(1, 10, "RF:OK");
    }

    OLED_ShowString(2, 1, "FV:");
    OLED_ShowSignedNum(2, 4, car.delayed_radio.speed, 4);
    OLED_ShowString(2, 10, "FT:");
    OLED_ShowSignedNum(2, 13, car.delayed_radio.turn, 3);

    OLED_ShowString(3, 1, "RX:");
    OLED_ShowNum(3, 4, car.radio.seq, 3);
    OLED_ShowString(3, 8, "EX:");
    if (car.mode == REAR_MODE_BUFFERING) {
        OLED_ShowString(3, 11, "---");
    } else {
        OLED_ShowNum(3, 11, car.delayed_radio.seq, 3);
    }

    OLED_ShowString(4, 1, "D");
    OLED_ShowNum(4, 2, (distance_mm + 5U) / 10U, 4);
    OLED_ShowString(4, 6, "Y");
    OLED_ShowString(4, 7, (yaw_x10 < 0) ? "-" : "+");
    OLED_ShowNum(4, 8, yaw_abs_x10 / 10U, 3);
    OLED_ShowString(4, 11, ".");
    OLED_ShowNum(4, 12, yaw_abs_x10 % 10U, 1);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_SPI2_Init();

  /* USER CODE BEGIN 2 */
  Motor_Init();
  SpeedPID_Init();
  AnglePID_Init();
  OLED_Init();
  MPU6500_Init();
  MPU6500_CalibrateGyro();
  Yaw_Reset();

  HCSR04_Init();
  (void)NRF24L01_Init();

  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_Base_Start_IT(&htim1);

  OLED_Clear();
  OLED_ShowString(1, 1, "Rear car ready");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    static uint32_t last_distance_tick = 0;
    static uint32_t last_control_tick = 0;
    static uint32_t last_oled_tick = 0;
    static uint16_t filtered_distance_cm = 0U;
    uint32_t now = HAL_GetTick();
    NRF24L01_Packet packet;
    NRF24L01_Status radio_status;

    /* Drain the three-entry NRF RX FIFO before running slower peripheral tasks. */
    for (uint8_t read_count = 0U; read_count < 3U; read_count++) {
        radio_status = NRF24L01_ReadPacket(&packet);
        if (radio_status == NRF24L01_OK) {
            RearCar_RecordPacket(&packet);
        } else if ((radio_status == NRF24L01_NO_DATA) ||
                   (radio_status == NRF24L01_SPI_ERROR)) {
            break;
        }
    }

    /* Keep the motor control ahead of the blocking ultrasonic measurement. */
    if ((uint32_t)(now - last_control_tick) >= FOLLOW_CONTROL_MS) {
        last_control_tick = now;
        RearCar_ControlTask();
    }

    if ((uint32_t)(now - last_distance_tick) >= FOLLOW_DISTANCE_MS) {
        int16_t turn_abs = (car.delayed_radio.turn < 0) ?
                           (int16_t)(-(int32_t)car.delayed_radio.turn) :
                           car.delayed_radio.turn;
        uint8_t turning = ((playback_active != 0U) &&
                           (turn_abs >= FOLLOW_TURN_DETECT_PWM)) ? 1U : 0U;

        last_distance_tick = now;
        if (turning != 0U) {
            car.distance_valid = 0U;
            distance_stable_count = 0U;
            last_raw_distance_cm = 0U;
            filtered_distance_cm = 0U;
        } else {
            uint16_t distance = HCSR04_ReadDistanceCm();
            uint8_t distance_valid = RearCar_IsDistanceValid(distance);

            if (distance_valid == 0U) {
                car.distance_valid = 0U;
                distance_stable_count = 0U;
                last_raw_distance_cm = 0U;
                filtered_distance_cm = 0U;
            } else {
                uint16_t jump_cm = (distance >= last_raw_distance_cm) ?
                                   (uint16_t)(distance - last_raw_distance_cm) :
                                   (uint16_t)(last_raw_distance_cm - distance);

                if ((last_raw_distance_cm == 0U) ||
                    (jump_cm > FOLLOW_DISTANCE_MAX_JUMP_CM)) {
                    distance_stable_count = 1U;
                    filtered_distance_cm = distance;
                } else {
                    if (distance_stable_count < FOLLOW_DISTANCE_STABLE_N) {
                        distance_stable_count++;
                    }
                    filtered_distance_cm = (uint16_t)(
                        ((uint32_t)filtered_distance_cm * 3U + distance + 2U) / 4U);
                }
                last_raw_distance_cm = distance;
                car.distance_valid = 1U;
                car.distance_cm = filtered_distance_cm;
            }
        }
    }

    if ((uint32_t)(now - last_oled_tick) >= FOLLOW_OLED_MS) {
        last_oled_tick = now;
        RearCar_UpdateOLED();
    }
  }
  /* USER CODE END WHILE */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 7199;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 99;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim2);
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOA, AIN1_Pin|AIN2_Pin|TRIG_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, BIN1_Pin|BIN2_Pin|CE_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, Buzzer_Pin|SCL_OLED_Pin|SDA_OLED_Pin, GPIO_PIN_SET);

  GPIO_InitStruct.Pin = AIN1_Pin|AIN2_Pin|TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = BIN1_Pin|BIN2_Pin|CSN_Pin|CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = Buzzer_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(Buzzer_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = SCL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SCL_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SDA_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = key1_Pin|key2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = ECHO_Pin|INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = SCL_OLED_Pin|SDA_OLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    int16_t speed_l = 0;
    int16_t speed_r = 0;

    if (htim->Instance != TIM1) {
        return;
    }

    Get_Motor_Speed(&speed_l, &speed_r);
    /* Right encoder is mounted with opposite polarity to the left encoder. */
    speed_r = (int16_t)(-(int32_t)speed_r);
    last_speedL = speed_l;
    last_speedR = speed_r;
    {
        uint16_t left_ticks = (speed_l < 0) ?
                              (uint16_t)(-(int32_t)speed_l) :
                              (uint16_t)speed_l;
        uint16_t right_ticks = (speed_r < 0) ?
                               (uint16_t)(-(int32_t)speed_r) :
                               (uint16_t)speed_r;
        uint32_t tick_sum = (uint32_t)left_ticks + right_ticks;
        uint32_t odometer_sum = tick_sum + rear_odometer_half_tick;

        rear_odometer_ticks += odometer_sum / 2U;
        rear_odometer_half_tick = (uint8_t)(odometer_sum & 1U);
        if (playback_active != 0U) {
            uint32_t path_sum = tick_sum + rear_path_half_tick;
            rear_path_ticks += path_sum / 2U;
            rear_path_half_tick = (uint8_t)(path_sum & 1U);
        }
    }
    Yaw_Update();
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif
