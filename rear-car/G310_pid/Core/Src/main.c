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
    REAR_MODE_NORMAL,
    REAR_MODE_PREDICT
} RearFollowMode;

typedef struct {
    NRF24L01_Packet packet;
    uint32_t rx_tick;
} TrackPoint;

typedef struct {
    NRF24L01_Packet radio;
    NRF24L01_Packet delayed_radio;
    uint16_t distance_cm;
    uint8_t distance_valid;
    uint8_t radio_ok;
    uint8_t near_stop_count;
    RearFollowMode mode;
    int16_t left_pwm;
    int16_t right_pwm;
} RearCarState;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FOLLOW_TARGET_CM        20
#define FOLLOW_SAFE_STOP_CM     10
#define FOLLOW_STOP_CONFIRM_N   3U
#define FOLLOW_SLOW_ZONE_CM     18
#define FOLLOW_VALID_MIN_CM     3
#define FOLLOW_VALID_MAX_CM     80
#define FOLLOW_DISTANCE_KP      12.0f
#define FOLLOW_HEADING_KP       2.0f
#define FOLLOW_TURN_FF_GAIN     1.0f
#define FOLLOW_TURN_YAW_GAIN    0.045f
#define FOLLOW_PWM_TO_CM_S      0.05f
#define FOLLOW_MIN_DELAY_MS     200U
#define FOLLOW_MAX_DELAY_MS     2500U
#define FOLLOW_MAX_BASE_PWM     550
#define FOLLOW_MAX_TURN_PWM     260
#define FOLLOW_MAX_HEADING_PWM  120
#define FOLLOW_CONTROL_MS       20U
#define FOLLOW_DISTANCE_MS      50U
#define FOLLOW_OLED_MS          200U
#define TRACK_BUFFER_SIZE       64U
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
static uint8_t track_head = 0;
static uint8_t track_count = 0;
static float rear_target_yaw = 0.0f;
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
static float angle_error_deg(float target, float current);
static float normalize_angle_deg(float angle);
static uint8_t RearCar_IsDistanceValid(uint16_t distance_cm);
static void TrackBuffer_Push(const NRF24L01_Packet *packet, uint32_t rx_tick);
static const NRF24L01_Packet *TrackBuffer_SelectDelayed(uint32_t now);
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

static void TrackBuffer_Push(const NRF24L01_Packet *packet, uint32_t rx_tick)
{
    track_buffer[track_head].packet = *packet;
    track_buffer[track_head].rx_tick = rx_tick;
    track_head = (uint8_t)((track_head + 1U) % TRACK_BUFFER_SIZE);

    if (track_count < TRACK_BUFFER_SIZE) {
        track_count++;
    }
}

static const NRF24L01_Packet *TrackBuffer_SelectDelayed(uint32_t now)
{
    uint8_t i;
    uint32_t delay_ms;
    uint32_t target_tick;
    uint32_t best_error = 0xFFFFFFFFUL;
    const TrackPoint *best = 0;
    const TrackPoint *newest;
    int16_t speed_abs;
    float speed_cm_s;

    if (track_count == 0U) {
        return 0;
    }

    newest = &track_buffer[(uint8_t)((track_head + TRACK_BUFFER_SIZE - 1U) % TRACK_BUFFER_SIZE)];
    speed_abs = newest->packet.speed;
    if (speed_abs < 0) {
        speed_abs = (int16_t)-speed_abs;
    }

    speed_cm_s = (float)speed_abs * FOLLOW_PWM_TO_CM_S;
    if (speed_cm_s < 1.0f) {
        speed_cm_s = 1.0f;
    }

    delay_ms = (uint32_t)(((float)FOLLOW_TARGET_CM * 1000.0f) / speed_cm_s);
    if (delay_ms < FOLLOW_MIN_DELAY_MS) {
        delay_ms = FOLLOW_MIN_DELAY_MS;
    }
    if (delay_ms > FOLLOW_MAX_DELAY_MS) {
        delay_ms = FOLLOW_MAX_DELAY_MS;
    }

    target_tick = (now > delay_ms) ? (now - delay_ms) : 0U;

    for (i = 0U; i < track_count; i++) {
        uint8_t index = (uint8_t)((track_head + TRACK_BUFFER_SIZE - 1U - i) % TRACK_BUFFER_SIZE);
        const TrackPoint *point = &track_buffer[index];
        uint32_t error;

        if (point->rx_tick > target_tick) {
            error = point->rx_tick - target_tick;
        } else {
            error = target_tick - point->rx_tick;
        }

        if (error < best_error) {
            best_error = error;
            best = point;
        }
    }

    return (best != 0) ? &best->packet : &newest->packet;
}

static void RearCar_Stop(void)
{
    car.left_pwm = 0;
    car.right_pwm = 0;
    car.mode = REAR_MODE_STOP;
    Motor_SetPWM(0, 0);
}

static void RearCar_ControlTask(void)
{
    float distance_error;
    float distance_pwm;
    float heading_error;
    float heading_pwm;
    float dt_s = (float)FOLLOW_CONTROL_MS / 1000.0f;
    int16_t base_pwm;
    int16_t turn_pwm;
    int32_t left;
    int32_t right;
    uint32_t now = HAL_GetTick();
    const NRF24L01_Packet *delayed_packet;

    car.radio_ok = NRF24L01_IsConnected(now);
    if (!car.radio_ok) {
        RearCar_Stop();
        return;
    }

    if (car.distance_valid && car.distance_cm <= FOLLOW_SAFE_STOP_CM) {
        if (car.near_stop_count < 255U) {
            car.near_stop_count++;
        }
    } else {
        car.near_stop_count = 0U;
    }

    if (car.near_stop_count >= FOLLOW_STOP_CONFIRM_N) {
        RearCar_Stop();
        return;
    }

    delayed_packet = TrackBuffer_SelectDelayed(now);
    if (delayed_packet == 0) {
        RearCar_Stop();
        return;
    }

    car.delayed_radio = *delayed_packet;

    if (car.distance_valid) {
        distance_error = (float)car.distance_cm - (float)FOLLOW_TARGET_CM;
        distance_pwm = distance_error * FOLLOW_DISTANCE_KP;

        if ((distance_error > -2.0f) && (distance_error < 2.0f)) {
            distance_pwm = 0.0f;
        }

        base_pwm = clamp_i16((int32_t)delayed_packet->speed + (int32_t)distance_pwm,
                             -FOLLOW_MAX_BASE_PWM,
                             FOLLOW_MAX_BASE_PWM);

        if ((car.distance_cm < FOLLOW_SLOW_ZONE_CM) && (base_pwm > 0)) {
            base_pwm = 0;
        }

        car.mode = REAR_MODE_NORMAL;
    } else {
        /*
         * Right-angle and S turns can make the ultrasonic beam miss the front
         * car board. Do not stop only because the echo is missing; follow the
         * delayed trajectory from the radio buffer.
         */
        base_pwm = clamp_i16(delayed_packet->speed,
                             -FOLLOW_MAX_BASE_PWM,
                             FOLLOW_MAX_BASE_PWM);
        car.mode = REAR_MODE_PREDICT;
    }

    turn_pwm = clamp_i16((int32_t)((float)delayed_packet->turn * FOLLOW_TURN_FF_GAIN),
                         -FOLLOW_MAX_TURN_PWM,
                         FOLLOW_MAX_TURN_PWM);
    rear_target_yaw = normalize_angle_deg(rear_target_yaw +
        ((float)delayed_packet->turn * FOLLOW_TURN_YAW_GAIN * dt_s));
    heading_error = angle_error_deg(rear_target_yaw, Yaw_GetAngle());
    heading_pwm = heading_error * FOLLOW_HEADING_KP;
    heading_pwm = (float)clamp_i16((int32_t)heading_pwm,
                                   -FOLLOW_MAX_HEADING_PWM,
                                   FOLLOW_MAX_HEADING_PWM);

    left = (int32_t)base_pwm - (int32_t)turn_pwm - (int32_t)heading_pwm;
    right = (int32_t)base_pwm + (int32_t)turn_pwm + (int32_t)heading_pwm;

    car.left_pwm = clamp_i16(left, -999, 999);
    car.right_pwm = clamp_i16(right, -999, 999);
    Motor_SetPWM(car.left_pwm, car.right_pwm);
}

static void RearCar_UpdateOLED(void)
{
    OLED_Clear();
    OLED_ShowString(1, 1, "D:");
    if (car.distance_valid) {
        OLED_ShowNum(1, 3, car.distance_cm, 3);
        OLED_ShowString(1, 6, "cm");
    } else {
        OLED_ShowString(1, 3, "---cm");
    }

    OLED_ShowString(1, 10, car.radio_ok ? "RF:OK" : "RF:NO");

    OLED_ShowString(2, 1, "FV:");
    OLED_ShowSignedNum(2, 4, car.radio.speed, 4);
    OLED_ShowString(2, 10, "FT:");
    OLED_ShowSignedNum(2, 13, car.radio.turn, 3);

    OLED_ShowString(3, 1, "FY:");
    OLED_ShowSignedNum(3, 4, (int32_t)(car.radio.yaw / 10), 4);
    OLED_ShowString(3, 10, "SEQ:");
    OLED_ShowNum(3, 14, car.radio.seq, 2);

    OLED_ShowString(4, 1, "RAWY:");
    OLED_ShowSignedNum(4, 6, car.radio.yaw, 5);
    OLED_ShowString(4, 12, "C:");
    OLED_ShowNum(4, 14, car.radio.checksum, 2);
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
    uint32_t now = HAL_GetTick();
    NRF24L01_Packet packet;
    NRF24L01_Status radio_status;

    radio_status = NRF24L01_ReadPacket(&packet);
    if (radio_status == NRF24L01_OK) {
        car.radio = packet;
        TrackBuffer_Push(&packet, now);
    }

    if ((uint32_t)(now - last_distance_tick) >= FOLLOW_DISTANCE_MS) {
        uint16_t distance = HCSR04_ReadDistanceCm();
        last_distance_tick = now;
        car.distance_cm = distance;
        car.distance_valid = RearCar_IsDistanceValid(distance);
    }

    if ((uint32_t)(now - last_control_tick) >= FOLLOW_CONTROL_MS) {
        last_control_tick = now;
        RearCar_ControlTask();
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
    last_speedL = speed_l;
    last_speedR = speed_r;
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
