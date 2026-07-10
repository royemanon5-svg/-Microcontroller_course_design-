/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include "mpu6500.h"
#include "I2C.h"
#include "adc.h"
#include "OLED.h"
#include "NRF24L01.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t LineFlag = 0;
int16_t debug_pos = 0;       // 用于显示的偏差值
volatile int16_t Target_speed[2] = {0}; // 用于显示的速度值
volatile int16_t last_speedL = 0; 
volatile int16_t last_speedR = 0;
int16_t left_speed = 0;  
int16_t right_speed = 0; 

volatile float g_yaw = 0.0f;
volatile int16_t g_left = 0;
volatile int16_t g_right = 0;
volatile int16_t g_motor_cmd_left = 0;
volatile int16_t g_motor_cmd_right = 0;
volatile uint8_t g_nrf_ready = 0;
volatile uint8_t g_nrf_tx_ok = 0;
volatile uint8_t g_turning = 0;
int16_t g_l_val = 0;
int16_t g_r_val = 0;
char buf[64];
volatile uint8_t line_detect_count = 0;
volatile float g_target_yaw = 0.0f;
volatile uint16_t lost_line_confirm = 0;
extern MPU6500_Data mpu_data;
typedef enum {
    STATE_TRACKING,   // 循迹
    STATE_STRAIGHT    // 直线
} RobotState;

RobotState state = STATE_STRAIGHT ;

/* 转弯状态机 */
volatile uint8_t  step = 1;
volatile uint8_t  corner_det = 0;
volatile uint16_t turn_ticks = 0;
volatile uint8_t  turn_lost_line = 0;
volatile uint32_t corner_lock = 0;
volatile uint8_t  corner_dir = 0;    // 0=左转 1=右转
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
#include<motor.h>
#include"pid.h"
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int16_t Get_Encoder_Speed(TIM_HandleTypeDef *htim) {
    int16_t count = (int16_t)__HAL_TIM_GET_COUNTER(htim);
    __HAL_TIM_SET_COUNTER(htim, 0); // 读取后清零，得到这 10ms 内的增量
    return count;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	int16_t Target_Speed[2] = {0, 0};
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  ADC_Init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
	//--------------初始化----------------
	Motor_Init();
	PID_Init();
	MPU6500_Init();
	MPU6500_CalibrateGyro();
	g_nrf_ready = NRF24L01_Init();
	//初始化软串口 (PB3=TX, PB4=RX)
	//开启编码器读取（硬件计数）
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);
	//开启控制定时器（关键：触发10ms中断）
	HAL_TIM_Base_Start_IT(&htim1);
        //初始化 OLED
        OLED_Init();
        OLED_Clear();
        OLED_ShowString(1, 1, "Tracking...");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	/* 在 main 函数的初始化部分 */
	Yaw_Reset();  // 当前方向设为0
	g_target_yaw = 0.0f;
	AnglePID_Enable();
	corner_lock = HAL_GetTick() + 500;  // 上电3秒内不检测直角
//
//	//--- 发送数据 ---
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // OLED 显示 8 路 + 总和（调试用）
    {
        char t[18];
        sprintf(t, "L%4d%4d%4d%4d", adc_values[0], adc_values[1], adc_values[2], adc_values[3]);
        OLED_ShowString(3, 1, t);
        sprintf(t, "R%4d%4d%4d%4d", adc_values[4], adc_values[5], adc_values[6], adc_values[7]);
        OLED_ShowString(4, 1, t);
    }
    {
        static uint32_t last_nrf_retry = 0U;
        int16_t left = g_motor_cmd_left;
        int16_t right = g_motor_cmd_right;
        int16_t speed = (int16_t)(((int32_t)left + right) / 2);
        int16_t turn = (int16_t)(((int32_t)right - left) / 2);
        int16_t yaw_x10 = (int16_t)(Yaw_GetAngle() * 10.0f);
        char radio_text[18];

        sprintf(radio_text, "V:%4d T:%4d", speed, turn);
        OLED_ShowString(1, 1, radio_text);

        if (g_nrf_ready)
        {
            g_nrf_tx_ok = (NRF24L01_SendCarData(speed, turn, yaw_x10) == NRF24L01_TX_OK);
            OLED_ShowString(2, 1, g_nrf_tx_ok ? "RF:OK Q:   " : "RF:ERR Q:  ");
            OLED_ShowNum(2, 9, NRF24L01_LastSequence(), 3);
        }
        else
        {
            uint32_t now = HAL_GetTick();

            if ((uint32_t)(now - last_nrf_retry) >= 500U)
            {
                last_nrf_retry = now;
                g_nrf_ready = NRF24L01_Init();
            }

            OLED_ShowString(2, 1, "RF:E C:  S:  ");
            OLED_ShowHexNum(2, 8, NRF24L01_ReadRegister(0x05U), 2);
            OLED_ShowHexNum(2, 13, NRF24L01_ReadRegister(0x07U), 2);
        }
    }
    // 转弯完成检测
    if (step == 3)
    {
        if (!line_detected && turn_ticks > 10)
            turn_lost_line = 1;
        if (turn_lost_line && line_detected && turn_ticks > 15)
        {
            line_detect_count++;
            if (line_detect_count > 2)
            {
                line_detect_count = 0;
                AnglePID_Disable();
                Tracking_Reset();
                corner_lock = HAL_GetTick() + CORNER_LOCK_MS;
                PID_SpeedL.integral = 0; PID_SpeedR.integral = 0;
                OLED_Clear();
                OLED_ShowString(1, 1, "Tracking...");
                step = 1;
            }
        }
        else line_detect_count = 0;
    }
    HAL_Delay(20);
  /* USER CODE END 3 */
}
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
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

  /** Initializes the CPU, AHB and APB buses clocks
  */
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

}
/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
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
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
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
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
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
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
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
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, AIN1_Pin|AIN2_Pin|GPIO_PIN_5|SCK_Pin
                          |CE_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BIN1_Pin|BIN2_Pin|SCL_Pin|SDA_Pin
                          |GPIO_PIN_12|GPIO_PIN_13|MOSI_Pin|UART_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SCL_OLED_Pin|SDA_OLED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : AIN1_Pin AIN2_Pin */
  GPIO_InitStruct.Pin = AIN1_Pin|AIN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 SCK_Pin CE_Pin CSN_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_5|SCK_Pin|CE_Pin|CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BIN1_Pin BIN2_Pin */
  GPIO_InitStruct.Pin = BIN1_Pin|BIN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SCL_Pin UART_1_Pin */
  GPIO_InitStruct.Pin = SCL_Pin|UART_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SDA_Pin */
  GPIO_InitStruct.Pin = SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SDA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 MOSI_Pin(PB14) */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : MISO_Pin(PB15) UART_1B4_Pin */
  GPIO_InitStruct.Pin = MISO_Pin|UART_1B4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : INT_Pin */
  GPIO_InitStruct.Pin = INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SCL_OLED_Pin SDA_OLED_Pin */
  GPIO_InitStruct.Pin = SCL_OLED_Pin|SDA_OLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1)
    {
        ADC_ReadAll();
        last_speedL = Get_Encoder_Speed(&htim3);
        last_speedR = Get_Encoder_Speed(&htim4);
        Yaw_Update();

        if (step == 1)
        {
            // 角检测：8路总和低于阈值 → 直角
            if ((int32_t)(HAL_GetTick() - corner_lock) >= 0)
            {
                uint16_t sum = 0, sumL = 0, sumR = 0;
                for (uint8_t i = 0; i < 4; i++) { sumL += adc_values[i]; sumR += adc_values[i+4]; }
                sum = sumL + sumR;
                if (sum < CORNER_SUM_THRESH)
                {
                    int16_t diff = (int16_t)(sumL - sumR);
                    if      (diff < -200) { corner_dir = 0; corner_det++; }
                    else if (diff >  200) { corner_dir = 1; corner_det++; }
                }
                else { corner_det = 0; }
                if (corner_det > 1)
                {
                    corner_det = 0;
                    turn_ticks = 0;
                    OLED_Clear();
                    OLED_ShowString(1, 1, corner_dir ? "TURN RIGHT" : "TURN LEFT");
                    step = 2;
                }
            }
            debug_pos = (int16_t)Tracking_Task((int16_t *)Target_speed);
            int16_t out_L = PID_Caculate(&PID_SpeedL, (float)Target_speed[0] - last_speedL);
            int16_t out_R = PID_Caculate(&PID_SpeedR, (float)Target_speed[1] - last_speedR);
            g_motor_cmd_left = out_L;
            g_motor_cmd_right = out_R;
            Motor_SetPWM(out_L, out_R);
        }
        else if (step == 2)
        {
            g_motor_cmd_left = 160;
            g_motor_cmd_right = 160;
            Motor_SetPWM(160, 160);
            turn_ticks++;
            if (turn_ticks >= 25)
            {
                AnglePID_Enable();
                g_target_yaw = Yaw_GetAngle() + (corner_dir ? -90.0f : 90.0f);
                if (g_target_yaw >  180.0f) g_target_yaw -= 360.0f;
                if (g_target_yaw < -180.0f) g_target_yaw += 360.0f;
                PID_SpeedL.integral = 0; PID_SpeedR.integral = 0;
                PID_SpeedL.last_error = 0; PID_SpeedR.last_error = 0;
                line_detect_count = 0;
                turn_ticks = 0;
                turn_lost_line = 0;
                step = 3;
            }
        }
        else if (step == 3)
        {
            if (turn_ticks < 255) turn_ticks++;
            int16_t L, R;
            AnglePID_Process(0, g_target_yaw, &L, &R);
            if (L >  TURN_MAX_PWM) L =  TURN_MAX_PWM;
            if (L < -TURN_MAX_PWM) L = -TURN_MAX_PWM;
            if (R >  TURN_MAX_PWM) R =  TURN_MAX_PWM;
            if (R < -TURN_MAX_PWM) R = -TURN_MAX_PWM;
            if (L > 0 && L < TURN_MIN_PWM) L = TURN_MIN_PWM;
            if (L < 0 && L > -TURN_MIN_PWM) L = -TURN_MIN_PWM;
            if (R > 0 && R < TURN_MIN_PWM) R = TURN_MIN_PWM;
            if (R < 0 && R > -TURN_MIN_PWM) R = -TURN_MIN_PWM;
            Target_speed[0] = L; Target_speed[1] = R;
            g_motor_cmd_left = L;
            g_motor_cmd_right = R;
            Motor_SetPWM(L, R);
        }
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
