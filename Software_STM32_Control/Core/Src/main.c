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
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    float x;   // state estimate
    float p;   // error covariance
    float q;   // process noise  — higher: trusts signal more
    float r;   // measurement noise — higher: smoother output
} Kalman1D;

typedef struct {
    float kp;          // proportional gain
    float ki;          // integral gain
    float kd;          // derivative gain
    float integral;    // accumulated error
    float prev_error;  // previous error (for derivative)
    float i_limit;     // anti-windup clamp
    float d_filtered;  // low-pass filtered derivative output
    float d_alpha;     // derivative filter coefficient (0.1–0.3)
} PID;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PAN_MIN        500
#define PAN_MAX        2000
#define TILT_MIN       500
#define TILT_MAX       1300
#define SMOOTH_ALPHA   0.18f   // exponential position smoother coefficient
#define TOL_ENTER      80      // hysteresis: motor activates above this error
#define TOL_EXIT       120     // hysteresis: motor deactivates below this error
#define MA_SIZE        5       // moving average window size
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint16_t ldr_TL = 0, ldr_TR = 0, ldr_BL = 0, ldr_BR = 0;

// Kalman filter instances — one per LDR channel
Kalman1D k_TL = {2048, 1.0f, 2.0f, 80.0f};
Kalman1D k_TR = {2048, 1.0f, 2.0f, 80.0f};
Kalman1D k_BL = {2048, 1.0f, 2.0f, 80.0f};
Kalman1D k_BR = {2048, 1.0f, 2.0f, 80.0f};

//         kp      ki       kd     integral  prev  i_limit  d_filt  d_alpha
PID pid_pan  = {0.008f, 0.0002f, 0.003f, 0, 0, 50.0f, 0, 0.15f};
PID pid_tilt = {0.008f, 0.0002f, 0.003f, 0, 0, 50.0f, 0, 0.15f};

float target_pan  = 1250;
float target_tilt = 900;
float pos_pan     = 1250;
float pos_tilt    = 900;

// Moving average circular buffers — pre-filled to avoid startup transient
float ma_TL[MA_SIZE] = {2048, 2048, 2048, 2048, 2048}; uint8_t ma_idx_TL = 0;
float ma_TR[MA_SIZE] = {2048, 2048, 2048, 2048, 2048}; uint8_t ma_idx_TR = 0;
float ma_BL[MA_SIZE] = {2048, 2048, 2048, 2048, 2048}; uint8_t ma_idx_BL = 0;
float ma_BR[MA_SIZE] = {2048, 2048, 2048, 2048, 2048}; uint8_t ma_idx_BR = 0;

// Hysteresis state flags (0 = inside dead band, 1 = active)
uint8_t pan_active  = 0;
uint8_t tilt_active = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Single-channel ADC read — reconfigures channel each call */
uint16_t Read_LDR(uint32_t Channel) {
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = Channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    uint16_t value = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    return value;
}

/* 1D Kalman filter — predict → update cycle */
float Kalman_Update(Kalman1D *k, float measurement) {
    k->p = k->p + k->q;
    float kg = k->p / (k->p + k->r);
    k->x = k->x + kg * (measurement - k->x);
    k->p = (1.0f - kg) * k->p;
    return k->x;
}

/* PID with hysteresis dead band, anti-windup clamping and filtered derivative */
float PID_Update(PID *pid, float error, uint8_t *active) {
    uint16_t thr = (*active) ? TOL_ENTER : TOL_EXIT;

    if (error > -(float)thr && error < (float)thr) {
        pid->integral   = 0.0f;
        pid->prev_error = 0.0f;
        pid->d_filtered = 0.0f;
        *active = 0;
        return 0.0f;
    }
    *active = 1;

    // Integral with hard clamp (anti-windup)
    pid->integral += error;
    if (pid->integral >  pid->i_limit) pid->integral =  pid->i_limit;
    if (pid->integral < -pid->i_limit) pid->integral = -pid->i_limit;

    // Low-pass filtered derivative — suppresses noise amplification
    float raw_d     = error - pid->prev_error;
    pid->d_filtered = pid->d_alpha * raw_d + (1.0f - pid->d_alpha) * pid->d_filtered;
    pid->prev_error = error;

    return (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * pid->d_filtered);
}

/* Circular buffer moving average */
float Moving_Avg(float *buf, uint8_t *idx, float new_val) {
    buf[*idx % MA_SIZE] = new_val;
    (*idx)++;
    float sum = 0;
    for (int i = 0; i < MA_SIZE; i++) sum += buf[i];
    return sum / MA_SIZE;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

  /* Soft start — ramp servos to home position over 1 second to prevent snap */
  for (int i = 1; i <= 50; i++) {
        uint16_t p = (uint16_t)(1250.0f * i / 50);
        uint16_t t = (uint16_t)( 900.0f * i / 50);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, p);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, t);
        HAL_Delay(20);
    }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
        // 1) Raw ADC read
        ldr_TL = Read_LDR(ADC_CHANNEL_4);
        ldr_TR = Read_LDR(ADC_CHANNEL_1);
        ldr_BL = Read_LDR(ADC_CHANNEL_5);
        ldr_BR = Read_LDR(ADC_CHANNEL_0);

        // 2) Kalman filter — removes high-frequency sensor noise
        float f_TL = Kalman_Update(&k_TL, ldr_TL);
        float f_TR = Kalman_Update(&k_TR, ldr_TR);
        float f_BL = Kalman_Update(&k_BL, ldr_BL);
        float f_BR = Kalman_Update(&k_BR, ldr_BR);

        // 3) Moving average — secondary smoothing on Kalman output
        f_TL = Moving_Avg(ma_TL, &ma_idx_TL, f_TL);
        f_TR = Moving_Avg(ma_TR, &ma_idx_TR, f_TR);
        f_BL = Moving_Avg(ma_BL, &ma_idx_BL, f_BL);
        f_BR = Moving_Avg(ma_BR, &ma_idx_BR, f_BR);

        float avg_top    = (f_TL + f_TR) / 2.0f;
        float avg_bottom = (f_BL + f_BR) / 2.0f;
        float avg_left   = (f_TL + f_BL) / 2.0f;
        float avg_right  = (f_TR + f_BR) / 2.0f;

        float error_tilt = avg_top   - avg_bottom;
        float error_pan  = avg_left  - avg_right;

        // 4) PID — compute target position correction
        float out_pan  = PID_Update(&pid_pan,  error_pan,  &pan_active);
        float out_tilt = PID_Update(&pid_tilt, error_tilt, &tilt_active);

        target_pan  += out_pan;
        target_tilt -= out_tilt;   // sign depends on mechanical orientation

        // 5) Clamp target within mechanical limits
        if (target_pan  > PAN_MAX)   target_pan  = PAN_MAX;
        if (target_pan  < PAN_MIN)   target_pan  = PAN_MIN;
        if (target_tilt > TILT_MAX)  target_tilt = TILT_MAX;
        if (target_tilt < TILT_MIN)  target_tilt = TILT_MIN;

        // 6) Exponential smoother — organic motion profile toward target
        pos_pan  = pos_pan  + SMOOTH_ALPHA * (target_pan  - pos_pan);
        pos_tilt = pos_tilt + SMOOTH_ALPHA * (target_tilt - pos_tilt);

        // (legacy threshold check — superseded by +0.5f rounding below)
        static float pwm_pan  = 1250;
        static float pwm_tilt = 900;

        if (pos_pan  - pwm_pan  >  0.5f || pwm_pan  - pos_pan  >  0.5f) pwm_pan  = pos_pan;
        if (pos_tilt - pwm_tilt >  0.5f || pwm_tilt - pos_tilt >  0.5f) pwm_tilt = pos_tilt;

        // 7) PWM write — +0.5f converts truncation to proper rounding,
        //    eliminating micro-stepping artifacts at low delta values
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint16_t)(pos_pan  + 0.5f));
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint16_t)(pos_tilt + 0.5f));

        HAL_Delay(20);
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 15;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 19999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_7B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : T_NRST_Pin */
  GPIO_InitStruct.Pin = T_NRST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(T_NRST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
