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
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* 1-D Kalman filter state for LDR noise reduction */
typedef struct {
    float x;    /* State estimate */
    float p;    /* Estimate covariance */
    float q;    /* Process noise */
    float r;    /* Measurement noise */
} Kalman1D;

/* PID controller with derivative-on-measurement and filtered derivative */
typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
    float prev_measurement;
    float i_limit;
    float d_filtered;
    float d_alpha;
    float prev_raw_d;
} PID;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Servo PWM pulse limits (microseconds) */
#define PAN_MIN              500
#define PAN_MAX              2000
#define TILT_MIN             500
#define TILT_MAX             1300

/* Output low-pass filter (1.0 = disabled, smaller = more smoothing) */
#define SMOOTH_ALPHA         0.95f

/* Adaptive deadband bounds (LDR ADC units) */
#define TOL_MIN              50.0f
#define TOL_MAX              80.0f

/* Number of ADC samples averaged per LDR read */
#define LDR_SAMPLES          3

/* Consecutive out-of-band samples required before PID activates */
#define CONFIRM_THRESH       2

/* Slew rate limit: max change in servo target per control loop (us) */
#define MAX_DELTA_PER_LOOP   15.0f

/* Fixed control loop period (ms) for deterministic sampling */
#define LOOP_PERIOD_MS       5
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* Raw ADC values from the four LDR sensors (Top-Left, Top-Right, Bottom-Left, Bottom-Right) */
uint16_t ldr_TL = 0, ldr_TR = 0, ldr_BL = 0, ldr_BR = 0;

/* Per-sensor Kalman filters: initial state 2048 (mid-range), tuned Q/R for slow light changes */
Kalman1D k_TL = {2048, 1.0f, 2.0f, 200.0f};
Kalman1D k_TR = {2048, 1.0f, 2.0f, 200.0f};
Kalman1D k_BL = {2048, 1.0f, 2.0f, 200.0f};
Kalman1D k_BR = {2048, 1.0f, 2.0f, 200.0f};

/* PID gains tuned for MG996R servo + 10k pull-down LDR bridge at 200 Hz loop rate.
 * Field order: kp, ki, kd, integral, prev_error, prev_measurement, i_limit, d_filtered, d_alpha, prev_raw_d
 */
PID pid_pan  = {0.0045f, 0.002f, 0.0005f, 0, 0, 0, 20.0f, 0, 0.15f, 0};
PID pid_tilt = {0.0045f, 0.002f, 0.0005f, 0, 0, 0, 20.0f, 0, 0.15f, 0};

/* Servo target (PID output) and smoothed position (PWM command) */
float target_pan  = 1250;
float target_tilt = 900;
float pos_pan     = 1250;
float pos_tilt    = 900;

/* PID active flags - controllers idle when error stays within deadband */
uint8_t pan_active  = 0;
uint8_t tilt_active = 0;

/* Counters used to debounce PID activation on transient noise */
uint8_t pan_confirm  = 0;
uint8_t tilt_confirm = 0;
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

/* Reads a single ADC channel multiple times and returns the average.
 * Software oversampling reduces noise from LDR thermal drift and supply ripple.
 */
uint16_t Read_LDR_Avg(uint32_t Channel) {
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = Channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    uint32_t sum = 0;
    for (int i = 0; i < LDR_SAMPLES; i++) {
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, 10);
        sum += HAL_ADC_GetValue(&hadc1);
    }
    HAL_ADC_Stop(&hadc1);

    return (uint16_t)(sum / LDR_SAMPLES);
}

/* Standard 1-D Kalman filter update step.
 * Predicts forward, then corrects with the new measurement.
 */
float Kalman_Update(Kalman1D *k, float measurement) {
    k->p = k->p + k->q;
    float kg = k->p / (k->p + k->r);
    k->x = k->x + kg * (measurement - k->x);
    k->p = (1.0f - kg) * k->p;
    return k->x;
}

/* Adaptive deadband scales with ambient light level.
 * Brighter ambient means more noise on the differential signal,
 * so the deadband widens proportionally.
 */
float Calc_Adaptive_Tol(float ambient) {
    float ratio = ambient / 4096.0f;
    return TOL_MIN + ratio * (TOL_MAX - TOL_MIN);
}

/* PID update with several industrial-grade features:
 *  - Tustin (trapezoidal) integration for accurate discrete-time behavior
 *  - Derivative on measurement (not error) to prevent derivative kick on setpoint changes
 *  - Low-pass filter on derivative to attenuate sensor noise
 *  - Hysteresis-based deadband with confirmation counter (debounced activation)
 *  - Asymmetric enter/exit thresholds prevent limit cycling at the boundary
 *  - Integral clamping for anti-windup
 */
float PID_Update(PID *pid, float setpoint, float measurement, float dt,
                 uint8_t *active, float tol, uint8_t *confirm) {
    float error = setpoint - measurement;
    float thr_enter = tol;
    float thr_exit  = tol * 2.5f;

    if (error > -thr_enter && error < thr_enter) {
        /* Inside deadband */
        *confirm = 0;
        if (*active) {
            /* Already active - require deeper return before deactivating (hysteresis) */
            if (error > -thr_exit && error < thr_exit) {
                pid->integral   = 0.0f;
                pid->prev_error = 0.0f;
                pid->prev_measurement = measurement;
                pid->d_filtered = 0.0f;
                pid->prev_raw_d = 0.0f;
                *active = 0;
            }
        }
        if (!(*active)) {
            /* Track measurement while idle so derivative is sane on next activation */
            pid->prev_measurement = measurement;
            return 0.0f;
        }
    } else {
        /* Outside deadband */
        if (!(*active)) {
            (*confirm)++;
            pid->prev_measurement = measurement;
            if (*confirm < CONFIRM_THRESH) {
                return 0.0f;
            }
            *active = 1;
            pid->prev_error = error;
        }
    }

    /* Trapezoidal integration with clamping anti-windup */
    pid->integral += (error + pid->prev_error) * 0.5f * dt;
    if (pid->integral >  pid->i_limit) pid->integral =  pid->i_limit;
    if (pid->integral < -pid->i_limit) pid->integral = -pid->i_limit;

    /* Derivative on measurement (negated to match error-derivative sign) with bilinear LPF */
    float raw_d = -(measurement - pid->prev_measurement) / dt;
    float k  = pid->d_alpha * 0.5f;
    float c1 = (1.0f - k) / (1.0f + k);
    float c2 =  k         / (1.0f + k);
    pid->d_filtered = c1 * pid->d_filtered + c2 * (raw_d + pid->prev_raw_d);

    pid->prev_error = error;
    pid->prev_measurement = measurement;
    pid->prev_raw_d = raw_d;

    return (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * pid->d_filtered);
}

/* Slew rate limiter: caps how much the target can move in a single loop iteration.
 * Prevents large PID outputs from commanding mechanically jarring servo movements.
 */
float Apply_Slew(float delta) {
    if (delta >  MAX_DELTA_PER_LOOP) return  MAX_DELTA_PER_LOOP;
    if (delta < -MAX_DELTA_PER_LOOP) return -MAX_DELTA_PER_LOOP;
    return delta;
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

  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

  /* Soft start: smoothly ramp servos from minimum to home position over 1 second.
   * Prevents inrush current spikes and mechanical shock on power-up.
   * Blocking HAL_Delay is acceptable here since this runs only once at boot.
   */
  for (int i = 1; i <= 50; i++) {
        uint16_t p = PAN_MIN + (uint16_t)((1250.0f - PAN_MIN) * i / 50.0f);
        uint16_t t = TILT_MIN + (uint16_t)((900.0f - TILT_MIN) * i / 50.0f);

        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, p);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, t);
        HAL_Delay(20);
    }

  uint32_t prev_time = HAL_GetTick();
  uint32_t next_tick = HAL_GetTick() + LOOP_PERIOD_MS;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* Non-blocking timer: run control loop every LOOP_PERIOD_MS milliseconds.
     * Signed cast handles 32-bit tick counter overflow correctly (~49 day rollover).
     */
    if ((int32_t)(HAL_GetTick() - next_tick) >= 0) {
        next_tick += LOOP_PERIOD_MS;

        /* If execution fell far behind (e.g. due to a debug breakpoint),
         * skip the catch-up burst and resync to the current time.
         */
        if ((int32_t)(HAL_GetTick() - next_tick) > 100) {
            next_tick = HAL_GetTick() + LOOP_PERIOD_MS;
        }

        /* Measure actual loop period for PID time-step.
         * Even with the fixed schedule, using measured dt keeps the math correct
         * if the schedule slips.
         */
        uint32_t current_time = HAL_GetTick();
        float dt = (current_time - prev_time) / 1000.0f;
        if (dt < 0.001f) dt = 0.001f;
        prev_time = current_time;

        /* Read the four LDRs. Channel mapping is set by the PCB layout. */
        ldr_TL = Read_LDR_Avg(ADC_CHANNEL_4);
        ldr_TR = Read_LDR_Avg(ADC_CHANNEL_1);
        ldr_BL = Read_LDR_Avg(ADC_CHANNEL_5);
        ldr_BR = Read_LDR_Avg(ADC_CHANNEL_0);

        /* Kalman filter each sensor independently for noise reduction. */
        float f_TL = Kalman_Update(&k_TL, ldr_TL);
        float f_TR = Kalman_Update(&k_TR, ldr_TR);
        float f_BL = Kalman_Update(&k_BL, ldr_BL);
        float f_BR = Kalman_Update(&k_BR, ldr_BR);

        /* Compute the four edge averages used for differential error signals. */
        float avg_top    = (f_TL + f_TR) / 2.0f;
        float avg_bottom = (f_BL + f_BR) / 2.0f;
        float avg_left   = (f_TL + f_BL) / 2.0f;
        float avg_right  = (f_TR + f_BR) / 2.0f;

        /* Ambient light estimate drives the adaptive deadband. */
        float ambient      = (f_TL + f_TR + f_BL + f_BR) / 4.0f;
        float adaptive_tol = Calc_Adaptive_Tol(ambient);

        /* Setpoint is zero: we want left=right and top=bottom (perfectly aimed at the sun). */
        float setpoint = 0.0f;

        /* Differential measurements: positive pan means right side is brighter,
         * positive tilt means bottom is brighter.
         */
        float meas_pan  = avg_right - avg_left;
        float meas_tilt = avg_bottom - avg_top;

        /* Run both PID controllers. */
        float out_pan  = PID_Update(&pid_pan,  setpoint, meas_pan,  dt, &pan_active,  adaptive_tol, &pan_confirm);
        float out_tilt = PID_Update(&pid_tilt, setpoint, meas_tilt, dt, &tilt_active, adaptive_tol, &tilt_confirm);

        /* Apply slew limit and update servo targets.
         * Sign is negated: positive error (light unbalanced right) needs servo to move toward light.
         */
        float dpan  = Apply_Slew(-out_pan);
        float dtilt = Apply_Slew(-out_tilt);
        target_pan  += dpan;
        target_tilt += dtilt;

        /* Conditional anti-windup: when the target saturates against a limit,
         * clear any integral pushing further into the saturation.
         */
        if (target_pan > PAN_MAX) {
            target_pan = PAN_MAX;
            if (pid_pan.integral < 0) pid_pan.integral = 0;
        } else if (target_pan < PAN_MIN) {
            target_pan = PAN_MIN;
            if (pid_pan.integral > 0) pid_pan.integral = 0;
        }

        if (target_tilt > TILT_MAX) {
            target_tilt = TILT_MAX;
            if (pid_tilt.integral < 0) pid_tilt.integral = 0;
        } else if (target_tilt < TILT_MIN) {
            target_tilt = TILT_MIN;
            if (pid_tilt.integral > 0) pid_tilt.integral = 0;
        }

        /* Optional low-pass on the commanded position (currently near-disabled at 0.95). */
        pos_pan  = pos_pan  + SMOOTH_ALPHA * (target_pan  - pos_pan);
        pos_tilt = pos_tilt + SMOOTH_ALPHA * (target_tilt - pos_tilt);

        /* Send PWM command to the servos. */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint16_t)(pos_pan + 0.5f));
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint16_t)(pos_tilt + 0.5f));
    }

    /* CPU is free between control loop iterations - background tasks can run here. */
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

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

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
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_160CYCLES_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_160CYCLES_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

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
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
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

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = T_NRST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(T_NRST_GPIO_Port, &GPIO_InitStruct);

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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
