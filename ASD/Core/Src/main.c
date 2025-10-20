/* Main file of the Active Swing Damper*/
/* Author: Yanislav Radyul */

/****************************************************
 *                 Standard Includes               *
 ****************************************************/
#include <math.h>
#include <string.h>
#include <stdio.h>

/****************************************************
 *                 Project Includes                *
 ****************************************************/
#include "main.h"
#include "fatfs.h"
#include "IMU.h"
#include "DSC.h"
#include "PID.h"
#include "MOT_ENC.h"
#include "MOT_DRV.h"
#include "SD.h"

/****************************************************
 *                 Peripheral Handles              *
 ****************************************************/
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
UART_HandleTypeDef huart1;

/****************************************************
 *                 Global Variables                *
 ****************************************************/
char TxBuffer[250];
int decel_sts;
int32_t ticks = 0;
uint32_t time_del;
int PIN_IN3_STATE;
int PIN_IN4_STATE;

/****************************************************
 *                 Constants / Macros              *
 ****************************************************/
#define SCALE_FACTOR 0.02 //best scaling for the accelerating/braking capabilities of the motor with flywheel
#define CLKWISE ((PIN_IN3_STATE==1) && (PIN_IN4_STATE == 0))
#define ACLKWISE ((PIN_IN3_STATE==0) && (PIN_IN4_STATE == 1))

/****************************************************
 *                 Function Prototypes             *
 ****************************************************/
// System configuration
static void SystemClock_Config(void);
static void SysClock_Init(void);
static void PeriphConfig_Init(Diag_lst_t *diag_lst);

// Peripheral initialization
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI2_Init(void);

// Utility functions
static int16_t LP_Filter(int16_t unfilt_val, int16_t filt_val_z1, double alpha);
static void DelayedVals_Update(Motor_t *motor, Rocker_IMU_t *IMU);

// HAL callbacks
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim2);


int main(void)
{
	HAL_Init();

	// Initialize the system clock
	SysClock_Init();

	//Define all structs
	PID_t PID;
	Rocker_IMU_t IMU;
	Motor_t Motor;
	Diag_lst_t Diag_lst;

	// Initialize all struct attributes
	PID_Init(&PID);
	IMU_Init(&IMU);
	Motor_Init(&Motor);
	Diag_Init(&Diag_lst);

	// Initialize all configured peripherals
	PeriphConfig_Init(&Diag_lst);

	// Micro SD Startup
	SDLog_Startup();

	while (1)
	{
		  // IMU data reading
		  IMU_Read(&IMU);
		  IMU.Acc_X = LP_Filter(IMU.Acc_X_raw, IMU.Acc_X_z1, 0.9);
		  IMU.Omega_Y = LP_Filter(IMU.Omega_Y_raw, IMU.Omega_Y_z1, 0.9);

		  // Encoder data reading
		  Motor_Speed_Calc(&Motor, &delta_us, &delta_s);
		  Motor.Motor_spd = LP_Filter(Motor.Motor_spd_raw, Motor.Motor_spd_z1, 0.6);

		  // Check whether the IMU is decelerating or not
		  IMU_Decel_Check(&decel_sts, &IMU);

		  // Select target speed based on accelerating state of the IMU
		  if (decel_sts)
		  {
			  PID.Tgt_spd = 0;
		  }
		  else
		  {
			  PID.Tgt_spd = (int16_t) (SCALE_FACTOR * IMU.Omega_Y);
		  }

		  // Diagnostic check of the system
		  Diag_Check(&Diag_lst);

		  // Set Motor rotation direction or braking mode
		  Motor_Drive_PinState_Set(PID.Tgt_spd, decel_sts, &PIN_IN3_STATE, &PIN_IN4_STATE);

		  // PID control
		  PID_Control(&PID, Motor.Motor_spd);
		  PID_PWM_Limiter(&PID, decel_sts, ACLKWISE, CLKWISE);

		  // Set motor speed and direction of motion
		  Motor_Drive_Speed_Set(PID.PWM_Output, PIN_IN3_STATE, PIN_IN4_STATE);

		  // Delayed values update
		  DelayedVals_Update(&Motor,&IMU);

		  // Micro SD data log
		  SDLog_Write(&Motor, &PID, &Diag_lst, &IMU, decel_sts, delta_us);

		  // Set 10ms cycle */
		  if (delta_us < 10000)
		  {
		      time_del = (uint32_t)(10000 - delta_us);  // microseconds
		      HAL_Delay(time_del / 1000);               // convert to milliseconds
		  }

  }
}


/**
 * @brief Initializes all peripherals required for the system.
 *
 * This function configures GPIO, I2C, UART, timers, encoder, PWM, SPI,
 * and the FAT filesystem. It also initializes the MPU6050 IMU and starts
 * the encoder and PWM channels. A small delay ensures proper I2C connection
 * before accessing the IMU.
 *
 * @param[in,out] diag_lst Pointer to the Diag_lst_t structure, used to
 *                         initialize the IMU and update diagnostic state.
 *
 * @return None
 *
 * @note
 * - Ensure that HAL is properly initialized before calling this function.
 * - MX_FATFS_Init() must be called after SPI initialization for SD card usage.
 * - The HAL_TIM_PWM_Start and HAL_TIM_Encoder_Start functions start
 *   the timers for motor control.
 */
static void PeriphConfig_Init(Diag_lst_t *diag_lst)
{
	  MX_GPIO_Init();
	  MX_I2C1_Init();
	  HAL_I2C_Init(&hi2c1);
	  MX_USART1_UART_Init();
	  MX_TIM1_Init();
	  MX_TIM2_Init();
	  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	  //Small delay to initalize everything during I2C connection to IMU
	  HAL_Delay(200);
	  MPU6050_Init(diag_lst);
	  MX_FATFS_Init();
	  MX_SPI2_Init();
	  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
}


static void SysClock_Init(void)
{
	SystemClock_Config();
	// Enable tracing
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	// Reset cycle counter
	DWT->CYCCNT = 0;
	// Enable cycle counter
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/**
 * @brief Applies a first-order low-pass filter to a signal.
 *
 * This function filters an input value using the previous filtered value and
 * a smoothing factor alpha. It is useful for reducing noise in sensor readings
 * or control signals.
 *
 * @param[in] unfilt_val Current raw/unfiltered input value.
 * @param[in] filt_val_z1 Previous filtered value (from the last call).
 * @param[in] alpha Smoothing factor (0 < alpha < 1):
 *                  - Closer to 1: slower response, smoother signal.
 *                  - Closer to 0: faster response, less smoothing.
 *
 * @return int16_t Filtered output value.
 *
 * @note
 * - This is a simple first-order filter: filt_val = alpha*filt_val_z1 + (1-alpha)*unfilt_val.
 * - Ensure alpha is within the range (0,1) for proper filtering behavior.
 */
static int16_t LP_Filter(int16_t unfilt_val, int16_t filt_val_z1, double alpha)
{
    int16_t filt_val = alpha * filt_val_z1 + (1-alpha) * unfilt_val;
    return filt_val;
}


/**
 * @brief Updates previous sample values for motor and IMU measurements.
 *
 * This function stores the current raw and filtered motor speeds and IMU
 * readings into their respective previous-sample variables. These delayed
 * values are used for derivative calculations, filtering, or control loops
 * that require knowledge of the previous state.
 *
 * @param[in,out] motor Pointer to the Motor_t structure containing current and previous motor speeds.
 * @param[in,out] IMU Pointer to the Rocker_IMU_t structure containing current and previous IMU measurements.
 *
 * @return None
 *
 * @note
 * - Should be called at the end of a control loop iteration to update
 *   previous values for the next cycle.
 */
static void DelayedVals_Update(Motor_t *motor, Rocker_IMU_t *IMU)
{
	 motor->Motor_spd_raw_z1 = motor->Motor_spd_raw;
     motor->Motor_spd_z1 = motor->Motor_spd;
     IMU->Omega_Y_z1 = IMU->Omega_Y;
	 IMU->Acc_X_z1 = IMU->Acc_X;
}


static void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
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
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }

}

static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 9;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 19999;
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
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967294;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 5;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 5;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

static void MX_USART1_UART_Init(void)
{

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

}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8
                          |GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB4 PB5 PB8
                           PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


}

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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
