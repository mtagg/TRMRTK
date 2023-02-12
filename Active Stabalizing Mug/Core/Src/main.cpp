/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include <math.h>

#include "MC3479.h"

#include "MP6543H.h"

#include "controlSystem.h"

#include "AS5048A.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define INACTIVITY_THRESHOLD 5000 //value where incactivity_counter will trigger sleep/idle

//#define __DEBUG_EN
#define __TEST_CODE_EN
//#define __IDLE_CURRENT_TEST_EN
#define __NORMAL_MODE_EN
//#define __SIMULINK_EN
//#define __OCP_MEASUREMENT_EN

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC2_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
HAL_StatusTypeDef UART_Send_16bit(UART_HandleTypeDef* uart, uint16_t data);
//bool UART_Send_String(UART_HandleTypeDef* uart, char str[], int char_cnt);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_ADC2_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */


  uint8_t xData [2];
  uint8_t yData [2];
  uint8_t zData [2];
//  uint8_t motionFlagStatus;
//  uint8_t motionIrqStatus;

// TODO: implement sleep routine
//  int inactivity_counter = 0;
//  bool x_inactive = false;
//  bool y_inactive = false;

  int16_t x_theta = 0;
  int16_t last_x_theta = 0;
  int16_t delta_x_theta;
  int16_t x_nominal = 0;
  //  int16_t y_theta = 0;
  int8_t allowableAngle = 3;
  uint16_t currentEncoderAngle;
  uint16_t lastEncoderAngle;
  int16_t deltaEncoderAngle = 0;
  //TODO: Update after installing on mug
  uint16_t encoderAngleOffset = 0;

// Initialize phase offsets:
//TODO: initialize phase angle to 0 position of handle
 uint8_t phaseAindex = 0;
 uint8_t phaseBindex = phaseAindex + 60;
 uint8_t phaseCindex = phaseAindex + 120;
 TIM1->CCR1 = ControlSystem.sineWave[phaseAindex];
 TIM1->CCR2 = ControlSystem.sineWave[phaseBindex];
 TIM1->CCR3 = ControlSystem.sineWave[phaseCindex];


#ifdef	 __SIMULINK_EN
	  //UART3 used for Simulink output/input
  uint8_t SimulinkPwm[2] = {0};
  uint8_t Simulink_Packet[8] = {0};
#endif //__SIMULINK_EN//

  //Configure Accelerometer using PA4 for CSn, in SPI mode
  MC3479.setSerialSPI(&hspi1, GPIOA, SPI1_CSn_Pin);
  MC3479.configAccelerometer();

  //Configure GPIO variables for x-axis
  MP6543H.x_configMotorController(TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, &htim1,
		  	  	  	  	  	  	  MP6543H_EN_A_GPIO_Port, MP6543H_EN_A_Pin,
								  	  MP6543H_EN_B_GPIO_Port, MP6543H_EN_B_Pin,
									  	  MP6543H_EN_C_GPIO_Port, MP6543H_EN_C_Pin,
									  	  	  MP6543H_nSLEEP_X_GPIO_Port, MP6543H_nSLEEP_X_Pin,
											  	  MP6543H_nFAULT_X_GPIO_Port, MP6543H_nFAULT_X_Pin);

  ControlSystem.initControlSystem(x_nominal, allowableAngle);
  AS5048A.SPI_Init(&hspi2, SPI2_SCn_GPIO_Port, SPI2_SCn_Pin);

  // Start all phase PWM signals
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  // Enable all Phase outputs
  HAL_GPIO_WritePin(MP6543H_EN_A_GPIO_Port, MP6543H_EN_A_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(MP6543H_EN_B_GPIO_Port, MP6543H_EN_B_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(MP6543H_EN_C_GPIO_Port, MP6543H_EN_C_Pin, GPIO_PIN_SET);

  // Read angle once to start read sequence cycle:
  lastEncoderAngle = AS5048A.readAngleSequential();
  // Initial encoder values:
  lastEncoderAngle = AS5048A.readAngleSequential() + encoderAngleOffset;
  currentEncoderAngle = AS5048A.readAngleSequential() + encoderAngleOffset;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	  // Brake if any motor fault or if tilt button is pressed.
	  while(HAL_GPIO_ReadPin(nTILT_BUTTON_GPIO_Port, nTILT_BUTTON_Pin) == 0){
		  HAL_Delay(1);
	  }
	  while(MP6543H.x_motorFault()){
		  HAL_Delay(1);
#ifdef __OCP_MEASUREMENT_EN
		  //This should allow us to see how long (approximately) we stay in the nFault loop
		  uint8_t fault_current [2] = {0xFF, 0xFF};
		  HAL_UART_Transmit(&huart3, &fault_current[0], sizeof(fault_current), HAL_MAX_DELAY);
#endif
	  }

#ifdef __NORMAL_MODE_EN

	  // fetch and normalize theta:
	  MC3479.getXYZ(xData, yData, zData);
	  last_x_theta = x_theta;
	  x_theta = ControlSystem.x_normalizeTheta(xData[0], xData[1], zData[0], zData[1]);
	  delta_x_theta = x_theta - last_x_theta;
	  //	  y_theta = ControlSystem.normalizeTheta(yData[0], yData[1], zData[0], zData[1]);

#ifdef	 __SIMULINK_EN

	  // NOTE: UART3 used for Simulink output/input

	  // Construct the Simulink Packet:
//	  Simulink_Packet[0] = (uint8_t)(x_theta & 0x00FF); // Bottom 8 bits of xTheta
//	  Simulink_Packet[1] = (uint8_t)(x_theta >> 8);		// Upper 8 bits of xTheta
//	  Simulink_Packet[2] = 0;							// Bottom 8 bits of xNominal
//	  Simulink_Packet[3] = 0;							// Upper 8 bits of xNominal
//	  Simulink_Packet[4] = (uint8_t)(y_theta & 0x00FF);	// Bottom 8 bits of yTheta
//	  Simulink_Packet[5] = (uint8_t)(y_theta >> 8);		// Upper 8 bits of yTheta
//	  Simulink_Packet[6] = 0;							// Bottom 8 bits of yNominal
//	  Simulink_Packet[7] = 0;							// Upper 8 bits of yNominal

// Send the Simulink Packet - Least significant Byte first - Byte 0 : Byte 7
// dividing simulink packet size by 4 to only send x_theta
//	  HAL_UART_Transmit(&huart3, &Simulink_Packet[0], sizeof(Simulink_Packet)/4, 10);
//	  HAL_UART_Transmit(&huart3, &Simulink_Packet[4], sizeof(Simulink_Packet)/4, 10);
	  UART_Send_16bit(&huart3, y_theta);


// Check for UART Rx Buffer:
// Loop until we receive xAxis PWM value in SimulinkPwm[0]:
//	  while (HAL_UART_Receive(&huart3, &SimulinkPwm[0], sizeof(SimulinkPwm[0]), 0) == HAL_TIMEOUT){
//		  HAL_Delay(1);
//	  }
//	  HAL_UART_Receive(&huart3, &SimulinkPwm[0], sizeof(SimulinkPwm[0]), 1);
#endif //__SIMULINK_EN//
#endif //__NORMAL_MODE_EN//
#ifdef __TEST_CODE_EN


		  // Update 3phase PWM values
		  int P = (20*((double)x_theta/90));			// Kp can be negative
		  double scalar = 0.3;
		  phaseAindex = (phaseAindex + 180 + P) % 180; // takes care of both directions
		  phaseBindex = (phaseAindex + 60) % 180;
		  phaseCindex = (phaseAindex + 120) % 180;

//		  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
//		  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
//		  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
		  TIM1->CCR1 = scalar*ControlSystem.sineWave[phaseAindex];
		  TIM1->CCR2 = scalar*ControlSystem.sineWave[phaseBindex];
		  TIM1->CCR3 = scalar*ControlSystem.sineWave[phaseCindex];
//		  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
//		  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
//		  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

// Get AS5048 Encoder PWM Value
//  	  HAL_ADC_Start(&hadc1);
//  	  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
//  	  uint16_t encoderPWM = HAL_ADC_GetValue(&hadc1);
//  	  uint8_t encoderMsg [2] = {(uint8_t)encoderPWM, (uint8_t)((encoderPWM>>8)&0xFF)};
	  lastEncoderAngle = currentEncoderAngle;
  	  currentEncoderAngle = AS5048A.readAngleSequential() + encoderAngleOffset;
  	  deltaEncoderAngle = currentEncoderAngle - lastEncoderAngle;
  	  UART_Send_16bit(&huart3, currentEncoderAngle);
//  	  uint8_t encoderMsg [2] = {(uint8_t)encoderAngle, (uint8_t)((encoderAngle>>8)&0xFF)};
//  	  HAL_UART_Transmit(&huart3, &encoderMsg[0], sizeof(encoderMsg), 1);
//  	  uint16_t Nop = AS5048A.Transcieve_Nop();
//  	  uint8_t NopRcv [2] = {(uint8_t)Nop, (uint8_t)((Nop>>8)&0xFF)};
//  	  HAL_UART_Transmit(&huart3, &NopRcv[0], sizeof(NopRcv), 10);
//  	  HAL_Delay(1);
//  	  UART_Send_16bit(&huart3, x_theta);



#endif //__TEST_CODE_EN//
#ifdef __OCP_MEASUREMENT_EN

// Read ADC Channel 4:s
	HAL_ADC_Start(&hadc2);
	HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);
	uint16_t adc_current = HAL_ADC_GetValue(&hadc2);
	uint8_t p4[2] = {(uint8_t)(adc_current & 0x00FF), (uint8_t)(adc_current >> 8)};

// Read ADC Channel 5:
	HAL_ADC_Start(&hadc2);
	HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);
	adc_current = HAL_ADC_GetValue(&hadc2);
	uint8_t p5[2] = {(uint8_t)(adc_current & 0x00FF), (uint8_t)(adc_current >> 8)};

// Read ADC Channel 6:
	HAL_ADC_Start(&hadc2);
	HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);
	adc_current = HAL_ADC_GetValue(&hadc2);
	HAL_ADC_Stop(&hadc2);
	uint8_t p6[2] = {(uint8_t)(adc_current & 0x00FF), (uint8_t)(adc_current >> 8)};

// Transmit ADC 4,5,6 data:
	HAL_UART_Transmit(&huart3, &p4[0], sizeof(p4), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart3, &p5[0], sizeof(p5), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart3, &p6[0], sizeof(p6), HAL_MAX_DELAY);
//	HAL_Delay(4000);

#endif //__OCP_MEASUREMENT_EN//

#ifdef __IDLE_CURRENT_TEST_EN
	  MC3479.setSampleRate(RATE0_50Hz);
//	  MP6543H.x_motorBrake(true);
//	  MP6543H.y_motorBrake(true);
	  MP6543H.x_motorSleep();
	  MP6543H.y_motorSleep();
	  HAL_Delay(5000);
	  MP6543H.x_motorWake();
	  MP6543H.y_motorWake();
//	  MP6543H.x_motorBrake(false);
//	  MP6543H.y_motorBrake(false);
	  HAL_Delay(5000);
#endif //__IDLE_CURRENT_TEST_EN//


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = ENABLE;
  hadc2.Init.NbrOfDiscConversion = 3;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 3;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 255;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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
  huart2.Init.BaudRate = 230400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 230400;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MP6543H_EN_A_Pin|MP6543H_nSLEEP_X_Pin|SPI1_CSn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MP6543H_EN_B_Pin|MP6543H_EN_C_Pin|SPI2_SCn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : MP6543H_EN_A_Pin MP6543H_nSLEEP_X_Pin SPI1_CSn_Pin */
  GPIO_InitStruct.Pin = MP6543H_EN_A_Pin|MP6543H_nSLEEP_X_Pin|SPI1_CSn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : MP6543H_EN_B_Pin MP6543H_EN_C_Pin SPI2_SCn_Pin */
  GPIO_InitStruct.Pin = MP6543H_EN_B_Pin|MP6543H_EN_C_Pin|SPI2_SCn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : MP6543H_nFAULT_X_Pin MC3479_INTN1_Pin MC3479_INTN2_Pin */
  GPIO_InitStruct.Pin = MP6543H_nFAULT_X_Pin|MC3479_INTN1_Pin|MC3479_INTN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : nTILT_BUTTON_Pin */
  GPIO_InitStruct.Pin = nTILT_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(nTILT_BUTTON_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
HAL_StatusTypeDef UART_Send_16bit(UART_HandleTypeDef* uart, uint16_t data)
{
	uint8_t Data[2]= {(uint8_t)(data & 0xFF), (uint8_t)((data >> 8) & 0xFF)};
	return HAL_UART_Transmit(uart, &Data[0], sizeof(Data), 1);
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
