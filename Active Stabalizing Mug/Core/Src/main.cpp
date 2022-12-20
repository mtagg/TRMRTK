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
#include "MC3479.h"

#include "MP6543H.h"

#include "controlSystem.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define INACTIVITY_THRESHOLD 5000 //value where incactivity_counter will trigger sleep/idle

//#define __DEBUG_EN
//#define __TEST_CODE_EN
//#define __IDLE_CURRENT_TEST_EN
//#define __UART_TEST_EN
#define __NORMAL_MODE_EN
#define __SIMULINK_EN

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc2;

SPI_HandleTypeDef hspi1;

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
  /* USER CODE BEGIN 2 */

  //Configure Accelerometer using PA4 for CSn, in SPI mode
  MC3479.setSerialSPI(&hspi1, GPIOA, SPI1_CSn_Pin);
  MC3479.configAccelerometer();

  //Configure GPIO variables for x-axis
  MP6543H.x_configMotorController(TIM_CHANNEL_1, &htim1,
		  	  	  	  	  	  	  GPIOB, MP6543H_DIR_X_Pin,
								  	  GPIOB, MP6543H_nBRAKE_X_Pin,
									  	  GPIOB, MP6543H_nSLEEP_X_Pin,
										  	  GPIOB, MP6543H_nFAULT_X_Pin);
  // Configure GPIO variables for y-axis
  MP6543H.y_configMotorController(TIM_CHANNEL_2, &htim1,
		  	  	  	  	  	  	  GPIOA, MP6543H_DIR_Y_Pin,
								  	  GPIOA, MP6543H_nBRAKE_Y_Pin,
									  	  GPIOA, MP6543H_nSLEEP_Y_Pin,
										  	  GPIOB, MP6543H_nFAULT_Y_Pin);


//#ifdef __NORMAL_MODE_EN
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
  int16_t y_theta = 0;
  int16_t x_nominal = 0;
  int16_t y_nominal = 0;
  uint8_t xPWM = 0;
  uint8_t yPWM = 0;
  int8_t allowableAngle = 5;

  ControlSystem.updateControlSystem(x_nominal, y_nominal, allowableAngle, allowableAngle);

#ifdef	 __SIMULINK_EN
	  //UART3 used for Simulink output/input
  uint8_t SimulinkPwm[2] = {0};
  uint8_t Simulink_Packet[8] = {0};
#endif //__SIMULINK_EN//
//#endif //_NORMAL_MODE_EN//

#ifdef __DEBUG_EN
  uint8_t newLine [] = "\n\r";
#endif //__DEBUG_EN//



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  while (1)
  {
	  // Brake if any motor fault or if tilt button is pressed.
	  while(HAL_GPIO_ReadPin(nTILT_BUTTON_GPIO_Port, nTILT_BUTTON_Pin) == 0){
//		  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
		  MP6543H.x_motorBrake(true);
		  MP6543H.y_motorBrake(true);
		  HAL_Delay(1);
	  }
	  while(MP6543H.x_motorFault() || MP6543H.y_motorFault()){
//		  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
		  MP6543H.x_motorBrake(true);
		  MP6543H.y_motorBrake(true);
		  HAL_Delay(1);
	  }
	  MP6543H.x_motorBrake(false);
	  MP6543H.y_motorBrake(false);

#ifdef __NORMAL_MODE_EN


	  // Loop Delay (for testing)
//	  HAL_Delay(1);
	  // fetch and normalize theta:
	  MC3479.getXYZ(xData, yData, zData);
//	  HAL_UART_Transmit(&huart3, &xData[0], sizeof(xData), 10);
//	  HAL_UART_Transmit(&huart3, &yData[0], sizeof(yData), 10);
//	  HAL_UART_Transmit(&huart3, &zData[0], sizeof(zData), 10);

	  x_theta = ControlSystem.normalizeTheta(xData[0], xData[1], zData[0], zData[1]);
	  y_theta = ControlSystem.normalizeTheta(yData[0], yData[1], zData[0], zData[1]);
//	  motionFlagStatus = MC3479.getMotionFlagStatus();
//	  HAL_UART_Transmit(&huart2, &motionFlagStatus, sizeof(motionFlagStatus), 10);
//	  motionIrqStatus = MC3479.getMotionIrqStatus();
//	  HAL_UART_Transmit(&huart2, &motionIrqStatus, sizeof(motionIrqStatus), 10);
//	  MC3479.clearMotionIrqStatus();


	  //TEST
	  // set correct motor directions:
//	  if (x_theta < 0){
//		  MP6543H.x_setMotorDir(!CLOCKWISE_DIR);
//	  }else{
//		  MP6543H.x_setMotorDir(CLOCKWISE_DIR);
//	  }
	  if (y_theta < 0){
		  MP6543H.x_setMotorDir(!CLOCKWISE_DIR);
//		  MP6543H.y_setMotorDir(!CLOCKWISE_DIR);
	  }else{
//		  MP6543H.y_setMotorDir(CLOCKWISE_DIR);
		  MP6543H.x_setMotorDir(CLOCKWISE_DIR);
	  }
	  // END TEST

#ifdef	 __SIMULINK_EN

	  // NOTE: UART3 used for Simulink output/input

	  // Construct the Simulink Packet:
	  Simulink_Packet[0] = (uint8_t)(x_theta & 0x00FF); // Bottom 8 bits of xTheta
	  Simulink_Packet[1] = (uint8_t)(x_theta >> 8);		// Upper 8 bits of xTheta
	  Simulink_Packet[2] = 0;							// Bottom 8 bits of xNominal
	  Simulink_Packet[3] = 0;							// Upper 8 bits of xNominal
	  Simulink_Packet[4] = (uint8_t)(y_theta & 0x00FF);	// Bottom 8 bits of yTheta
	  Simulink_Packet[5] = (uint8_t)(y_theta >> 8);		// Upper 8 bits of yTheta
	  Simulink_Packet[6] = 0;							// Bottom 8 bits of yNominal
	  Simulink_Packet[7] = 0;							// Upper 8 bits of yNominal

	  // Send the Simulink Packet - Least significant Byte first - Byte 0 : Byte 7
	  // dividing simulink packet size by 4 to only send x_theta
//	  HAL_UART_Transmit(&huart3, &Simulink_Packet[0], sizeof(Simulink_Packet)/4, 10);
	  HAL_UART_Transmit(&huart3, &Simulink_Packet[4], sizeof(Simulink_Packet)/4, 10);
	  // Check for UART Rx Buffer:
	  // Loop until we recieve xAxis PWM value in SimulinkPwm[0]:
	  while (HAL_UART_Receive(&huart3, &SimulinkPwm[0], sizeof(SimulinkPwm[0]), 0) == HAL_TIMEOUT){
		  HAL_Delay(1);
	  }
	  xPWM = SimulinkPwm[0];
//	  //yPWM = SimulinkPwm[1];
	  MP6543H.setMotorPwm(xPWM);
//	  if (!MP6543H.x_motorFault()){
//
//	  	MP6543H.x_startMotorPwmDuration(MOTOR_CONTROL_DURATION);
//	  }
	  //if (!MP6543H.y_motorFault()){
		//  MP6543H.setMotorPwm(yPWM);
		//  MP6543H.y_startMotorPwmDuration(MOTOR_CONTROL_DURATION);
	  //}


#else

//	uint8_t xAng [] = {(uint8_t)x_theta, (uint8_t)(x_theta >> 8)};
//	HAL_UART_Transmit(&huart3, &xAng[0], sizeof(xAng), 1);
//	xPWM = ControlSystem.x_calcPwm(x_nominal, x_theta, y_nominal, y_theta);
//	MP6543H.setMotorPwm(xPWM);
//	if (!MP6543H.x_motorFault()){
//		MP6543H.x_motorBrake(false);
////		MP6543H.x_startMotorPwmDuration(MOTOR_CONTROL_DURATION);
//	}

//	uint8_t yAng [] = {(uint8_t)y_theta, (uint8_t)(y_theta >> 8)};
//	HAL_UART_Transmit(&huart3, &yAng[0], sizeof(yAng), 1);
	yPWM = ControlSystem.y_calcPwm(x_nominal, x_theta, y_nominal, y_theta);
	MP6543H.setMotorPwm(yPWM);
//	if (!MP6543H.y_motorFault()){
//		MP6543H.setMotorPwm(yPWM);
//		MP6543H.y_startMotorPwmDuration(MOTOR_CONTROL_DURATION);
//	}

#endif //__SIMULINK_EN//

#endif //__NORMAL_MODE_EN//

#ifdef __DEBUG_EN

	  HAL_UART_Transmit(&huart2, &newLine, sizeof(newLine), 10);
	  HAL_UART_Transmit(&huart2, &motionFlagStatus, sizeof(motionFlagStatus), 10);
	  HAL_UART_Transmit(&huart2, &motionIrqStatus, sizeof(motionIrqStatus), 10);

	  HAL_UART_Transmit(&huart2, &newLine, sizeof(newLine), 10);
	  HAL_UART_Transmit(&huart2, &xData[0], sizeof(xData), 10);
	  UART_Send_16bit(&huart2, x_theta);

	  HAL_UART_Transmit(&huart2, &newLine, sizeof(newLine), 10);
	  HAL_UART_Transmit(&huart2, &yData[0], sizeof(yData), 10);
	  UART_Send_16bit(&huart2, y_theta);

	  HAL_UART_Transmit(&huart2, &newLine, sizeof(newLine), 10);
	  HAL_UART_Transmit(&huart2, &zData[0], sizeof(zData), 10);

	  HAL_Delay(2000);
#endif //__DEBUG_EN//

#ifdef __TEST_CODE_EN


//	  // Start PWM for x motor
//	  MP6543H.x_motorBrake(false);
//	  MP6543H.setMotorPwm(0);
//	  MP6543H.x_startMotorPwmDuration(2000);
//
//	  MP6543H.x_motorBrake(true);
//	  MP6543H.setMotorPwm(25);
//	  MP6543H.x_startMotorPwmDuration(2000);
//
	  MP6543H.x_setMotorDir(COUNTER_CLOCKWISE_DIR);
	  MP6543H.x_motorBrake(false);
	  MP6543H.setMotorPwm(50);
	  MP6543H.x_startMotorPwmDuration(1000);
	  HAL_Delay(4000);
	  MP6543H.x_setMotorDir(CLOCKWISE_DIR);
	  MP6543H.x_startMotorPwmDuration(1000);
	  HAL_Delay(4000);
//
//	  MP6543H.x_motorBrake(true);
//	  MP6543H.setMotorPwm(75);
//	  MP6543H.x_startMotorPwmDuration(2000);
//
//	  MP6543H.x_motorBrake(true);
//	  MP6543H.setMotorPwm(100);
//	  MP6543H.x_startMotorPwmDuration(2000);






#endif //__TEST_CODE_EN//


#ifdef __IDLE_CURRENT_TEST_EN
	  MC3479.setSampleRate(RATE0_50Hz);
	  MP6543H.x_motorBrake(true);
	  MP6543H.y_motorBrake(true);
	  MP6543H.x_motorSleep();
	  MP6543H.y_motorSleep();
	  HAL_Delay(5000);
	  MP6543H.x_motorWake();
	  MP6543H.y_motorWake();
	  MP6543H.x_motorBrake(false);
	  MP6543H.y_motorBrake(false);
	  HAL_Delay(5000);
#endif //__IDLE_CURRENT_TEST_EN//


#ifdef __UART_TEST_EN
	  uint8_t test1 [] = "Hello";
	  HAL_UART_Transmit(&huart2, &test1[0], sizeof(test1), 10);
	  HAL_UART_Transmit(&huart2, &newLine[0], sizeof(newLine), 10);
	  HAL_UART_Transmit(&huart2, &newLine[0], sizeof(newLine), 10);
	  HAL_Delay(500);

	  uint8_t test4 [] = "Do sum Flip";
	  HAL_UART_Transmit(&huart3, &test4[0], sizeof(test4), 10);
	  HAL_UART_Transmit(&huart3, &newLine[0], sizeof(newLine), 10);
	  HAL_UART_Transmit(&huart3, &newLine[0], sizeof(newLine), 10);
	  HAL_Delay(500);
#endif //__UART_TEST_EN

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
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
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
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
  htim1.Init.Period = 100;
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
  sConfigOC.Pulse = 50;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MP6543H_DIR_X_Pin|MP6543H_nSLEEP_X_Pin|MP6543H_nBRAKE_X_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MP6543H_DIR_Y_Pin|MP6543H_nSLEEP_Y_Pin|MP6543H_nBRAKE_Y_Pin|SPI1_CSn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : nTILT_BUTTON_Pin */
  GPIO_InitStruct.Pin = nTILT_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(nTILT_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MP6543H_nFAULT_Y_Pin MP6543H_nFAULT_X_Pin MC3479_INTN1_Pin MC3479_INTN2_Pin */
  GPIO_InitStruct.Pin = MP6543H_nFAULT_Y_Pin|MP6543H_nFAULT_X_Pin|MC3479_INTN1_Pin|MC3479_INTN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : MP6543H_DIR_X_Pin MP6543H_nSLEEP_X_Pin MP6543H_nBRAKE_X_Pin */
  GPIO_InitStruct.Pin = MP6543H_DIR_X_Pin|MP6543H_nSLEEP_X_Pin|MP6543H_nBRAKE_X_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : MP6543H_DIR_Y_Pin MP6543H_nSLEEP_Y_Pin MP6543H_nBRAKE_Y_Pin SPI1_CSn_Pin */
  GPIO_InitStruct.Pin = MP6543H_DIR_Y_Pin|MP6543H_nSLEEP_Y_Pin|MP6543H_nBRAKE_Y_Pin|SPI1_CSn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
HAL_StatusTypeDef UART_Send_16bit(UART_HandleTypeDef* uart, uint16_t data)
{
	uint8_t Data[2]= {(uint8_t)(data & 0xFF), (uint8_t)((data >> 8) & 0xFF)};
	return HAL_UART_Transmit(uart, &Data[0], sizeof(Data), 10);
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
