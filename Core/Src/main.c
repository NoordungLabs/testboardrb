/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "valve.h"
#include "nslp_dma.h"
#include "i2c_dma_sens.h"
//remove in stm32g4xx_it.c in usart1_irqHandler and after uartinit
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
ValveController bal1 = {
    .pinO = GPIO_PIN_13,
    .busO = (int)GPIOC,
    .pinC = GPIO_PIN_14,
    .busC = (int)GPIOC,
	.funPin = GPIO_PIN_12,
	.funBus = (int)GPIOB,
    .timeO = 6500,  // Full open time in ms
    .timeC = 6500,  // Full close time in ms
    .current_openness = 0,
    .target_openness = 0,
    .state = VALVE_IDLE,
    .start_time = 0,
    .move_duration = 0,
	.valvecal = 2.83
};

struct Packet *rx;

ValveController bal2 = {
    .pinO = GPIO_PIN_6,
    .busO = (int)GPIOB,
    .pinC = GPIO_PIN_7,
    .busC = (int)GPIOB,
	.funPin = GPIO_PIN_11,
	.funBus = (int)GPIOB,
    .timeO = 6500,  // Full open time in ms
    .timeC = 6500,  // Full close time in ms
    .current_openness = 0,
    .target_openness = 0,
    .state = VALVE_IDLE,
    .start_time = 0,
    .move_duration = 0,
	.valvecal = 2.83
};

/*
Solenoid air1 	=	{GPIO_PIN_2, GPIOD, GPIO_PIN_1, GPIOC, GPIO_PIN_0, GPIOC, 0, 0};
Solenoid air2 	=	{GPIO_PIN_3, GPIOB, GPIO_PIN_3, GPIOC, GPIO_PIN_2, GPIOC, 0, 0};
Solenoid liq1 	=	{GPIO_PIN_4, GPIOB, GPIO_PIN_1, GPIOA, GPIO_PIN_0, GPIOA, 0, 0};
Solenoid liq2 	=	{GPIO_PIN_5, GPIOB, GPIO_PIN_3, GPIOA, GPIO_PIN_2, GPIOA, 0, 0};
Solenoid ven1 	=	{GPIO_PIN_6, GPIOB, GPIO_PIN_5, GPIOA, GPIO_PIN_4, GPIOA, 0, 0};
Solenoid ven2 	=	{GPIO_PIN_7, GPIOB, GPIO_PIN_7, GPIOA, GPIO_PIN_6, GPIOA, 0, 0};
Ignitor ig1 	=	{GPIO_PIN_14, GPIOC, GPIO_PIN_10, GPIOB, GPIO_PIN_2, GPIOB, 0, 0};
*/
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c3;
DMA_HandleTypeDef hdma_i2c3_tx;
DMA_HandleTypeDef hdma_i2c3_rx;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */
/*
void on_packet_received(struct Packet *p) {
    switch (p->type) {
        case 'a':
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); // Example: toggle LED on type 'a'
            break;
        case 'b':
            // Do something else
            break;
        default:
            break;
    }
}
*/
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t isCon;
uint8_t isOn;
uint8_t ballin;
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
  	uint32_t opento1 = 0;
  	uint32_t opento2 = 0;
  	uint8_t debug = 0;
  	uint8_t flag = 0;
  	uint32_t timepre = 0;
  	uint32_t timeref1 = 0;
  	uint32_t timec = 0;



  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C3_Init();
  MX_USART1_UART_Init();
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */
  muxInit();
  startSensorReadSequence();
  nslp_dma_init(&huart1, &hcrc);
  //nslp_set_rx_callback(on_packet_received);

  HAL_GPIO_WritePin(bal2.busC, bal2.pinC, 0);
  HAL_GPIO_WritePin(bal2.busO, bal2.pinO, 0);
  /*
  HAL_Delay(10000);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 0);

  valve_set_openness(&bal1, 127);
  valve_update(&bal1);
  */
  //valve_calibrate(&bal1);
  //valve_calibrate(&bal2);
  //valve_close(&bal1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  timec = HAL_GetTick();
	  if (timec - timeref1 > 10000){
		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15);
		  timeref1 = timec;
		  flag = !flag;

		  if (flag){
			  valve_set_openness(&bal1, 255);
			  valve_set_openness(&bal2, 128);
			  /*
			  HAL_GPIO_WritePin(bal1.busC, bal1.pinC, 0);
			  HAL_GPIO_WritePin(bal1.busO, bal1.pinO, 0);
			  HAL_Delay(1000);
			  HAL_GPIO_WritePin(bal1.busC, bal1.pinC, 1);
			  */

		  }
		  else {
			  valve_set_openness(&bal1, 100);
			  valve_set_openness(&bal2, 0);
			  /*
			  HAL_GPIO_WritePin(bal1.busC, bal1.pinC, 0);
			  HAL_GPIO_WritePin(bal1.busO, bal1.pinO, 0);
			  HAL_Delay(1000);
			  HAL_GPIO_WritePin(bal1.busO, bal1.pinO, 1);
			  */
		  }

	  }

	  //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15);
	  //HAL_Delay(1000);
	  isOn = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0);
	  isCon = !HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1);

	  struct Packet Pressure = {
			.type = 'p',
			.size = sizeof(pressureArray),
			.payload = pressureArray
		};

	  struct Packet Temperature = {
			.type = 't',
			.size = sizeof(temperatureArray),
			.payload = temperatureArray
		};

	  send_packet_dma(&Temperature);
	  send_packet_dma(&Pressure);

	  valve_update(&bal1); //Purely while debugging
	  valve_update(&bal2); //Purely while debugging

	  uint32_t time = HAL_GetTick();

	  if (time - timepre > 100){
		  if (!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15)){
			  switch (debug){
			  case 0:
				  valve_set_openness(&bal1, 0);
				  break;
			  case 1:
				  valve_set_openness(&bal1, 255);
				  break;
			  case 2:
				  valve_set_openness(&bal1, 127);
				  break;
			  case 3:
				  valve_set_openness(&bal1, 100);
				  break;
			  case 4:
				  valve_set_openness(&bal1, 200);
				  break;
			  default:
				  break;

			  }
			  if (debug > 5 ){debug = 0;}
			  else debug++;

		  }
		  timepre = time;
	  }

	  rx = nslp_get_received_packet();
	  if (rx && rx->payload != NULL) {

		  uint8_t yay = 1;
	  }


	  /*


			switch(safeCopy.type){
	  		case (SADD):
				(c.payload & (1 << 0)) ? HAL_GPIO_WritePin(air1.onbus, air1.onpin, 1) : HAL_GPIO_WritePin(air1.onbus, air1.onpin, 0);
				(c.payload & (1 << 1)) ? HAL_GPIO_WritePin(air2.onbus, air2.onpin, 1) : HAL_GPIO_WritePin(air2.onbus, air2.onpin, 0);
				(c.payload & (1 << 2)) ? HAL_GPIO_WritePin(liq1.onbus, liq1.onpin, 1) : HAL_GPIO_WritePin(liq1.onbus, liq1.onpin, 0);
				(c.payload & (1 << 3)) ? HAL_GPIO_WritePin(liq2.onbus, liq2.onpin, 1) : HAL_GPIO_WritePin(liq2.onbus, liq2.onpin, 0);
				(c.payload & (1 << 4)) ? HAL_GPIO_WritePin(ven1.onbus, ven1.onpin, 1) : HAL_GPIO_WritePin(ven1.onbus, ven1.onpin, 0);
				(c.payload & (1 << 5)) ? HAL_GPIO_WritePin(ven2.onbus, ven2.onpin, 1) : HAL_GPIO_WritePin(ven2.onbus, ven2.onpin, 0);
				(c.payload & (1 << 6)) ? HAL_GPIO_WritePin(ig1.onbus, ig1.onpin, 1)   : HAL_GPIO_WritePin(ig1.onbus, ig1.onpin, 0);
				break;
	  		case (BADA1):
				opento1 = c.payload;
	  			break;
	  		case (BADA2):
				opento2 = c.payload;
				break;
			}

	  //Is line really on
	  		uint8_t isOn = 0;
	  		air1.isOn = HAL_GPIO_ReadPin(air1.onbus, air1.onpin);
	  		air2.isOn = HAL_GPIO_ReadPin(air2.onbus, air2.onpin);
	  		liq1.isOn = HAL_GPIO_ReadPin(liq1.onbus, liq1.onpin);
	  		liq2.isOn = HAL_GPIO_ReadPin(liq2.onbus, liq2.onpin);
	  		ven1.isOn = HAL_GPIO_ReadPin(ven1.onbus, ven1.onpin);
	  		ven2.isOn = HAL_GPIO_ReadPin(ven2.onbus, ven2.onpin);
	  		ig1.isOn  = HAL_GPIO_ReadPin(ig1.onbus, ig1.onpin);
	  		(air1.isOn) ? (isOn |= (1 << 0)) : (isOn &= ~(1 << 0));
	  		(air2.isOn) ? (isOn |= (1 << 1)) : (isOn &= ~(1 << 1));
	  		(liq1.isOn) ? (isOn |= (1 << 2)) : (isOn &= ~(1 << 2));
	  		(liq2.isOn) ? (isOn |= (1 << 3)) : (isOn &= ~(1 << 3));
	  		(ven1.isOn) ? (isOn |= (1 << 4)) : (isOn &= ~(1 << 4));
	  		(ven2.isOn) ? (isOn |= (1 << 5)) : (isOn &= ~(1 << 5));
	  		(ig1.isOn)  ? (isOn |= (1 << 6)) : (isOn &= ~(1 << 6));

	  		//Check if relay is connected
	  		uint8_t isCon = 0;
	  		air1.isCon = !HAL_GPIO_ReadPin(air1.conBus, air1.conPin);
	  		air2.isCon = !HAL_GPIO_ReadPin(air2.conBus, air2.conPin);
	  		liq1.isCon = !HAL_GPIO_ReadPin(liq1.conBus, liq1.conPin);
	  		liq2.isCon = !HAL_GPIO_ReadPin(liq2.conBus, liq2.conPin);
	  		ven1.isCon = !HAL_GPIO_ReadPin(ven1.conBus, ven1.conPin);
	  		ven2.isCon = !HAL_GPIO_ReadPin(ven2.conBus, ven2.conPin);
	  		ig1.isCon  = !HAL_GPIO_ReadPin(ig1.conBus, ig1.conPin);
	  		(air1.isCon) ? (isCon |= (1 << 0)) : (isCon &= ~(1 << 0));
	  		(air2.isCon) ? (isCon |= (1 << 1)) : (isCon &= ~(1 << 1));
	  		(liq1.isCon) ? (isCon |= (1 << 2)) : (isCon &= ~(1 << 2));
	  		(liq2.isCon) ? (isCon |= (1 << 3)) : (isCon &= ~(1 << 3));
	  		(ven1.isCon) ? (isCon |= (1 << 4)) : (isCon &= ~(1 << 4));
	  		(ven2.isCon) ? (isCon |= (1 << 5)) : (isCon &= ~(1 << 5));
	  		(ig1.isCon)  ? (isCon |= (1 << 6)) : (isCon &= ~(1 << 6));

	  	  	//Check if relay is open
	  		uint8_t isFun = 0;
	  		air1.isFun = HAL_GPIO_ReadPin(air1.funBus, air1.funPin);
	  		air2.isFun = HAL_GPIO_ReadPin(air2.funBus, air2.funPin);
	  		liq1.isFun = HAL_GPIO_ReadPin(liq1.funBus, liq1.funPin);
	  		liq2.isFun = HAL_GPIO_ReadPin(liq2.funBus, liq2.funPin);
	  		ven1.isFun = HAL_GPIO_ReadPin(ven1.funBus, ven1.funPin);
	  		ven2.isFun = HAL_GPIO_ReadPin(ven2.funBus, ven2.funPin);
	  		ig1.isFun  = HAL_GPIO_ReadPin(ig1.funBus, ig1.funPin);
	  		(air1.isFun) ? (isFun |= (1 << 0)) : (isFun &= ~(1 << 0));
	  		(air2.isFun) ? (isFun |= (1 << 1)) : (isFun &= ~(1 << 1));
	  		(liq1.isFun) ? (isFun |= (1 << 2)) : (isFun &= ~(1 << 2));
	  		(liq2.isFun) ? (isFun |= (1 << 3)) : (isFun &= ~(1 << 3));
	  		(ven1.isFun) ? (isFun |= (1 << 4)) : (isFun &= ~(1 << 4));
	  		(ven2.isFun) ? (isFun |= (1 << 5)) : (isFun &= ~(1 << 5));
	  		(ig1.isFun)  ? (isFun |= (1 << 6)) : (isFun &= ~(1 << 6));

	  		valve_set_openness(&bal1, opento1);
	  		valve_set_openness(&bal2, opento2);
	  		valve_update(&bal1);
	  		valve_update(&bal2);


  struct Packet Pressure = {
		.type = 0xA0,
		.size = sizeof(pressureArray),
		.payload = pressureArray
	};

  struct Packet Temperature = {
		.type = 0xA1,
		.size = sizeof(temperatureArray),
		.payload = temperatureArray
	};

  struct Packet Ball1State = {
		.type = 0xA3,
		.size = sizeof(pressureArray),
		.payload = pressureArray
	};

  struct Packet Ball1CurrentPos = {
		.type = 0xA4,
		.size = sizeof(temperatureArray),
		.payload = temperatureArray
	};


  struct Packet Ball2State = {
		.type = 0xA5,
		.size = sizeof(pressureArray),
		.payload = pressureArray
	};

  struct Packet Ball2CurrentPos = {
		.type = 0xA6,
		.size = sizeof(temperatureArray),
		.payload = temperatureArray
	};

  struct Packet SolIsCon = {
		.type = 0xA7,
		.size = sizeof(isCon),
		.payload = isCon
	};

  struct Packet SolIsOn = {
		.type = 0xA8,
		.size = sizeof(isOn),
		.payload = isOn
	};

  struct Packet SolISFun = {
		.type = 0xA9,
		.size = sizeof(isFun),
		.payload = isFun
	};

  send_packet_dma(&Temperature);
  send_packet_dma(&Pressure);
  send_packet_dma(&Bal1State);
  send_packet_dma(&Bal1CurrentPos);
  send_packet_dma(&Bal2State);
  send_packet_dma(&Bal2CurrentPos);
  send_packet_dma(&SolIsCon);
  send_packet_dma(&SolIsOn);
  send_packet_dma(&SolISFun);



	  */




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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x00503D58;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */
  //__HAL_RCC_I2C3_CONFIG(RCC_I2C3CLKSOURCE_HSI);
  //__HAL_RCC_I2C3_CLK_ENABLE();
  /* USER CODE END I2C3_Init 2 */

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
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC15 PC0 PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB11 PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB13 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 PA10 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
