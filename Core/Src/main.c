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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MUXE	GPIO_PIN_7
#define MUXP1	GPIO_PIN_10
#define MUXP2	GPIO_PIN_9
#define MUXP3	GPIO_PIN_8
#define MGO		GPIOA

#define NUM_OF_SENSORS 4
#define RUNAVGAM 10
#define Fullscale_P 4000.0f

uint8_t currentSensor = 0;
uint8_t dmaStep = 0;

uint8_t instructionArray[2] = {0x30, 0x0A};
uint8_t addressArray[1] = {0x06};
uint8_t receiveArray[5];

float pressureArray[NUM_OF_SENSORS];
float temperatureArray[NUM_OF_SENSORS];
float calibration[NUM_OF_SENSORS] = { 1000  / 1.5 / 1.10/5, 300 / 1.40/2, 200, 120.0/1.3, 1, 1, 1, 1, 1, 1 };;
float tempcal[NUM_OF_SENSORS] = { 2.15, 2, 2.15, 2, 1, 1, 1, 1, 1, 1 };;
float runningAveragePressure[NUM_OF_SENSORS][RUNAVGAM];
float runningAverageTemperature[NUM_OF_SENSORS][RUNAVGAM];

const int selectPins[3] = { GPIO_PIN_10, GPIO_PIN_9, GPIO_PIN_8 };

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
    .timeO = 8000,  // Full open time in ms
    .timeC = 8000,  // Full close time in ms
    .current_openness = 0,
    .target_openness = 0,
    .state = VALVE_IDLE,
    .start_time = 0,
    .move_duration = 0
};
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c3;
DMA_HandleTypeDef hdma_i2c3_tx;
DMA_HandleTypeDef hdma_i2c3_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C3_Init(void);
/* USER CODE BEGIN PFP */
void selectMuxPin(uint8_t pin) {
	for (uint8_t j = 0; j < 3; j++) {
		if (pin & (1 << j)) {
			if (j == 2) {
				HAL_GPIO_WritePin(GPIOA, selectPins[j], GPIO_PIN_SET);
			} else {
				HAL_GPIO_WritePin(GPIOA, selectPins[j], GPIO_PIN_SET);
			}
		} else {
			if (j == 2) {
				HAL_GPIO_WritePin(GPIOA, selectPins[j], GPIO_PIN_RESET);
			} else {
				HAL_GPIO_WritePin(GPIOA, selectPins[j], GPIO_PIN_RESET);
			}
		}
	}
}

void muxInit(){
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

		//Initialize MUX
		for (uint8_t i = 0; i < 3; i++) {
			if (i == 2) {
				HAL_GPIO_WritePin(GPIOA, selectPins[i], GPIO_PIN_SET);
			} else {
				HAL_GPIO_WritePin(GPIOA, selectPins[i], GPIO_PIN_SET);
			}
		}
}

void startSensorReadSequence() {
    currentSensor = 0;
    dmaStep = 0;
    selectMuxPin(currentSensor);
    HAL_I2C_Master_Transmit_DMA(&hi2c3, 0x7F << 1, instructionArray, 2);
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c != &hi2c3) return;

    if (dmaStep == 0) {
        dmaStep = 1;
        HAL_I2C_Master_Transmit_DMA(&hi2c3, 0x7F << 1, addressArray, 1);
    } else if (dmaStep == 1) {
        dmaStep = 2;
        HAL_I2C_Master_Receive_DMA(&hi2c3, 0x7F << 1, receiveArray, 5);
    }
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c != &hi2c3) return;

    uint32_t rawPressureData = (receiveArray[0] << 16) | (receiveArray[1] << 8) | receiveArray[2];
    float fpressureData = rawPressureData;
    float fpressureData2, pressureSum = 0;
    float temperatureSum = 0;

    if (fpressureData >= 8388608) {
        fpressureData2 = (fpressureData - 16777216.0f) * Fullscale_P * calibration[currentSensor] / 8388608.0f;
    } else {
        fpressureData2 = fpressureData / 8388608.0f * Fullscale_P * calibration[currentSensor];
    }
    float truePressureData = fpressureData2;

    for (uint8_t j = 0; j < RUNAVGAM; j++) {
        if (j == RUNAVGAM - 1) {
            runningAveragePressure[currentSensor][j] = truePressureData;
        } else {
            runningAveragePressure[currentSensor][j] = runningAveragePressure[currentSensor][j + 1];
        }
        pressureSum += runningAveragePressure[currentSensor][j];
    }
    float pressureAverage = pressureSum / RUNAVGAM;

    uint16_t rawTemperatureData = (receiveArray[3] << 8) | receiveArray[4];
    float ftemperatureData = rawTemperatureData;
    float trueTemperature = ftemperatureData / 256.0f * tempcal[currentSensor];

    for (uint8_t j = 0; j < RUNAVGAM; j++) {
        if (j == RUNAVGAM - 1) {
            runningAverageTemperature[currentSensor][j] = trueTemperature;
        } else {
            runningAverageTemperature[currentSensor][j] = runningAverageTemperature[currentSensor][j + 1];
        }
        temperatureSum += runningAverageTemperature[currentSensor][j];
    }
    float temperatureAverage = temperatureSum / RUNAVGAM;

    pressureArray[currentSensor] = pressureAverage;
    temperatureArray[currentSensor] = temperatureAverage;

    // Move to next sensor
    currentSensor++;
    currentSensor = (currentSensor + 1) % NUM_OF_SENSORS;
    dmaStep = 0;
    selectMuxPin(currentSensor);
    HAL_I2C_Master_Transmit_DMA(&hi2c3, 0x7F << 1, instructionArray, 2);
}




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
    uint8_t instructionArray[8];
  	uint8_t addressArray[8];
  	uint8_t receiveArray[5];
  	uint32_t rawPressureData;
  	int32_t rawTemperatureData;
  	float fpressureData;
  	float fpressureData2;
  	float truePressureData;
  	float runningAveragePressure[NUM_OF_SENSORS][5];
  	float ftemperatureData;
  	float runningAverageTemperature[NUM_OF_SENSORS][5];
  	float temperatureSum = 0;
  	float temperatureAverage = 0;
  	float trueTemparature;
  	float pressureSum = 0;
  	float pressureAverage = 0;
  	uint32_t timeRef1 = 0;
  	uint32_t timeRef2 = 0;
  	uint32_t timeRef3 = 0;
  	uint32_t timeRef4 = 0;
  	uint32_t timeOpen1 = 0;
  	uint32_t timeOpen2 = 0;
  	uint32_t opento1 = 0;
  	uint32_t opento2 = 0;
  	uint32_t opento3 = 0;
  	uint32_t opento4 = 0;
  	uint32_t openTick1 = 0;
  	uint32_t openTick2 = 0;
  	uint32_t openTick3 = 0;
  	uint32_t openTick4 = 0;


  	//Sensor calibration values
  	 //float calibration[10] = { 1000  / 1.5 / 1.10/5, 300 / 1.40/2, 200, 120.0/1.3, 1, 1, 1, 1, 1, 1 };
  	 //float tempcal[10] = { 2.15, 2, 2.15, 2, 1, 1, 1, 1, 1, 1 };
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */
  muxInit();
  	/*
  	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

	//Initialize MUX
	for (uint8_t i = 0; i < 3; i++) {
		if (i == 2) {
			HAL_GPIO_WritePin(GPIOA, selectPins[i], GPIO_PIN_SET);
		} else {
			HAL_GPIO_WritePin(GPIOA, selectPins[i], GPIO_PIN_SET);
		}
	}
	*/
  startSensorReadSequence();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET); // Manually set PC8 high
	  /*
	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15);
	  isCon = !HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1);
	  isOn =  HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0);
	  ballin =  HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);
	  HAL_Delay(4000);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 0);

	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 0);

	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
	*/
	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
	  HAL_Delay(100);
	  valve_set_openness(&bal1, 128);
	  valve_update(&bal1);
	  /*
	  for (uint8_t i = 0; i < NUM_OF_SENSORS; i++) {
		  	 selectMuxPin(i);
		  //Set instructions for temperature and pressure sensors
			instructionArray[0] = 0x30;
			instructionArray[1] = 0x0A;
			addressArray[0] = 0x06;
			HAL_StatusTypeDef status0 =  HAL_I2C_Master_Transmit(&hi2c3, 0x7f << 1, instructionArray, 2, 200);
			HAL_StatusTypeDef status1 =  HAL_I2C_Master_Transmit(&hi2c3, 0x7f << 1, addressArray, 1, 200);
			HAL_StatusTypeDef status2 =  HAL_I2C_Master_Receive(&hi2c3, 0x7f << 1, receiveArray, 5, 200);


			//Pressure data interpretation
			rawPressureData = (receiveArray[0] << 16) | (receiveArray[1] << 8) | (receiveArray[2]);
			fpressureData = rawPressureData;
			if (fpressureData >= 8388608) {
				fpressureData2 = (fpressureData - 16777216.0) * Fullscale_P * calibration[i] / 8388608.0;
			} else {
				fpressureData2 = fpressureData / 8388608.0 * Fullscale_P * calibration[i]; //delis zaradi max vrednosti
			}
			truePressureData = fpressureData2;

			//Running average  for stable data
			for (uint8_t j = 0; j < RUNAVGAM; j++) {
				if (j == 0) {
					pressureSum = 0;
				}
				if (j == RUNAVGAM - 1) {
					runningAveragePressure[i][j] = truePressureData;
					pressureSum += runningAveragePressure[i][j];
					break;
				}
				runningAveragePressure[i][j] = runningAveragePressure[i][j + 1];
				pressureSum += runningAveragePressure[i][j];
			}
			pressureAverage = pressureSum / RUNAVGAM;

			//Temperature data interpretation
			rawTemperatureData = (receiveArray[3] << 8) | receiveArray[4];
			ftemperatureData = rawTemperatureData;
			trueTemparature = ftemperatureData / 256.0 * tempcal[i];

			//Running average  for stable data
			for (uint8_t j = 0; j < RUNAVGAM; j++) {
				if (j == 0) {
					temperatureSum = 0;
				}
				if (j == RUNAVGAM - 1) {
					runningAverageTemperature[i][j] = trueTemparature;
					temperatureSum += runningAverageTemperature[i][j];
					break;
				}
				runningAverageTemperature[i][j] = runningAverageTemperature[i][j + 1];
				temperatureSum += runningAverageTemperature[i][j];
			}
			temperatureAverage = temperatureSum / RUNAVGAM;
			pressureArray[i] = pressureAverage;
			temperatureArray[i] = temperatureAverage;

	  }
	  */
	  /*
	  HAL_Delay(2000);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 0);

	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 1);

	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
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
