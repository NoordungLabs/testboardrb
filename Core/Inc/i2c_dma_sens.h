/*
 * i2c_dma_sens.h
 *
 *  Created on: Jul 19, 2025
 *      Author: Leon
 */

#ifndef INC_I2C_DMA_SENS_H_
#define INC_I2C_DMA_SENS_H_

#include "stm32g4xx_hal.h"

// GPIO pin definitions
#define MUXE    GPIO_PIN_7
#define MUXP1   GPIO_PIN_10
#define MUXP2   GPIO_PIN_9
#define MUXP3   GPIO_PIN_8
#define MGO     GPIOA

// Configuration constants
#define NUM_OF_SENSORS 6
#define RUNAVGAM 5
#define Fullscale_P 4000.0f

// External I2C handle (must be defined elsewhere)
extern I2C_HandleTypeDef hi2c3;

// Sensor DMA state enumeration
typedef enum {
    SENSOR_STEP_INSTRUCTION,
    SENSOR_STEP_ADDRESS,
    SENSOR_STEP_RECEIVE
} SensorDMAState;

// Global variables
extern volatile uint8_t currentSensor;
extern volatile uint8_t dmaStep;

extern uint8_t instructionArray[2];
extern uint8_t addressArray[1];
extern uint8_t receiveArray[5];

extern float pressureArray[NUM_OF_SENSORS];
extern float temperatureArray[NUM_OF_SENSORS];
extern float calibration[NUM_OF_SENSORS];
extern float tempcal[NUM_OF_SENSORS];
extern float runningAveragePressure[NUM_OF_SENSORS][RUNAVGAM];
extern float runningAverageTemperature[NUM_OF_SENSORS][RUNAVGAM];

// Function declarations
void selectMuxPin(uint8_t pin);
void muxInit(void);
void startSensorReadSequence(void);
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c);




#endif /* INC_I2C_DMA_SENS_H_ */
