/*
 * i2c_dma_sens.c
 *
 *  Created on: Jul 19, 2025
 *      Author: Leon
 */

#include "i2c_dma_sens.h"

// Global variable definitions
volatile uint8_t currentSensor;
volatile uint8_t dmaStep;

uint8_t instructionArray[2] = {0x30, 0x0A};
uint8_t addressArray[1] = {0x06};
uint8_t receiveArray[5];

float pressureArray[NUM_OF_SENSORS];
float temperatureArray[NUM_OF_SENSORS];
float calibration[NUM_OF_SENSORS] = { 10000 / 1.5  / 5/1.3*1.9*1.4, 3000 / 1.40 /2 , 2000/1.1/1.5/1.1, 1200.0*1.1/1.4, 1, 1, 1, 1, 1, 1 };
float tempcal[NUM_OF_SENSORS] = { 2.15, 2, 2.15, 2, 1, 1, 1, 1, 1, 1 };
float runningAveragePressure[NUM_OF_SENSORS][RUNAVGAM];
float runningAverageTemperature[NUM_OF_SENSORS][RUNAVGAM];
float rawPressureArray[NUM_OF_SENSORS];
float rawTemperatureArray[NUM_OF_SENSORS];

const int selectPins[3] = { GPIO_PIN_10, GPIO_PIN_9, GPIO_PIN_8 };

void selectMuxPin(uint8_t pin) {
    for (uint8_t j = 0; j < 3; j++) {
        if (pin & (1 << j)) {
            HAL_GPIO_WritePin(GPIOA, selectPins[j], GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(GPIOA, selectPins[j], GPIO_PIN_RESET);
        }
    }
}

void muxInit() {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

    // Initialize MUX
    for (uint8_t i = 0; i < 3; i++) {
        HAL_GPIO_WritePin(GPIOA, selectPins[i], GPIO_PIN_SET);
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
/*
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

    currentSensor = (currentSensor + 1) % NUM_OF_SENSORS;
    dmaStep = 0;
    selectMuxPin(currentSensor);
    HAL_I2C_Master_Transmit_DMA(&hi2c3, 0x7F << 1, instructionArray, 2);
}
*/

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c != &hi2c3) return;

    // Extract raw data from receiveArray
    uint32_t rawPressure = (receiveArray[0] << 16) | (receiveArray[1] << 8) | receiveArray[2];
    uint16_t rawTemperature = (receiveArray[3] << 8) | receiveArray[4];

    // Store raw data into arrays for later processing
    rawPressureArray[currentSensor] = rawPressure;
    rawTemperatureArray[currentSensor] = rawTemperature;

    // Move to next sensor and trigger next DMA
    currentSensor = (currentSensor + 1) % NUM_OF_SENSORS;
    dmaStep = 0;
    selectMuxPin(currentSensor);
    HAL_I2C_Master_Transmit_DMA(&hi2c3, 0x7F << 1, instructionArray, 2);
}




void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c != &hi2c3) return;

    HAL_I2C_DeInit(hi2c);
    HAL_I2C_Init(hi2c);

    currentSensor = (currentSensor + 1) % NUM_OF_SENSORS;
    dmaStep = 0;
    selectMuxPin(currentSensor);
    HAL_I2C_Master_Transmit_DMA(&hi2c3, 0x7F << 1, instructionArray, 2);
}

void ProcessSensorData(uint8_t sensorIndex) {
    uint32_t rawPressure = rawPressureArray[sensorIndex];
    uint16_t rawTemperature = rawTemperatureArray[sensorIndex];

    float fpressureData = rawPressure;
    float fpressureData2;

    if (fpressureData >= 8388608) {
        fpressureData2 = (fpressureData - 16777216.0f) * Fullscale_P * calibration[sensorIndex] / 8388608.0f;
    } else {
        fpressureData2 = fpressureData / 8388608.0f * Fullscale_P * calibration[sensorIndex];
    }

    float truePressure = fpressureData2;
    float pressureSum = 0;

    for (uint8_t j = 0; j < RUNAVGAM; j++) {
        if (j == RUNAVGAM - 1) {
            runningAveragePressure[sensorIndex][j] = truePressure;
        } else {
            runningAveragePressure[sensorIndex][j] = runningAveragePressure[sensorIndex][j + 1];
        }
        pressureSum += runningAveragePressure[sensorIndex][j];
    }

    float pressureAverage = pressureSum / RUNAVGAM;

    float ftemperature = rawTemperature;
    float trueTemperature = ftemperature / 256.0f * tempcal[sensorIndex];
    float temperatureSum = 0;

    for (uint8_t j = 0; j < RUNAVGAM; j++) {
        if (j == RUNAVGAM - 1) {
            runningAverageTemperature[sensorIndex][j] = trueTemperature;
        } else {
            runningAverageTemperature[sensorIndex][j] = runningAverageTemperature[sensorIndex][j + 1];
        }
        temperatureSum += runningAverageTemperature[sensorIndex][j];
    }

    float temperatureAverage = temperatureSum / RUNAVGAM;

    pressureArray[sensorIndex] = pressureAverage;
    temperatureArray[sensorIndex] = temperatureAverage;
}



