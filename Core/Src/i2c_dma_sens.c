/*
 * i2c_dma_sens.c
 *
 *  Created on: Jul 19, 2025
 *      Author: Leon
 */
#include "nslp_dma.h"  // For nslp_rx_active  declaration
#include "i2c_dma_sens.h"


#define MAX_I2C_RETRIES 3

// Global variable definitions
volatile uint8_t currentSensor;
volatile uint8_t dmaStep;
volatile uint8_t i2c_paused = 0;
extern volatile uint8_t nslp_rx_active;  // Declaration (no initialization)


uint8_t instructionArray[2] = {0x30, 0x0A};
uint8_t addressArray[1] = {0x06};
uint8_t receiveArray[5];

float pressureArray[NUM_OF_SENSORS];
float temperatureArray[NUM_OF_SENSORS];
float calibration[NUM_OF_SENSORS] = { 10000 / 1.5  / 5/1.3*1.9*1.4/2.4, 3000 / 1.40 /2*1.6/2*1.2 , 2000/1.1/1.5/1.1*2/1.16*3/1.4/2*1.3, 1200.0*1.1/1.4*3/3*1.2, 1, 1, 1, 1, 1, 1 };
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

void pauseI2COperations() {
    i2c_paused = 1;
}

void resumeI2COperations() {
    if(i2c_paused) {
        i2c_paused = 0;
        startSensorReadSequence();
    }
}

void startSensorReadSequence() {
    // Don't start if UART is active or operations are paused
    if(nslp_rx_active  || i2c_paused) {
        return;
    }

    currentSensor = 0;
    dmaStep = 0;
    selectMuxPin(currentSensor);

    for (int attempt = 0; attempt < MAX_I2C_RETRIES; attempt++) {
        if (HAL_I2C_Master_Transmit_DMA(&hi2c3, 0x7F << 1, instructionArray, 2) == HAL_OK) {
            return;
        }
    }

    // Skip to next sensor on repeated failure
    HAL_I2C_ErrorCallback(&hi2c3);
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c != &hi2c3 || nslp_rx_active  || i2c_paused) {
        return;
    }

    HAL_StatusTypeDef status;

    if (dmaStep == 0) {
        dmaStep = 1;
        for (int attempt = 0; attempt < MAX_I2C_RETRIES; attempt++) {
            status = HAL_I2C_Master_Transmit_DMA(&hi2c3, 0x7F << 1, addressArray, 1);
            if (status == HAL_OK) return;
        }
        HAL_I2C_ErrorCallback(hi2c);
    } else if (dmaStep == 1) {
        dmaStep = 2;
        for (int attempt = 0; attempt < MAX_I2C_RETRIES; attempt++) {
            status = HAL_I2C_Master_Receive_DMA(&hi2c3, 0x7F << 1, receiveArray, 5);
            if (status == HAL_OK) return;
        }
        HAL_I2C_ErrorCallback(hi2c);
    }
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c != &hi2c3 || nslp_rx_active  || i2c_paused) {
        return;
    }

    // Extract raw data from receiveArray
    uint32_t rawPressure = (receiveArray[0] << 16) | (receiveArray[1] << 8) | receiveArray[2];
    uint16_t rawTemperature = (receiveArray[3] << 8) | receiveArray[4];

    rawPressureArray[currentSensor] = rawPressure;
    rawTemperatureArray[currentSensor] = rawTemperature;

    // Process the data immediately
    ProcessSensorData(currentSensor);

    // Move to next sensor
    currentSensor = (currentSensor + 1) % NUM_OF_SENSORS;
    dmaStep = 0;
    selectMuxPin(currentSensor);

    // Start next reading if not paused
    if(!nslp_rx_active  && !i2c_paused) {
        for (int attempt = 0; attempt < MAX_I2C_RETRIES; attempt++) {
            if (HAL_I2C_Master_Transmit_DMA(&hi2c3, 0x7F << 1, instructionArray, 2) == HAL_OK) {
                return;
            }
        }
        HAL_I2C_ErrorCallback(hi2c);
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c != &hi2c3) return;

    HAL_I2C_DeInit(hi2c);
    HAL_I2C_Init(hi2c);

    // Only continue if not paused
    if(!nslp_rx_active  && !i2c_paused) {
        currentSensor = (currentSensor + 1) % NUM_OF_SENSORS;
        dmaStep = 0;
        selectMuxPin(currentSensor);
        HAL_I2C_Master_Transmit_DMA(&hi2c3, 0x7F << 1, instructionArray, 2);
    }
}

void ProcessSensorData(uint8_t sensorIndex) {
    uint32_t rawPressure = rawPressureArray[sensorIndex];
    uint16_t rawTemperature = rawTemperatureArray[sensorIndex];

    float fpressureData = (float)rawPressure;
    float fpressureData2;

    if (fpressureData >= 8388608.0f) {
        fpressureData2 = (fpressureData - 16777216.0f) * Fullscale_P * calibration[sensorIndex] / 8388608.0f;
    } else {
        fpressureData2 = fpressureData * Fullscale_P * calibration[sensorIndex] / 8388608.0f;
    }

    // Ignore negative pressure
    if (fpressureData2 < 0.0f) {
        // Shift the array but keep previous value
        for (uint8_t j = 0; j < RUNAVGAM - 1; j++) {
            runningAveragePressure[sensorIndex][j] = runningAveragePressure[sensorIndex][j + 1];
        }
        runningAveragePressure[sensorIndex][RUNAVGAM - 1] = pressureArray[sensorIndex];
        return;
    }

    float truePressure = fpressureData2;
    float pressureSum = 0.0f;

    for (uint8_t j = 0; j < RUNAVGAM; j++) {
        if (j == RUNAVGAM - 1) {
            runningAveragePressure[sensorIndex][j] = truePressure;
        } else {
            runningAveragePressure[sensorIndex][j] = runningAveragePressure[sensorIndex][j + 1];
        }
        pressureSum += runningAveragePressure[sensorIndex][j];
    }

    float pressureAverage = pressureSum / RUNAVGAM;

    float ftemperature = (float)rawTemperature;
    float trueTemperature = ftemperature / 256.0f * tempcal[sensorIndex];
    float temperatureSum = 0.0f;

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

void CheckAndResumeI2C() {
    if(!nslp_rx_active  && i2c_paused) {
        resumeI2COperations();
    }
}
