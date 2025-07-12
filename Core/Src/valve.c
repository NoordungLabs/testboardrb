/*
 * valve.c
 *
 *  Created on: Jun 6, 2025
 *      Author: Leon
 */
#include "valve.h"


void valve_set_openness(ValveController* valve, uint8_t openness) {
    valve->target_openness = openness;

    if (valve->state == VALVE_OPENING && valve->target_openness < valve->current_openness) {
        HAL_GPIO_WritePin((GPIO_TypeDef*)valve->busO, valve->pinO, GPIO_PIN_RESET);
        valve->state = VALVE_IDLE;
    } else if (valve->state == VALVE_CLOSING && valve->target_openness > valve->current_openness) {
        HAL_GPIO_WritePin((GPIO_TypeDef*)valve->busC, valve->pinC, GPIO_PIN_RESET);
        valve->state = VALVE_IDLE;
    }
}

/*
void valve_update(ValveController* valve) {
    uint32_t now = HAL_GetTick();

    switch (valve->state) {
        case VALVE_IDLE:
            if (valve->target_openness != valve->current_openness) {
                int16_t delta = (int16_t)valve->target_openness - (int16_t)valve->current_openness;

                if (delta > 0) {
                    // Open
                    valve->move_duration = ((uint32_t)delta * valve->timeO) / 255;
                    valve->start_time = now;
                    HAL_GPIO_WritePin((GPIO_TypeDef*)valve->busO, valve->pinO, GPIO_PIN_SET);
                    valve->state = VALVE_OPENING;
                } else {
                    // Close
                    valve->move_duration = ((uint32_t)(-delta) * valve->timeC) / 255;
                    valve->start_time = now;
                    HAL_GPIO_WritePin((GPIO_TypeDef*)valve->busC, valve->pinC, GPIO_PIN_SET);
                    valve->state = VALVE_CLOSING;
                }
            }
            break;

        case VALVE_OPENING:
            if ((now - valve->start_time) >= valve->move_duration) {
                HAL_GPIO_WritePin((GPIO_TypeDef*)valve->busO, valve->pinO, GPIO_PIN_RESET);
                valve->current_openness = valve->target_openness;
                valve->state = VALVE_IDLE;
            }
            break;

        case VALVE_CLOSING:
            if ((now - valve->start_time) >= valve->move_duration) {
                HAL_GPIO_WritePin((GPIO_TypeDef*)valve->busC, valve->pinC, GPIO_PIN_RESET);
                valve->current_openness = valve->target_openness;
                valve->state = VALVE_IDLE;
            }
            break;
    }
}
*/

#define ADC_MOVING_THRESHOLD 100  // Adjust to match your signal levels

void valve_update(ValveController* valve) {
    uint32_t now = HAL_GetTick();

    switch (valve->state) {
        case VALVE_IDLE:
            if (valve->target_openness != valve->current_openness) {
                int16_t delta = (int16_t)valve->target_openness - (int16_t)valve->current_openness;

                if (delta > 0) {
                    // Start opening
                    valve->move_duration = ((uint32_t)delta * valve->timeO) / 255;
                    valve->start_time = now;
                    HAL_GPIO_WritePin((GPIO_TypeDef*)valve->busO, valve->pinO, GPIO_PIN_SET);
                    valve->state = VALVE_OPENING;
                } else {
                    // Start closing
                    valve->move_duration = ((uint32_t)(-delta) * valve->timeC) / 255;
                    valve->start_time = now;
                    HAL_GPIO_WritePin((GPIO_TypeDef*)valve->busC, valve->pinC, GPIO_PIN_SET);
                    valve->state = VALVE_CLOSING;
                }
            }
            break;

        case VALVE_OPENING:
            if (valve->adcVal <= ADC_MOVING_THRESHOLD) {
                // Not moving when it should be → possibly fully open
                HAL_GPIO_WritePin((GPIO_TypeDef*)valve->busO, valve->pinO, GPIO_PIN_RESET);

                // Assume fully open → recalibrate
                valve->current_openness = 255;

                // Retry to reach target if needed
                valve->state = VALVE_IDLE;
                break;
            }

            if ((now - valve->start_time) >= valve->move_duration) {
                HAL_GPIO_WritePin((GPIO_TypeDef*)valve->busO, valve->pinO, GPIO_PIN_RESET);
                valve->current_openness = valve->target_openness;
                valve->state = VALVE_IDLE;
            }
            break;

        case VALVE_CLOSING:
            if (valve->adcVal <= ADC_MOVING_THRESHOLD) {
                // Not moving when it should be → possibly fully closed
                HAL_GPIO_WritePin((GPIO_TypeDef*)valve->busC, valve->pinC, GPIO_PIN_RESET);

                // Assume fully closed → recalibrate
                valve->current_openness = 0;

                // Retry to reach target if needed
                valve->state = VALVE_IDLE;
                break;
            }

            if ((now - valve->start_time) >= valve->move_duration) {
                HAL_GPIO_WritePin((GPIO_TypeDef*)valve->busC, valve->pinC, GPIO_PIN_RESET);
                valve->current_openness = valve->target_openness;
                valve->state = VALVE_IDLE;
            }
            break;
    }
}





