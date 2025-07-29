/*
 * valve.c
 *
 *  Created on: Jun 6, 2025
 *      Author: Leon
 */
#include "valve.h"

void valve_set_openness(ValveController* valve, uint8_t openness) {
    if (openness > 255) openness = 255;
    valve->target_openness = openness;
    // New logic will handle real-time change in valve_update
}



#define DIRECTION_CHANGE_DELAY    1000    // Delay before changing direction
#define MOVEMENT_COOLDOWN_DELAY   1000    // Delay after movement completes
#define POSITION_TOLERANCE         0     // Allowable position difference

void valve_update(ValveController* valve) {
    uint32_t now = HAL_GetTick();
    int16_t delta;
    uint32_t elapsed;

    switch (valve->state) {
        case VALVE_IDLE:
            // Check if we need to start new movement after cooldown
            if (now >= valve->cooldown_end_time) {
                delta = valve->target_openness - valve->current_openness;

                // Only move if beyond tolerance threshold
                if (delta > POSITION_TOLERANCE || -delta > POSITION_TOLERANCE) {
                    if (delta > 0) {
                        valve->move_duration = ((uint32_t)delta * valve->timeO) / 255;
                        valve->start_time = now;
                        HAL_GPIO_WritePin(valve->busO, valve->pinO, GPIO_PIN_SET);
                        valve->state = VALVE_OPENING;
                    }
                    else {
                        valve->move_duration = ((uint32_t)(-delta) * valve->timeC) / 255;
                        valve->start_time = now;
                        HAL_GPIO_WritePin(valve->busC, valve->pinC, GPIO_PIN_SET);
                        valve->state = VALVE_CLOSING;
                    }
                }
            }
            break;

        case VALVE_OPENING:
        case VALVE_CLOSING:
            // Immediately stop movement if target changes
            delta = valve->target_openness - valve->current_openness;
            if ((valve->state == VALVE_OPENING && delta <= POSITION_TOLERANCE) ||
                (valve->state == VALVE_CLOSING && -delta <= POSITION_TOLERANCE)) {
                // Stop movement and enter cooldown
                HAL_GPIO_WritePin(valve->state == VALVE_OPENING ? valve->busO : valve->busC,
                                 valve->state == VALVE_OPENING ? valve->pinO : valve->pinC,
                                 GPIO_PIN_RESET);
                valve->cooldown_end_time = now + MOVEMENT_COOLDOWN_DELAY;
                valve->state = VALVE_COOLDOWN;
                break;
            }

            // Continue normal movement
            elapsed = now - valve->start_time;
            if (elapsed >= valve->move_duration) {
                // Movement complete
                HAL_GPIO_WritePin(valve->state == VALVE_OPENING ? valve->busO : valve->busC,
                                 valve->state == VALVE_OPENING ? valve->pinO : valve->pinC,
                                 GPIO_PIN_RESET);
                valve->current_openness = valve->target_openness;
                valve->cooldown_end_time = now + MOVEMENT_COOLDOWN_DELAY;
                valve->state = VALVE_COOLDOWN;
            } else {
                // Update position based on elapsed time
                if (valve->state == VALVE_OPENING) {
                    valve->current_openness = ((elapsed * 255) / valve->timeO) +
                                            (valve->target_openness - ((valve->move_duration * 255) / valve->timeO));
                } else {
                    valve->current_openness = valve->target_openness +
                                            ((valve->move_duration * 255) / valve->timeC) -
                                            ((elapsed * 255) / valve->timeC);
                }
            }
            break;

        case VALVE_COOLDOWN:
            if (now >= valve->cooldown_end_time) {
                valve->state = VALVE_IDLE;
            }
            break;

        default:
            valve->state = VALVE_IDLE;
            break;
    }
}


/*
 * OG VERSION DONT DELETE
void valve_update(ValveController* valve) {
    uint32_t now = HAL_GetTick();
    int16_t delta;
    uint32_t elapsed;
    uint8_t new_position;

    switch (valve->state) {
        case VALVE_IDLE:
            if (valve->target_openness != valve->current_openness) {
                delta = (int16_t)valve->target_openness - (int16_t)valve->current_openness;

                if (delta > 0) {
                    valve->move_duration = ((uint32_t)delta * valve->timeO) / 255;
                    valve->start_time = now;
                    HAL_GPIO_WritePin((GPIO_TypeDef*)valve->busO, valve->pinO, GPIO_PIN_SET);
                    valve->state = VALVE_OPENING;
                } else if (delta < 0) {
                    valve->move_duration = ((uint32_t)(-delta) * valve->timeC) / 255;
                    valve->start_time = now;
                    HAL_GPIO_WritePin((GPIO_TypeDef*)valve->busC, valve->pinC, GPIO_PIN_SET);
                    valve->state = VALVE_CLOSING;
                }
                else {
                	HAL_GPIO_WritePin((GPIO_TypeDef*)valve->busC, valve->pinC, GPIO_PIN_RESET);
                	HAL_GPIO_WritePin((GPIO_TypeDef*)valve->busO, valve->pinO, GPIO_PIN_RESET);
                }
            }
            break;

        case VALVE_OPENING:
            // Check for new target during movement
            elapsed = now - valve->start_time;
            if (elapsed >= valve->move_duration) {
                HAL_GPIO_WritePin((GPIO_TypeDef*)valve->busO, valve->pinO, GPIO_PIN_RESET);
                valve->current_openness = valve->target_openness;
                valve->state = VALVE_IDLE;
                break;
            }

            // Calculate new openness based on time
            new_position = valve->current_openness + ((uint32_t)elapsed * 255 / valve->timeO);
            if (new_position > 255) new_position = 255;

            // If target changed mid-movement, recalculate
            if (valve->target_openness < new_position) {
                HAL_GPIO_WritePin((GPIO_TypeDef*)valve->busO, valve->pinO, GPIO_PIN_RESET);
                valve->current_openness = new_position;
                valve->state = VALVE_IDLE;
                break;
            }

            // Safety fallback: end movement if valve stopped moving
            if (!HAL_GPIO_ReadPin((GPIO_TypeDef*)valve->funBus, valve->funPin)) {
                HAL_GPIO_WritePin((GPIO_TypeDef*)valve->busO, valve->pinO, GPIO_PIN_RESET);
                valve->current_openness = 255;
                valve->state = VALVE_IDLE;
            }
            break;

        case VALVE_CLOSING:
            elapsed = now - valve->start_time;
            if (elapsed >= valve->move_duration) {
                HAL_GPIO_WritePin((GPIO_TypeDef*)valve->busC, valve->pinC, GPIO_PIN_RESET);
                valve->current_openness = valve->target_openness;
                valve->state = VALVE_IDLE;
                break;
            }

            new_position = valve->current_openness - ((uint32_t)elapsed * 255 / valve->timeC);
            if (new_position > valve->current_openness) new_position = 0; // wrap protection

            if (valve->target_openness > new_position) {
                HAL_GPIO_WritePin((GPIO_TypeDef*)valve->busC, valve->pinC, GPIO_PIN_RESET);
                valve->current_openness = new_position;
                valve->state = VALVE_IDLE;
                break;
            }

            if (!HAL_GPIO_ReadPin((GPIO_TypeDef*)valve->funBus, valve->funPin)) {
                HAL_GPIO_WritePin((GPIO_TypeDef*)valve->busC, valve->pinC, GPIO_PIN_RESET);
                valve->current_openness = 0;
                valve->state = VALVE_IDLE;
            }
            break;

        default:
            valve->state = VALVE_IDLE;
            break;
    }
}
*/




void valve_calibrate(ValveController* valve){
  	uint32_t timeRef1 = 0;
  	uint32_t timeRef2 = 0;
  	HAL_GPIO_WritePin((GPIO_TypeDef*)valve->busO, valve->pinO, 0);
	HAL_GPIO_WritePin(valve->busC, valve->pinC, 0);
	HAL_GPIO_WritePin((GPIO_TypeDef*)valve->busO, valve->pinO, 1);
	HAL_Delay(4000);
	HAL_GPIO_WritePin((GPIO_TypeDef*)valve->busO, valve->pinO, 0);
	HAL_GPIO_WritePin(valve->busC, valve->pinC, 0);
	HAL_Delay(1000);
	while(1){
		HAL_GPIO_WritePin((GPIO_TypeDef*)valve->busO, valve->pinO, 0);
		HAL_GPIO_WritePin(valve->busC, valve->pinC, 1);
		valve->isMax = 0;
		HAL_Delay(1000);

		while (!valve->isMax){
			if (!HAL_GPIO_ReadPin((GPIO_TypeDef*)valve->funBus, valve->funPin)){
				HAL_GPIO_WritePin(valve->busC, valve->pinC, 0);
				valve->isMax = 1;
			}
		}
		HAL_Delay(1000);
		timeRef1 = HAL_GetTick();
		HAL_GPIO_WritePin(valve->busC, valve->pinC, 0);
		HAL_GPIO_WritePin(valve->busO, valve->pinO, 1);
		valve->isMax = 0;
		HAL_Delay(6000);
		while (!valve->isMax){
			if (!HAL_GPIO_ReadPin((GPIO_TypeDef*)valve->funBus, valve->funPin)){
				valve->timeO = (HAL_GetTick() - timeRef1);///valve->valvecal;
				HAL_GPIO_WritePin(valve->busO, valve->pinO, 0);
				valve->isMax = 1;
			}
		}

		HAL_Delay(1000);
		timeRef2 = HAL_GetTick();
		HAL_GPIO_WritePin(valve->busO, valve->pinO, 0);
		HAL_GPIO_WritePin(valve->busC, valve->pinC, 1);
		valve->isMax = 0;
		HAL_Delay(6000);
		while (!valve->isMax){
			if (!HAL_GPIO_ReadPin((GPIO_TypeDef*)valve->funBus, valve->funPin)){
				valve->timeC = (HAL_GetTick() - timeRef2);
				HAL_GPIO_WritePin(valve->busC, valve->pinC, 0);
				valve->isMax = 1;
			}
		}
		break;
	}
}

void valve_close(ValveController* valve){
	HAL_Delay(1000);
	HAL_GPIO_WritePin(valve->busO, valve->pinO, 0);
	HAL_GPIO_WritePin(valve->busC, valve->pinC, 1);
	valve->isMax = 0;
	HAL_Delay(4000);
	while (!valve->isMax){
		if (!HAL_GPIO_ReadPin((GPIO_TypeDef*)valve->funBus, valve->funPin)){
			HAL_GPIO_WritePin(valve->busC, valve->pinC, 0);
			valve->isMax = 1;
		}
	}
}


#define ADC_MOVING_THRESHOLD 100  // Adjust to match your signal levels
/*
//ANALOG
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
*/

//DISCRETE







