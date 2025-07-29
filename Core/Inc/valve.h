/*
 * valve.h
 *
 *  Created on: May 22, 2025
 *      Author: Leon
 */

#ifndef INC_VALVE_H_
#define INC_VALVE_H_

#include "stm32g4xx_hal.h"

typedef enum {
    VALVE_IDLE,
    VALVE_OPENING,
    VALVE_CLOSING,
	VALVE_WAIT_DIRECTION_CHANGE,  // Waiting before reversing
	VALVE_COOLDOWN
} ValveState;

typedef struct {
	int pinO;
	int busO;
	int pinC;
	int busC;
	int funPin;
	int funBus;
	uint8_t isMax;
	uint16_t adcVal;
	uint32_t timeO;
	uint32_t timeC;
    uint8_t current_openness;      // 0–255
    uint8_t target_openness;       // 0–255
    ValveState state;
    uint32_t start_time;           // ms
    uint32_t move_duration;        // ms
    uint8_t is_opening;
    uint32_t wait_start_time;
    float valvecal;
    /*
    uint32_t endstop_check_time;
	uint32_t direction_change_time;
	uint32_t cooldown_end_time;    // New variable for cooldown period
	uint8_t last_direction;
	*/
    // Timing control
    // Timing control
    uint32_t direction_change_time;
    uint32_t cooldown_end_time;

    // Movement tracking
    uint8_t last_direction;          // 0=closing, 1=opening
    uint8_t movement_start_position; // Position when movement began
    uint32_t last_update_time;       // Last position update time
} ValveController;

//ValveController valve = {0, 0, VALVE_IDLE, 0, 0};


typedef struct {
	int onpin;
	int onbus;
	int conPin;
	int conBus;
	int funPin;
	int funBus;
	uint8_t isOn;
	uint8_t isCon;
	uint8_t isFun;
} Solenoid;

typedef struct {
	int onpin;
	int onbus;
	int conPin;
	int conBus;
	int funPin;
	int funBus;
	uint8_t isOn;
	uint8_t isCon;
	uint8_t isFun;
} Ignitor;

typedef struct {
	int pinO;
	int busO;
	int pinC;
	int busC;
	int funPin;
	int funBus;
	uint8_t isMax;
	uint16_t adcVal;
	uint32_t timeO;
	uint32_t timeC;
} Ball;

struct Openess{
	int openess1;
	int openess2;
};



void valve_set_openness(ValveController* valve, uint8_t openness);
void valve_update(ValveController* valve);
void valve_calibrate(ValveController* valve);

#endif /* INC_VALVE_H_ */
