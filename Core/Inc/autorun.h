/*
 * autorun.h
 *
 *  Created on: Aug 4, 2025
 *      Author: Leon
 */

#ifndef INC_AUTORUN_H_
#define INC_AUTORUN_H_
#include "stm32g4xx_hal.h"
#include "nslp_dma.h"
#include "i2c_dma_sens.h"

typedef enum {
	AUTO_IDLE,
	AUTO_FILL,
	AUTO_WAIT,
	AUTO_IGNT,
	AUTO_FLAM,
	AUTO_DGAS,
	AUTO_VENT,
	AUTO_STOP
} AutoRunState;

void fautoRun();


#endif /* INC_AUTORUN_H_ */
