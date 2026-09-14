#ifndef ASSESSMOTORSTATE_H
#define ASSESSMOTORSTATE_H

#include "freertos/FreeRTOS.h"

typedef enum{
	STATE_NONE = -1,
	STATE_UNKNOWN,
	STATE_NORMAL,
	STATE_WARNING,
	STATE_ALARM,
	STATE_FAULT_PENDING,
	STATE_FAULT
} Motor_State;

void vTask_AssessMotorState(void *pvParameters);

#endif
