#ifndef ASSESSMOTORSTATE_H
#define ASSESSMOTORSTATE_H

#include <stdbool.h>

#include "freertos/FreeRTOS.h"

#include "ReadMotor.h"

// Default threshold values

// Accel RMS (g)
#define A_RMS_WARNING_ENTER  0.50f
#define A_RMS_WARNING_EXIT   0.40f

#define A_RMS_ALARM_ENTER    1.20f
#define A_RMS_ALARM_EXIT     1.00f

#define A_RMS_FAULT_ENTER    1.80f
#define A_RMS_FAULT_EXIT     1.60f

// Crest Factor
#define CF_WARNING_ENTER     3.5f
#define CF_WARNING_EXIT      3.0f

#define CF_ALARM_ENTER       5.0f
#define CF_ALARM_EXIT        4.5f

#define CF_FAULT_ENTER       7.0f
#define CF_FAULT_EXIT        6.0f

// Temperature (°C)
#define TEMP_WARNING_ENTER   55.0f
#define TEMP_WARNING_EXIT    50.0f

#define TEMP_ALARM_ENTER     70.0f
#define TEMP_ALARM_EXIT      65.0f

#define TEMP_FAULT_ENTER     82.0f
#define TEMP_FAULT_EXIT      77.0f

// Consecutive count required before entering FAULT state
#define FAULT_PERSISTENCE_CNT 5

typedef struct{
	float enter;
  float exit;
} threshold_value_pair_t;

typedef struct{
	threshold_value_pair_t warning;
	threshold_value_pair_t alarm;
	threshold_value_pair_t fault;
} state_profile_t;

typedef struct{
	state_profile_t accel_rms;
	state_profile_t crest_factor;
	state_profile_t temperature;
} metric_thresholds_t;

typedef enum{
	METRIC_ACCEL_RMS = 0,
	METRIC_CREST_FACTOR,
	METRIC_TEMPERATURE
} Metric_Type;

typedef enum{
	STATE_NONE = -1,
	STATE_UNKNOWN,
	STATE_NORMAL,
	STATE_WARNING,
	STATE_ALARM,
	STATE_FAULT,
	STATE_FAULT_PENDING
} Motor_State;

typedef struct{
	motor_metrics_t metrics;
	Motor_State state;
} motor_data_set_t;

void vTask_AssessMotorState(void *pvParameters);

void thresholds_set_value(const Metric_Type metric, const Motor_State state, const bool enter, const float new_value);
float thresholds_get_value(const Metric_Type metric, const Motor_State state, const bool enter);

esp_err_t motor_thresholds_mutex_init();

#endif
