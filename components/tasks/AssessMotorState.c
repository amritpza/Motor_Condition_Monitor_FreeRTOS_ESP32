#include "AssessMotorState.h"

#include "freertos/FreeRTOS.h"

#include "ReadMotor.h"

#include "lcd_i2c.h"

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

static bool warning_enter(const motor_metrics_t* const m){
	return ((m->accel_rms >= A_RMS_WARNING_ENTER) ||
			(m->crest_factor >= CF_WARNING_ENTER) ||
			(m->temperature_c >= TEMP_WARNING_ENTER));
}

static bool warning_exit(const motor_metrics_t* const m){
	return ((m->accel_rms < A_RMS_WARNING_EXIT) &&
			(m->crest_factor < CF_WARNING_EXIT) &&
			(m->temperature_c < TEMP_WARNING_EXIT));
}

static bool alarm_enter(const motor_metrics_t* const m){
	return ((m->accel_rms >= A_RMS_ALARM_ENTER) ||
			(m->crest_factor >= CF_ALARM_ENTER) ||
			(m->temperature_c >= TEMP_ALARM_ENTER));
}

static bool alarm_exit(const motor_metrics_t* const m){
	return ((m->accel_rms < A_RMS_ALARM_EXIT) &&
			(m->crest_factor < CF_ALARM_EXIT) &&
			(m->temperature_c < TEMP_ALARM_EXIT));
}

static bool fault_enter(const motor_metrics_t* const m){
	return ((m->accel_rms >= A_RMS_FAULT_ENTER) ||
			(m->crest_factor >= CF_FAULT_ENTER) ||
			(m->temperature_c >= TEMP_FAULT_ENTER));
}

static bool fault_exit(const motor_metrics_t* const m){
	return ((m->accel_rms < A_RMS_FAULT_EXIT) &&
			(m->crest_factor < CF_FAULT_EXIT) &&
			(m->temperature_c < TEMP_FAULT_EXIT));
}

static Motor_State get_motor_state(const motor_metrics_t* const m){
	if(motor_metrics_is_zero(m)) return STATE_UNKNOWN;

	if(alarm_enter(m)) return STATE_ALARM;

	if(warning_enter(m)) return STATE_WARNING;

	return STATE_NORMAL;
}

void vTask_AssessMotorState(void *pvParameters){
	motor_metrics_t curr_motor_stats = {0};
	uint8_t fault_counter = 0;

	motor_metrics_queue_receive(&curr_motor_stats);

	Motor_State curr_motor_state = STATE_NONE;
	Motor_State prev_motor_state = STATE_NONE;

	if(fault_enter(&curr_motor_stats)){
		fault_counter = 1;
		curr_motor_state = STATE_FAULT_PENDING;
	}

	else curr_motor_state = get_motor_state(&curr_motor_stats);

	while(1){
		if(motor_metrics_queue_receive(&curr_motor_stats) != pdPASS){
			curr_motor_state = STATE_UNKNOWN;
		}

		// State transition

		if(curr_motor_state == STATE_UNKNOWN){
			curr_motor_state = get_motor_state(&curr_motor_stats);
		}

		else if(curr_motor_state == STATE_NORMAL){
			if(fault_enter(&curr_motor_stats)){
				fault_counter = 1;
				curr_motor_state = STATE_FAULT_PENDING;
			}

			else if(alarm_enter(&curr_motor_stats)){
				curr_motor_state = STATE_ALARM;
			}

			else if(warning_enter(&curr_motor_stats)){
				curr_motor_state = STATE_WARNING;
			}
		}

		else if(curr_motor_state == STATE_WARNING){
			if(fault_enter(&curr_motor_stats)){
				fault_counter = 1;
				curr_motor_state = STATE_FAULT_PENDING;
			}

			else if(alarm_enter(&curr_motor_stats)){
				curr_motor_state = STATE_ALARM;
			}

			else if(warning_exit(&curr_motor_stats)){
				curr_motor_state = STATE_NORMAL;
			}
		}

		else if(curr_motor_state == STATE_ALARM){
			if(fault_enter(&curr_motor_stats)){
				fault_counter = 1;
				curr_motor_state = STATE_FAULT_PENDING;
			}

			else if(alarm_exit(&curr_motor_stats)){
				curr_motor_state = get_motor_state(&curr_motor_stats);
			}
		}

		else if(curr_motor_state == STATE_FAULT_PENDING){
			if(fault_enter(&curr_motor_stats)){
				fault_counter++;

				if(fault_counter >= FAULT_PERSISTENCE_CNT){
					fault_counter = 0;
					curr_motor_state = STATE_FAULT;
				}
			}

			else{
				fault_counter = 0;
				curr_motor_state = get_motor_state(&curr_motor_stats);
			}
		}

		else if(curr_motor_state == STATE_FAULT){
			if(fault_exit(&curr_motor_stats)){
				curr_motor_state = get_motor_state(&curr_motor_stats);
			}
		}

		// Update state

		if(curr_motor_state != prev_motor_state){
			const char *lcd_msg = "";

			if(curr_motor_state == STATE_UNKNOWN)            lcd_msg = "determine state!";
			else if(curr_motor_state == STATE_NORMAL)        lcd_msg = "NORMAL OPERATION";
			else if(curr_motor_state == STATE_WARNING)       lcd_msg = "MOTOR WARNING";
			else if(curr_motor_state == STATE_ALARM)         lcd_msg = "CRITICAL ALARM";
			else if(curr_motor_state == STATE_FAULT_PENDING) lcd_msg = "CHECKING FAULT..";
			else if(curr_motor_state == STATE_FAULT)         lcd_msg = "MOTOR FAULTED";

			if(take_i2c_mutex()){
				ESP_ERROR_CHECK(lcd_i2c_clear());

				ESP_ERROR_CHECK(lcd_i2c_set_cursor(0, 0));
				if(curr_motor_state != STATE_UNKNOWN) ESP_ERROR_CHECK(lcd_i2c_write("Motor Condition"));
				else ESP_ERROR_CHECK(lcd_i2c_write("ERROR: Unable to"));

				ESP_ERROR_CHECK(lcd_i2c_set_cursor(0, 1));
				ESP_ERROR_CHECK(lcd_i2c_write(lcd_msg));

				give_i2c_mutex();
			}

			prev_motor_state = curr_motor_state;
		}
	}
}
