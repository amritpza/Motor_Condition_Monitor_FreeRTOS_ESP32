#include "AssessMotorState.h"

#include <float.h>
#include <math.h>

#include "freertos/FreeRTOS.h"

#include "ReadMotor.h"
#include "UIManager.h"

#include "lcd_i2c.h"

#define MEASUREMENTS_SIZE 10

static bool warning_enter(const motor_metrics_t* const m){
	return ((m->accel_rms >= thresholds_get_value(METRIC_ACCEL_RMS, STATE_WARNING, true)) ||
			(m->crest_factor >= thresholds_get_value(METRIC_CREST_FACTOR, STATE_WARNING, true)) ||
			(m->temperature_c >= thresholds_get_value(METRIC_TEMPERATURE, STATE_WARNING, true)));
}

static bool warning_exit(const motor_metrics_t* const m){
	return ((m->accel_rms < thresholds_get_value(METRIC_ACCEL_RMS, STATE_WARNING, false)) &&
			(m->crest_factor < thresholds_get_value(METRIC_CREST_FACTOR, STATE_WARNING, false)) &&
			(m->temperature_c < thresholds_get_value(METRIC_TEMPERATURE, STATE_WARNING, false)));
}

static bool alarm_enter(const motor_metrics_t* const m){
	return ((m->accel_rms >= thresholds_get_value(METRIC_ACCEL_RMS, STATE_ALARM, true)) ||
			(m->crest_factor >= thresholds_get_value(METRIC_CREST_FACTOR, STATE_ALARM, true)) ||
			(m->temperature_c >= thresholds_get_value(METRIC_TEMPERATURE, STATE_ALARM, true)));
}

static bool alarm_exit(const motor_metrics_t* const m){
	return ((m->accel_rms < thresholds_get_value(METRIC_ACCEL_RMS, STATE_ALARM, false)) &&
			(m->crest_factor < thresholds_get_value(METRIC_CREST_FACTOR, STATE_ALARM, false)) &&
			(m->temperature_c < thresholds_get_value(METRIC_TEMPERATURE, STATE_ALARM, false)));
}

static bool fault_enter(const motor_metrics_t* const m){
	return ((m->accel_rms >= thresholds_get_value(METRIC_ACCEL_RMS, STATE_FAULT, true)) ||
			(m->crest_factor >= thresholds_get_value(METRIC_CREST_FACTOR, STATE_FAULT, true)) ||
			(m->temperature_c >= thresholds_get_value(METRIC_TEMPERATURE, STATE_FAULT, true)));
}

static bool fault_exit(const motor_metrics_t* const m){
	return ((m->accel_rms < thresholds_get_value(METRIC_ACCEL_RMS, STATE_FAULT, false)) &&
			(m->crest_factor < thresholds_get_value(METRIC_CREST_FACTOR, STATE_FAULT, false)) &&
			(m->temperature_c < thresholds_get_value(METRIC_TEMPERATURE, STATE_FAULT, false)));
}

static Motor_State get_motor_state(const motor_metrics_t* const m){
	if(motor_metrics_is_zero(m)) return STATE_UNKNOWN;

	if(alarm_enter(m)) return STATE_ALARM;

	if(warning_enter(m)) return STATE_WARNING;

	return STATE_NORMAL;
}

static SemaphoreHandle_t motor_thresholds_mutex;

static metric_thresholds_t motor_thresholds = {
	.accel_rms = {
		.warning = {.enter = A_RMS_WARNING_ENTER, .exit = A_RMS_WARNING_EXIT},
		.alarm   = {.enter = A_RMS_ALARM_ENTER, .exit = A_RMS_ALARM_EXIT},
		.fault   = {.enter = A_RMS_FAULT_ENTER, .exit = A_RMS_FAULT_EXIT}
	},

	.crest_factor = {
		.warning = {.enter = CF_WARNING_ENTER, .exit = CF_WARNING_EXIT},
		.alarm   = {.enter = CF_ALARM_ENTER, .exit = CF_ALARM_EXIT},
		.fault   = {.enter = CF_FAULT_ENTER, .exit = CF_FAULT_EXIT}
	},

	.temperature = {
		.warning = { .enter = TEMP_WARNING_ENTER, .exit = TEMP_WARNING_EXIT},
		.alarm   = { .enter = TEMP_ALARM_ENTER, .exit = TEMP_ALARM_EXIT},
		.fault   = { .enter = TEMP_FAULT_ENTER, .exit = TEMP_FAULT_EXIT}
	}
};

static motor_data_set_t measurements[MEASUREMENTS_SIZE];
static float sum[3] = {0}; // accel_rms, crest_factor, temperature_c;
static float accel_peak_min_max[2] = {FLT_MAX, -FLT_MAX}; // min, max
static size_t sample_cnt = 0;

void vTask_AssessMotorState(void *pvParameters){
	motor_metrics_t curr_motor_info = {0};
	uint8_t fault_counter = 0;

	motor_info_queue_receive(&curr_motor_info);

	Motor_State curr_motor_state = STATE_NONE;
	Motor_State prev_motor_state = STATE_NONE;

	if(fault_enter(&curr_motor_info)){
		fault_counter = 1;
		curr_motor_state = STATE_FAULT_PENDING;
	}

	else curr_motor_state = get_motor_state(&curr_motor_info);

	while(1){
		if(motor_info_queue_receive(&curr_motor_info) != pdPASS){
			curr_motor_state = STATE_UNKNOWN;
		}

		// State transition

		if(curr_motor_state == STATE_UNKNOWN){
			curr_motor_state = get_motor_state(&curr_motor_info);
		}

		else if(curr_motor_state == STATE_NORMAL){
			if(fault_enter(&curr_motor_info)){
				fault_counter = 1;
				curr_motor_state = STATE_FAULT_PENDING;
			}

			else if(alarm_enter(&curr_motor_info)){
				curr_motor_state = STATE_ALARM;
			}

			else if(warning_enter(&curr_motor_info)){
				curr_motor_state = STATE_WARNING;
			}
		}

		else if(curr_motor_state == STATE_WARNING){
			if(fault_enter(&curr_motor_info)){
				fault_counter = 1;
				curr_motor_state = STATE_FAULT_PENDING;
			}

			else if(alarm_enter(&curr_motor_info)){
				curr_motor_state = STATE_ALARM;
			}

			else if(warning_exit(&curr_motor_info)){
				curr_motor_state = STATE_NORMAL;
			}
		}

		else if(curr_motor_state == STATE_ALARM){
			if(fault_enter(&curr_motor_info)){
				fault_counter = 1;
				curr_motor_state = STATE_FAULT_PENDING;
			}

			else if(alarm_exit(&curr_motor_info)){
				curr_motor_state = get_motor_state(&curr_motor_info);
			}
		}

		else if(curr_motor_state == STATE_FAULT_PENDING){
			if(fault_enter(&curr_motor_info)){
				fault_counter++;

				if(fault_counter >= FAULT_PERSISTENCE_CNT){
					fault_counter = 0;
					curr_motor_state = STATE_FAULT;
				}
			}

			else{
				fault_counter = 0;
				curr_motor_state = get_motor_state(&curr_motor_info);
			}
		}

		else if(curr_motor_state == STATE_FAULT){
			if(fault_exit(&curr_motor_info)){
				curr_motor_state = get_motor_state(&curr_motor_info);
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

		// Log data

    motor_data_set_t motor_log_entry = {curr_motor_info, curr_motor_state};

    measurements[sample_cnt % MEASUREMENTS_SIZE] = motor_log_entry;
    sum[0] += motor_log_entry.metrics.accel_rms;
    sum[1] += motor_log_entry.metrics.crest_factor;
    sum[2] += motor_log_entry.metrics.temperature_c;
    accel_peak_min_max[0] = fminf(motor_log_entry.metrics.accel_peak, accel_peak_min_max[0]);
    accel_peak_min_max[1] = fmaxf(motor_log_entry.metrics.accel_peak, accel_peak_min_max[1]);
    sample_cnt++;

		EventBits_t menu_option_bits = rx_user_menu_option();

		if((menu_option_bits & BIT_MENU_SHOW_MEASUREMENTS) == BIT_MENU_SHOW_MEASUREMENTS){
			char log_str[10];

			serial_print_from_task("Accel. RMS, Gyro. RMS, Accel. Peak, Crest Factor, Dominant_Freq., Temperature, State\r\n");

			for(int i = MEASUREMENTS_SIZE - 1; i >= 0; i--){
				snprintf(log_str, sizeof(log_str), "%f", measurements[i].metrics.accel_rms);
				serial_print_from_task(log_str);
				serial_print_from_task(", ");

				snprintf(log_str, sizeof(log_str), "%f", measurements[i].metrics.gyro_rms);
				serial_print_from_task(log_str);
				serial_print_from_task(", ");

				snprintf(log_str, sizeof(log_str), "%f", measurements[i].metrics.accel_peak);
				serial_print_from_task(log_str);
				serial_print_from_task(", ");

				snprintf(log_str, sizeof(log_str), "%f", measurements[i].metrics.crest_factor);
				serial_print_from_task(log_str);
				serial_print_from_task(", ");

				snprintf(log_str, sizeof(log_str), "%f", measurements[i].metrics.dominant_freq_hz);
				serial_print_from_task(log_str);
				serial_print_from_task(", ");

				snprintf(log_str, sizeof(log_str), "%f", measurements[i].metrics.temperature_c);
				serial_print_from_task(log_str);
				serial_print_from_task(", ");

				switch(measurements[i].state){
					case STATE_NONE:
							serial_print_from_task("None");
							break;

					case STATE_UNKNOWN:
							serial_print_from_task("Unknown");
							break;

					case STATE_NORMAL:
							serial_print_from_task("Normal");
							break;

					case STATE_WARNING:
							serial_print_from_task("Warning");
							break;

					case STATE_ALARM:
							serial_print_from_task("Alarm");
							break;

					case STATE_FAULT:
							serial_print_from_task("Fault");
							break;

					case STATE_FAULT_PENDING:
							serial_print_from_task("Fault Pending");
							break;
				}

				serial_print_from_task("\r\n");
			}

			set_bit_finished_menu_rq_option();
		}

		else if((menu_option_bits & BIT_MENU_VIEW_STATS) == BIT_MENU_VIEW_STATS){
			char avg_str[10];
			float avg[3] = {sum[0]/sample_cnt, sum[1]/sample_cnt, sum[2]/sample_cnt}; // accel_rms, crest_factor, temperature_c;

			serial_print_from_task("Average Accel. RMS (g): ");
			snprintf(avg_str, sizeof(avg_str), "%f", avg[0]);
			serial_print_from_task(avg_str);
			serial_print_from_task("\r\n");

			serial_print_from_task("Average Crest Factor: ");
			snprintf(avg_str, sizeof(avg_str), "%f", avg[1]);
			serial_print_from_task(avg_str);
			serial_print_from_task("\r\n");

			serial_print_from_task("Average Temperature (°C): ");
			snprintf(avg_str, sizeof(avg_str), "%f", avg[2]);
			serial_print_from_task(avg_str);
			serial_print_from_task("\r\n");

			serial_print_from_task("Smallest Accel. peak (g): ");
			snprintf(avg_str, sizeof(avg_str), "%f", accel_peak_min_max[0]);
			serial_print_from_task(avg_str);
			serial_print_from_task("\r\n");

			serial_print_from_task("Largest Accel. peak (g): ");
			snprintf(avg_str, sizeof(avg_str), "%f", accel_peak_min_max[1]);
			serial_print_from_task(avg_str);
			serial_print_from_task("\r\n");

			serial_print_from_task("Samples Processed: ");
			snprintf(avg_str, sizeof(avg_str), "%zu", sample_cnt);
			serial_print_from_task(avg_str);

			set_bit_finished_menu_rq_option();
		}

		else if((menu_option_bits & BIT_MENU_RESET_STATS) == BIT_MENU_RESET_STATS){
			sum[0] = 0;
			sum[1] = 0;
			sum[2] = 0;
			accel_peak_min_max[0] = FLT_MAX;
			accel_peak_min_max[1] = -FLT_MAX;
			sample_cnt = 0;

			serial_print_from_task("Statistics data reset successfully.");
			set_bit_finished_menu_rq_option();
		}
	}
}

void thresholds_set_value(const Metric_Type metric, const Motor_State state, const bool enter, const float new_value){
	if(xSemaphoreTake(motor_thresholds_mutex, portMAX_DELAY)){
		metric_thresholds_t *ptr = &motor_thresholds;

		state_profile_t *chosen_metric = NULL;
		switch(metric){
			case METRIC_ACCEL_RMS:    chosen_metric = &ptr->accel_rms; break;
			case METRIC_CREST_FACTOR: chosen_metric = &ptr->crest_factor; break;
			case METRIC_TEMPERATURE:  chosen_metric = &ptr->temperature; break;
		}

		threshold_value_pair_t *value_pair = NULL;
		switch(state){
			case STATE_WARNING: value_pair = &chosen_metric->warning; break;
			case STATE_ALARM:   value_pair = &chosen_metric->alarm; break;
			case STATE_FAULT:   value_pair = &chosen_metric->fault; break;
			default:            value_pair = NULL; break;
		}

		if(value_pair != NULL){
			if(enter) value_pair->enter = new_value;
			else value_pair->exit = new_value;
		}

		xSemaphoreGive(motor_thresholds_mutex);
	}
}

float thresholds_get_value(const Metric_Type metric, const Motor_State state, const bool enter){
	float result = -FLT_MAX;

	if(xSemaphoreTake(motor_thresholds_mutex, portMAX_DELAY)){
		metric_thresholds_t *ptr = &motor_thresholds;

		const state_profile_t *chosen_metric = NULL;
		switch(metric){
			case METRIC_ACCEL_RMS:    chosen_metric = &ptr->accel_rms; break;
			case METRIC_CREST_FACTOR: chosen_metric = &ptr->crest_factor; break;
			case METRIC_TEMPERATURE:  chosen_metric = &ptr->temperature; break;
		}

		const threshold_value_pair_t *value_pair = NULL;
		switch(state){
			case STATE_WARNING: value_pair = &chosen_metric->warning; break;
			case STATE_ALARM:   value_pair = &chosen_metric->alarm; break;
			case STATE_FAULT:   value_pair = &chosen_metric->fault; break;
			default:            value_pair = NULL; break;
		}

		if(value_pair != NULL){
			if(enter) result = value_pair->enter;
			else result = value_pair->exit;
		}

		xSemaphoreGive(motor_thresholds_mutex);
	}

	return result;
}

esp_err_t motor_thresholds_mutex_init(){
	motor_thresholds_mutex = xSemaphoreCreateMutex();
	if(motor_thresholds_mutex == NULL) return ESP_ERR_NOT_FOUND;
	return ESP_OK;
}
