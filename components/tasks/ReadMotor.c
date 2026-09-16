#include "ReadMotor.h"

#include <stdio.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_dsp.h"

#include "mpu6050.h"

#define FFT_SIZE  256
static float fft_buffer[FFT_SIZE * 2];
static float hann_window[FFT_SIZE];

static float calculate_dominant_frequency(const float* const samples, const uint16_t sample_count, const float sample_rate_hz){
	float mean = 0.0f;

	for(uint16_t i = 0; i < sample_count; i++) mean += samples[i];
	mean /= sample_count;

	for(uint16_t i = 0; i < sample_count; i++){
	    fft_buffer[2 * i] = (samples[i] - mean) * hann_window[i]; // re
	    fft_buffer[2 * i + 1] = 0.0f; // im
	}

	dsps_fft2r_fc32(fft_buffer, sample_count);
	dsps_bit_rev_fc32(fft_buffer, sample_count);
	dsps_cplx2reC_fc32(fft_buffer, sample_count);

	uint16_t peak_bin = 1;
	float peak_mag = 0.0f;

	for(uint16_t bin = 1; bin < sample_count / 2; bin++){
	    float re = fft_buffer[2 * bin];
	    float im = fft_buffer[2 * bin + 1];

	    float mag = re * re + im * im;

	    if(mag > peak_mag){
	        peak_mag = mag;
	        peak_bin = bin;
	    }
	}

	return ((float)peak_bin * sample_rate_hz) / sample_count;
}

static float accel_mag_samples[FFT_SIZE];
static uint16_t sample_index = 0;

static QueueHandle_t motor_metrics_queue;

void vTask_ReadMotor(void *pvParameters){
	float accel_sum_sqr = 0.0f;
	float gyro_sum_sqr  = 0.0f;

	float accel_peak = 0.0f;

	dsps_wind_hann_f32(hann_window, FFT_SIZE);

	while(1){
		uint32_t mpu6050_sample_ready = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		if(mpu6050_sample_ready){ // RX MPU6050 sample when data ready via ISR 
			mpu6050_data_t motor_data = {0};

			if(mpu6050_read(&motor_data) != ESP_OK){
				motor_metrics_t motor_stats = {0};
				xQueueOverwrite(motor_metrics_queue, &motor_stats);
				continue;
			}

			// Remove Earth's gravity
			float accel_mag = sqrtf((motor_data.accel_x_g * motor_data.accel_x_g) +
									(motor_data.accel_y_g * motor_data.accel_y_g) +
									(motor_data.accel_z_g * motor_data.accel_z_g)) - 1.0f;

			float gyro_mag = sqrtf((motor_data.gyro_x_dps * motor_data.gyro_x_dps) +
								   (motor_data.gyro_y_dps * motor_data.gyro_y_dps) +
								   (motor_data.gyro_z_dps * motor_data.gyro_z_dps));

			accel_mag_samples[sample_index] = accel_mag;
			sample_index++;

			accel_sum_sqr += accel_mag * accel_mag;
			gyro_sum_sqr  += gyro_mag  * gyro_mag;

			accel_peak = fmaxf(accel_peak, fabsf(accel_mag));

			if(sample_index >= FFT_SIZE){
				motor_metrics_t motor_stats = {0};

				motor_stats.accel_rms = sqrtf(accel_sum_sqr / FFT_SIZE);

				motor_stats.gyro_rms = sqrtf(gyro_sum_sqr / FFT_SIZE);

				motor_stats.accel_peak = accel_peak;

				if(motor_stats.accel_rms > 0.0001f)
					motor_stats.crest_factor = accel_peak / motor_stats.accel_rms;

				motor_stats.dominant_freq_hz = calculate_dominant_frequency(accel_mag_samples, FFT_SIZE, MPU6050_SAMPLE_RATE_HZ);

				motor_stats.temperature_c = motor_data.temperature_c;

				xQueueOverwrite(motor_metrics_queue, &motor_stats);

				sample_index = 0;

				accel_sum_sqr = 0.0f;
				gyro_sum_sqr  = 0.0f;

				accel_peak = 0.0f;
			}
		}

		else{
			motor_metrics_t motor_stats = {0};
			xQueueOverwrite(motor_metrics_queue, &motor_stats);
		}
	}
}

esp_err_t motor_metrics_queue_init(){
	motor_metrics_queue = xQueueCreate(1, sizeof(motor_metrics_t));
	if(motor_metrics_queue == NULL) return ESP_ERR_NOT_FOUND;
	return ESP_OK;
}

BaseType_t motor_metrics_queue_receive(motor_metrics_t* const data_rx){
	return xQueueReceive(motor_metrics_queue, data_rx, portMAX_DELAY);
}

bool motor_metrics_is_zero(const motor_metrics_t* const m){
	return m->accel_rms        == 0.0f &&
		   m->gyro_rms         == 0.0f &&
		   m->accel_peak       == 0.0f &&
           m->crest_factor     == 0.0f &&
		   m->dominant_freq_hz == 0.0f &&
		   m->temperature_c    == 0.0f;
}
