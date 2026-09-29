#include "ReadMotor.h"

#include <math.h>

#include "freertos/FreeRTOS.h"

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

static QueueHandle_t motor_info_queue;

static SemaphoreHandle_t i2c_mutex;

void vTask_ReadMotor(void *pvParameters){
	float accel_sum_sqr = 0.0f;
	float gyro_sum_sqr  = 0.0f;

	float accel_peak = 0.0f;

	dsps_wind_hann_f32(hann_window, FFT_SIZE);

	motor_info_queue = xQueueCreate(1, sizeof(motor_metrics_t));
	if(motor_info_queue == NULL) ESP_ERROR_CHECK(ESP_ERR_NOT_FOUND);

	while(1){
		uint32_t mpu6050_sample_ready = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		if(mpu6050_sample_ready){ // RX MPU6050 sample when data ready via ISR 
			mpu6050_data_t motor_data = {0};

			esp_err_t err = ESP_FAIL;
			if(take_i2c_mutex()){
				err = mpu6050_read(&motor_data);
				give_i2c_mutex();
			}

			if(err != ESP_OK){
				motor_metrics_t motor_info = {0};
				xQueueOverwrite(motor_info_queue, &motor_info);
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
				motor_metrics_t motor_info = {0};

				motor_info.accel_rms = sqrtf(accel_sum_sqr / FFT_SIZE);

				motor_info.gyro_rms = sqrtf(gyro_sum_sqr / FFT_SIZE);

				motor_info.accel_peak = accel_peak;

				if(motor_info.accel_rms > 0.0001f)
					motor_info.crest_factor = accel_peak / motor_info.accel_rms;

				motor_info.dominant_freq_hz = calculate_dominant_frequency(accel_mag_samples, FFT_SIZE, MPU6050_SAMPLE_RATE_HZ);

				motor_info.temperature_c = motor_data.temperature_c;

				xQueueOverwrite(motor_info_queue, &motor_info);

				sample_index = 0;

				accel_sum_sqr = 0.0f;
				gyro_sum_sqr  = 0.0f;

				accel_peak = 0.0f;
			}
		}

		else{
			motor_metrics_t motor_info = {0};
			xQueueOverwrite(motor_info_queue, &motor_info);
		}
	}
}

BaseType_t motor_info_queue_receive(motor_metrics_t* const data_rx){
	return xQueueReceive(motor_info_queue, data_rx, portMAX_DELAY);
}

esp_err_t i2c_mutex_init(){
	i2c_mutex = xSemaphoreCreateMutex();
	if(i2c_mutex == NULL) return ESP_ERR_NOT_FOUND;
	return ESP_OK;
}

BaseType_t take_i2c_mutex(){
	return xSemaphoreTake(i2c_mutex, portMAX_DELAY);
}

BaseType_t give_i2c_mutex(){
	return xSemaphoreGive(i2c_mutex);
}

bool motor_metrics_is_zero(const motor_metrics_t* const m){
	return m->accel_rms        == 0.0f &&
		   m->gyro_rms         == 0.0f &&
		   m->accel_peak       == 0.0f &&
           m->crest_factor     == 0.0f &&
		   m->dominant_freq_hz == 0.0f &&
		   m->temperature_c    == 0.0f;
}
