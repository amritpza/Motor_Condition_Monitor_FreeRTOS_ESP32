#ifndef READMOTOR_H
#define READMOTOR_H

#include "freertos/FreeRTOS.h"

typedef struct{
	float accel_rms;         // Average vibration energy
	float gyro_rms;          // Average rotational vibration energy
	float accel_peak;        // Highest vibration value (shock)
	float crest_factor;      // Level of vibration impulsiveness (bearing defects, looseness, etc.)
	float dominant_freq_hz;  // Highest vibration energy in frequency domain
	float temperature_c;     // Motor temperature
} motor_metrics_t;

void vTask_ReadMotor(void *pvParameters);

BaseType_t motor_info_queue_receive(motor_metrics_t* const data_rx);

esp_err_t i2c_mutex_init();
BaseType_t take_i2c_mutex();
BaseType_t give_i2c_mutex();

bool motor_metrics_is_zero(const motor_metrics_t* const m);

#endif
