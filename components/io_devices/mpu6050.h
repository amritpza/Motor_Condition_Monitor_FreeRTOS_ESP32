#ifndef MPU6050_H
#define MPU6050_H

#include "driver/i2c_master.h"

#define MPU6050_ADDRESS_LOW       0x68
#define MPU6050_ADDRESS_HIGH      0x69
#define MPU6050_CLK_HZ            400000

#define MPU6050_SMPLRT_DIV        0x04
#define MPU6050_SAMPLE_RATE_HZ    (1000.0f / (1 + MPU6050_SMPLRT_DIV))
#define MPU6050_SAMPLE_PERIOD_MS  (1000.0f / MPU6050_SAMPLE_RATE_HZ)

typedef enum{
	MPU6050_ACCEL_RANGE_2G = 0,
	MPU6050_ACCEL_RANGE_4G,
	MPU6050_ACCEL_RANGE_8G,
	MPU6050_ACCEL_RANGE_16G,
} mpu6050_accel_range_t;

typedef enum{
	MPU6050_GYRO_RANGE_250_DPS = 0,
	MPU6050_GYRO_RANGE_500_DPS,
	MPU6050_GYRO_RANGE_1000_DPS,
	MPU6050_GYRO_RANGE_2000_DPS,
} mpu6050_gyro_range_t;

typedef struct{
	uint8_t address;
	gpio_num_t int_pin_gpio;
	mpu6050_accel_range_t accel_range;
	mpu6050_gyro_range_t gyro_range;
	uint32_t clock_hz;
	i2c_master_bus_handle_t i2c_bus;
} mpu6050_config_t;

typedef struct{
	int16_t accel_x;
	int16_t accel_y;
	int16_t accel_z;
	int16_t temperature;
	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;
} mpu6050_raw_data_t;

typedef struct{
	float accel_x_g;
	float accel_y_g;
	float accel_z_g;
	float temperature_c;
	float gyro_x_dps;
	float gyro_y_dps;
	float gyro_z_dps;
} mpu6050_data_t;

esp_err_t mpu6050_init(const mpu6050_config_t *config);
esp_err_t mpu6050_deinit();

esp_err_t mpu6050_read_raw(mpu6050_raw_data_t *data);
esp_err_t mpu6050_read(mpu6050_data_t *data);

#endif
