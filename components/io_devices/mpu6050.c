#include "mpu6050.h"

#include "driver/i2c_master.h"
#include "driver/gpio.h"

#define MPU6050_REG_WHO_AM_I        0x75

#define MPU6050_REG_PWR_MGMT_1      0x6B
#define MPU6050_CLKSEL_PLL_X        0x01

#define MPU6050_REG_CONFIG          0x1A
#define MPU6050_DLPF_CFG_3          0x03

#define MPU6050_REG_SMPRT_DIV       0x19
// Value of this reg is in h file

#define MPU6050_REG_INT_PIN_CFG     0x37
#define MPU6050_LOW_OPEN_LATCH_CLR  0xF0

#define MPU6050_REG_INT_ENABLE      0x38
#define MPU6050_DATA_RDY_EN         0x01

#define MPU6050_REG_ACCEL_CONFIG    0x1C

#define MPU6050_REG_GYRO_CONFIG     0x1B

#define MPU6050_REG_ACCEL_XOUT_H    0x3B

#define MPU6050_TX_RX_TIMEOUT_MS    100

static i2c_master_dev_handle_t mpu6050_device;
static mpu6050_accel_range_t mpu6050_accel_range;
static mpu6050_gyro_range_t mpu6050_gyro_range;
static bool mpu6050_initialized = false;

static esp_err_t mpu6050_write_register(const uint8_t reg, const uint8_t value){
    const uint8_t data[] = {reg, value};
    return i2c_master_transmit(mpu6050_device, data, sizeof(data), MPU6050_TX_RX_TIMEOUT_MS);
}

static esp_err_t mpu6050_read_register(const uint8_t reg_addr, uint8_t *data, const size_t length){
    if(data == NULL || length == 0) return ESP_ERR_INVALID_ARG;
    return i2c_master_transmit_receive(mpu6050_device, &reg_addr, 1, data, length, MPU6050_TX_RX_TIMEOUT_MS);
}

static int16_t mpu6050_make_int16(const uint8_t high, const uint8_t low){
    return (int16_t)(((uint16_t)high << 8) | low);
}

static float mpu6050_accel_scale(const mpu6050_accel_range_t range){
    const float scale[] = {16384.0f, 8192.0f, 4096.0f, 2048.0f};
    return scale[range];
}

static float mpu6050_gyro_scale(const mpu6050_gyro_range_t range){
	const float scale[] = {131.0f, 65.5f, 32.8f, 16.4f};
    return scale[range];
}

esp_err_t mpu6050_init(const mpu6050_config_t *config){
	if(mpu6050_initialized) return ESP_ERR_INVALID_STATE;

	if(config == NULL || (config->address != MPU6050_ADDRESS_LOW && config->address != MPU6050_ADDRESS_HIGH) || 
	   config->accel_range > MPU6050_ACCEL_RANGE_16G || config->gyro_range > MPU6050_GYRO_RANGE_2000_DPS)
		return ESP_ERR_INVALID_ARG;

	const gpio_config_t int_pin_config = {
		.intr_type = GPIO_INTR_NEGEDGE,
		.mode = GPIO_MODE_INPUT,
		.pin_bit_mask = (1ULL << config->int_pin_gpio),
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.pull_up_en = GPIO_PULLUP_ENABLE
	};
	esp_err_t err = gpio_config(&int_pin_config);
	if(err != ESP_OK) return err;

	const i2c_device_config_t device_config = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address = config->address,
		.scl_speed_hz = config->clock_hz ? config->clock_hz : MPU6050_CLK_HZ,
	};
	err = i2c_master_bus_add_device(config->i2c_bus, &device_config, &mpu6050_device);
	if(err != ESP_OK) return err;
	mpu6050_initialized = true;

	uint8_t who_am_i = 0;
	err = mpu6050_read_register(MPU6050_REG_WHO_AM_I, &who_am_i, 1);
	if (err == ESP_OK && (who_am_i & 0x7E) != MPU6050_ADDRESS_LOW) err = ESP_ERR_NOT_FOUND;

	if(err == ESP_OK) err = mpu6050_write_register(MPU6050_REG_PWR_MGMT_1, MPU6050_CLKSEL_PLL_X);
	if(err == ESP_OK) err = mpu6050_write_register(MPU6050_REG_CONFIG, MPU6050_DLPF_CFG_3);
	if(err == ESP_OK) err = mpu6050_write_register(MPU6050_REG_SMPRT_DIV, MPU6050_SMPLRT_DIV);
	if(err == ESP_OK) err = mpu6050_write_register(MPU6050_REG_INT_PIN_CFG, MPU6050_LOW_OPEN_LATCH_CLR);
	if(err == ESP_OK) err = mpu6050_write_register(MPU6050_REG_INT_ENABLE, MPU6050_DATA_RDY_EN);
	if(err == ESP_OK) err = mpu6050_write_register(MPU6050_REG_ACCEL_CONFIG, (uint8_t)config->accel_range << 3);
	if(err == ESP_OK) err = mpu6050_write_register(MPU6050_REG_GYRO_CONFIG, (uint8_t)config->gyro_range << 3);

	if(err != ESP_OK) {
		mpu6050_deinit();
		return err;
	}

	mpu6050_accel_range = config->accel_range;
	mpu6050_gyro_range = config->gyro_range;

	return ESP_OK;
}

esp_err_t mpu6050_deinit(){
	if(!mpu6050_initialized) return ESP_ERR_INVALID_STATE;

	esp_err_t err = i2c_master_bus_rm_device(mpu6050_device);
	if(err != ESP_OK) return err;

	mpu6050_device = NULL;
	mpu6050_initialized = false;

	return ESP_OK;
}

esp_err_t mpu6050_read_raw(mpu6050_raw_data_t *data){
	if(!mpu6050_initialized) return ESP_ERR_INVALID_STATE;
	if(data == NULL) return ESP_ERR_INVALID_ARG;

	uint8_t buffer[14];
	esp_err_t err = mpu6050_read_register(MPU6050_REG_ACCEL_XOUT_H, buffer, sizeof(buffer));
	if(err != ESP_OK) return err;

	data->accel_x = mpu6050_make_int16(buffer[0], buffer[1]);
	data->accel_y = mpu6050_make_int16(buffer[2], buffer[3]);
	data->accel_z = mpu6050_make_int16(buffer[4], buffer[5]);
	data->temperature = mpu6050_make_int16(buffer[6], buffer[7]);
	data->gyro_x = mpu6050_make_int16(buffer[8], buffer[9]);
	data->gyro_y = mpu6050_make_int16(buffer[10], buffer[11]);
	data->gyro_z = mpu6050_make_int16(buffer[12], buffer[13]);

	return ESP_OK;
}

esp_err_t mpu6050_read(mpu6050_data_t *data){
	if(!mpu6050_initialized) return ESP_ERR_INVALID_STATE;
	if(data == NULL) return ESP_ERR_INVALID_ARG;

	mpu6050_raw_data_t raw = {0};
	esp_err_t err = mpu6050_read_raw(&raw);
	if(err != ESP_OK) return err;

	const float accel_scale = mpu6050_accel_scale(mpu6050_accel_range);
	const float gyro_scale = mpu6050_gyro_scale(mpu6050_gyro_range);

	data->accel_x_g = raw.accel_x / accel_scale;
	data->accel_y_g = raw.accel_y / accel_scale;
	data->accel_z_g = raw.accel_z / accel_scale;
	data->temperature_c = (raw.temperature / 340.0f) + 36.53f;
	data->gyro_x_dps = raw.gyro_x / gyro_scale;
	data->gyro_y_dps = raw.gyro_y / gyro_scale;
	data->gyro_z_dps = raw.gyro_z / gyro_scale;

	return ESP_OK;
}
