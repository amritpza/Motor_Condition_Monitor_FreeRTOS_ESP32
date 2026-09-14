#include <stdio.h>
#include <time.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2c_master.h"
#include "driver/gpio.h"

#include "lcd_i2c.h"
#include "mpu6050.h"
#include "ext_serial.h"

#include "ReadMotor.h"
#include "AssessMotorState.h"
#include "UIManager.h"

static TaskHandle_t read_motor_task_handle = NULL;

static void IRAM_ATTR mpu6050_int_pin_isr(void* arg){
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	vTaskNotifyGiveFromISR(read_motor_task_handle, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void app_main(){
	const ext_serial_config_t ext_serial_config = {
		.uart_port = EXT_SERIAL_UART_PORT, 
		.tx_pin    = EXT_SERIAL_UART_TX_GPIO,
		.rx_pin    = EXT_SERIAL_UART_RX_GPIO,
		.baud_rate = EXT_SERIAL_UART_BAUD_RATE,
		.buff_size = EXT_SERIAL_UART_BUFF_SIZE,
	};
	ESP_ERROR_CHECK(ext_serial_init(&ext_serial_config));

	const i2c_master_bus_config_t i2c_bus_config = {
		.i2c_port                     = I2C_NUM_0,
		.sda_io_num                   = GPIO_NUM_21,
		.scl_io_num                   = GPIO_NUM_22,
		.clk_source                   = I2C_CLK_SRC_DEFAULT,
		.glitch_ignore_cnt            = 7,
		.flags.enable_internal_pullup = true,
	};
	i2c_master_bus_handle_t i2c_bus;
	ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &i2c_bus));

	const lcd_i2c_config_t lcd_config = {
		.address  = LCD_I2C_ADDRESS,
		.columns  = 16,
		.rows     = 2,
		.clock_hz = LCD_I2C_CLK_HZ,
		.i2c_bus  = i2c_bus,
	};
	ESP_ERROR_CHECK(lcd_i2c_init(&lcd_config));
	ESP_ERROR_CHECK(lcd_i2c_clear());

	const gpio_num_t mpu6050_int_pin = GPIO_NUM_5;
	const mpu6050_config_t mpu_config = {
		.address      = MPU6050_ADDRESS_LOW,
		.int_pin_gpio = mpu6050_int_pin,
		.accel_range  = MPU6050_ACCEL_RANGE_4G,
		.gyro_range   = MPU6050_GYRO_RANGE_250_DPS,
		.clock_hz     = MPU6050_CLK_HZ,
		.i2c_bus      = i2c_bus,
	};
	ESP_ERROR_CHECK(mpu6050_init(&mpu_config));

	gpio_install_isr_service(0);
	gpio_isr_handler_add(mpu6050_int_pin, mpu6050_int_pin_isr, NULL);

	ESP_ERROR_CHECK(motor_metrics_queue_init());

	xTaskCreate(vTask_ReadMotor, "read_motor", 4096, NULL, 5, &read_motor_task_handle);
	xTaskCreate(vTask_AssessMotorState, "assess_motor_state", 4096, NULL, 4, NULL);
	xTaskCreate(vTask_UIManager, "ui_manager", 4096, NULL, 3, NULL);
}
