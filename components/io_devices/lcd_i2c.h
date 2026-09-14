#ifndef LCD_I2C_H
#define LCD_I2C_H

#include "driver/i2c_master.h"

#define LCD_I2C_ADDRESS  0x27
#define LCD_I2C_CLK_HZ   100000

typedef struct{
	uint8_t address;
	uint8_t columns;
	uint8_t rows;
	uint32_t clock_hz;
	i2c_master_bus_handle_t i2c_bus;
} lcd_i2c_config_t;

esp_err_t lcd_i2c_init(const lcd_i2c_config_t *config);
esp_err_t lcd_i2c_deinit();

esp_err_t lcd_i2c_clear();
esp_err_t lcd_i2c_home();
esp_err_t lcd_i2c_set_cursor(const uint8_t column, const uint8_t row);
esp_err_t lcd_i2c_write(const char *text);

#endif
