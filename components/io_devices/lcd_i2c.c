#include "lcd_i2c.h"

#include "driver/i2c_master.h"
#include "esp_rom_sys.h"

#define LCD_RS                    0x01
#define LCD_ENABLE                0x04
#define LCD_BACKLIGHT             0x08

#define LCD_CMD_CLEAR             0x01
#define LCD_CMD_HOME              0x02
#define LCD_CMD_ENTRY_MODE        0x06
#define LCD_CMD_DISPLAY_ON        0x0C
#define LCD_CMD_FUNCTION          0x28
#define LCD_CMD_SET_DDRAM         0x80

#define LCD_I2C_TX_RX_TIMEOUT_MS  100

static i2c_master_dev_handle_t lcd_dev;
static uint8_t lcd_columns;
static uint8_t lcd_rows;
static bool lcd_initialized = false;

static esp_err_t lcd_write_nibble(const uint8_t nibble, const uint8_t mode){
	uint8_t data = (uint8_t)((nibble & 0x0F) << 4) | mode | LCD_BACKLIGHT | LCD_ENABLE;

	esp_err_t err = i2c_master_transmit(lcd_dev, &data, 1, LCD_I2C_TX_RX_TIMEOUT_MS);
	if(err != ESP_OK) return err;
	esp_rom_delay_us(1);

	data = data & (uint8_t)~LCD_ENABLE;
	err = i2c_master_transmit(lcd_dev, &data, 1, LCD_I2C_TX_RX_TIMEOUT_MS);
	esp_rom_delay_us(50);

	return err;
}

static esp_err_t lcd_write_byte(const uint8_t value, const uint8_t mode){
	esp_err_t err = lcd_write_nibble((uint8_t)(value >> 4), mode);
	if(err != ESP_OK) return err;

	return lcd_write_nibble(value, mode);
}

static esp_err_t lcd_command(const uint8_t command){
	esp_err_t err = lcd_write_byte(command, 0);

	if(err == ESP_OK && (command == LCD_CMD_CLEAR || command == LCD_CMD_HOME))
		esp_rom_delay_us(2000);

	return err;
}

esp_err_t lcd_i2c_init(const lcd_i2c_config_t *config){
	if(lcd_initialized) return ESP_ERR_INVALID_STATE;

	if(config == NULL || config->address > 0x7F ||
	   config->columns == 0 || config->columns > 20 ||
	   config->rows == 0 || config->rows > 4)
		return ESP_ERR_INVALID_ARG;

	const i2c_device_config_t device_config = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address = config->address,
		.scl_speed_hz = config->clock_hz ? config->clock_hz : LCD_I2C_CLK_HZ,
	};
	esp_err_t err = i2c_master_bus_add_device(config->i2c_bus, &device_config, &lcd_dev);
	if(err != ESP_OK) return err;
	lcd_initialized = true;

	esp_rom_delay_us(45000);

	err = lcd_write_nibble(0x03, 0);
	if(err == ESP_OK){
		esp_rom_delay_us(4500);
		err = lcd_write_nibble(0x03, 0);
	}

	if(err == ESP_OK){
		esp_rom_delay_us(105);
		err = lcd_write_nibble(0x03, 0);
	}

	if(err == ESP_OK){
		esp_rom_delay_us(150);
		err = lcd_write_nibble(0x02, 0);
	}

	if(err == ESP_OK) err = lcd_command(LCD_CMD_FUNCTION);
	if(err == ESP_OK) err = lcd_command(LCD_CMD_DISPLAY_ON);
	if(err == ESP_OK) err = lcd_command(LCD_CMD_CLEAR);
	if(err == ESP_OK) err = lcd_command(LCD_CMD_ENTRY_MODE);

	if(err != ESP_OK){
		lcd_i2c_deinit();
		return err;
	}

	lcd_columns = config->columns;
	lcd_rows = config->rows;

	return ESP_OK;
}

esp_err_t lcd_i2c_deinit(){
	if(!lcd_initialized) return ESP_ERR_INVALID_STATE;

	esp_err_t err = i2c_master_bus_rm_device(lcd_dev);
	if(err != ESP_OK) return err;

	lcd_dev = NULL;
	lcd_initialized = false;

	return ESP_OK;
}

esp_err_t lcd_i2c_clear(){
	return lcd_initialized ? lcd_command(LCD_CMD_CLEAR) : ESP_ERR_INVALID_STATE;
}

esp_err_t lcd_i2c_home(){
	return lcd_initialized ? lcd_command(LCD_CMD_HOME) : ESP_ERR_INVALID_STATE;
}

esp_err_t lcd_i2c_set_cursor(const uint8_t column, const uint8_t row){
	if(!lcd_initialized) return ESP_ERR_INVALID_STATE;
	if(row >= lcd_rows || column >= lcd_columns) return ESP_ERR_INVALID_ARG;

	const uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54};

	return lcd_command((uint8_t)(LCD_CMD_SET_DDRAM | (row_offsets[row] + column)));
}

esp_err_t lcd_i2c_write(const char *text){
	if(!lcd_initialized) return ESP_ERR_INVALID_STATE;
	if(text == NULL) return ESP_ERR_INVALID_ARG;

	while(*text != '\0'){
		esp_err_t err = lcd_write_byte((uint8_t)*text++, LCD_RS);
		if (err != ESP_OK) return err;
	}

	return ESP_OK;
}
