#include "ext_serial.h"

#include <string.h>

#include "driver/uart.h"

static uart_port_t ext_serial_uart_port;
static bool ext_serial_initialized = false;

esp_err_t ext_serial_init(const ext_serial_config_t *config){
  if(ext_serial_initialized) return ESP_ERR_INVALID_STATE;

	if(config == NULL || config->uart_port >= UART_NUM_MAX ||
	   config->baud_rate == 0 || config->buff_size == 0)
		return ESP_ERR_INVALID_ARG;

	const uart_config_t uart_config = {
		.baud_rate =  config->baud_rate,
		.data_bits = UART_DATA_8_BITS,
		.parity    = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
	};

  esp_err_t err = uart_driver_install(config->uart_port, config->buff_size, config->buff_size, 0, NULL, 0);
  if(err == ESP_OK) err = uart_param_config(config->uart_port, &uart_config);
  if(err == ESP_OK) err = uart_set_pin(config->uart_port, config->tx_pin, config->rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

  ext_serial_initialized = true;

  if(err != ESP_OK){
    ext_serial_deinit();
    return err;
  }

  ext_serial_uart_port = config->uart_port;

  return ESP_OK;
}

esp_err_t ext_serial_deinit(){
  if(!ext_serial_initialized) return ESP_ERR_INVALID_STATE;

  esp_err_t err = uart_driver_delete(ext_serial_uart_port);
  if(err != ESP_OK) return err;

  ext_serial_initialized = false;

  return ESP_OK;
}

int ext_serial_print(const char *msg){
  return ext_serial_initialized ? uart_write_bytes(ext_serial_uart_port, msg, strlen(msg)) : ESP_FAIL;
}

int ext_serial_read(char *rx_data, const uint32_t rx_data_size){
  return ext_serial_initialized ? uart_read_bytes(ext_serial_uart_port, rx_data, rx_data_size, portMAX_DELAY) : ESP_FAIL;
}
