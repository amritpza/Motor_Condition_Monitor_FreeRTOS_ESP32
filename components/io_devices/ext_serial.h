#ifndef EXT_SERIAL_H
#define EXT_SERIAL_H

#include "driver/uart.h"
#include "driver/gpio.h"

#define EXT_SERIAL_UART_PORT       UART_NUM_2
#define EXT_SERIAL_UART_TX_GPIO    GPIO_NUM_17
#define EXT_SERIAL_UART_RX_GPIO    GPIO_NUM_16
#define EXT_SERIAL_UART_BAUD_RATE  115200
#define EXT_SERIAL_UART_BUFF_SIZE  1024

typedef struct{
	uart_port_t uart_port;
	gpio_num_t tx_pin;
	gpio_num_t rx_pin;
	uint32_t baud_rate;
	uint32_t buff_size;
} ext_serial_config_t;

esp_err_t ext_serial_init(const ext_serial_config_t *config);
esp_err_t ext_serial_deinit();

int ext_serial_print(const char *msg);
int ext_serial_read(char *rx_data, const uint32_t rx_data_size);

#endif
