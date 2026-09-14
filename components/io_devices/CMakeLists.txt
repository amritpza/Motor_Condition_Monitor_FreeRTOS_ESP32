idf_component_register(
	SRCS "ext_serial.c" "lcd_i2c.c" "mpu6050.c"             # List all .c files in this folder
	INCLUDE_DIRS "."                                        # Tells the compiler headers are in this directory
	REQUIRES esp_driver_uart esp_driver_i2c esp_driver_gpio # List the public requirements (component names)
)
