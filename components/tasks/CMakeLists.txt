idf_component_register(
	SRCS "AssessMotorState.c" "ReadMotor.c" "UIManager.c" # List all .c files in this folder
	INCLUDE_DIRS "."                                      # Tells the compiler headers are in this directory
	REQUIRES io_devices esp-dsp                           # List the public requirements (component names)
)
