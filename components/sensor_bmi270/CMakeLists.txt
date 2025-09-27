idf_component_register(
    SRC_DIRS "src"
    INCLUDE_DIRS "include"     
    REQUIRES esp_driver_i2c
	REQUIRES esp_driver_gpio
    REQUIRES esp_timer
    REQUIRES i2c_bus
)

