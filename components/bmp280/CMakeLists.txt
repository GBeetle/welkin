set(COMPONENT_SRCS "bmp280.c")

set(COMPONENT_REQUIRES log_sys error_handle spi_bus i2c_bus io_define mpu_driver)
set(COMPONENT_ADD_INCLUDEDIRS "include")

register_component()
