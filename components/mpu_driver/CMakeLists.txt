set(COMPONENT_SRCS "mpu_driver.c"
                   "mpu6500_dmp.c"
                   "ml_math_func.c"
                   "motion.c"
                   "AHRS.c")
set(COMPONENT_REQUIRES spi_bus i2c_bus lis3mdl log_sys error_handle)

set(COMPONENT_ADD_INCLUDEDIRS "include")

register_component()
