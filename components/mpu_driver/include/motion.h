#ifndef MPU_MOTION_H__
#define MPU_MOTION_H__

#include <math.h>
#include "mpu_types.h"
#include "mpu_math.h"
#include "ml_math_func.h"

void imuUpdate(float_axes_t acc, float_axes_t gyro, motion_state_t *state , float dt);

#endif
