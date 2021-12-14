#ifndef AHRS_H__
#define AHRS_H__

#include <math.h>
#include "mpu_types.h"
#include "mpu_math.h"
#include "ml_math_func.h"

void imuInit(void);
void imuUpdateAttitude(float_axes_t acc, float_axes_t gyro, float_axes_t mag, motion_state_t *state, float dt);

#endif
