#pragma once
#include "types.h"

void ekf_init();
void ekf_reset();
void ekf_predict(float accel_world_x, float accel_world_y, float dt);
void ekf_correct(float flow_vel_x, float flow_vel_y, int quality);
void ekf_get_state(float *pos_x, float *pos_y, float *vel_x, float *vel_y);
