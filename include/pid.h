#ifndef PID_H
#define PID_H
#include "types.h"

void pid_init();
void pid_init_params(DroneState *drone); // <--- AJOUT
void pid_reset_integral();
void pid_reset_altitude_integral();       // Reset I altitude seul
void pid_preload_altitude_integral(float preload_value);
void pid_compute_setpoints(DroneState *drone);
void pid_compute(DroneState *drone);
void pid_compute_altitude(DroneState *drone);
void pid_reset_position();
void pid_compute_position(DroneState *drone);
float pid_get_quality_fade();

#endif