#ifndef MOTORS_H
#define MOTORS_H
#include "types.h"

void motors_init();
void motors_stop();
void motors_mix(DroneState *drone);
void motors_write(); // Ecrit les valeurs calculées
void motors_write_direct(int m1, int m2, int m3, int m4); // Pour la calibration

#endif