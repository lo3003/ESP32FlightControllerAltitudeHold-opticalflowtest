#ifndef LIDAR_H
#define LIDAR_H

#include "types.h"

// Initialise le TF-Luna sur I2C et démarre la tâche FreeRTOS (Core 0, 50 Hz)
void lidar_init();

// Copie thread-safe des dernières valeurs LiDAR dans le DroneState
void lidar_update(DroneState *drone);

#endif
