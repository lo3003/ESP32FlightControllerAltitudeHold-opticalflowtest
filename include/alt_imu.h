#ifndef ALT_IMU_H
#define ALT_IMU_H

#include "types.h"

// Adresses I2C du Pololu AltIMU-10 v2
#define L3GD20_ADDR     0x6B   // Gyroscope
#define LSM303_ACC_ADDR 0x19   // Accéléromètre LSM303DLHC
#define LSM303_MAG_ADDR 0x1E   // Magnétomètre LSM303DLHC

// Initialisation des capteurs L3GD20 (gyro), LSM303DLHC (accel + mag)
void alt_imu_init();

// Calibration magnétomètre (rotation pendant 10s)
void alt_imu_calibrate_mag();

// --- FreeRTOS Task API ---
void alt_imu_start_task();
void alt_imu_update(DroneState *drone);

#endif
