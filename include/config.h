#ifndef CONFIG_H
#define CONFIG_H

// --- PINOUT (ESP32 30-PIN) ---
// Adapté à votre câblage physique :
#define PIN_MOTOR_1    27
#define PIN_MOTOR_2    13
#define PIN_MOTOR_3    25
#define PIN_MOTOR_4    26

// --- BATTERIE ---
#define PIN_BATTERY    34
#define BAT_SCALE      5.88f  // Calibré: 12.19V mesurés pour 2.09V lus

#define ESC_FREQ       250 

// --- RADIO (S.BUS) ---
#define PIN_SBUS_RX    4   
#define D_FILTER_COEFF 0.50f 
#define PIN_LED        5

// --- I2C ---
#define GYRO_ADDRESS   0x68
#define I2C_SPEED      400000

// Mettre à 1 si ton yaw est juste dans le mauvais sens (commande pilote inversée)
#define IMU_INVERT_YAW 1
#define RC_INVERT_YAW  0

// --- PARAMETRES DE VOL (PID) ---
#define PID_P_ROLL     1.35
#define PID_I_ROLL     0.021
#define PID_D_ROLL     2
#define PID_MAX_ROLL   400

#define PID_P_PITCH    1.35
#define PID_I_PITCH    0.021
#define PID_D_PITCH    2
#define PID_MAX_PITCH  400

#define PID_P_YAW      2
#define PID_I_YAW      0.01
#define PID_D_YAW      1.5
#define PID_MAX_YAW    400

// --- REGLAGES MOTEURS ---
#define MAX_THROTTLE_FLIGHT 2000
#define MIN_THROTTLE_IDLE   1050
#define MOTOR_OFF           1000

#define LOOP_TIME_US   4000 

// --- ALTITUDE HOLD ---
#define PIN_AUX1_CHANNEL    5           // Canal SBUS du switch AUX1
#define LIDAR_I2C_ADDR      0x10        // Adresse I2C du TF-Luna

// Plage de hauteur contrôlée par le stick (cm)
#define ALT_MIN_HEIGHT      15.0f       // Stick bas → 15 cm
#define ALT_MAX_HEIGHT      200.0f      // Stick haut → 200 cm (max fiable TF-Luna)
// Sécurité coupe-moteur en AltHold
#define ALT_HOLD_KILL_THROTTLE  1150    // Si gaz < cette valeur → coupe AltHold + retour ARMED

// Rampe de throttle au décollage en Alt Hold (µs ajoutés par cycle à 250Hz)
// 2 µs/cycle → atteint hover_throttle depuis MIN_THROTTLE_IDLE en ~240 cycles = ~1 seconde
#define ALT_THROTTLE_RAMP_RATE  2

// Lissage de la consigne altitude (filtre passe-bas)
// 0.005 = très doux (~3s), 0.01 = lent (~2s), 0.02 = modéré (~1s), 0.05 = réactif (~0.5s)
#define ALT_SMOOTH_DEFAULT      0.01f

// Gains PID Altitude par défaut
#define PID_P_ALT           1.5f
#define PID_I_ALT           0.05f
#define PID_D_ALT           1.0f
#define PID_MAX_ALT         400
#define HOVER_THROTTLE      1530        // Gaz de hover par défaut (µs)

// --- OPTICAL FLOW / POSITION HOLD ---
#define PIN_FLOW_RX         16
#define PIN_FLOW_TX         17
#define FLOW_BAUD           115200
#define FLOW_SCALE_FACTOR   420.0f   // Augmenté: valeurs étaient ~20x trop élevées
#define FLOW_QUALITY_MIN    50
#define PID_P_POS           0.3f
#define PID_I_POS           0.0f
#define PID_D_POS           0.5f
#define PID_MAX_POS_ANGLE   12.0f   // Angle max commandé par le position hold (degrés)
#define POS_SMOOTH_DEFAULT  0.02f
#define PIN_AUX2_CHANNEL    6       // Canal SBUS du switch AUX2

// Inversion axes optical flow (selon montage capteur)
// Mettre 1 pour normal, -1 pour inversé
#define FLOW_SIGN_X         -1
#define FLOW_SIGN_Y         -1

// --- Position Hold EKF ---
#define POS_STICK_DEADBAND      50      // us, deadband stick pour override pilote
#define EKF_Q_ACCEL             2.0f    // Bruit processus accelerometre (cm/s^2)^2
#define EKF_R_BASE              600.0f  // Bruit mesure OF de base (cm/s)^2

// --- IMU Calibration tuning ---
#define IMU_CALIB_SETTLE_MS     200     // temps après config (stabilisation capteur)
#define IMU_CALIB_DISCARD       50      // lectures jetées au début
#define IMU_CALIB_SAMPLES       1500    // échantillons utiles (plus = plus stable)
#define IMU_CALIB_DELAY_US      2000    // spacing entre échantillons
#define IMU_CALIB_STD_MAX_RAW   8.0f    // seuil "ça bouge" (en unités brutes gyro)

#endif