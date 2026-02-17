#include <Arduino.h>
#include <Wire.h>
#include "alt_imu.h"
#include "imu.h"      // Pour accéder au mutex I2C partagé
#include "config.h"
#include "kalman.h"
#include "motors.h"   // Pour maintenir le signal ESC pendant la calibration

// --- FreeRTOS ---
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// ==================== REGISTRES L3GD20 ====================
#define L3GD20_CTRL_REG1    0x20
#define L3GD20_CTRL_REG4    0x23
#define L3GD20_OUT_X_L      0x28

// ==================== REGISTRES LSM303DLHC ACCELEROMETRE ====================
#define LSM303_CTRL_REG1_A  0x20
#define LSM303_CTRL_REG4_A  0x23
#define LSM303_OUT_X_L_A    0x28

// ==================== REGISTRES LSM303DLHC MAGNETOMETRE ====================
#define LSM303_CRA_REG_M    0x00  // Data rate configuration
#define LSM303_CRB_REG_M    0x01  // Gain configuration
#define LSM303_MR_REG_M     0x02  // Mode select
#define LSM303_OUT_X_H_M    0x03  // Data output (X,Z,Y order!)

// ==================== CALIBRATION OFFSETS GYRO ====================
static float alt_gyro_off_x = 0.0f;
static float alt_gyro_off_y = 0.0f;
static float alt_gyro_off_z = 0.0f;

// ==================== CALIBRATION OFFSETS MAGNETOMETRE ====================
static float mag_offset_x = 0.0f;
static float mag_offset_y = 0.0f;
static float mag_offset_z = 0.0f;
static float mag_scale_x = 1.0f;
static float mag_scale_y = 1.0f;
static float mag_scale_z = 1.0f;
static bool mag_calibrated = false;

// ==================== FILTRES KALMAN (INSTANCE LOCALE) ====================
static Kalman alt_kalman_roll;
static Kalman alt_kalman_pitch;
static bool alt_kalman_initialized = false;
static bool alt_imu_available = false;  // Flag pour indiquer si l'AltIMU est connecté

// ==================== FILTRE PT1 GYRO ====================
#define ALT_GYRO_PT1_COEFF 0.5f
static float alt_gyro_roll_filt = 0.0f;
static float alt_gyro_pitch_filt = 0.0f;
static float alt_gyro_yaw_filt = 0.0f;

// ==================== SNAPSHOT ALT_IMU ====================
typedef struct {
    float alt_acc_x;
    float alt_acc_y;
    float alt_acc_z;
    float alt_gyro_roll;
    float alt_gyro_pitch;
    float alt_gyro_yaw;
    float alt_angle_roll;
    float alt_angle_pitch;
    float alt_angle_yaw;     // Heading magnétomètre
    unsigned long last_dur_us;
    unsigned long last_ok_ms;
    bool ok;
} AltImuSnapshot;

static portMUX_TYPE alt_imu_mux = portMUX_INITIALIZER_UNLOCKED;
static AltImuSnapshot alt_imu_snap = {0};

static DroneState alt_imu_state;
static TaskHandle_t alt_imu_task_handle = nullptr;

// ==================== LECTURE MAGNETOMETRE RAW ====================
static bool read_mag_raw(int16_t *mx, int16_t *my, int16_t *mz) {
    Wire.beginTransmission(LSM303_MAG_ADDR);
    Wire.write(LSM303_OUT_X_H_M);
    if (Wire.endTransmission() != 0) return false;

    uint8_t count = Wire.requestFrom(LSM303_MAG_ADDR, 6);
    if (count < 6) return false;

    // LSM303DLHC: ordre des données = X_H, X_L, Z_H, Z_L, Y_H, Y_L (attention: Z avant Y!)
    uint8_t xh = Wire.read();
    uint8_t xl = Wire.read();
    uint8_t zh = Wire.read();
    uint8_t zl = Wire.read();
    uint8_t yh = Wire.read();
    uint8_t yl = Wire.read();

    *mx = (int16_t)((xh << 8) | xl);
    *mz = (int16_t)((zh << 8) | zl);
    *my = (int16_t)((yh << 8) | yl);

    return true;
}

// ==================== ALT_IMU_INIT ====================
void alt_imu_init() {
    Serial.println(F("ALT_IMU: Checking L3GD20 (Gyro)..."));

    // Vérifier si le L3GD20 est présent
    Wire.beginTransmission(L3GD20_ADDR);
    uint8_t err = Wire.endTransmission();
    if (err != 0) {
        Serial.println(F("ALT_IMU: L3GD20 NOT FOUND! Skipping AltIMU init."));
        alt_imu_available = false;
        return;
    }

    Serial.println(F("ALT_IMU: Init L3GD20 (Gyro 500dps)..."));

    // L3GD20: Wake up + 380Hz ODR, cut-off 25Hz
    Wire.beginTransmission(L3GD20_ADDR);
    Wire.write(L3GD20_CTRL_REG1);
    Wire.write(0xAF);  // 10101111: PD=1, Zen=1, Yen=1, Xen=1, BW=10 (25Hz), DR=10 (380Hz)
    Wire.endTransmission();

    // L3GD20: 500dps full scale
    Wire.beginTransmission(L3GD20_ADDR);
    Wire.write(L3GD20_CTRL_REG4);
    Wire.write(0x10);  // FS=01 (500dps), BLE=0 (little endian)
    Wire.endTransmission();

    Serial.println(F("ALT_IMU: Init LSM303DLHC (Accel ±8g)..."));

    // LSM303DLHC Accel: 400Hz ODR, normal mode, all axes enabled
    Wire.beginTransmission(LSM303_ACC_ADDR);
    Wire.write(LSM303_CTRL_REG1_A);
    Wire.write(0x77);  // ODR=0111 (400Hz), LPen=0, Zen=1, Yen=1, Xen=1
    Wire.endTransmission();

    // LSM303DLHC Accel: ±8g full scale, high resolution mode
    Wire.beginTransmission(LSM303_ACC_ADDR);
    Wire.write(LSM303_CTRL_REG4_A);
    Wire.write(0x28);  // BDU=0, BLE=0, FS=10 (±8g), HR=1 (high res), 00
    Wire.endTransmission();

    // ========== INITIALISATION MAGNETOMETRE LSM303DLHC ==========
    Serial.println(F("ALT_IMU: Init LSM303DLHC (Mag)..."));

    // CRA_REG_M: 75Hz ODR, temperature sensor disabled
    Wire.beginTransmission(LSM303_MAG_ADDR);
    Wire.write(LSM303_CRA_REG_M);
    Wire.write(0x18);  // DO[2:0]=110 (75Hz), TEMP_EN=0
    Wire.endTransmission();

    // CRB_REG_M: ±1.3 gauss (GN=001, default)
    Wire.beginTransmission(LSM303_MAG_ADDR);
    Wire.write(LSM303_CRB_REG_M);
    Wire.write(0x20);  // GN[2:0]=001 (±1.3 gauss, 1100 LSB/gauss XY, 980 LSB/gauss Z)
    Wire.endTransmission();

    // MR_REG_M: Continuous conversion mode
    Wire.beginTransmission(LSM303_MAG_ADDR);
    Wire.write(LSM303_MR_REG_M);
    Wire.write(0x00);  // MD[1:0]=00 (continuous)
    Wire.endTransmission();

    // Calibration gyroscope (5 secondes)
    Serial.println(F(""));
    Serial.println(F("ALT_IMU: Calibration Gyro L3GD20 - NE PAS BOUGER LE DRONE!"));
    Serial.println(F("ALT_IMU: Calibration en cours (5 secondes)..."));

    long gyro_sum_x = 0, gyro_sum_y = 0, gyro_sum_z = 0;
    int valid_samples = 0;

    // Calibration pendant 5 secondes avec LED clignotante rapide (50ms)
    unsigned long calib_start = millis();
    unsigned long last_led_toggle = 0;
    bool led_state = false;

    while (millis() - calib_start < 5000) {
        // Clignotement LED rapide (50ms on, 50ms off)
        if (millis() - last_led_toggle >= 50) {
            led_state = !led_state;
            digitalWrite(PIN_LED, led_state);
            last_led_toggle = millis();
        }

        // Signal ESC "heartbeat" variable (1000-1019µs) pour éviter timeout ESC
        // Le signal oscille au lieu d'être statique, ce qui empêche la protection timeout
        int pwm_heartbeat = 1000 + (millis() % 20);
        motors_write_direct(pwm_heartbeat, pwm_heartbeat, pwm_heartbeat, pwm_heartbeat);

        // Lecture avec auto-increment (bit 7 = 1)
        Wire.beginTransmission(L3GD20_ADDR);
        Wire.write(L3GD20_OUT_X_L | 0x80);
        Wire.endTransmission();
        Wire.requestFrom(L3GD20_ADDR, 6);

        if (Wire.available() < 6) {
            delayMicroseconds(2000);
            continue;
        }

        int16_t gx = (int16_t)(Wire.read() | (Wire.read() << 8));
        int16_t gy = (int16_t)(Wire.read() | (Wire.read() << 8));
        int16_t gz = (int16_t)(Wire.read() | (Wire.read() << 8));

        gyro_sum_x += gx;
        gyro_sum_y += gy;
        gyro_sum_z += gz;
        valid_samples++;

        delayMicroseconds(2000);
    }

    digitalWrite(PIN_LED, LOW);

    if (valid_samples > 0) {
        alt_gyro_off_x = (float)gyro_sum_x / valid_samples;
        alt_gyro_off_y = (float)gyro_sum_y / valid_samples;
        alt_gyro_off_z = (float)gyro_sum_z / valid_samples;
    }

    Serial.printf("ALT_IMU: %d echantillons collectes\n", valid_samples);

    // Configuration Kalman pour AltIMU
    alt_kalman_roll.setQangle(0.001f);
    alt_kalman_roll.setQbias(0.003f);
    alt_kalman_roll.setRmeasure(0.03f);

    alt_kalman_pitch.setQangle(0.001f);
    alt_kalman_pitch.setQbias(0.003f);
    alt_kalman_pitch.setRmeasure(0.03f);

    alt_imu_available = true;  // AltIMU initialisé avec succès
    Serial.println(F("ALT_IMU: Init OK"));
}

// ==================== CALIBRATION MAGNETOMETRE ====================
void alt_imu_calibrate_mag() {
    if (!alt_imu_available) {
        Serial.println(F("ALT_IMU: Sensor not available, cannot calibrate mag"));
        return;
    }

    Serial.println(F(""));
    Serial.println(F("========================================"));
    Serial.println(F("MAG CALIBRATION: TOURNER LE DRONE DANS"));
    Serial.println(F("TOUTES LES DIRECTIONS pendant 20 secondes"));
    Serial.println(F("(Pitch, Roll, Yaw 360 degres)"));
    Serial.println(F("========================================"));

    float min_x = 32767.0f, max_x = -32768.0f;
    float min_y = 32767.0f, max_y = -32768.0f;
    float min_z = 32767.0f, max_z = -32768.0f;

    unsigned long start_time = millis();
    unsigned long last_led_toggle = 0;
    bool led_state = false;
    int sample_count = 0;

    // LED clignotante lente (500ms) pendant calibration 20 secondes
    while (millis() - start_time < 20000) {
        // Clignotement LED lent (500ms on, 500ms off)
        if (millis() - last_led_toggle >= 500) {
            led_state = !led_state;
            digitalWrite(PIN_LED, led_state);
            last_led_toggle = millis();
        }

        // Signal ESC "heartbeat" variable (1000-1019µs) pour éviter timeout ESC
        // Le signal oscille au lieu d'être statique, ce qui empêche la protection timeout
        int pwm_heartbeat = 1000 + (millis() % 20);
        motors_write_direct(pwm_heartbeat, pwm_heartbeat, pwm_heartbeat, pwm_heartbeat);

        int16_t mx, my, mz;
        if (read_mag_raw(&mx, &my, &mz)) {
            // Track min/max
            if (mx < min_x) min_x = mx;
            if (mx > max_x) max_x = mx;
            if (my < min_y) min_y = my;
            if (my > max_y) max_y = my;
            if (mz < min_z) min_z = mz;
            if (mz > max_z) max_z = mz;
            sample_count++;
        }

        delay(20);  // ~50Hz sampling
    }

    digitalWrite(PIN_LED, LOW);

    if (sample_count < 100) {
        Serial.println(F("MAG CALIBRATION: FAILED - not enough samples!"));
        return;
    }

    // Calculate hard-iron offsets (centre de l'ellipse)
    mag_offset_x = (max_x + min_x) / 2.0f;
    mag_offset_y = (max_y + min_y) / 2.0f;
    mag_offset_z = (max_z + min_z) / 2.0f;

    // Calculate soft-iron scales (normaliser les axes)
    float delta_x = (max_x - min_x) / 2.0f;
    float delta_y = (max_y - min_y) / 2.0f;
    float delta_z = (max_z - min_z) / 2.0f;

    // Éviter division par zéro
    if (delta_x < 1.0f) delta_x = 1.0f;
    if (delta_y < 1.0f) delta_y = 1.0f;
    if (delta_z < 1.0f) delta_z = 1.0f;

    float avg_delta = (delta_x + delta_y + delta_z) / 3.0f;
    mag_scale_x = avg_delta / delta_x;
    mag_scale_y = avg_delta / delta_y;
    mag_scale_z = avg_delta / delta_z;

    mag_calibrated = true;

    Serial.println(F(""));
    Serial.println(F("MAG CALIBRATION: Complete!"));
    Serial.printf("  Samples: %d\n", sample_count);
    Serial.printf("  Offsets: X=%.1f Y=%.1f Z=%.1f\n", mag_offset_x, mag_offset_y, mag_offset_z);
    Serial.printf("  Scales:  X=%.3f Y=%.3f Z=%.3f\n", mag_scale_x, mag_scale_y, mag_scale_z);
    Serial.println(F("========================================"));
}

// ==================== ALT_IMU_READ_INTERNAL ====================
static void alt_imu_read_internal(DroneState *drone) {
    static unsigned long last_us = 0;
    static unsigned long fail_count = 0;

    // Si l'AltIMU n'est pas disponible, ne rien faire
    if (!alt_imu_available) return;

    const unsigned long now_us = micros();
    float dt_s = 0.004f;
    if (last_us != 0) {
        dt_s = (now_us - last_us) * 1e-6f;
        if (dt_s < 0.002f) dt_s = 0.002f;
        if (dt_s > 0.010f) dt_s = 0.010f;
    }
    last_us = now_us;

    // ========== SECTION I2C PROTEGEE PAR MUTEX ==========
    // Prendre le mutex avant d'accéder au bus I2C
    if (i2c_mutex != nullptr) {
        if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(8)) != pdTRUE) {
            // Timeout: le mutex n'a pas pu être pris, skip cette lecture
            return;
        }
    }

    // ========== LECTURE GYROSCOPE L3GD20 ==========
    Wire.beginTransmission(L3GD20_ADDR);
    Wire.write(L3GD20_OUT_X_L | 0x80);  // Auto-increment
    uint8_t err_g = Wire.endTransmission();

    if (err_g != 0) {
        if (i2c_mutex != nullptr) xSemaphoreGive(i2c_mutex);
        fail_count++;
        if (fail_count >= 10) {
            // Log supprimé pour éviter les lags
            alt_imu_available = false;
        }
        return;
    }

    fail_count = 0;  // Reset si succès
    uint8_t count_g = Wire.requestFrom(L3GD20_ADDR, 6);

    int16_t gx = 0, gy = 0, gz = 0;
    if (count_g >= 6) {
        gx = (int16_t)(Wire.read() | (Wire.read() << 8));
        gy = (int16_t)(Wire.read() | (Wire.read() << 8));
        gz = (int16_t)(Wire.read() | (Wire.read() << 8));
    }

    // Correction offsets
    float gyro_x_cal = gx - alt_gyro_off_x;
    float gyro_y_cal = gy - alt_gyro_off_y;
    float gyro_z_cal = gz - alt_gyro_off_z;

    // Conversion en deg/s (500dps → 17.50 mdps/digit → 0.01750 dps/digit)
    const float GYRO_SCALE = 17.50f / 1000.0f;  // mdps to dps
    float gyro_roll_dps  = gyro_x_cal * GYRO_SCALE;
    float gyro_pitch_dps = -gyro_y_cal * GYRO_SCALE;  // Inverser selon orientation
    float gyro_yaw_dps   = gyro_z_cal * GYRO_SCALE;

    // Filtre PT1 passe-bas (identique au MPU6050)
    alt_gyro_roll_filt  += ALT_GYRO_PT1_COEFF * (gyro_roll_dps  - alt_gyro_roll_filt);
    alt_gyro_pitch_filt += ALT_GYRO_PT1_COEFF * (gyro_pitch_dps - alt_gyro_pitch_filt);
    alt_gyro_yaw_filt   += ALT_GYRO_PT1_COEFF * (gyro_yaw_dps   - alt_gyro_yaw_filt);

    drone->alt_gyro_roll  = alt_gyro_roll_filt;
    drone->alt_gyro_pitch = alt_gyro_pitch_filt;
    drone->alt_gyro_yaw   = alt_gyro_yaw_filt;

    // ========== LECTURE ACCELEROMETRE LSM303DLHC ==========
    Wire.beginTransmission(LSM303_ACC_ADDR);
    Wire.write(LSM303_OUT_X_L_A | 0x80);  // Auto-increment
    Wire.endTransmission();
    uint8_t count_a = Wire.requestFrom(LSM303_ACC_ADDR, 6);

    int16_t ax = 0, ay = 0, az = 0;
    if (count_a >= 6) {
        // LSM303DLHC: données 12-bit alignées à gauche (shift right 4)
        ax = (int16_t)(Wire.read() | (Wire.read() << 8)) >> 4;
        ay = (int16_t)(Wire.read() | (Wire.read() << 8)) >> 4;
        az = (int16_t)(Wire.read() | (Wire.read() << 8)) >> 4;
    }

    // Conversion en G (±8g mode, 12-bit: 4mg/LSB → 0.004 G/LSB)
    const float ACC_SCALE = 0.004f;  // G per LSB
    drone->alt_acc_x = ax * ACC_SCALE;
    drone->alt_acc_y = ay * ACC_SCALE;
    drone->alt_acc_z = az * ACC_SCALE;

    // ========== CALCUL DES ANGLES ROLL/PITCH ==========
    float acc_total = sqrtf(drone->alt_acc_x * drone->alt_acc_x +
                            drone->alt_acc_y * drone->alt_acc_y +
                            drone->alt_acc_z * drone->alt_acc_z);

    float angle_roll_acc = 0.0f, angle_pitch_acc = 0.0f;

    if (acc_total > 0.1f) {  // Éviter division par zéro
        if (fabsf(drone->alt_acc_y) < acc_total) {
            angle_roll_acc = asinf(drone->alt_acc_y / acc_total) * 57.296f;
        }
        if (fabsf(drone->alt_acc_x) < acc_total) {
            angle_pitch_acc = asinf(drone->alt_acc_x / acc_total) * 57.296f;
        }
    }

    // ========== FILTRE KALMAN ROLL/PITCH ==========
    if (!alt_kalman_initialized) {
        alt_kalman_roll.setAngle(angle_roll_acc);
        alt_kalman_pitch.setAngle(angle_pitch_acc);
        drone->alt_angle_roll = angle_roll_acc;
        drone->alt_angle_pitch = angle_pitch_acc;
        alt_kalman_initialized = true;
    } else {
        drone->alt_angle_roll  = alt_kalman_roll.update(angle_roll_acc, gyro_roll_dps, dt_s);
        drone->alt_angle_pitch = alt_kalman_pitch.update(angle_pitch_acc, gyro_pitch_dps, dt_s);
    }

    // ========== LECTURE MAGNETOMETRE LSM303DLHC ==========
    int16_t mx_raw = 0, my_raw = 0, mz_raw = 0;
    bool mag_ok = read_mag_raw(&mx_raw, &my_raw, &mz_raw);
    
    // Libérer le mutex après toutes les lectures I2C
    if (i2c_mutex != nullptr) xSemaphoreGive(i2c_mutex);
    // ========== FIN SECTION I2C PROTEGEE ==========

    if (mag_ok) {
        // Appliquer calibration
        float mx = (mx_raw - mag_offset_x) * mag_scale_x;
        float my = (my_raw - mag_offset_y) * mag_scale_y;
        float mz = (mz_raw - mag_offset_z) * mag_scale_z;

        // Compensation de l'inclinaison (tilt compensation)
        float roll_rad = drone->alt_angle_roll * 0.01745329f;   // DEG_TO_RAD
        float pitch_rad = drone->alt_angle_pitch * 0.01745329f;

        float cos_roll = cosf(roll_rad);
        float sin_roll = sinf(roll_rad);
        float cos_pitch = cosf(pitch_rad);
        float sin_pitch = sinf(pitch_rad);

        // Formules de compensation tilt
        float mx_comp = mx * cos_pitch + mz * sin_pitch;
        float my_comp = mx * sin_roll * sin_pitch + my * cos_roll - mz * sin_roll * cos_pitch;

        // Calcul du heading (cap magnétique)
        float heading = atan2f(-my_comp, mx_comp) * 57.296f;  // RAD_TO_DEG

        // Normaliser entre 0 et 360
        if (heading < 0.0f) heading += 360.0f;
        if (heading >= 360.0f) heading -= 360.0f;

        drone->alt_angle_yaw = heading;
    }
}

// ==================== TASK ALT_IMU ====================
static void alt_imu_task(void *parameter) {
    (void)parameter;

    memset(&alt_imu_state, 0, sizeof(alt_imu_state));

    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        unsigned long t0 = micros();
        alt_imu_read_internal(&alt_imu_state);
        unsigned long dur = micros() - t0;

        bool ok = alt_imu_available;

        portENTER_CRITICAL(&alt_imu_mux);
        alt_imu_snap.alt_acc_x       = alt_imu_state.alt_acc_x;
        alt_imu_snap.alt_acc_y       = alt_imu_state.alt_acc_y;
        alt_imu_snap.alt_acc_z       = alt_imu_state.alt_acc_z;
        alt_imu_snap.alt_gyro_roll   = alt_imu_state.alt_gyro_roll;
        alt_imu_snap.alt_gyro_pitch  = alt_imu_state.alt_gyro_pitch;
        alt_imu_snap.alt_gyro_yaw    = alt_imu_state.alt_gyro_yaw;
        alt_imu_snap.alt_angle_roll  = alt_imu_state.alt_angle_roll;
        alt_imu_snap.alt_angle_pitch = alt_imu_state.alt_angle_pitch;
        alt_imu_snap.alt_angle_yaw   = alt_imu_state.alt_angle_yaw;
        alt_imu_snap.last_dur_us     = dur;
        alt_imu_snap.ok              = ok;
        if (ok) alt_imu_snap.last_ok_ms = millis();
        portEXIT_CRITICAL(&alt_imu_mux);

        // 25 Hz suffit largement pour le magnétomètre/fusion (réduit la contention I2C)
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(40)); // 25 Hz au lieu de 50 Hz
    }
}

// ==================== API PUBLIQUE ====================
void alt_imu_start_task() {
    if (alt_imu_task_handle != nullptr) {
        Serial.println(F("ALT_IMU: Task already running!"));
        return;
    }

    if (!alt_imu_available) {
        Serial.println(F("ALT_IMU: Sensor not available, task not started"));
        return;
    }

    // Attendre que le mutex I2C soit créé par imu_start_task()
    if (i2c_mutex == nullptr) {
        Serial.println(F("ALT_IMU: WARNING - I2C mutex not yet created!"));
    }

    Serial.println(F("ALT_IMU: Starting FreeRTOS task..."));

    xTaskCreatePinnedToCore(
        alt_imu_task,
        "alt_imu",
        3072,           // Stack réduite (suffisant pour cette tâche)
        nullptr,
        1,              // Priority 1 (plus basse que l'IMU principal qui est à 4)
        &alt_imu_task_handle,
        0               // Core 0 (avec IMU principal, WiFi sur Core 1)
    );

    Serial.println(F("ALT_IMU: Task started on Core 0 @ 25Hz"));
}

void alt_imu_update(DroneState *drone) {
    AltImuSnapshot s;

    portENTER_CRITICAL(&alt_imu_mux);
    s = alt_imu_snap;
    portEXIT_CRITICAL(&alt_imu_mux);

    drone->alt_acc_x       = s.alt_acc_x;
    drone->alt_acc_y       = s.alt_acc_y;
    drone->alt_acc_z       = s.alt_acc_z;
    drone->alt_gyro_roll   = s.alt_gyro_roll;
    drone->alt_gyro_pitch  = s.alt_gyro_pitch;
    drone->alt_gyro_yaw    = s.alt_gyro_yaw;
    drone->alt_angle_roll  = s.alt_angle_roll;
    drone->alt_angle_pitch = s.alt_angle_pitch;
    drone->alt_angle_yaw   = s.alt_angle_yaw;
}
