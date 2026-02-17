#include <Arduino.h>
#include <Wire.h>
#include "imu.h"
#include "config.h"
#include "motors.h"
#include "kalman.h"

// --- FreeRTOS ---
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// ==================== MUTEX I2C GLOBAL ====================
// Ce mutex est partagé entre imu.cpp et alt_imu.cpp pour éviter les conflits I2C
SemaphoreHandle_t i2c_mutex = nullptr;

// --- Offsets calibration ---
static double gyro_off_x = 0;
static double gyro_off_y = 0;
static double gyro_off_z = 0;

// --- Variables brutes ---
static int16_t acc_raw[3];
static int16_t gyro_raw[3];
static int16_t temperature;

// --- Filtres PT1 pour gyro (entrées PID) ---
static float gyro_roll_filt = 0.0f;
static float gyro_pitch_filt = 0.0f;
static float gyro_yaw_filt = 0.0f;
#define GYRO_PT1_COEFF 0.85f

// --- Filtres de Kalman pour Roll et Pitch ---
static Kalman kalman_roll;
static Kalman kalman_pitch;
static float yaw_angle = 0.0f;  // AJOUT: intégration simple du Yaw
static bool kalman_initialized = false;

// ==================== SNAPSHOT IMU ====================
typedef struct {
    float gyro_roll_input;
    float gyro_pitch_input;
    float gyro_yaw_input;
    float angle_roll;
    float angle_pitch;
    float angle_yaw;          // AJOUT
    float acc_total_vector;
    float acc_x;              // Accélération X en G
    float acc_y;              // Accélération Y en G
    float acc_z;              // Accélération Z en G
    unsigned long last_dur_us;
    unsigned long last_ok_ms;
    bool ok;
} ImuSnapshot;

static portMUX_TYPE imu_mux = portMUX_INITIALIZER_UNLOCKED;
static ImuSnapshot imu_snap = {0};

static volatile FlightMode imu_in_mode = MODE_SAFE;
static volatile int imu_in_ch3 = 1000;
static volatile bool imu_reset_req = false;
static volatile float imu_in_trim_roll  = 2.4f;
static volatile float imu_in_trim_pitch = -3.8f;

static DroneState imu_state;
static TaskHandle_t imu_task_handle = nullptr;

// ==================== IMU INIT ====================
void imu_init() {
    motors_write_direct(1000, 1000, 1000, 1000);

    Serial.println(F("IMU: Init Raw I2C..."));

    Wire.beginTransmission(MPU_ADDR); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission();
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x1B); Wire.write(0x08); Wire.endTransmission();
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x1C); Wire.write(0x10); Wire.endTransmission();
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x1A); Wire.write(0x03); Wire.endTransmission();

    Serial.println(F(""));
    Serial.println(F("IMU: Calibration Gyro MPU6050 - NE PAS BOUGER LE DRONE!"));
    Serial.println(F("IMU: Calibration en cours (5 secondes)..."));

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

        Wire.beginTransmission(MPU_ADDR);
        Wire.write(0x43);
        Wire.endTransmission();
        Wire.requestFrom(MPU_ADDR, 6);

        if (Wire.available() < 6) { delay(2); continue; }

        gyro_sum_x += (int16_t)(Wire.read() << 8 | Wire.read());
        gyro_sum_y += (int16_t)(Wire.read() << 8 | Wire.read());
        gyro_sum_z += (int16_t)(Wire.read() << 8 | Wire.read());
        valid_samples++;

        delayMicroseconds(2000);
    }

    if (valid_samples > 0) {
        gyro_off_x = (double)gyro_sum_x / valid_samples;
        gyro_off_y = (double)gyro_sum_y / valid_samples;
        gyro_off_z = (double)gyro_sum_z / valid_samples;
    }

    Serial.printf("IMU: %d echantillons collectes\n", valid_samples);

    // Configuration Kalman pour MPU6050
    kalman_roll.setQangle(0.001f);
    kalman_roll.setQbias(0.003f);
    kalman_roll.setRmeasure(0.03f);
    
    kalman_pitch.setQangle(0.001f);
    kalman_pitch.setQbias(0.003f);
    kalman_pitch.setRmeasure(0.03f);

    digitalWrite(PIN_LED, LOW);
    Serial.println(F("IMU: Calibration OK (Kalman Filter Enabled)"));
}

// ==================== IMU READ INTERNAL ====================
static void imu_read_internal(DroneState *drone) {
    static unsigned long last_us = 0;
    static unsigned long fail_count = 0;  // DEBUG: compteur d'échecs
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
    // Timeout de 2ms max pour garantir la priorité de l'IMU principal sans blocage trop long
    if (i2c_mutex != nullptr) {
        if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
            // Mutex occupé après 2ms: skip cette lecture (l'AltIMU utilise le bus)
            // Les anciennes valeurs seront conservées, pas de blocage
            return;
        }
    }

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    uint8_t err = Wire.endTransmission();

    if (err != 0) {
        if (i2c_mutex != nullptr) xSemaphoreGive(i2c_mutex);
        fail_count++;
        // Log supprimé pour éviter les lags (Serial.printf est bloquant)
        drone->max_time_imu = 888888;
        return;
    }

    uint8_t count = Wire.requestFrom(MPU_ADDR, 14);

    if (count < 14) {
        if (i2c_mutex != nullptr) xSemaphoreGive(i2c_mutex);
        fail_count++;
        // Log supprimé pour éviter les lags
        drone->max_time_imu = 888888;
        return;
    }
    
    fail_count = 0;  // Reset si succès

    acc_raw[0] = (int16_t)(Wire.read() << 8 | Wire.read());
    acc_raw[1] = (int16_t)(Wire.read() << 8 | Wire.read());
    acc_raw[2] = (int16_t)(Wire.read() << 8 | Wire.read());
    temperature = (int16_t)(Wire.read() << 8 | Wire.read());
    gyro_raw[0] = (int16_t)(Wire.read() << 8 | Wire.read());
    gyro_raw[1] = (int16_t)(Wire.read() << 8 | Wire.read());
    gyro_raw[2] = (int16_t)(Wire.read() << 8 | Wire.read());

    // Libérer le mutex après la lecture I2C
    if (i2c_mutex != nullptr) xSemaphoreGive(i2c_mutex);
    // ========== FIN SECTION I2C PROTEGEE ==========

    double gyro_x_cal = gyro_raw[0] - gyro_off_x;
    double gyro_y_cal = gyro_raw[1] - gyro_off_y;
    double gyro_z_cal = gyro_raw[2] - gyro_off_z;

    // Mapping axes
    double gyro_roll  = gyro_x_cal;
    long acc_roll_val = acc_raw[1];

    double gyro_pitch = -gyro_y_cal;
    long acc_pitch_val = acc_raw[0];

    // CORRECTION: Inverser le signe du gyro Yaw
       // YAW : selon l'orientation de la carte, il peut être nécessaire d'inverser le signe.
    #if IMU_INVERT_YAW
        double gyro_yaw = -gyro_z_cal;
    #else
        double gyro_yaw = gyro_z_cal;
    #endif
        long acc_yaw_val  = acc_raw[2];


    const float gyro_scale = 65.5f;

    // Gyro en degrés/seconde
    float gyro_roll_dps  = (float)(gyro_roll / gyro_scale);
    float gyro_pitch_dps = (float)(gyro_pitch / gyro_scale);
    float gyro_yaw_dps   = (float)(gyro_yaw / gyro_scale);

    // Filtre PT1 pour les entrées PID
    gyro_roll_filt  += GYRO_PT1_COEFF * (gyro_roll_dps  - gyro_roll_filt);
    gyro_pitch_filt += GYRO_PT1_COEFF * (gyro_pitch_dps - gyro_pitch_filt);
    gyro_yaw_filt   += GYRO_PT1_COEFF * (gyro_yaw_dps   - gyro_yaw_filt);

    drone->gyro_roll_input  = gyro_roll_filt;
    drone->gyro_pitch_input = gyro_pitch_filt;
    drone->gyro_yaw_input   = gyro_yaw_filt;

    // Calcul du vecteur accélération total
    drone->acc_total_vector = sqrtf((float)(acc_roll_val * acc_roll_val) +
                                    (float)(acc_pitch_val * acc_pitch_val) +
                                    (float)(acc_yaw_val * acc_yaw_val));

    // Accélération normalisée en G (1G = 4096 LSB pour ±8g)
    const float ACC_SCALE = 4096.0f;
    drone->acc_x = (float)acc_pitch_val / ACC_SCALE;  // X = acc_pitch
    drone->acc_y = (float)acc_roll_val / ACC_SCALE;   // Y = acc_roll
    drone->acc_z = (float)acc_yaw_val / ACC_SCALE;    // Z = acc_z

    // Calcul angles accéléromètre
    float angle_pitch_acc = 0.0f, angle_roll_acc = 0.0f;

    if (fabsf((float)acc_pitch_val) < drone->acc_total_vector) {
        angle_pitch_acc = asinf((float)acc_pitch_val / drone->acc_total_vector) * RAD_TO_DEG;
    }
    if (fabsf((float)acc_roll_val) < drone->acc_total_vector) {
        angle_roll_acc = asinf((float)acc_roll_val / drone->acc_total_vector) * RAD_TO_DEG;
    }

    // Trims supprimés de l'IMU : l'angle physique brut est nécessaire
    // pour la soustraction de gravité dans l'EKF. Les trims sont
    // maintenant appliqués dans pid_compute_setpoints() côté pilotage.

    // --- FILTRE DE KALMAN ---
    if (!kalman_initialized) {
        kalman_roll.setAngle(angle_roll_acc);
        kalman_pitch.setAngle(angle_pitch_acc);
        drone->angle_roll = angle_roll_acc;
        drone->angle_pitch = angle_pitch_acc;
        kalman_initialized = true;
    } else {
        // Mode adaptatif
        if (drone->current_mode == MODE_SAFE && drone->channel_3 < 1050) {
            kalman_roll.setRmeasure(0.01f);
            kalman_pitch.setRmeasure(0.01f);
        } else {
            kalman_roll.setRmeasure(0.03f);
            kalman_pitch.setRmeasure(0.03f);
        }

        drone->angle_roll  = kalman_roll.update(angle_roll_acc, gyro_roll_dps, dt_s);
        drone->angle_pitch = kalman_pitch.update(angle_pitch_acc, gyro_pitch_dps, dt_s);
    }

    // Intégration Yaw gyro (référence interne uniquement, pour reset)
    // NE PAS écrire dans drone->angle_yaw : le yaw est géré
    // exclusivement par yaw_fusion.cpp (gyro + magnétomètre)
    yaw_angle += gyro_yaw_dps * dt_s;
    while (yaw_angle > 180.0f) yaw_angle -= 360.0f;
    while (yaw_angle < -180.0f) yaw_angle += 360.0f;

    // Compensation yaw - DESACTIVER POUR TEST
    #if 0  // <-- Mettre 1 pour réactiver
    const float yaw_rad = gyro_yaw_dps * dt_s * (PI / 180.0f);
    const float s = sinf(yaw_rad);

    const float roll0  = drone->angle_roll;
    const float pitch0 = drone->angle_pitch;

    drone->angle_pitch = pitch0 - roll0  * s;
    drone->angle_roll  = roll0  + pitch0 * s;
    #endif
}

// ==================== TASK IMU ====================
static void imu_task(void *parameter) {
    (void)parameter;

    memset(&imu_state, 0, sizeof(imu_state));
    imu_state.current_mode = MODE_SAFE;
    imu_state.channel_3 = 1000;

    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        FlightMode m;
        int ch3;
        bool do_reset = false;

        portENTER_CRITICAL(&imu_mux);
        m = imu_in_mode;
        ch3 = imu_in_ch3;
        if (imu_reset_req) { imu_reset_req = false; do_reset = true; }
        imu_state.trim_roll  = imu_in_trim_roll;
        imu_state.trim_pitch = imu_in_trim_pitch;
        portEXIT_CRITICAL(&imu_mux);

        imu_state.current_mode = m;
        imu_state.channel_3 = ch3;

        if (do_reset) {
            imu_state.angle_pitch = 0.0f;
            imu_state.angle_roll  = 0.0f;
            imu_state.gyro_roll_input = 0.0f;
            imu_state.gyro_pitch_input = 0.0f;
            imu_state.gyro_yaw_input = 0.0f;
            
            kalman_roll.setAngle(0.0f);
            kalman_pitch.setAngle(0.0f);
            kalman_initialized = false;
            yaw_angle = 0.0f;  // AJOUT: reset angle yaw
            
            gyro_roll_filt = 0.0f;
            gyro_pitch_filt = 0.0f;
            gyro_yaw_filt = 0.0f;
        }

        unsigned long t0 = micros();
        imu_read_internal(&imu_state);
        unsigned long dur = micros() - t0;

        bool ok = (imu_state.max_time_imu != 888888);

        portENTER_CRITICAL(&imu_mux);
        imu_snap.gyro_roll_input  = imu_state.gyro_roll_input;
        imu_snap.gyro_pitch_input = imu_state.gyro_pitch_input;
        imu_snap.gyro_yaw_input   = imu_state.gyro_yaw_input;
        imu_snap.angle_roll       = imu_state.angle_roll;
        imu_snap.angle_pitch      = imu_state.angle_pitch;
        // angle_yaw n'est plus copié ici : géré par yaw_fusion exclusivement
        imu_snap.acc_total_vector = imu_state.acc_total_vector;
        imu_snap.acc_x            = imu_state.acc_x;
        imu_snap.acc_y            = imu_state.acc_y;
        imu_snap.acc_z            = imu_state.acc_z;
        imu_snap.last_dur_us      = dur;
        imu_snap.ok               = ok;
        if (ok) imu_snap.last_ok_ms = millis();
        portEXIT_CRITICAL(&imu_mux);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(4)); // 250 Hz
    }
}

// ==================== API PUBLIQUE ====================
void imu_start_task() {
    if (imu_task_handle != nullptr) {
        Serial.println(F("IMU: Task already running!"));
        return;
    }

    // Créer le mutex I2C s'il n'existe pas encore
    if (i2c_mutex == nullptr) {
        i2c_mutex = xSemaphoreCreateMutex();
        if (i2c_mutex == nullptr) {
            Serial.println(F("IMU: ERREUR - Impossible de créer le mutex I2C!"));
            return;
        }
        Serial.println(F("IMU: Mutex I2C créé"));
    }

    Serial.println(F("IMU: Starting FreeRTOS task..."));

    xTaskCreatePinnedToCore(
        imu_task,
        "imu_i2c",
        4096,
        nullptr,
        4,
        &imu_task_handle,
        0
    );
    
    Serial.println(F("IMU: Task started successfully"));
}

void imu_update(DroneState *drone) {
    static FlightMode last_mode = MODE_SAFE;

    ImuSnapshot s;

    portENTER_CRITICAL(&imu_mux);
    imu_in_mode = drone->current_mode;
    imu_in_ch3  = drone->channel_3;
    imu_in_trim_roll  = drone->trim_roll;
    imu_in_trim_pitch = drone->trim_pitch;
    s = imu_snap;
    portEXIT_CRITICAL(&imu_mux);

    if ((drone->current_mode != last_mode) &&
        (drone->current_mode == MODE_SAFE || drone->current_mode == MODE_ARMED)) {
        imu_request_reset();
    }
    last_mode = drone->current_mode;

    drone->gyro_roll_input  = s.gyro_roll_input;
    drone->gyro_pitch_input = s.gyro_pitch_input;
    drone->gyro_yaw_input   = s.gyro_yaw_input;
    drone->angle_roll       = s.angle_roll;
    drone->angle_pitch      = s.angle_pitch;
    // drone->angle_yaw n'est plus écrit ici : géré par yaw_fusion exclusivement
    drone->acc_total_vector = s.acc_total_vector;
    drone->acc_x            = s.acc_x;
    drone->acc_y            = s.acc_y;
    drone->acc_z            = s.acc_z;
    drone->current_time_imu = s.last_dur_us;
}

void imu_request_reset() {
    portENTER_CRITICAL(&imu_mux);
    imu_reset_req = true;
    portEXIT_CRITICAL(&imu_mux);
}

void imu_read(DroneState *drone) {
    imu_read_internal(drone);
}