#include <Arduino.h>
#include <Wire.h>
#include "lidar.h"
#include "config.h"
#include "imu.h"   // pour i2c_mutex

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// --- Données partagées (protégées par mutex) ---
static portMUX_TYPE lidar_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile float lidar_dist_cm    = 0.0f;   // Distance brute (cm)
static volatile float lidar_alt_cm     = 0.0f;   // Altitude compensée inclinaison (cm)
static volatile float lidar_vz_cms     = 0.0f;   // Vitesse verticale (cm/s)
static volatile bool  lidar_valid      = false;

static TaskHandle_t lidar_task_handle = nullptr;

// ----------------------------------------------------------------
//  Lecture I2C du TF-Luna (registre 0x00-0x01 = distance en cm)
// ----------------------------------------------------------------
static bool tfluna_read_distance(uint16_t *dist_cm) {
    // Prend le mutex I2C partagé avec l'IMU
    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(5)) != pdTRUE) return false;

    Wire.beginTransmission(LIDAR_I2C_ADDR);
    Wire.write(0x00);   // Registre distance LSB
    uint8_t err = Wire.endTransmission(false);
    if (err != 0) {
        xSemaphoreGive(i2c_mutex);
        return false;
    }

    uint8_t got = Wire.requestFrom((uint8_t)LIDAR_I2C_ADDR, (uint8_t)2);
    xSemaphoreGive(i2c_mutex);

    if (got < 2) return false;

    uint8_t lo = Wire.read();
    uint8_t hi = Wire.read();
    *dist_cm = (uint16_t)hi << 8 | lo;
    return true;
}

// ----------------------------------------------------------------
//  Tâche FreeRTOS — tourne sur Core 0 à ~50 Hz
// ----------------------------------------------------------------
static void lidar_task(void *parameter) {
    (void)parameter;

    float prev_alt   = 0.0f;
    float filt_alt   = 0.0f;
    float filt_vz    = 0.0f;
    bool  first      = true;

    const float dt       = 0.02f;           // 50 Hz → 20 ms
    const float alpha_d  = 0.3f;            // Filtre passe-bas distance
    const float alpha_vz = 0.4f;            // Filtre passe-bas Vz

    TickType_t xLastWake = xTaskGetTickCount();

    for (;;) {
        uint16_t raw_cm = 0;
        bool ok = tfluna_read_distance(&raw_cm);

        if (ok && raw_cm < 1200) {           // TF-Luna range max ~12 m
            float dist = (float)raw_cm;

            // --- Compensation d'inclinaison (Tilt Compensation) ---
            // On lit angle_roll / angle_pitch depuis les variables globales
            // (pas besoin de mutex, ce sont des floats atomiques sur ESP32)
            float roll_rad  = lidar_dist_cm;  // placeholder — sera recalculé juste en dessous
            (void)roll_rad;

            // Récupérer angles en degrés (écrits par la boucle principale)
            // On n'a pas directement accès à drone->angle_*, mais la tâche
            // reçoit la distance brute; la compensation sera appliquée ci-dessous.
            // On utilise les derniers angles transmis via un petit buffer statique.
            extern volatile float _lidar_angle_roll;
            extern volatile float _lidar_angle_pitch;

            float roll_r  = _lidar_angle_roll  * 0.01745329f;  // deg → rad
            float pitch_r = _lidar_angle_pitch * 0.01745329f;

            float cos_tilt = cosf(roll_r) * cosf(pitch_r);
            if (cos_tilt < 0.5f) cos_tilt = 0.5f;             // Sécurité si > 60°

            float alt = dist * cos_tilt;

            // Filtre passe-bas sur l'altitude
            if (first) {
                filt_alt = alt;
                prev_alt = alt;
                first = false;
            } else {
                filt_alt = filt_alt + alpha_d * (alt - filt_alt);
            }

            // Vitesse verticale (dérivée filtrée)
            float vz_raw = (filt_alt - prev_alt) / dt;
            filt_vz = filt_vz + alpha_vz * (vz_raw - filt_vz);
            prev_alt = filt_alt;

            // Publication thread-safe
            portENTER_CRITICAL(&lidar_mux);
            lidar_dist_cm = dist;
            lidar_alt_cm  = filt_alt;
            lidar_vz_cms  = filt_vz;
            lidar_valid   = true;
            portEXIT_CRITICAL(&lidar_mux);
        }

        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(20));   // 50 Hz
    }
}

// ----------------------------------------------------------------
//  Variables d'angle partagées avec la boucle principale
// ----------------------------------------------------------------
volatile float _lidar_angle_roll  = 0.0f;
volatile float _lidar_angle_pitch = 0.0f;

// ----------------------------------------------------------------
//  API publiques
// ----------------------------------------------------------------
void lidar_init() {
    // Vérification de présence du TF-Luna sur le bus I2C
    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        Wire.beginTransmission(LIDAR_I2C_ADDR);
        uint8_t err = Wire.endTransmission();
        xSemaphoreGive(i2c_mutex);

        if (err == 0) {
            Serial.println(F("[LIDAR] TF-Luna détecté sur I2C"));
        } else {
            Serial.printf("[LIDAR] TF-Luna NON détecté (err=%d) — Altitude Hold désactivé\n", err);
        }
    }

    xTaskCreatePinnedToCore(
        lidar_task,
        "lidar",
        4096,
        nullptr,
        2,                 // Priorité moyenne
        &lidar_task_handle,
        0                  // Core 0 (comme radio / IMU secondaire)
    );

    Serial.println(F("[LIDAR] Tâche démarrée (Core 0, 50 Hz)"));
}

void lidar_update(DroneState *drone) {
    // Transmettre les angles actuels pour la compensation d'inclinaison
    _lidar_angle_roll  = drone->angle_roll;
    _lidar_angle_pitch = drone->angle_pitch;

    // Copie thread-safe des données LiDAR
    portENTER_CRITICAL(&lidar_mux);
    drone->lidar_distance = lidar_alt_cm;
    drone->lidar_velocity = lidar_vz_cms;
    portEXIT_CRITICAL(&lidar_mux);
}
