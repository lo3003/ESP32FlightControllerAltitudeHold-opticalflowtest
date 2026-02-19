#include <Arduino.h>
#include "optical_flow.h"
#include "config.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// --- MSPv2 Function IDs ---
#define MSP2_SENSOR_OPTIC_FLOW  0x1F02
#define MSP2_SENSOR_RANGEFINDER 0x1F01

// --- MSPv2 Parser States ---
enum MspState {
    MSP_IDLE,
    MSP_HEADER_X,
    MSP_HEADER_DIR,
    MSP_FLAG,
    MSP_FUNC_LO,
    MSP_FUNC_HI,
    MSP_SIZE_LO,
    MSP_SIZE_HI,
    MSP_PAYLOAD,
    MSP_CRC
};

// --- Données partagées (protégées par mutex) ---
static portMUX_TYPE flow_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile float flow_vel_x     = 0.0f;
static volatile float flow_vel_y     = 0.0f;
static volatile int   flow_qual      = 0;
static volatile float flow_raw_dx    = 0.0f;
static volatile float flow_raw_dy    = 0.0f;
static volatile bool  flow_new_data  = false;
static volatile float flow_pos_x     = 0.0f;  // Position intégrée X (cm)
static volatile float flow_pos_y     = 0.0f;  // Position intégrée Y (cm)

// Flag de reset position (set depuis main loop, lu par flow_task)
static volatile bool flow_reset_pos  = false;

static TaskHandle_t flow_task_handle = nullptr;

// --- Scale factor runtime-modifiable ---
static volatile float flow_scale_runtime = FLOW_SCALE_FACTOR;

// --- Variables volatiles pour recevoir les angles depuis la boucle principale ---
volatile float _flow_angle_yaw    = 0.0f;
volatile float _flow_lidar_dist   = 0.0f;

// --- Variables volatiles pour compensation gyroscopique ---
volatile float _flow_gyro_pitch   = 0.0f;  // deg/s
volatile float _flow_gyro_roll    = 0.0f;  // deg/s

// --- Variables volatiles pour compensation inclinaison ---
volatile float _flow_angle_roll   = 0.0f;  // deg
volatile float _flow_angle_pitch  = 0.0f;  // deg

// --- CRC8 DVB-S2 ---
static uint8_t crc8_dvb_s2(uint8_t crc, uint8_t byte) {
    crc ^= byte;
    for (int i = 0; i < 8; i++) {
        if (crc & 0x80) crc = (crc << 1) ^ 0xD5;
        else crc = crc << 1;
    }
    return crc;
}

// --- Tâche FreeRTOS — tourne sur Core 0 à ~100 Hz ---
static void flow_task(void *parameter) {
    (void)parameter;

    // Parser state
    MspState state = MSP_IDLE;
    uint8_t  msp_flag = 0;
    uint16_t msp_func = 0;
    uint16_t msp_size = 0;
    uint16_t msp_idx  = 0;
    uint8_t  msp_crc  = 0;
    uint8_t  msp_payload[16];  // Optic flow payload = 9 bytes max

    // Filtered velocities
    float filt_vel_x = 0.0f;
    float filt_vel_y = 0.0f;

    // Position intégrée (accumulateurs locaux)
    float pos_x_int = 0.0f;
    float pos_y_int = 0.0f;

    // Timing
    unsigned long last_frame_us = micros();

    const float ALPHA = 0.5f;  // Filtre passe-bas coefficient

    TickType_t xLastWake = xTaskGetTickCount();

    for (;;) {
        // Vérifier demande de reset position
        if (flow_reset_pos) {
            pos_x_int = 0.0f;
            pos_y_int = 0.0f;
            filt_vel_x = 0.0f;
            filt_vel_y = 0.0f;
            flow_reset_pos = false;
        }

        // Lire tous les bytes disponibles sur Serial1
        while (Serial1.available()) {
            uint8_t c = Serial1.read();

            switch (state) {
                case MSP_IDLE:
                    if (c == '$') state = MSP_HEADER_X;
                    break;

                case MSP_HEADER_X:
                    state = (c == 'X') ? MSP_HEADER_DIR : MSP_IDLE;
                    break;

                case MSP_HEADER_DIR:
                    if (c == '<' || c == '>') {
                        state = MSP_FLAG;
                    } else {
                        state = MSP_IDLE;
                    }
                    break;

                case MSP_FLAG:
                    msp_flag = c;
                    msp_crc = 0;
                    msp_crc = crc8_dvb_s2(msp_crc, c);
                    state = MSP_FUNC_LO;
                    break;

                case MSP_FUNC_LO:
                    msp_func = c;
                    msp_crc = crc8_dvb_s2(msp_crc, c);
                    state = MSP_FUNC_HI;
                    break;

                case MSP_FUNC_HI:
                    msp_func |= ((uint16_t)c << 8);
                    msp_crc = crc8_dvb_s2(msp_crc, c);
                    state = MSP_SIZE_LO;
                    break;

                case MSP_SIZE_LO:
                    msp_size = c;
                    msp_crc = crc8_dvb_s2(msp_crc, c);
                    state = MSP_SIZE_HI;
                    break;

                case MSP_SIZE_HI:
                    msp_size |= ((uint16_t)c << 8);
                    msp_crc = crc8_dvb_s2(msp_crc, c);
                    msp_idx = 0;
                    if (msp_size > 0 && msp_size <= sizeof(msp_payload)) {
                        state = MSP_PAYLOAD;
                    } else if (msp_size == 0) {
                        state = MSP_CRC;
                    } else {
                        // Payload trop grand, on ignore
                        state = MSP_IDLE;
                    }
                    break;

                case MSP_PAYLOAD:
                    msp_payload[msp_idx++] = c;
                    msp_crc = crc8_dvb_s2(msp_crc, c);
                    if (msp_idx >= msp_size) {
                        state = MSP_CRC;
                    }
                    break;

                case MSP_CRC:
                    if (c == msp_crc && msp_func == MSP2_SENSOR_OPTIC_FLOW && msp_size == 9) {
                        // --- Trame OPTIC_FLOW valide ---
                        uint8_t quality = msp_payload[0];

                        int32_t motion_x = (int32_t)(
                            (uint32_t)msp_payload[1]       |
                            ((uint32_t)msp_payload[2] << 8)  |
                            ((uint32_t)msp_payload[3] << 16) |
                            ((uint32_t)msp_payload[4] << 24)
                        );

                        int32_t motion_y = (int32_t)(
                            (uint32_t)msp_payload[5]       |
                            ((uint32_t)msp_payload[6] << 8)  |
                            ((uint32_t)msp_payload[7] << 16) |
                            ((uint32_t)msp_payload[8] << 24)
                        );

                        // motion_x/y sont déjà des deltas instantanés (Matek 3901-L0X)
                        {
                            int32_t delta_x = motion_x;
                            int32_t delta_y = motion_y;

                            // Calculer dt
                            unsigned long now_us = micros();
                            float dt_s = (float)(now_us - last_frame_us) * 1e-6f;
                            last_frame_us = now_us;
                            if (dt_s < 0.001f) dt_s = 0.001f;  // Sécurité
                            if (dt_s > 0.5f) dt_s = 0.5f;

                            // Lire les valeurs partagées
                            float lidar_dist = _flow_lidar_dist;
                            float yaw_deg    = _flow_angle_yaw;

                            // === COMPENSATION TILT ===
                            // Le capteur voit : mouvement_réel + rotation_drone
                            // La rotation est un phénomène optique indépendant de la hauteur.
                            // gyro (deg/s) → rad/s, puis pixels = gyro_rad * dt * scale_factor
                            float gyro_roll_rad  = _flow_gyro_roll  * 0.01745329f;  // deg/s → rad/s
                            float gyro_pitch_rad = _flow_gyro_pitch * 0.01745329f;  // deg/s → rad/s

                            // Roll compense delta_x (axe latéral), Pitch compense delta_y (axe longitudinal)
                            float gyro_comp_x = gyro_roll_rad  * dt_s * flow_scale_runtime;
                            float gyro_comp_y = gyro_pitch_rad * dt_s * flow_scale_runtime;

                            // TODO: vérifier signe avec test tilt — si dérive augmente, inverser en -
                            float corrected_delta_x = (float)delta_x + gyro_comp_x;
                            float corrected_delta_y = (float)delta_y + gyro_comp_y;

                            // Convertir en vitesse body frame (cm/s)
                            // FLOW_SIGN_X/Y permettent d'inverser selon le montage
                            // (axes swappés : delta_y → vel_x, delta_x → vel_y)
                            float vel_x_body = FLOW_SIGN_X * (corrected_delta_y * lidar_dist) / (dt_s * flow_scale_runtime);
                            float vel_y_body = FLOW_SIGN_Y * (corrected_delta_x * lidar_dist) / (dt_s * flow_scale_runtime);
                            // Rejet de spike : vitesse > 200 cm/s physiquement impossible
                            if (fabsf(vel_x_body) > 200.0f) vel_x_body = 0.0f;
                            if (fabsf(vel_y_body) > 200.0f) vel_y_body = 0.0f;

                            // Rotation body → world frame (yaw)
                            /*float yaw_rad = yaw_deg * 0.01745329f;
                            float cos_yaw = cosf(yaw_rad);
                            float sin_yaw = sinf(yaw_rad);
                            float vel_x_world = vel_x_body * cos_yaw - vel_y_body * sin_yaw;
                            float vel_y_world = vel_x_body * sin_yaw + vel_y_body * cos_yaw;
                            */
                            // Filtre passe-bas
                            filt_vel_x = filt_vel_x + ALPHA * (vel_x_body - filt_vel_x);
                            filt_vel_y = filt_vel_y + ALPHA * (vel_y_body - filt_vel_y);

                            // Intégration de position (seulement si qualité suffisante)
                            /*if (quality >= FLOW_QUALITY_MIN) {
                                pos_x_int += filt_vel_x * dt_s;
                                pos_y_int += filt_vel_y * dt_s;
                            }
                            */
                            // Intégration de position (seulement si qualité suffisante)
                            if (quality >= FLOW_QUALITY_MIN) {
                                // Deadband vélocité : ne pas intégrer les petites vitesses résiduelles
                                float int_vx = (fabsf(filt_vel_x) > 2.0f) ? filt_vel_x : 0.0f;
                                float int_vy = (fabsf(filt_vel_y) > 2.0f) ? filt_vel_y : 0.0f;
                                pos_x_int += int_vx * dt_s;
                                pos_y_int += int_vy * dt_s;
                            }

                            // Publication thread-safe
                            portENTER_CRITICAL(&flow_mux);
                            flow_vel_x    = filt_vel_x;
                            flow_vel_y    = filt_vel_y;
                            flow_qual     = quality;
                            flow_raw_dx   = (float)delta_x;
                            flow_raw_dy   = (float)delta_y;
                            flow_new_data = true;
                            flow_pos_x    = pos_x_int;
                            flow_pos_y    = pos_y_int;
                            portEXIT_CRITICAL(&flow_mux);
                        }
                    }
                    // Ignorer MSP2_SENSOR_RANGEFINDER (0x1F01) — on utilise le TF-Luna
                    state = MSP_IDLE;
                    break;
            }
        }

        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(10));  // ~100 Hz
    }
}

// --- API publiques ---
void optical_flow_set_scale(float scale) {
    if (scale > 0.1f) flow_scale_runtime = scale;
}

void optical_flow_init() {
    Serial1.begin(FLOW_BAUD, SERIAL_8N1, PIN_FLOW_RX, PIN_FLOW_TX);
    Serial.println(F("[FLOW] Serial1 initialisé (MSPv2, 115200)"));
}

void optical_flow_start_task() {
    if (flow_task_handle != nullptr) return;

    xTaskCreatePinnedToCore(
        flow_task,
        "opt_flow",
        4096,
        nullptr,
        2,                 // Priorité moyenne
        &flow_task_handle,
        0                  // Core 0 (comme radio / IMU secondaire / lidar)
    );

    Serial.println(F("[FLOW] Tâche démarrée (Core 0, ~100 Hz)"));
}

void optical_flow_reset_position() {
    flow_reset_pos = true;
    // Le reset effectif se fait dans flow_task au prochain cycle
}

void optical_flow_update(DroneState *drone) {
    // Transmettre les valeurs actuelles pour la conversion body→world
    _flow_angle_yaw  = drone->angle_yaw;        // Yaw fusionné (gyro+mag, plus stable)
    _flow_lidar_dist = drone->lidar_distance;   // Altitude compensée (cm)

    _flow_gyro_roll  = drone->gyro_roll_input;   // deg/s
    _flow_gyro_pitch = drone->gyro_pitch_input;   // deg/s
    _flow_angle_roll  = drone->angle_roll;         // deg
    _flow_angle_pitch = drone->angle_pitch;        // deg

    // Copie thread-safe des données optical flow
    portENTER_CRITICAL(&flow_mux);
    drone->flow_velocity_x = flow_vel_x;
    drone->flow_velocity_y = flow_vel_y;
    drone->flow_quality    = flow_qual;
    drone->flow_raw_x      = flow_raw_dx;
    drone->flow_raw_y      = flow_raw_dy;
    drone->new_flow_data   = flow_new_data;
    flow_new_data          = false;
    // Position intégrée directement depuis l'OF (sans EKF)
    drone->pos_est_x       = flow_pos_x;
    drone->pos_est_y       = flow_pos_y;
    portEXIT_CRITICAL(&flow_mux);
}
