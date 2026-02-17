#include "pid.h"
#include "config.h"
#include <Arduino.h>

// --- TENSION DE REFERENCE POUR LA COMPENSATION (A ajuster selon ta batterie) ---
// 12.0f pour 3S (11.1V nominal, 12.6V max)
// 16.0f pour 4S (14.8V nominal, 16.8V max)
#define PID_REF_VOLTAGE 12.6f 

// --- FLAG POUR DESACTIVER LE HEADING HOLD ---
// Mettre à 1 pour activer, 0 pour désactiver (mode simple)
#define HEADING_HOLD_ENABLED 1

// --- MEMOIRES PID ---
// Note : pid_last_..._input stocke la dernière MESURE (Gyro) pour le D
static float pid_i_mem_roll,  pid_last_roll_input;
static float pid_i_mem_pitch, pid_last_pitch_input;
static float pid_i_mem_yaw,   pid_last_yaw_input;

static float pid_roll_d_filter_old;
static float pid_pitch_d_filter_old;
static float pid_yaw_d_filter_old;

// FeedForward : consigne "pilote" (rate) mémorisée depuis pid_compute_setpoints()
static float ff_sp_roll = 0.0f;
static float ff_sp_pitch = 0.0f;
static float ff_sp_yaw = 0.0f;

// --- HEADING HOLD (Maintien de Cap) ---
static float yaw_target_angle = 0.0f;   // Cap cible mémorisé
static bool yaw_heading_lock = false;   // Flag: lock de cap actif
static float last_valid_yaw = 0.0f;     // Dernier yaw valide mémorisé

// Paramètres Heading Hold
static const float YAW_DEADBAND = 20.0f;        // Deadband stick yaw (±20 autour de 1500)
static const float YAW_RATE_LIMIT = 200.0f;     // Limite max de la consigne rate générée (deg/s)
static const float YAW_JUMP_THRESHOLD = 30.0f;  // Seuil de détection de saut aberrant (deg)

// Timer de sécurité pour ne pas resetter le I sur une micro-coupure radio
static unsigned long pid_inflight_timer = 0;

// --- INITIALISATION DES PARAMETRES PID ---
// Appelé au démarrage pour charger des valeurs par défaut
void pid_init_params(DroneState *drone) {
    // ROLL/PITCH
    drone->p_pitch_roll = PID_P_ROLL;
    drone->i_pitch_roll = PID_I_ROLL;
    drone->d_pitch_roll = PID_D_ROLL;

    // YAW (rate)
    drone->p_yaw = PID_P_YAW;
    drone->i_yaw = PID_I_YAW;
    drone->d_yaw = PID_D_YAW;

    // FEEDFORWARD
    drone->ff_pitch_roll = 0;
    drone->ff_yaw        = 0;

    // AUTO LEVEL
    drone->p_level = 1.55f;

    // HEADING HOLD
    drone->p_heading = 1.1f;

    // ALTITUDE HOLD
    drone->p_alt = PID_P_ALT;
    drone->i_alt = PID_I_ALT;
    drone->d_alt = PID_D_ALT;
    drone->hover_throttle = HOVER_THROTTLE;
    drone->alt_setpoint = 0.0f;
    drone->pid_output_alt = 0.0f;
    drone->alt_stick_offset = 0.0f;
    drone->throttle_base_ramp = MIN_THROTTLE_IDLE;
    drone->takeoff_ramp_active = false;
    drone->alt_smooth = ALT_SMOOTH_DEFAULT;
    drone->auto_takeoff_active = false;
    drone->throttle_command = 1000;

    // POSITION HOLD
    drone->p_pos = PID_P_POS;
    drone->i_pos = PID_I_POS;
    drone->d_pos = PID_D_POS;
    drone->pos_smooth = POS_SMOOTH_DEFAULT;
    drone->pos_est_x = 0; drone->pos_est_y = 0;
    drone->pos_setpoint_x = 0; drone->pos_setpoint_y = 0;
    drone->pid_output_pos_roll = 0; drone->pid_output_pos_pitch = 0;

    Serial.println("PID Params Initialized (Flight-Ready Mode + VBat Comp + AltHold)");;
}

void pid_init() {
    pid_reset_integral();
}

void pid_reset_integral() {
    pid_i_mem_roll = 0;  pid_last_roll_input = 0;  pid_roll_d_filter_old = 0;
    pid_i_mem_pitch = 0; pid_last_pitch_input = 0; pid_pitch_d_filter_old = 0;
    pid_i_mem_yaw = 0;   pid_last_yaw_input = 0;   pid_yaw_d_filter_old = 0;
    pid_inflight_timer = 0;

    ff_sp_roll = ff_sp_pitch = ff_sp_yaw = 0.0f;

    // Reset Heading Hold
    yaw_target_angle = 0.0f;
    yaw_heading_lock = false;
    last_valid_yaw = 0.0f;
}

// --- CALCUL DES CONSIGNES (Setpoints) ---
void pid_compute_setpoints(DroneState *drone) {
    // === POSITION HOLD INJECTION ===
    float pos_hold_roll_offset = 0.0f;
    float pos_hold_pitch_offset = 0.0f;
    if (drone->current_mode == MODE_AUTO_POS || drone->current_mode == MODE_POS_HOLD) {
        pos_hold_roll_offset  = drone->pid_output_pos_roll  * drone->p_level;
        pos_hold_pitch_offset = drone->pid_output_pos_pitch * drone->p_level;
    }

    // --- ROLL (stick) ---
    float stick_roll = 0.0f;
    if (drone->channel_1 > 1520)      stick_roll = (float)(drone->channel_1 - 1520);
    else if (drone->channel_1 < 1480) stick_roll = (float)(drone->channel_1 - 1480);

    // --- PITCH (stick) ---
    float stick_pitch = 0.0f;
    if (drone->channel_2 > 1520)      stick_pitch = (float)(drone->channel_2 - 1520);
    else if (drone->channel_2 < 1480) stick_pitch = (float)(drone->channel_2 - 1480);

    // Ajouter les offsets position hold
    stick_roll  += pos_hold_roll_offset;
    stick_pitch += pos_hold_pitch_offset;

    // FeedForward = uniquement la commande pilote (rate)
    ff_sp_roll = stick_roll / 3.0f;
    ff_sp_pitch = stick_pitch / 3.0f;

    // Self-level (outer loop)
    // Les trims sont ajoutés à l'angle pour compenser le déséquilibre mécanique
    // sans contaminer l'angle physique brut utilisé par l'EKF pour la gravité
    float input_roll = stick_roll - ((drone->angle_roll + drone->trim_roll) * drone->p_level);
    drone->pid_setpoint_roll = input_roll / 3.0f;

    float input_pitch = stick_pitch - ((drone->angle_pitch + drone->trim_pitch) * drone->p_level);
    drone->pid_setpoint_pitch = input_pitch / 3.0f;

    // --- YAW ---
#if HEADING_HOLD_ENABLED
    float raw_stick_yaw = 0.0f;
    if (drone->channel_3 > 1050) {
        raw_stick_yaw = (float)(drone->channel_4 - 1500);
    }

#if RC_INVERT_YAW
    raw_stick_yaw = -raw_stick_yaw;
#endif

    bool stick_in_deadband = (fabsf(raw_stick_yaw) < YAW_DEADBAND);

    if (stick_in_deadband) {
        // --- Heading Hold ---
        float current_yaw = drone->angle_yaw;

        float yaw_delta = current_yaw - last_valid_yaw;
        while (yaw_delta > 180.0f)  yaw_delta -= 360.0f;
        while (yaw_delta < -180.0f) yaw_delta += 360.0f;

        if (fabsf(yaw_delta) > YAW_JUMP_THRESHOLD && yaw_heading_lock) {
            current_yaw = last_valid_yaw;
        } else {
            last_valid_yaw = current_yaw;
        }

        if (!yaw_heading_lock) {
            yaw_target_angle = current_yaw;
            last_valid_yaw = current_yaw;
            yaw_heading_lock = true;
        }

        float error_angle = yaw_target_angle - current_yaw;
        while (error_angle > 180.0f)  error_angle -= 360.0f;
        while (error_angle < -180.0f) error_angle += 360.0f;

        float rate_setpoint = error_angle * drone->p_heading;

        if (rate_setpoint > YAW_RATE_LIMIT) rate_setpoint = YAW_RATE_LIMIT;
        else if (rate_setpoint < -YAW_RATE_LIMIT) rate_setpoint = -YAW_RATE_LIMIT;

        drone->pid_setpoint_yaw = rate_setpoint;
        ff_sp_yaw = 0.0f;

    } else {
        // --- Pilot Control ---
        yaw_heading_lock = false;
        last_valid_yaw = drone->angle_yaw;

        float stick_yaw = 0.0f;
        if (drone->channel_4 > 1520)      stick_yaw = (float)(drone->channel_4 - 1520);
        else if (drone->channel_4 < 1480) stick_yaw = (float)(drone->channel_4 - 1480);

#if RC_INVERT_YAW
        stick_yaw = -stick_yaw;
#endif

        ff_sp_yaw = stick_yaw;
        drone->pid_setpoint_yaw = stick_yaw;
    }

#else
    // --- YAW SIMPLE ---
    float stick_yaw = 0.0f;
    if (drone->channel_3 > 1050) {
        if (drone->channel_4 > 1520)      stick_yaw = (float)(drone->channel_4 - 1520);
        else if (drone->channel_4 < 1480) stick_yaw = (float)(drone->channel_4 - 1480);
    }

#if RC_INVERT_YAW
    stick_yaw = -stick_yaw;
#endif

    ff_sp_yaw = stick_yaw;
    drone->pid_setpoint_yaw = stick_yaw;
#endif
}

// --- BOUCLE PID PRINCIPALE (Avec V-Bat Compensation) ---
void pid_compute(DroneState *drone) {
    float d_err_raw, d_err_filtered;

    // 1) DETECTION "IN FLIGHT"
    if (drone->channel_3 > 1350) {
        pid_inflight_timer = millis();
    }
    bool in_flight = (millis() - pid_inflight_timer < 500);

    // 2) TPA (Throtte PID Attenuation)
    // Réduit les gains quand on met plein gaz pour éviter les oscillations
    float tpa_factor = 1.0f;
    if (drone->channel_3 > 1500) {
        // Décommenter la ligne suivante pour activer le TPA
        tpa_factor = map(drone->channel_3, 1500, 2000, 100, 90) / 100.0f;
    }

    // 3) COMPENSATION TENSION BATTERIE (V-Bat)
    // Augmente les gains quand la batterie faiblit (les moteurs ont moins de couple)
    float vbat_compensation = 1.0f;
    if (drone->voltage_bat > 6.0f) { // Sécurité : on ne compense que si lecture > 6V
        vbat_compensation = PID_REF_VOLTAGE / drone->voltage_bat;
        
        // Bornes de sécurité (±30%)
        if (vbat_compensation > 1.3f) vbat_compensation = 1.3f;
        if (vbat_compensation < 0.7f) vbat_compensation = 0.7f;
    }

    // SCALER GLOBAL : Combine TPA et V-Bat
    float pid_scaler = tpa_factor * vbat_compensation;


    // ---------------- ROLL ----------------
    float error = drone->gyro_roll_input - drone->pid_setpoint_roll;

    float p_term_roll = (drone->p_pitch_roll * pid_scaler) * error;

    d_err_raw = pid_last_roll_input - drone->gyro_roll_input;
    d_err_filtered = pid_roll_d_filter_old + D_FILTER_COEFF * (d_err_raw - pid_roll_d_filter_old);
    pid_roll_d_filter_old = d_err_filtered;
    pid_last_roll_input = drone->gyro_roll_input;
    
    float d_term_roll = (drone->d_pitch_roll * pid_scaler) * d_err_filtered;

    if (in_flight) {
        float output_before_i = p_term_roll + pid_i_mem_roll + d_term_roll;
        if (fabsf(output_before_i) < PID_MAX_ROLL) {
            pid_i_mem_roll += drone->i_pitch_roll * error;
        }
        if (pid_i_mem_roll > PID_MAX_ROLL) pid_i_mem_roll = PID_MAX_ROLL;
        else if (pid_i_mem_roll < -PID_MAX_ROLL) pid_i_mem_roll = -PID_MAX_ROLL;
    } else {
        pid_i_mem_roll = 0;
    }

    float ff_term_roll = (drone->ff_pitch_roll * pid_scaler) * ff_sp_roll;

    drone->pid_output_roll = p_term_roll + pid_i_mem_roll + d_term_roll + ff_term_roll;
    if (drone->pid_output_roll > PID_MAX_ROLL) drone->pid_output_roll = PID_MAX_ROLL;
    else if (drone->pid_output_roll < -PID_MAX_ROLL) drone->pid_output_roll = -PID_MAX_ROLL;

    // ---------------- PITCH ----------------
    error = drone->gyro_pitch_input - drone->pid_setpoint_pitch;

    float p_term_pitch = (drone->p_pitch_roll * pid_scaler) * error;

    d_err_raw = pid_last_pitch_input - drone->gyro_pitch_input;
    d_err_filtered = pid_pitch_d_filter_old + D_FILTER_COEFF * (d_err_raw - pid_pitch_d_filter_old);
    pid_pitch_d_filter_old = d_err_filtered;
    pid_last_pitch_input = drone->gyro_pitch_input;
    
    float d_term_pitch = (drone->d_pitch_roll * pid_scaler) * d_err_filtered;

    if (in_flight) {
        float output_before_i = p_term_pitch + pid_i_mem_pitch + d_term_pitch;
        if (fabsf(output_before_i) < PID_MAX_PITCH) {
            pid_i_mem_pitch += drone->i_pitch_roll * error;
        }
        if (pid_i_mem_pitch > PID_MAX_PITCH) pid_i_mem_pitch = PID_MAX_PITCH;
        else if (pid_i_mem_pitch < -PID_MAX_PITCH) pid_i_mem_pitch = -PID_MAX_PITCH;
    } else {
        pid_i_mem_pitch = 0;
    }

    float ff_term_pitch = (drone->ff_pitch_roll * pid_scaler) * ff_sp_pitch;

    drone->pid_output_pitch = p_term_pitch + pid_i_mem_pitch + d_term_pitch + ff_term_pitch;
    if (drone->pid_output_pitch > PID_MAX_PITCH) drone->pid_output_pitch = PID_MAX_PITCH;
    else if (drone->pid_output_pitch < -PID_MAX_PITCH) drone->pid_output_pitch = -PID_MAX_PITCH;

    // ---------------- YAW ----------------
    error = drone->gyro_yaw_input - drone->pid_setpoint_yaw;

    float p_term_yaw = (drone->p_yaw * pid_scaler) * error;

    d_err_raw = pid_last_yaw_input - drone->gyro_yaw_input;
    d_err_filtered = pid_yaw_d_filter_old + D_FILTER_COEFF * (d_err_raw - pid_yaw_d_filter_old);
    pid_yaw_d_filter_old = d_err_filtered;
    pid_last_yaw_input = drone->gyro_yaw_input;
    
    float d_term_yaw = (drone->d_yaw * pid_scaler) * d_err_filtered;

    if (in_flight) {
        float output_before_i = p_term_yaw + pid_i_mem_yaw + d_term_yaw;
        if (fabsf(output_before_i) < PID_MAX_YAW) {
            pid_i_mem_yaw += drone->i_yaw * error;
        }
        if (pid_i_mem_yaw > PID_MAX_YAW) pid_i_mem_yaw = PID_MAX_YAW;
        else if (pid_i_mem_yaw < -PID_MAX_YAW) pid_i_mem_yaw = -PID_MAX_YAW;
    } else {
        pid_i_mem_yaw = 0;
    }

    float ff_term_yaw = (drone->ff_yaw * pid_scaler) * ff_sp_yaw;

    drone->pid_output_yaw = p_term_yaw + pid_i_mem_yaw + d_term_yaw + ff_term_yaw;
    if (drone->pid_output_yaw > PID_MAX_YAW) drone->pid_output_yaw = PID_MAX_YAW;
    else if (drone->pid_output_yaw < -PID_MAX_YAW) drone->pid_output_yaw = -PID_MAX_YAW;
}

// ================================================================
//  PID ALTITUDE HOLD — Contrôle de Position Absolue
//  Le stick gaz mappe directement une altitude cible (15-200 cm).
//  Un filtre passe-bas lisse la consigne pour des transitions douces.
// ================================================================
static float pid_i_mem_alt = 0.0f;

// --- MEMOIRES PID POSITION HOLD ---
static float pid_i_mem_pos_x = 0.0f;
static float pid_i_mem_pos_y = 0.0f;

void pid_reset_position() {
    pid_i_mem_pos_x = 0.0f;
    pid_i_mem_pos_y = 0.0f;
}

void pid_compute_position(DroneState *drone) {
    // Stick override : si le pilote pousse un stick, on relache le hold
    bool stick_active = (abs(drone->channel_1 - 1500) > POS_STICK_DEADBAND) ||
                        (abs(drone->channel_2 - 1500) > POS_STICK_DEADBAND);

    if (stick_active) {
        // Le pilote controle -> le setpoint suit la position estimee courante
        drone->pos_setpoint_x = drone->pos_est_x;
        drone->pos_setpoint_y = drone->pos_est_y;
        pid_i_mem_pos_x = 0.0f;
        pid_i_mem_pos_y = 0.0f;
        drone->pid_output_pos_pitch = 0.0f;
        drone->pid_output_pos_roll  = 0.0f;
        return;
    }

    // Qualité insuffisante → zéro les sorties, pas de correction
    if (drone->flow_quality < FLOW_QUALITY_MIN) {
        pid_i_mem_pos_x = 0.0f;
        pid_i_mem_pos_y = 0.0f;
        drone->pid_output_pos_pitch = 0.0f;
        drone->pid_output_pos_roll  = 0.0f;
        return;
    }

    // Position et vélocité directement depuis l'optical flow (sans EKF)
    // pos_est_x/y est rempli par optical_flow_update()
    // flow_velocity_x/y est la vélocité world filtrée

    // Erreur de position
    float err_x = drone->pos_setpoint_x - drone->pos_est_x;
    float err_y = drone->pos_setpoint_y - drone->pos_est_y;

    // Deadband : ignorer les erreurs < 2 cm (bruit)
    if (fabsf(err_x) < 2.0f) err_x = 0.0f;
    if (fabsf(err_y) < 2.0f) err_y = 0.0f;

    // Terme P
    float px = drone->p_pos * err_x;
    float py = drone->p_pos * err_y;

    // Terme I (avec anti-windup)
    pid_i_mem_pos_x += drone->i_pos * err_x;
    pid_i_mem_pos_y += drone->i_pos * err_y;
    if (pid_i_mem_pos_x > PID_MAX_POS_ANGLE) pid_i_mem_pos_x = PID_MAX_POS_ANGLE;
    else if (pid_i_mem_pos_x < -PID_MAX_POS_ANGLE) pid_i_mem_pos_x = -PID_MAX_POS_ANGLE;
    if (pid_i_mem_pos_y > PID_MAX_POS_ANGLE) pid_i_mem_pos_y = PID_MAX_POS_ANGLE;
    else if (pid_i_mem_pos_y < -PID_MAX_POS_ANGLE) pid_i_mem_pos_y = -PID_MAX_POS_ANGLE;

    // Terme D (D on measurement — utilise flow_velocity directement)
    float vx = drone->flow_velocity_x;
    float vy = drone->flow_velocity_y;
    if (fabsf(vx) < 1.0f) vx = 0.0f;
    if (fabsf(vy) < 1.0f) vy = 0.0f;
    float dx = -drone->d_pos * (vx / 100.0f);
    float dy = -drone->d_pos * (vy / 100.0f);

    // Somme + saturation
    float out_x = px + pid_i_mem_pos_x + dx;
    float out_y = py + pid_i_mem_pos_y + dy;

    if (out_x > PID_MAX_POS_ANGLE) out_x = PID_MAX_POS_ANGLE;
    else if (out_x < -PID_MAX_POS_ANGLE) out_x = -PID_MAX_POS_ANGLE;
    if (out_y > PID_MAX_POS_ANGLE) out_y = PID_MAX_POS_ANGLE;
    else if (out_y < -PID_MAX_POS_ANGLE) out_y = -PID_MAX_POS_ANGLE;

    // Affecter (derive +X -> corriger -pitch, derive +Y -> corriger -roll)
    drone->pid_output_pos_pitch = -out_x;
    drone->pid_output_pos_roll  = -out_y;
}

void pid_reset_altitude_integral() {
    pid_i_mem_alt = 0.0f;
}

void pid_preload_altitude_integral(float preload_value) {
    pid_i_mem_alt = preload_value;
    if (pid_i_mem_alt > (float)PID_MAX_ALT) pid_i_mem_alt = (float)PID_MAX_ALT;
    else if (pid_i_mem_alt < -(float)PID_MAX_ALT) pid_i_mem_alt = -(float)PID_MAX_ALT;
}

void pid_compute_altitude(DroneState *drone) {
    int ch3 = drone->channel_3;

    // --- SECURITE : stick très bas → idle + reset ---
    if (ch3 < ALT_HOLD_KILL_THROTTLE) {
        pid_i_mem_alt = 0.0f;
        drone->pid_output_alt = 0.0f;
        drone->throttle_command = MIN_THROTTLE_IDLE;
        drone->alt_setpoint = drone->lidar_distance;
        return;
    }

    // --- MAPPING DIRECT : position stick → altitude cible ---
    // ch3 [1150..2000] → [ALT_MIN_HEIGHT..ALT_MAX_HEIGHT]
    float target;
    if (ch3 >= 2000) {
        target = ALT_MAX_HEIGHT;
    } else if (ch3 <= ALT_HOLD_KILL_THROTTLE) {
        target = ALT_MIN_HEIGHT;
    } else {
        target = ALT_MIN_HEIGHT + (ALT_MAX_HEIGHT - ALT_MIN_HEIGHT)
                 * ((float)(ch3 - ALT_HOLD_KILL_THROTTLE) / (2000.0f - (float)ALT_HOLD_KILL_THROTTLE));
    }

    target += drone->alt_stick_offset;
    if (target < ALT_MIN_HEIGHT) target = ALT_MIN_HEIGHT;
    if (target > ALT_MAX_HEIGHT) target = ALT_MAX_HEIGHT;

    // Lissage de la consigne (filtre passe-bas)
    // Empêche les sauts brutaux si le pilote bouge le stick vite
    const float SMOOTH = drone->alt_smooth;
    drone->alt_setpoint = drone->alt_setpoint * (1.0f - SMOOTH) + target * SMOOTH;

    // --- PID POSITION ---
    float error = drone->alt_setpoint - drone->lidar_distance;

    float p_term = drone->p_alt * error;

    pid_i_mem_alt += drone->i_alt * error;
    if (pid_i_mem_alt > (float)PID_MAX_ALT) pid_i_mem_alt = (float)PID_MAX_ALT;
    else if (pid_i_mem_alt < -(float)PID_MAX_ALT) pid_i_mem_alt = -(float)PID_MAX_ALT;

    float d_term = -drone->d_alt * drone->lidar_velocity;

    float output = p_term + pid_i_mem_alt + d_term;
    if (output > (float)PID_MAX_ALT) output = (float)PID_MAX_ALT;
    else if (output < -(float)PID_MAX_ALT) output = -(float)PID_MAX_ALT;

    drone->pid_output_alt = output;

    // --- GESTION RAMPE DE DECOLLAGE ---
    int base_throttle;
    if (drone->takeoff_ramp_active) {
        // Pendant la rampe : incrémenter progressivement le throttle de base
        drone->throttle_base_ramp += ALT_THROTTLE_RAMP_RATE;
        if (drone->throttle_base_ramp >= drone->hover_throttle) {
            drone->throttle_base_ramp = drone->hover_throttle;
            drone->takeoff_ramp_active = false;

            // Rampe terminée : calculer l'offset bumpless maintenant
            float stick_maps_to = ALT_MIN_HEIGHT + (ALT_MAX_HEIGHT - ALT_MIN_HEIGHT)
                                  * ((float)(ch3 - ALT_HOLD_KILL_THROTTLE)
                                  / (2000.0f - (float)ALT_HOLD_KILL_THROTTLE));
            if (stick_maps_to < ALT_MIN_HEIGHT) stick_maps_to = ALT_MIN_HEIGHT;
            if (stick_maps_to > ALT_MAX_HEIGHT) stick_maps_to = ALT_MAX_HEIGHT;
            drone->alt_stick_offset = drone->lidar_distance - stick_maps_to;
        }

        // Pendant la rampe : forcer le setpoint à l'altitude courante
        // (on veut juste décoller, pas encore naviguer vers la cible)
        drone->alt_setpoint = drone->lidar_distance;
        base_throttle = drone->throttle_base_ramp;
    } else {
        base_throttle = drone->hover_throttle;
    }

    // --- COMMANDE GAZ FINALE ---
    int throttle = base_throttle + (int)output;
    if (throttle < MIN_THROTTLE_IDLE) throttle = MIN_THROTTLE_IDLE;
    if (throttle > MAX_THROTTLE_FLIGHT) throttle = MAX_THROTTLE_FLIGHT;

    drone->throttle_command = throttle;
}