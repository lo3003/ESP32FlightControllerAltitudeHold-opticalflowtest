#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "types.h"
#include "radio.h"
#include "imu.h"
#include "alt_imu.h"
#include "yaw_fusion.h"
#include "pid.h"
#include "motors.h"
#include "esc_calibrate.h"
#include "telemetry.h"
#include "lidar.h"
#include "optical_flow.h"
#include "position_ekf.h"

// --- FLAG POUR DESACTIVER LA FUSION YAW ---
// Mettre à 1 pour activer, 0 pour désactiver
#define YAW_FUSION_ENABLED 1

DroneState drone;
unsigned long loop_timer;
unsigned long arming_timer = 0;
unsigned long disarm_debounce_timer = 0; // Chrono pour la coupure Radio
unsigned long angle_security_timer = 0;  // Chrono pour l'angle excessif
int error_code = 0;                      // 0=OK, 1=CRASH ANGLE, 2=PERTE RADIO

void setup() {
    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(I2C_SPEED);
    Wire.setTimeOut(1);

    pinMode(PIN_LED, OUTPUT);
    pinMode(PIN_BATTERY, INPUT);
    analogReadResolution(12);

    // --- CORRECTION ESC : ON N'INITIALISE PAS LES MOTEURS TOUT DE SUITE ---
    // Si on lance motors_init() ici, on envoie 1000us pendant les 30s de calibration.
    // Cela provoque la mise en sécurité de certains ESC.
    
    // À la place, on force les pins à 0V (LOW). Les ESC vont biper "Signal Lost", mais ne se verrouilleront pas.
    // NOTE : Vérifie que ces pins correspondent bien à tes defines dans config.h (souvent 12, 13, 14, 15 sur ESP32)
    pinMode(12, OUTPUT); digitalWrite(12, LOW);
    pinMode(13, OUTPUT); digitalWrite(13, LOW);
    pinMode(14, OUTPUT); digitalWrite(14, LOW);
    pinMode(15, OUTPUT); digitalWrite(15, LOW);

    radio_init();
    radio_start_task(); // Radio indépendante de la loop()

    // On ne lance PAS motors_write_direct(2000...) ici. On attend de connaître le mode.

    // 3. DEMARRAGE TÂCHE TELEMETRIE (WIFI)
    start_telemetry_task(&drone); 

    // On attend un signal valide. Pendant ce temps, les ESC bipent (Signal Lost) ou attendent.
    unsigned long wait_start = millis();
    while(drone.channel_3 < 900) {
        radio_update(&drone);
        // Clignotement rapide
        if((millis() / 50) % 2) digitalWrite(PIN_LED, HIGH); else digitalWrite(PIN_LED, LOW);
        delay(5);
        if(millis() - wait_start > 15000) break; // Sécurité 15s
    }

    // 4. DECISION SELON LE STICK
    if(drone.channel_3 > 1900) {
        // ================= MODE CALIBRATION ESC =================
        // L'utilisateur veut calibrer : ON INITIALISE LES MOTEURS MAINTENANT
        motors_init(); 
        // On envoie direct le MAX throttle pour entrer en mode prog ESC
        motors_write_direct(2000, 2000, 2000, 2000);

        drone.current_mode = MODE_CALIBRATION;
        esc_calibrate_init();

    } else {
        // ================= MODE VOL NORMAL (SAFE) =================
        // On reste silencieux vers les moteurs (LOW) pour ne pas déclencher le timeout ESC
        
        drone.current_mode = MODE_SAFE;
        drone.new_flow_data = false;

        // ========== PHASE 1 : CALIBRATION GYRO/ACCEL ==========
        Serial.println(F(""));
        Serial.println(F("############################################"));
        Serial.println(F("# PHASE 1 - CALIBRATION GYRO/ACCEL        #"));
        Serial.println(F("# >>> NE PAS BOUGER LE DRONE <<<          #"));
        Serial.println(F("############################################"));

        imu_init();            // Calibration MPU6050 (5s)
        alt_imu_init();        // Calibration AltIMU (5s)

        // Pause visuelle
        digitalWrite(PIN_LED, LOW);
        delay(1000);

        // ========== PHASE 2 : CALIBRATION MAGNETOMETRE ==========
        Serial.println(F(""));
        Serial.println(F("############################################"));
        Serial.println(F("# PHASE 2 - CALIBRATION MAGNETOMETRE      #"));
        Serial.println(F("# >>> TOURNER LE DRONE DANS TOUS LES SENS #"));
        Serial.println(F("############################################"));

        alt_imu_calibrate_mag();  // C'est ici que ça bloquait 20s

        // Fin de calibration - LED fixe 1 seconde
        Serial.println(F(""));
        Serial.println(F("############################################"));
        Serial.println(F("# CALIBRATION TERMINEE                    #"));
        Serial.println(F("############################################"));
        digitalWrite(PIN_LED, HIGH);
        delay(1000);
        digitalWrite(PIN_LED, LOW);

        // --- C'EST MAINTENANT QU'ON REVEILLE LES ESC ---
        // La calibration longue est finie, on peut envoyer le signal PWM 1000us
        Serial.println(F("INITIALISATION MOTEURS..."));
        motors_init(); 
        // Les ESC vont maintenant faire leur musique de démarrage "123" + Bips LiPo
        
        // Démarrage des tâches FreeRTOS
        imu_start_task();      
        alt_imu_start_task();  

        // Initialisation fusion yaw
        yaw_fusion_init();

        // Initialisation LiDAR TF-Luna (Altitude Hold)
        lidar_init();

        // Initialisation Optical Flow (Position Hold)
        optical_flow_init();
        optical_flow_start_task();

        pid_init();
        pid_init_params(&drone);
        ekf_init();

        // Trims accéléromètre (valeurs par défaut)
        drone.trim_roll  = 0.8f;
        drone.trim_pitch = -3.7f;
    }

    // Initialisation des compteurs de diag
    drone.max_time_radio = 0;
    drone.max_time_imu = 0;
    drone.max_time_pid = 0;
    
    loop_timer = micros();
}

void loop() {
    unsigned long t_start = micros();

    // Lecture tension batterie (filtrée) - seulement 1 fois sur 25 (10Hz)
    static uint8_t bat_counter = 0;
    static float vbat_filter = 11.1f;
    if (++bat_counter >= 25) {
        bat_counter = 0;
        int raw = analogRead(PIN_BATTERY);
        float v_pin = (raw / 4095.0f) * 3.3f;
        float v_bat = v_pin * BAT_SCALE;
        vbat_filter = (vbat_filter * 0.95f) + (v_bat * 0.05f);
        drone.voltage_bat = vbat_filter;
    }

    radio_update(&drone);
    unsigned long t_radio = micros();

    // 2. Gestion LED Erreur
    if (error_code > 0) {
        if ((millis() % 50) < 25) digitalWrite(PIN_LED, HIGH);
        else digitalWrite(PIN_LED, LOW);
    } 
    else if (drone.current_mode == MODE_SAFE) {
        digitalWrite(PIN_LED, HIGH); // Fixe en SAFE
    }

    if(drone.current_mode == MODE_CALIBRATION) {
        esc_calibrate_loop(&drone);
    } else {
        // Au lieu de bloquer sur I2C ici:
        // imu_read(&drone);

        unsigned long t_imu_start = micros();
        imu_update(&drone);      // <-- snapshot non-bloquant
        alt_imu_update(&drone);  // <-- snapshot alt_imu non-bloquant
        lidar_update(&drone);    // <-- snapshot LiDAR non-bloquant
        optical_flow_update(&drone);  // <-- snapshot Optical Flow non-bloquant

        // Fusion Yaw (gyro + magnétomètre) - réduit à 50Hz pour économiser du CPU
#if YAW_FUSION_ENABLED
        static uint8_t fusion_counter = 0;
        if (drone.current_mode == MODE_ARMED || drone.current_mode == MODE_FLYING || drone.current_mode == MODE_ALT_HOLD || drone.current_mode == MODE_AUTO_POS) {
            if (++fusion_counter >= 5) {
                fusion_counter = 0;
                const float dt_s = LOOP_TIME_US * 5.0f * 1e-6f;  // 5 cycles = 20ms
                yaw_fusion_update(&drone, dt_s);
            }
        } else {
            fusion_counter = 0;
        }
#endif

        // EKF supprimé : la position est maintenant intégrée directement
        // dans la tâche optical flow (optical_flow.cpp) sans accéléromètre.
        // optical_flow_update() remplit drone.pos_est_x/y directement.

        unsigned long t_imu = micros();
        (void)t_imu_start;   // duree "loop" IMU n'a plus de sens; drone.current_time_imu vient de la task

        // Variable commune pour le mode test position (partagee entre SAFE et ARMED)
        static bool pos_test_mode = false;

        switch(drone.current_mode) {
            // ---------------- MODE SAFE ----------------
            case MODE_SAFE:
                motors_stop();

                // --- MODE TEST POSITION (sans armement, sans moteurs) (T) ---
                if (drone.channel_6 > 1500) {
                    if (!pos_test_mode) {
                        optical_flow_reset_position();
                        drone.pos_est_x = 0; drone.pos_est_y = 0;
                        drone.pos_setpoint_x = 0; drone.pos_setpoint_y = 0;
                        drone.pid_output_pos_roll = 0; drone.pid_output_pos_pitch = 0;
                        pid_reset_position();
                        pos_test_mode = true;
                        Serial.println(F("[TEST] Position test mode ON (SAFE)"));
                    }
                    // Executer le PID position (sans affecter les moteurs)
                    pid_compute_position(&drone);
                    // NE PAS appeler motors_mix/motors_write
                } else {
                    if (pos_test_mode) {
                        pos_test_mode = false;
                        drone.pid_output_pos_roll = 0;
                        drone.pid_output_pos_pitch = 0;
                        Serial.println(F("[TEST] Position test mode OFF"));
                    }
                }

                // ARMEMENTS : Gaz Bas + Yaw Gauche
                if(drone.channel_3 < 1010 && drone.channel_4 < 1200) {
                     if(arming_timer == 0) arming_timer = millis();
                     else if(millis() - arming_timer > 1000) {
                        drone.current_mode = MODE_ARMED;
                        arming_timer = 0;
                        error_code = 0; // Reset erreurs
                        pid_reset_integral();
                        drone.angle_pitch = 0;
                        drone.angle_roll = 0;
                        imu_request_reset();      // Reset IMU angles
#if YAW_FUSION_ENABLED
                        yaw_fusion_reset(&drone); // Reset fusion yaw avec magnétomètre
#endif
                        pos_test_mode = false;
                        loop_timer = micros();
                     }
                } else { arming_timer = 0; }
                break;

            // ---------------- MODE ARMED ----------------
            case MODE_ARMED:
                motors_stop(); 
                // Clignotement lent
                if ((millis() % 500) < 100) digitalWrite(PIN_LED, HIGH); else digitalWrite(PIN_LED, LOW);

                // Décollage classique (Gaz > 1040)
                if(drone.channel_3 > 1040) {
                    drone.current_mode = MODE_FLYING;
                    disarm_debounce_timer = 0;
                    angle_security_timer = 0;
                }

                // --- MODE TEST POSITION (sans moteurs) (T) ---
                if (drone.channel_6 > 1500) {
                    if (!pos_test_mode) {
                        optical_flow_reset_position();
                        drone.pos_est_x = 0; drone.pos_est_y = 0;
                        drone.pos_setpoint_x = 0; drone.pos_setpoint_y = 0;
                        drone.pid_output_pos_roll = 0; drone.pid_output_pos_pitch = 0;
                        pid_reset_position();
                        pos_test_mode = true;
                        Serial.println(F("[TEST] Position test mode ON (ARMED)"));
                    }
                    // Executer le PID position (sans affecter les moteurs)
                    pid_compute_position(&drone);
                } else {
                    if (pos_test_mode) {
                        pos_test_mode = false;
                        drone.pid_output_pos_roll = 0;
                        drone.pid_output_pos_pitch = 0;
                        Serial.println(F("[TEST] Position test mode OFF"));
                    }
                }
                
                // Désarmement (Gaz Bas + Yaw Droite)
                if(drone.channel_3 < 1010 && drone.channel_4 > 1800) {
                     if(arming_timer == 0) arming_timer = millis();
                     else if(millis() - arming_timer > 1000) {
                        drone.current_mode = MODE_SAFE;
                        arming_timer = 0;
                     }
                } else { arming_timer = 0; }
                break;

            // ---------------- MODE FLYING ----------------
            case MODE_FLYING:
                digitalWrite(PIN_LED, LOW); 

                // --- LOGIQUE SWITCH AUX1 + AUX2 ---
                if (drone.channel_5 > 1500 && drone.channel_6 > 1500) {
                    // === TRANSITION VERS AUTO_POS (Alt Hold + Pos Hold) ===
                    drone.current_mode = MODE_AUTO_POS;
                    drone.alt_setpoint = drone.lidar_distance;
                    // Init Alt Hold
                    if (drone.lidar_distance < 20.0f) {
                        drone.takeoff_ramp_active = true;
                        drone.throttle_base_ramp = MIN_THROTTLE_IDLE;
                        drone.alt_stick_offset = 0.0f;
                        pid_reset_altitude_integral();
                    } else {
                        drone.takeoff_ramp_active = false;
                        drone.throttle_base_ramp = drone.hover_throttle;
                        float stick_maps_to = ALT_MIN_HEIGHT + (ALT_MAX_HEIGHT - ALT_MIN_HEIGHT)
                                              * ((float)(drone.channel_3 - ALT_HOLD_KILL_THROTTLE)
                                              / (2000.0f - (float)ALT_HOLD_KILL_THROTTLE));
                        if (stick_maps_to < ALT_MIN_HEIGHT) stick_maps_to = ALT_MIN_HEIGHT;
                        if (stick_maps_to > ALT_MAX_HEIGHT) stick_maps_to = ALT_MAX_HEIGHT;
                        drone.alt_stick_offset = drone.lidar_distance - stick_maps_to;
                        pid_preload_altitude_integral((float)(drone.channel_3 - drone.hover_throttle));
                    }
                    // Init Position Hold
                    optical_flow_reset_position();
                    drone.pos_est_x = 0; drone.pos_est_y = 0;
                    drone.pos_setpoint_x = 0; drone.pos_setpoint_y = 0;
                    drone.pid_output_pos_roll = 0; drone.pid_output_pos_pitch = 0;
                    pid_reset_position();
                    Serial.printf("[POS] Engage AUTO_POS @ %.0f cm\n", drone.lidar_distance);
                    break;
                } else if (drone.channel_5 > 1500) {
                    // === TRANSITION VERS ALT_HOLD (AUX1 seul) ===
                    drone.current_mode = MODE_ALT_HOLD;
                    drone.alt_setpoint = drone.lidar_distance;
                    if (drone.lidar_distance < 20.0f) {
                        drone.takeoff_ramp_active = true;
                        drone.throttle_base_ramp = MIN_THROTTLE_IDLE;
                        drone.alt_stick_offset = 0.0f;
                        pid_reset_altitude_integral();
                    } else {
                        drone.takeoff_ramp_active = false;
                        drone.throttle_base_ramp = drone.hover_throttle;
                        float stick_maps_to = ALT_MIN_HEIGHT + (ALT_MAX_HEIGHT - ALT_MIN_HEIGHT)
                                              * ((float)(drone.channel_3 - ALT_HOLD_KILL_THROTTLE)
                                              / (2000.0f - (float)ALT_HOLD_KILL_THROTTLE));
                        if (stick_maps_to < ALT_MIN_HEIGHT) stick_maps_to = ALT_MIN_HEIGHT;
                        if (stick_maps_to > ALT_MAX_HEIGHT) stick_maps_to = ALT_MAX_HEIGHT;
                        drone.alt_stick_offset = drone.lidar_distance - stick_maps_to;
                        pid_preload_altitude_integral((float)(drone.channel_3 - drone.hover_throttle));
                    }
                    break;
                } else if (drone.channel_6 > 1500) {
                    // === TRANSITION VERS POS_HOLD (AUX2 seul) ===
                    drone.current_mode = MODE_POS_HOLD;
                    optical_flow_reset_position();
                    drone.pos_est_x = 0; drone.pos_est_y = 0;
                    drone.pos_setpoint_x = 0; drone.pos_setpoint_y = 0;
                    drone.pid_output_pos_roll = 0; drone.pid_output_pos_pitch = 0;
                    pid_reset_position();
                    Serial.println(F("[POS] Engage POS_HOLD (manual throttle)"));
                    break;
                }

                // --- INTELLIGENCE DE VOL (MODE MANUEL) ---
                pid_compute_setpoints(&drone);
                pid_compute(&drone);
                motors_mix(&drone);
                motors_write();
                
                // --- DESARMEMENT MANUEL D'URGENCE ---
                if(drone.channel_3 < 1010 && drone.channel_4 > 1800) {
                     if(arming_timer == 0) arming_timer = millis();
                     else if(millis() - arming_timer > 1000) {
                        drone.current_mode = MODE_SAFE;
                        motors_stop(); 
                        arming_timer = 0;
                     }
                } else { 
                    arming_timer = 0; 
                }

                // --- SECURITE RADIO / AUTOLANDING ---
                if (drone.channel_3 < 1010) {
                    if (disarm_debounce_timer == 0) disarm_debounce_timer = millis();
                    
                    if (millis() - disarm_debounce_timer > 60000) { 
                        drone.current_mode = MODE_ARMED; 
                        error_code = 2; // PERTE RADIO
                        disarm_debounce_timer = 0;
                    }
                } else {
                    disarm_debounce_timer = 0; 
                }
                break;

            // ---------------- MODE ALT_HOLD ----------------
            case MODE_ALT_HOLD:
            {
                // LED clignotement rapide pour signaler Alt Hold
                if ((millis() % 200) < 100) digitalWrite(PIN_LED, HIGH);
                else digitalWrite(PIN_LED, LOW);

                // === AUX1 OFF → Retour en mode manuel ===
                if (drone.channel_5 < 1500) {
                    drone.current_mode = MODE_FLYING;
                    drone.takeoff_ramp_active = false;
                    pid_reset_altitude_integral();
                    Serial.println(F("[ALT] AUX1 OFF → Retour MANUEL"));
                    break;
                }

                // === Sécurité : stick gaz tout en bas → retour manuel immédiat ===
                if (drone.channel_3 < 1050) {
                    drone.current_mode = MODE_FLYING;
                    drone.takeoff_ramp_active = false;
                    pid_reset_altitude_integral();
                    Serial.println(F("[ALT] GAZ BAS → Retour MANUEL"));
                    break;
                }

                // === AUX2 ON → Upgrade vers AUTO_POS ===
                if (drone.channel_6 > 1500) {
                    drone.current_mode = MODE_AUTO_POS;
                    optical_flow_reset_position();
                    drone.pos_est_x = 0; drone.pos_est_y = 0;
                    drone.pos_setpoint_x = 0; drone.pos_setpoint_y = 0;
                    drone.pid_output_pos_roll = 0; drone.pid_output_pos_pitch = 0;
                    pid_reset_position();
                    Serial.println(F("[POS] ALT_HOLD → AUTO_POS"));
                    break;
                }

                // === Boucle de vol Alt Hold ===
                // pid_compute_altitude gère le tracking sol + lissage + PID
                pid_compute_altitude(&drone);    // PID altitude → throttle_command
                pid_compute_setpoints(&drone);   // Roll/Pitch/Yaw normaux
                pid_compute(&drone);             // PID angulaire
                motors_mix(&drone);              // Utilise throttle_command comme base
                motors_write();

                // === Désarmement manuel d'urgence (Gaz bas + Yaw droite) ===
                if (drone.channel_3 < 1010 && drone.channel_4 > 1800) {
                    if (arming_timer == 0) arming_timer = millis();
                    else if (millis() - arming_timer > 1000) {
                        drone.current_mode = MODE_SAFE;
                        pid_reset_altitude_integral();
                        motors_stop();
                        arming_timer = 0;
                    }
                } else {
                    arming_timer = 0;
                }
                break;
            }

            // ---------------- MODE POS_HOLD (position seule, throttle manuel) ----------------
            case MODE_POS_HOLD:
            {
                // LED triple flash rapide pour distinguer du Alt Hold
                unsigned long ms = millis() % 800;
                if (ms < 80 || (ms > 150 && ms < 230) || (ms > 300 && ms < 380)) digitalWrite(PIN_LED, HIGH);
                else digitalWrite(PIN_LED, LOW);

                // === AUX2 OFF → Retour FLYING ===
                if (drone.channel_6 < 1500) {
                    drone.current_mode = MODE_FLYING;
                    drone.pid_output_pos_roll = 0;
                    drone.pid_output_pos_pitch = 0;
                    pid_reset_position();
                    Serial.println(F("[POS] AUX2 OFF → FLYING"));
                    break;
                }

                // === AUX1 ON → Upgrade vers AUTO_POS (position + altitude) ===
                if (drone.channel_5 > 1500) {
                    drone.current_mode = MODE_AUTO_POS;
                    drone.alt_setpoint = drone.lidar_distance;
                    if (drone.lidar_distance < 20.0f) {
                        drone.takeoff_ramp_active = true;
                        drone.throttle_base_ramp = MIN_THROTTLE_IDLE;
                        drone.alt_stick_offset = 0.0f;
                        pid_reset_altitude_integral();
                    } else {
                        drone.takeoff_ramp_active = false;
                        drone.throttle_base_ramp = drone.hover_throttle;
                        float stick_maps_to = ALT_MIN_HEIGHT + (ALT_MAX_HEIGHT - ALT_MIN_HEIGHT)
                                              * ((float)(drone.channel_3 - ALT_HOLD_KILL_THROTTLE)
                                              / (2000.0f - (float)ALT_HOLD_KILL_THROTTLE));
                        if (stick_maps_to < ALT_MIN_HEIGHT) stick_maps_to = ALT_MIN_HEIGHT;
                        if (stick_maps_to > ALT_MAX_HEIGHT) stick_maps_to = ALT_MAX_HEIGHT;
                        drone.alt_stick_offset = drone.lidar_distance - stick_maps_to;
                        pid_preload_altitude_integral((float)(drone.channel_3 - drone.hover_throttle));
                    }
                    Serial.println(F("[POS] POS_HOLD → AUTO_POS"));
                    break;
                }

                // === Boucle de vol Position Hold (throttle manuel) ===
                pid_compute_position(&drone);    // PID position → corrections roll/pitch
                pid_compute_setpoints(&drone);   // Roll/Pitch/Yaw (avec corrections position)
                pid_compute(&drone);             // PID rate
                motors_mix(&drone);              // Throttle manuel (channel_3)
                motors_write();

                // === Désarmement urgence ===
                if (drone.channel_3 < 1010 && drone.channel_4 > 1800) {
                    if (arming_timer == 0) arming_timer = millis();
                    else if (millis() - arming_timer > 1000) {
                        drone.current_mode = MODE_SAFE;
                        drone.pid_output_pos_roll = 0;
                        drone.pid_output_pos_pitch = 0;
                        pid_reset_position();
                        motors_stop();
                        arming_timer = 0;
                    }
                } else arming_timer = 0;
                break;
            }

            // ---------------- MODE AUTO_POS ----------------
            case MODE_AUTO_POS:
            {
                // LED double flash pour distinguer visuellement
                unsigned long ms = millis() % 600;
                if (ms < 100 || (ms > 200 && ms < 300)) digitalWrite(PIN_LED, HIGH);
                else digitalWrite(PIN_LED, LOW);

                // === AUX1 OFF → Downgrade vers POS_HOLD (garde le position hold) ===
                if (drone.channel_5 < 1500) {
                    drone.current_mode = MODE_POS_HOLD;
                    drone.takeoff_ramp_active = false;
                    pid_reset_altitude_integral();
                    Serial.println(F("[POS] AUX1 OFF → POS_HOLD"));
                    break;
                }

                // === AUX2 OFF → Downgrade vers ALT_HOLD ===
                if (drone.channel_6 < 1500) {
                    drone.current_mode = MODE_ALT_HOLD;
                    drone.pid_output_pos_roll = 0;
                    drone.pid_output_pos_pitch = 0;
                    pid_reset_position();
                    Serial.println(F("[POS] AUX2 OFF → ALT_HOLD"));
                    break;
                }

                // === Sécurité : stick gaz tout en bas → retour FLYING ===
                if (drone.channel_3 < 1050) {
                    drone.current_mode = MODE_FLYING;
                    drone.takeoff_ramp_active = false;
                    drone.pid_output_pos_roll = 0;
                    drone.pid_output_pos_pitch = 0;
                    pid_reset_position();
                    pid_reset_altitude_integral();
                    Serial.println(F("[POS] GAZ BAS → FLYING"));
                    break;
                }

                // === Boucle de vol Auto Pos ===
                pid_compute_position(&drone);    // PID position → corrections roll/pitch
                pid_compute_altitude(&drone);    // PID altitude → throttle_command
                pid_compute_setpoints(&drone);   // Roll/Pitch/Yaw (corrections position injectées)
                pid_compute(&drone);             // PID rate
                motors_mix(&drone);
                motors_write();

                // === Désarmement urgence ===
                if (drone.channel_3 < 1010 && drone.channel_4 > 1800) {
                    if (arming_timer == 0) arming_timer = millis();
                    else if (millis() - arming_timer > 1000) {
                        drone.current_mode = MODE_SAFE;
                        drone.pid_output_pos_roll = 0;
                        drone.pid_output_pos_pitch = 0;
                        pid_reset_position();
                        pid_reset_altitude_integral();
                        motors_stop();
                        arming_timer = 0;
                    }
                } else arming_timer = 0;
                break;
            }

            case MODE_WEB_TEST:
                motors_write_direct(
                    drone.web_test_vals[1], drone.web_test_vals[2], 
                    drone.web_test_vals[3], drone.web_test_vals[4]
                );
                // Sortie du mode test si on touche aux gaz
                if(drone.channel_3 > 1100) {
                    drone.current_mode = MODE_SAFE;
                    motors_stop();
                }
                break;
        }

        // --- DIAGNOSTIC LAG (BOÎTE NOIRE) ---
        unsigned long t_end = micros();
        unsigned long dur_radio = t_radio - t_start;
        unsigned long dur_imu = t_imu - t_radio;
        unsigned long dur_pid_mix = t_end - t_imu;
        unsigned long total_loop = t_end - t_start;

        drone.loop_time = total_loop; 
        
        // Si on détecte un gros lag (> 6000us), on enregistre le coupable
        // MAIS on évite d'overwrite le code d'erreur 888888 (IMU CRASH)
        if (total_loop > 6000) {
            if(dur_radio > drone.max_time_radio) drone.max_time_radio = dur_radio;
            if(drone.max_time_imu != 888888 && dur_imu > drone.max_time_imu) drone.max_time_imu = dur_imu;
            if(dur_pid_mix > drone.max_time_pid) drone.max_time_pid = dur_pid_mix;
        }
    }

    // --- OPTIMISATION MULTITÂCHE ---
    // Gestion du Loop Time (250Hz = 4000µs)
    unsigned long time_now = micros();
    
    // Si on a fini les calculs tôt (qu'il reste du temps avant le prochain cycle)
    if (loop_timer + LOOP_TIME_US > time_now) {
        unsigned long remaining = (loop_timer + LOOP_TIME_US) - time_now;
        
        // Si on a plus de 1.5ms de marge, on "dort" 1ms pour laisser le WiFi travailler
        // C'est LA ligne qui sauve la télémétrie
        if (remaining > 1500) {
             vTaskDelay(1); // Libère le CPU pour ~1ms (WiFi, Télémétrie)
        }
        
        // On finit le reste du temps (quelques µs) en attente active pour la précision PID
        while(micros() - loop_timer < LOOP_TIME_US);
    }
    
    loop_timer = micros();
}