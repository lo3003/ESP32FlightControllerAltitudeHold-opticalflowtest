#ifndef TYPES_H
#define TYPES_H

// Modes de vol
typedef enum {
    MODE_SAFE,           // 0 - Moteurs coupés, LED Fixe
    MODE_CALIBRATION,    // 1 - Mode spécial Calibration ESC
    MODE_SETUP,          // 2 - Mode spécial Setup Wizard
    MODE_PRE_ARM,        // 3 - Attente armement
    MODE_ARMED,          // 4 - Moteurs au ralenti
    MODE_FLYING,         // 5 - Boucle de vol complète
    MODE_ALT_HOLD,       // 6 - Maintien d'altitude (LiDAR)
    MODE_POS_HOLD,       // 7 - Maintien de position seul (throttle manuel)
    MODE_AUTO_POS,       // 8 - Maintien d'altitude + position
    MODE_WEB_TEST        // 9 - Mode de test via interface web
} FlightMode;

// Etat global (partagé entre les modules)
typedef struct {
    // Mode
    FlightMode current_mode;

    // Radio (1000-2000)
    int channel_1; // Roll
    int channel_2; // Pitch
    int channel_3; // Throttle
    int channel_4; // Yaw

    // IMU
    float gyro_roll_input;   // deg/s
    float gyro_pitch_input;  // deg/s
    float gyro_yaw_input;    // deg/s
    float angle_roll;        // deg
    float angle_pitch;       // deg
    float angle_yaw;         // deg (intégré gyro, pas utilisé pour PID yaw)
    float acc_total_vector;
    
    // Accélération normalisée (en G) pour PoC drift
    float acc_x;             // G (axe X)
    float acc_y;             // G (axe Y)
    float acc_z;             // G (axe Z)

    // PID setpoints (rate)
    float pid_setpoint_roll;
    float pid_setpoint_pitch;
    float pid_setpoint_yaw;

    // PID outputs (mixage)
    float pid_output_roll;
    float pid_output_pitch;
    float pid_output_yaw;

    // Gains PID
    float p_pitch_roll;
    float i_pitch_roll;
    float d_pitch_roll;

    float p_yaw;
    float i_yaw;
    float d_yaw;

    // FeedForward (gain appliqué à la consigne pilote en RATE)
    float ff_pitch_roll;  // commun roll/pitch
    float ff_yaw;

    // Auto-level (outer loop)
    float p_level;

    // Heading Hold (maintien de cap)
    float p_heading;

    // Trims accéléromètre (offsets mécaniques)
    float trim_roll;
    float trim_pitch;

    // Web test
    int web_test_vals[5];

    // Timing / debug
    unsigned long loop_time;
    unsigned long max_time_radio;
    unsigned long max_time_imu;
    unsigned long max_time_pid;

    // Durée IMU (task) exposée au main
    unsigned long current_time_imu;

    // Tension batterie (V)
    float voltage_bat;

    // --- Second IMU (AltIMU-10 v2) pour comparaison ---
    float alt_acc_x;         // G (axe X)
    float alt_acc_y;         // G (axe Y)
    float alt_acc_z;         // G (axe Z)
    float alt_gyro_roll;     // deg/s
    float alt_gyro_pitch;    // deg/s
    float alt_gyro_yaw;      // deg/s
    float alt_angle_roll;    // deg (Kalman)
    float alt_angle_pitch;   // deg (Kalman)
    float alt_angle_yaw;     // deg (magnétomètre heading)

    // --- LiDAR / Altitude Hold ---
    float lidar_distance;    // Altitude compensée inclinaison (cm)
    float lidar_velocity;    // Vitesse verticale (cm/s)
    float alt_setpoint;      // Consigne d'altitude (cm)
    float alt_stick_offset;  // Offset bumpless pour le mapping stick→altitude
    float pid_output_alt;    // Sortie PID altitude
    int   throttle_base_ramp;    // Throttle de base progressif pendant le décollage
    bool  takeoff_ramp_active;   // True = rampe de décollage en cours

    // Gains PID Altitude (réglables en vol via telemetry)
    float p_alt;
    float i_alt;
    float d_alt;
    int   hover_throttle;    // Gaz de hover (µs, ~1450)
    float alt_smooth;        // Facteur de lissage setpoint altitude (0.005=lent, 0.05=réactif)

    // Radio AUX1 (Canal 5)
    int   channel_5;         // Valeur brute AUX1 (1000-2000)
    bool  auto_takeoff_active; // Flag auto-takeoff en cours

    // --- Radio AUX2 (Canal 6) ---
    int   channel_6;              // Valeur brute AUX2 (1000-2000)

    // --- Optical Flow / Position Hold ---
    float flow_velocity_x;       // Vitesse sol X world frame (cm/s)
    float flow_velocity_y;       // Vitesse sol Y world frame (cm/s)
    int   flow_quality;          // Qualité optical flow (0-255)
    bool  new_flow_data;            // Flag: nouvelle trame OF reçue
    float flow_raw_x;            // Delta flow brut X (debug)
    float flow_raw_y;            // Delta flow brut Y (debug)
    float pos_est_x;             // Position estimée X (cm, intégrale de vel)
    float pos_est_y;             // Position estimée Y (cm)
    float pos_setpoint_x;        // Consigne position X (cm, lockée à l'engagement)
    float pos_setpoint_y;        // Consigne position Y (cm)
    float pid_output_pos_roll;   // Correction PID pos → roll (degrés)
    float pid_output_pos_pitch;  // Correction PID pos → pitch (degrés)
    float p_pos;                 // Gain P position
    float i_pos;                 // Gain I position
    float d_pos;                 // Gain D position (utilise la vitesse directement)
    float pos_smooth;            // Filtre lissage position setpoint

    // --- EKF debug (telemetrie) ---
    float ekf_vel_x;            // Vitesse estimee EKF X (cm/s)
    float ekf_vel_y;            // Vitesse estimee EKF Y (cm/s)
    float quality_fade;         // Facteur confiance OF (0.0-1.0)

    // Commande gaz finale (après calcul PID alt ou manuel)
    int   throttle_command;
} DroneState;

#endif
