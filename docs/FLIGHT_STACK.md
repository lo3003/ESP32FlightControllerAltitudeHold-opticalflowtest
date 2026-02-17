# Flight Stack Architecture

**Version:** 1.0
**Plateforme:** ESP32 (dual-core) + MPU6050
**Auteur:** Documentation technique du projet Drone UMons 2025-2026

---

## Vue d'ensemble

La Flight Stack implémente une boucle de contrôle de vol classique pour quadricoptère en configuration "X". L'architecture repose sur trois étages principaux exécutés en pipeline :

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         FLIGHT STACK PIPELINE                           │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  ┌─────────────┐    ┌─────────────────┐    ┌──────────────────────┐    │
│  │   IMU Task  │───>│  PID Controller │───>│  Motor Mixer (LEDC)  │    │
│  │   (Core 0)  │    │    (Core 1)     │    │       (Core 1)       │    │
│  │   @ 250 Hz  │    │    @ 250 Hz     │    │       @ 250 Hz       │    │
│  └─────────────┘    └─────────────────┘    └──────────────────────┘    │
│        │                    │                        │                  │
│        v                    v                        v                  │
│   angle_roll          pid_output_roll           esc_1..esc_4           │
│   angle_pitch         pid_output_pitch          (PWM 14-bit)           │
│   gyro_*_input        pid_output_yaw                                   │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## 1. Acquisition & Estimation (IMU)

### 1.1 Architecture RTOS

L'acquisition IMU est isolée sur le **Core 0** de l'ESP32 via une tâche FreeRTOS dédiée, permettant de découpler le traitement temps-critique du bus I2C de la boucle principale de vol (Core 1).

```c
// src/imu.cpp:387-395
xTaskCreatePinnedToCore(
    imu_task,           // Fonction de la tâche
    "imu_i2c",          // Nom (debug)
    4096,               // Stack size (bytes)
    nullptr,            // Paramètre
    4,                  // Priorité (haute)
    &imu_task_handle,   // Handle
    0                   // Core 0 (dédié IMU)
);
```

La tâche s'exécute à **250 Hz** (période de 4 ms) grâce à `vTaskDelayUntil` qui garantit un timing régulier indépendamment du temps d'exécution :

```c
// src/imu.cpp:364
vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(4)); // 250 Hz
```

### 1.2 Protection des données

Deux mécanismes de synchronisation sont utilisés :

#### Mutex I2C (`i2c_mutex`)
Protège l'accès concurrent au bus I2C, partagé potentiellement avec d'autres capteurs (AltIMU-10 v2). Le mutex utilise un timeout de 2 ms pour éviter les blocages :

```c
// src/imu.cpp:152-158
if (i2c_mutex != nullptr) {
    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(2)) != pdTRUE) {
        // Mutex occupé : skip cette lecture (l'AltIMU utilise le bus)
        return;
    }
}
```

#### Spinlock IMU (`imu_mux`)
Protège l'intégrité du snapshot de données IMU entre la tâche productrice (Core 0) et le consommateur (Core 1). Un `portMUX_TYPE` est utilisé pour les sections critiques courtes :

```c
// src/imu.cpp:56
static portMUX_TYPE imu_mux = portMUX_INITIALIZER_UNLOCKED;

// Écriture (tâche IMU, Core 0) - src/imu.cpp:348-362
portENTER_CRITICAL(&imu_mux);
imu_snap.gyro_roll_input  = imu_state.gyro_roll_input;
imu_snap.angle_roll       = imu_state.angle_roll;
// ... autres champs ...
portEXIT_CRITICAL(&imu_mux);

// Lecture (boucle principale, Core 1) - src/imu.cpp:405-409
portENTER_CRITICAL(&imu_mux);
imu_in_mode = drone->current_mode;
s = imu_snap;  // Copie atomique du snapshot complet
portEXIT_CRITICAL(&imu_mux);
```

### 1.3 Chaîne de traitement

#### Étape 1 : Lecture I2C "Burst"

La lecture est optimisée en mode "burst" : une seule transaction I2C lit les 14 registres consécutifs (accéléromètre + température + gyroscope) :

```c
// src/imu.cpp:160-190
Wire.beginTransmission(MPU_ADDR);
Wire.write(0x3B);  // Registre de départ (ACCEL_XOUT_H)
Wire.endTransmission();
Wire.requestFrom(MPU_ADDR, 14);  // Lecture burst de 14 bytes

// Décodage Big-Endian
acc_raw[0] = (int16_t)(Wire.read() << 8 | Wire.read());  // ACCEL_X
acc_raw[1] = (int16_t)(Wire.read() << 8 | Wire.read());  // ACCEL_Y
acc_raw[2] = (int16_t)(Wire.read() << 8 | Wire.read());  // ACCEL_Z
temperature = (int16_t)(Wire.read() << 8 | Wire.read()); // TEMP
gyro_raw[0] = (int16_t)(Wire.read() << 8 | Wire.read()); // GYRO_X
gyro_raw[1] = (int16_t)(Wire.read() << 8 | Wire.read()); // GYRO_Y
gyro_raw[2] = (int16_t)(Wire.read() << 8 | Wire.read()); // GYRO_Z
```

#### Étape 2 : Calibration (offset au démarrage)

Au démarrage, une calibration de 5 secondes échantillonne le gyroscope au repos pour calculer les offsets de biais :

```c
// src/imu.cpp:81-118
long gyro_sum_x = 0, gyro_sum_y = 0, gyro_sum_z = 0;
int valid_samples = 0;

while (millis() - calib_start < 5000) {
    // ... lecture gyro ...
    gyro_sum_x += (int16_t)(Wire.read() << 8 | Wire.read());
    gyro_sum_y += (int16_t)(Wire.read() << 8 | Wire.read());
    gyro_sum_z += (int16_t)(Wire.read() << 8 | Wire.read());
    valid_samples++;
}

// Moyenne = offset de biais
gyro_off_x = (double)gyro_sum_x / valid_samples;
gyro_off_y = (double)gyro_sum_y / valid_samples;
gyro_off_z = (double)gyro_sum_z / valid_samples;
```

Les valeurs brutes sont ensuite corrigées à chaque lecture :

```c
// src/imu.cpp:196-198
double gyro_x_cal = gyro_raw[0] - gyro_off_x;
double gyro_y_cal = gyro_raw[1] - gyro_off_y;
double gyro_z_cal = gyro_raw[2] - gyro_off_z;
```

#### Étape 3 : Mapping des Axes (CRITIQUE)

**Attention :** Le mapping des axes est **croisé** pour adapter l'orientation physique du capteur MPU6050 au repère du drone :

| Axe Drone | Source Accéléromètre | Source Gyroscope |
|-----------|----------------------|------------------|
| **Roll**  | `acc_raw[1]` (Y)     | `gyro_x_cal`     |
| **Pitch** | `acc_raw[0]` (X)     | `-gyro_y_cal` (inversé) |
| **Yaw**   | `acc_raw[2]` (Z)     | `±gyro_z_cal` (configurable) |

```c
// src/imu.cpp:200-214
// Mapping axes
double gyro_roll  = gyro_x_cal;
long acc_roll_val = acc_raw[1];  // <-- Acc Y pilote le Roll

double gyro_pitch = -gyro_y_cal; // <-- INVERSION du signe Gyro Y
long acc_pitch_val = acc_raw[0]; // <-- Acc X pilote le Pitch

// YAW : inversion configurable
#if IMU_INVERT_YAW
    double gyro_yaw = -gyro_z_cal;
#else
    double gyro_yaw = gyro_z_cal;
#endif
```

**Explication :** Le MPU6050 est monté de telle sorte que :
- L'axe physique Y de l'accéléromètre mesure l'inclinaison Roll du drone
- L'axe physique X mesure l'inclinaison Pitch
- Le signe du gyroscope Y doit être inversé pour correspondre à la convention de rotation positive

#### Étape 4 : Filtrage

**Filtre PT1 (passe-bas du 1er ordre) sur le gyroscope :**

Atténue le bruit haute fréquence avant injection dans le PID. Le coefficient `GYRO_PT1_COEFF = 0.5` représente un compromis réactivité/lissage :

```c
// src/imu.cpp:31
#define GYRO_PT1_COEFF 0.5f

// src/imu.cpp:225-227
gyro_roll_filt  += GYRO_PT1_COEFF * (gyro_roll_dps  - gyro_roll_filt);
gyro_pitch_filt += GYRO_PT1_COEFF * (gyro_pitch_dps - gyro_pitch_filt);
gyro_yaw_filt   += GYRO_PT1_COEFF * (gyro_yaw_dps   - gyro_yaw_filt);
```

**Filtre de Kalman pour la fusion d'angle (Accéléromètre + Gyroscope) :**

Le filtre de Kalman fusionne les mesures complémentaires :
- **Gyroscope** : précis à court terme mais dérive dans le temps
- **Accéléromètre** : stable à long terme mais bruité et sensible aux vibrations

```c
// src/kalman.cpp:26-66
float Kalman::update(float angle_acc, float gyro_rate, float dt) {
    // ========== ÉTAPE 1: PRÉDICTION ==========
    rate = gyro_rate - bias;
    angle += dt * rate;

    // Mise à jour covariance P = A * P * A' + Q
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // ========== ÉTAPE 2: CORRECTION ==========
    float y = angle_acc - angle;    // Innovation
    float S = P[0][0] + R_measure;  // Covariance innovation

    // Gain de Kalman
    float K[2];
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    // Mise à jour état et covariance
    angle += K[0] * y;
    bias  += K[1] * y;
    // ... mise à jour P ...

    return angle;
}
```

**Paramètres de tuning Kalman :**

| Paramètre | Valeur | Description |
|-----------|--------|-------------|
| `Q_angle` | 0.001 | Confiance dans le modèle (gyro) |
| `Q_bias`  | 0.003 | Vitesse de dérive du biais gyro |
| `R_measure` | 0.03-0.05 | Bruit accéléromètre (adaptatif) |

Le paramètre `R_measure` est ajusté dynamiquement selon le mode de vol :

```c
// src/imu.cpp:267-273
if (drone->current_mode == MODE_SAFE && drone->channel_3 < 1050) {
    kalman_roll.setRmeasure(0.01f);   // Plus de confiance accéléromètre (au sol)
} else {
    kalman_roll.setRmeasure(0.05f);   // Moins de confiance (en vol, vibrations)
}
```

---

## 2. Boucle de Contrôle (PID Controller)

### 2.1 Architecture en Cascade

Le contrôleur PID utilise une architecture **cascade à deux boucles** :

```
                    ┌──────────────────────────────────────────────────────┐
                    │              BOUCLE EXTERNE (ANGLE)                  │
Stick ──────────────┤                                                      │
  │                 │  stick_roll - (angle_roll × P_level) = setpoint_rate │
  │                 └────────────────────────┬─────────────────────────────┘
  │                                          │
  │                                          v
  │                 ┌──────────────────────────────────────────────────────┐
  │                 │              BOUCLE INTERNE (RATE)                   │
  └── FeedForward ──┤                                                      │
                    │   PID(gyro_input - setpoint_rate) = pid_output       │
                    └────────────────────────┬─────────────────────────────┘
                                             │
                                             v
                                        Motor Mixer
```

**Outer Loop (Angle/Self-Level) :**
```c
// src/pid.cpp:96-97
float input_roll = stick_roll - (drone->angle_roll * drone->p_level);
drone->pid_setpoint_roll = input_roll / 3.0f;
```

- Convertit la commande stick en consigne de **taux de rotation** (rate)
- Le gain `p_level` (défaut: 5.0) amplifie l'erreur d'angle pour forcer le retour à l'horizontale

**Inner Loop (Rate) :**
Régule la vitesse angulaire mesurée par le gyroscope pour suivre la consigne rate.

### 2.2 Implémentation PID

#### Terme Proportionnel (P)

Réaction proportionnelle à l'erreur instantanée :

```c
// src/pid.cpp:235-237
float error = drone->gyro_roll_input - drone->pid_setpoint_roll;
float p_term_roll = (drone->p_pitch_roll * tpa_factor) * error;
```

#### Terme Intégral (I) avec Anti-Windup

L'intégrateur accumule l'erreur pour éliminer les erreurs statiques, mais plusieurs protections évitent le "windup" :

```c
// src/pid.cpp:245-254
if (in_flight) {
    // Anti-windup : N'accumule PAS si la sortie est déjà saturée
    float output_before_i = p_term_roll + pid_i_mem_roll + d_term_roll;
    if (fabsf(output_before_i) < PID_MAX_ROLL) {
        pid_i_mem_roll += drone->i_pitch_roll * error;
    }
    // Clamping de l'intégrateur
    if (pid_i_mem_roll > PID_MAX_ROLL) pid_i_mem_roll = PID_MAX_ROLL;
    else if (pid_i_mem_roll < -PID_MAX_ROLL) pid_i_mem_roll = -PID_MAX_ROLL;
} else {
    pid_i_mem_roll = 0;  // Reset si gaz coupés
}
```

**Conditions de blocage de l'intégrateur :**
1. Drone non "in-flight" (gaz < 1020 depuis plus de 500ms)
2. Sortie PID (sans I) déjà saturée → évite l'accumulation inutile

#### Terme Dérivé (D) - "Derivative on Measurement"

Le terme D est calculé sur la **mesure** (gyro) et non sur l'erreur, évitant les "derivative kicks" lors de changements brusques de consigne :

```c
// src/pid.cpp:239-243
// Calcul sur la variation de MESURE (pas l'erreur)
d_err_raw = pid_last_roll_input - drone->gyro_roll_input;

// Filtre passe-bas pour réduire le bruit haute fréquence
d_err_filtered = pid_roll_d_filter_old + D_FILTER_COEFF * (d_err_raw - pid_roll_d_filter_old);
pid_roll_d_filter_old = d_err_filtered;
pid_last_roll_input = drone->gyro_roll_input;

float d_term_roll = (drone->d_pitch_roll * tpa_factor) * d_err_filtered;
```

Le coefficient `D_FILTER_COEFF = 0.18` (dans `config.h`) lisse le signal dérivé pour éviter l'amplification du bruit.

#### FeedForward (FF)

Branche directe qui injecte la commande pilote dans la sortie, améliorant la réactivité sans passer par la boucle de rétroaction :

```c
// src/pid.cpp:93-94, 256
ff_sp_roll = stick_roll / 3.0f;  // Mémorisation de la consigne pilote brute
// ...
float ff_term_roll = (drone->ff_pitch_roll * tpa_factor) * ff_sp_roll;
```

#### Sortie Finale

```c
// src/pid.cpp:258-260
drone->pid_output_roll = p_term_roll + pid_i_mem_roll + d_term_roll + ff_term_roll;
if (drone->pid_output_roll > PID_MAX_ROLL) drone->pid_output_roll = PID_MAX_ROLL;
else if (drone->pid_output_roll < -PID_MAX_ROLL) drone->pid_output_roll = -PID_MAX_ROLL;
```

### 2.3 Sécurité : Détection "In-Flight"

Un timer de 500 ms empêche le reset intempestif du terme I sur des micro-coupures de gaz :

```c
// src/pid.cpp:222-225
if (drone->channel_3 > 1020) {
    pid_inflight_timer = millis();
}
bool in_flight = (millis() - pid_inflight_timer < 500);
```

**Comportement :**
- Tant que les gaz ont été > 1020 dans les dernières 500 ms, le drone est considéré "in-flight"
- Le terme I est préservé lors de brèves coupures de gaz (manoeuvres agressives)
- Reset complet après 500 ms de gaz coupés (atterrissage confirmé)

### 2.4 Paramètres PID par défaut

```c
// src/pid.cpp:40-61
// ROLL/PITCH (Rate loop)
drone->p_pitch_roll = 2.5f;
drone->i_pitch_roll = 0.01f;
drone->d_pitch_roll = 8.0f;

// YAW (Rate loop)
drone->p_yaw = 2.0f;
drone->i_yaw = 0.005f;
drone->d_yaw = 0.0f;

// FEEDFORWARD
drone->ff_pitch_roll = 0.16f;
drone->ff_yaw        = 0.10f;

// AUTO LEVEL (Outer loop)
drone->p_level = 5.0f;
```

---

## 3. Mixage & Motorisation (Actuators)

### 3.1 Hardware : Driver LEDC

L'ESP32 utilise son périphérique LEDC (LED Control) pour générer les signaux PWM des ESC :

```c
// src/motors.cpp:12-21
#define PWM_CH1 0
#define PWM_CH2 1
#define PWM_CH3 2
#define PWM_CH4 3

void motors_init() {
    ledcSetup(PWM_CH1, ESC_FREQ, 14);  // 250 Hz, résolution 14 bits
    ledcSetup(PWM_CH2, ESC_FREQ, 14);
    ledcSetup(PWM_CH3, ESC_FREQ, 14);
    ledcSetup(PWM_CH4, ESC_FREQ, 14);

    ledcAttachPin(PIN_MOTOR_1, PWM_CH1);  // GPIO 27
    ledcAttachPin(PIN_MOTOR_2, PWM_CH2);  // GPIO 13
    ledcAttachPin(PIN_MOTOR_3, PWM_CH3);  // GPIO 25
    ledcAttachPin(PIN_MOTOR_4, PWM_CH4);  // GPIO 26
}
```

**Conversion µs → duty cycle 14-bit :**

```c
// src/motors.cpp:73-77
void motors_write() {
    ledcWrite(PWM_CH1, (esc_1 * 16383) / 4000);
    ledcWrite(PWM_CH2, (esc_2 * 16383) / 4000);
    ledcWrite(PWM_CH3, (esc_3 * 16383) / 4000);
    ledcWrite(PWM_CH4, (esc_4 * 16383) / 4000);
}
```

- `esc_x` : valeur en microsecondes (1000-2000 typique)
- `16383` : valeur max pour 14 bits (2^14 - 1)
- `4000` : période totale à 250 Hz (1/250 = 4000 µs)

### 3.2 Mixer Quad-X

Configuration moteurs en X avec sens de rotation alternés :

```
        AVANT
    M4(CW)   M1(CCW)
        \   /
         \ /
          X
         / \
        /   \
    M3(CCW)  M2(CW)
       ARRIÈRE
```

**Matrice de mixage :**

| Moteur | Position | Sens | Throttle | Pitch | Roll | Yaw |
|--------|----------|------|----------|-------|------|-----|
| M1 | Avant-Droit | CCW | + | **-** | **+** | **-** |
| M2 | Arrière-Droit | CW | + | **+** | **+** | **+** |
| M3 | Arrière-Gauche | CCW | + | **+** | **-** | **-** |
| M4 | Avant-Gauche | CW | + | **-** | **-** | **+** |

```c
// src/motors.cpp:49-52
int esc_1_calc = throttle - drone->pid_output_pitch + drone->pid_output_roll - drone->pid_output_yaw;
int esc_2_calc = throttle + drone->pid_output_pitch + drone->pid_output_roll + drone->pid_output_yaw;
int esc_3_calc = throttle + drone->pid_output_pitch - drone->pid_output_roll - drone->pid_output_yaw;
int esc_4_calc = throttle - drone->pid_output_pitch - drone->pid_output_roll + drone->pid_output_yaw;
```

### 3.3 Gestion de Saturation Intelligente

Lorsqu'un moteur dépasse `MAX_THROTTLE_FLIGHT`, tous les moteurs sont réduits proportionnellement pour **préserver le différentiel** (et donc la maniabilité) :

```c
// src/motors.cpp:54-64
// Trouver le moteur le plus sollicité
int max_val = esc_1_calc;
if(esc_2_calc > max_val) max_val = esc_2_calc;
if(esc_3_calc > max_val) max_val = esc_3_calc;
if(esc_4_calc > max_val) max_val = esc_4_calc;

// Si saturation : réduire TOUS les moteurs du même montant
if(max_val > MAX_THROTTLE_FLIGHT) {
    int diff = max_val - MAX_THROTTLE_FLIGHT;
    esc_1_calc -= diff;
    esc_2_calc -= diff;
    esc_3_calc -= diff;
    esc_4_calc -= diff;
}
```

**Avantage :** Même à plein gaz, si le PID demande une correction, le différentiel entre moteurs est maintenu, permettant au drone de rester contrôlable.

**Bornage final :**
```c
// src/motors.cpp:67-70
esc_1 = constrain(esc_1_calc, MIN_THROTTLE_IDLE, MAX_THROTTLE_FLIGHT);
esc_2 = constrain(esc_2_calc, MIN_THROTTLE_IDLE, MAX_THROTTLE_FLIGHT);
esc_3 = constrain(esc_3_calc, MIN_THROTTLE_IDLE, MAX_THROTTLE_FLIGHT);
esc_4 = constrain(esc_4_calc, MIN_THROTTLE_IDLE, MAX_THROTTLE_FLIGHT);
```

### 3.4 Air Mode / Idle

Les constantes de `config.h` définissent les limites :

```c
// include/config.h:47-49
#define MAX_THROTTLE_FLIGHT 1700  // Limite haute (marge de manoeuvre PID)
#define MIN_THROTTLE_IDLE   1050  // Idle minimum (moteurs tournent toujours)
#define MOTOR_OFF           1000  // Arrêt complet ESC
```

**Comportement :**
- En mode armé (gaz à zéro), les moteurs tournent au minimum `MIN_THROTTLE_IDLE = 1050`
- Permet au PID de corriger l'attitude même gaz coupés ("Air Mode")
- `MOTOR_OFF = 1000` n'est utilisé qu'en mode SAFE (désarmé)

---

## Annexe : Structure de Données Principale

```c
// include/types.h
typedef struct {
    FlightMode current_mode;     // MODE_SAFE, MODE_ARMED, MODE_FLYING...

    // Radio (1000-2000 µs)
    int channel_1;               // Roll
    int channel_2;               // Pitch
    int channel_3;               // Throttle
    int channel_4;               // Yaw

    // IMU - Entrées filtrées
    float gyro_roll_input;       // deg/s (après PT1)
    float gyro_pitch_input;      // deg/s
    float gyro_yaw_input;        // deg/s
    float angle_roll;            // deg (Kalman)
    float angle_pitch;           // deg (Kalman)
    float angle_yaw;             // deg (intégration gyro)

    // PID - Consignes (rate)
    float pid_setpoint_roll;
    float pid_setpoint_pitch;
    float pid_setpoint_yaw;

    // PID - Sorties (mixage)
    float pid_output_roll;
    float pid_output_pitch;
    float pid_output_yaw;

    // Gains (modifiables via Web)
    float p_pitch_roll, i_pitch_roll, d_pitch_roll;
    float p_yaw, i_yaw, d_yaw;
    float ff_pitch_roll, ff_yaw;
    float p_level;               // Outer loop
    float p_heading;             // Heading Hold
} DroneState;
```

---

## Diagramme de flux complet

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              MAIN LOOP (Core 1)                             │
│                                  @ 250 Hz                                   │
└─────────────────────────────────────────────────────────────────────────────┘
                                      │
                                      v
┌─────────────────────────────────────────────────────────────────────────────┐
│  1. imu_update()                                                            │
│     └─> Copie atomique du snapshot IMU (imu_snap -> drone)                  │
│         ├─> gyro_roll/pitch/yaw_input  (filtré PT1)                         │
│         └─> angle_roll/pitch           (fusion Kalman)                      │
└─────────────────────────────────────────────────────────────────────────────┘
                                      │
                                      v
┌─────────────────────────────────────────────────────────────────────────────┐
│  2. pid_compute_setpoints()                                                 │
│     └─> Outer Loop (Angle -> Rate)                                          │
│         setpoint_rate = (stick - angle × p_level) / 3                       │
└─────────────────────────────────────────────────────────────────────────────┘
                                      │
                                      v
┌─────────────────────────────────────────────────────────────────────────────┐
│  3. pid_compute()                                                           │
│     └─> Inner Loop (Rate PID)                                               │
│         output = P×error + I_mem + D×(Δgyro filtered) + FF×stick            │
└─────────────────────────────────────────────────────────────────────────────┘
                                      │
                                      v
┌─────────────────────────────────────────────────────────────────────────────┐
│  4. motors_mix()                                                            │
│     ├─> Matrice Quad-X                                                      │
│     ├─> Gestion saturation (préserve différentiel)                          │
│     └─> Bornage [MIN_THROTTLE_IDLE, MAX_THROTTLE_FLIGHT]                    │
└─────────────────────────────────────────────────────────────────────────────┘
                                      │
                                      v
┌─────────────────────────────────────────────────────────────────────────────┐
│  5. motors_write()                                                          │
│     └─> LEDC PWM 14-bit @ 250 Hz -> ESC 1-4                                 │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

*Document généré le 2025-01-22 - Projet Drone UMons*
