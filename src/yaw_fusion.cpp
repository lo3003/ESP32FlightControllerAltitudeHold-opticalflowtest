#include <Arduino.h>
#include "yaw_fusion.h"

// ==================== CONFIGURATION ====================
// Coefficient du filtre complémentaire
// alpha = 0.98 : 98% gyro (réactif, court terme), 2% magnétomètre (stable, long terme)
#define YAW_FUSION_ALPHA 0.995f

// --- DETECTION VALEURS ABERRANTES ---
// Seuil max de changement d'angle par cycle (deg) - au-delà c'est considéré comme aberrant
// À 250Hz, le drone ne peut physiquement pas tourner de plus de 2000°/s = 8°/cycle
#define YAW_MAX_DELTA_PER_CYCLE     10.0f   // deg (tolérance large)

// Seuil max de rate gyro plausible (deg/s)
#define YAW_MAX_GYRO_RATE           800.0f  // deg/s (un drone ne tourne pas plus vite)

// Seuil de différence mag/gyro pour détecter une perturbation magnétomètre
#define YAW_MAG_OUTLIER_THRESHOLD   45.0f   // deg (si le mag diffère trop du gyro, c'est suspect)

// Nombre de cycles consécutifs aberrants avant d'ignorer le magnétomètre
#define YAW_OUTLIER_COUNT_LIMIT     10

// Coefficient filtre passe-bas supplémentaire pour le yaw final
#define YAW_OUTPUT_FILTER_COEFF     0.02f   // Plus c'est bas, plus c'est lissé

// ==================== VARIABLES D'ETAT ====================
static float fused_yaw = 0.0f;          // Angle yaw fusionné (0-360°)
static float fused_yaw_filtered = 0.0f; // Yaw filtré pour l'affichage/régulation
static float last_fused_yaw = 0.0f;     // Mémorisation pour détection de saut
static bool fusion_initialized = false;

// Compteur de valeurs aberrantes consécutives
static int mag_outlier_count = 0;

// Flag: magnétomètre considéré comme fiable
static bool mag_reliable = true;

// ==================== FONCTIONS UTILITAIRES ====================

/**
 * Normalise un angle entre 0 et 360 degrés
 */
static float normalize_angle_360(float angle) {
    while (angle < 0.0f) angle += 360.0f;
    while (angle >= 360.0f) angle -= 360.0f;
    return angle;
}

/**
 * Calcule la différence angulaire en tenant compte du wrap-around
 * Retourne une valeur entre -180 et +180
 */
static float angle_diff(float target, float current) {
    float diff = target - current;

    // Gérer le wrap-around
    if (diff > 180.0f) diff -= 360.0f;
    if (diff < -180.0f) diff += 360.0f;

    return diff;
}

/**
 * Vérifie si le rate gyro est plausible
 */
static bool is_gyro_rate_valid(float gyro_rate) {
    return (fabsf(gyro_rate) < YAW_MAX_GYRO_RATE);
}

/**
 * Vérifie si le changement d'angle est plausible
 */
static bool is_delta_valid(float delta) {
    return (fabsf(delta) < YAW_MAX_DELTA_PER_CYCLE);
}

// ==================== API PUBLIQUE ====================

void yaw_fusion_init() {
    fused_yaw = 0.0f;
    fused_yaw_filtered = 0.0f;
    last_fused_yaw = 0.0f;
    fusion_initialized = false;
    mag_outlier_count = 0;
    mag_reliable = true;
}

void yaw_fusion_update(DroneState *drone, float dt_s) {
    // Vérifier que le delta time est valide
    if (dt_s <= 0.0f || dt_s > 0.1f) {
        dt_s = 0.004f;  // Valeur par défaut (250Hz)
    }

    // Récupérer les entrées
    float gyro_rate = drone->gyro_yaw_input;  // deg/s du MPU6050
    float mag_heading = drone->alt_angle_yaw; // Heading magnétomètre (0-360°)

    // ========== VALIDATION DU GYRO ==========
    // Si le gyro donne une valeur aberrante, on le limite
    if (!is_gyro_rate_valid(gyro_rate)) {
        // Limiter à une valeur plausible (garder le signe)
        gyro_rate = (gyro_rate > 0.0f) ? YAW_MAX_GYRO_RATE : -YAW_MAX_GYRO_RATE;
    }

    // Première exécution : initialiser avec le magnétomètre
    if (!fusion_initialized) {
        fused_yaw = mag_heading;
        fused_yaw_filtered = mag_heading;
        last_fused_yaw = mag_heading;
        fusion_initialized = true;
        drone->angle_yaw = fused_yaw_filtered;
        return;
    }

    // ========== FILTRE COMPLEMENTAIRE ==========

    // 1. Prédiction par intégration gyro (court terme)
    float gyro_prediction = fused_yaw + gyro_rate * dt_s;
    gyro_prediction = normalize_angle_360(gyro_prediction);

    // 2. Détection d'anomalie du magnétomètre
    // Calculer l'erreur angulaire entre mag et prédiction gyro
    float mag_vs_gyro_error = angle_diff(mag_heading, gyro_prediction);

    // Si le magnétomètre diffère beaucoup de la prédiction gyro, c'est suspect
    if (fabsf(mag_vs_gyro_error) > YAW_MAG_OUTLIER_THRESHOLD) {
        mag_outlier_count++;
        if (mag_outlier_count > YAW_OUTLIER_COUNT_LIMIT) {
            mag_reliable = false;  // Trop d'erreurs consécutives, ignorer le mag
        }
    } else {
        // Valeur cohérente, reset du compteur
        if (mag_outlier_count > 0) mag_outlier_count--;
        if (mag_outlier_count == 0) mag_reliable = true;
    }

    // 3. Fusion adaptative
    float new_fused_yaw;
    if (mag_reliable) {
        // Fusion normale : alpha * gyro + (1-alpha) * correction magnétomètre
        new_fused_yaw = gyro_prediction + (1.0f - YAW_FUSION_ALPHA) * mag_vs_gyro_error;
    } else {
        // Magnétomètre non fiable : utiliser uniquement le gyro (avec dérive possible)
        new_fused_yaw = gyro_prediction;
    }

    // Normaliser le résultat
    new_fused_yaw = normalize_angle_360(new_fused_yaw);

    // ========== DETECTION DE SAUT ABERRANT ==========
    float delta_from_last = angle_diff(new_fused_yaw, last_fused_yaw);

    if (!is_delta_valid(delta_from_last)) {
        // Saut trop grand : c'est probablement une erreur
        // On garde la dernière valeur valide + une petite correction basée sur le gyro
        new_fused_yaw = normalize_angle_360(last_fused_yaw + gyro_rate * dt_s);
    }

    // Mettre à jour le yaw fusionné
    fused_yaw = new_fused_yaw;
    last_fused_yaw = fused_yaw;

    // ========== FILTRE PASSE-BAS DE SORTIE ==========
    // Lissage final pour éviter les micro-oscillations à l'affichage
    float delta_filtered = angle_diff(fused_yaw, fused_yaw_filtered);
    fused_yaw_filtered = normalize_angle_360(fused_yaw_filtered + delta_filtered * YAW_OUTPUT_FILTER_COEFF);

    // Stocker dans le drone state
    drone->angle_yaw = fused_yaw_filtered;
}

void yaw_fusion_reset(DroneState *drone) {
    // Réinitialiser avec la valeur actuelle du magnétomètre
    fused_yaw = drone->alt_angle_yaw;
    fused_yaw_filtered = fused_yaw;
    last_fused_yaw = fused_yaw;
    fusion_initialized = true;
    mag_outlier_count = 0;
    mag_reliable = true;
    drone->angle_yaw = fused_yaw_filtered;
}
