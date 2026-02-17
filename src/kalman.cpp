#include "kalman.h"

Kalman::Kalman() {
    // Valeurs initiales
    angle = 0.0f;
    bias = 0.0f;
    rate = 0.0f;
    
    // Matrice de covariance initiale
    P[0][0] = 0.0f;
    P[0][1] = 0.0f;
    P[1][0] = 0.0f;
    P[1][1] = 0.0f;
    
    // Paramètres de bruit par défaut (tunés pour MPU6050)
    Q_angle   = 0.001f;  // Confiance dans le modèle (gyro)
    Q_bias    = 0.003f;  // Vitesse de dérive du biais
    R_measure = 0.03f;   // Bruit accéléromètre (plus grand = moins de confiance)
}

void Kalman::setAngle(float a) { angle = a; }
void Kalman::setQangle(float q) { Q_angle = q; }
void Kalman::setQbias(float q) { Q_bias = q; }
void Kalman::setRmeasure(float r) { R_measure = r; }

float Kalman::update(float angle_acc, float gyro_rate, float dt) {
    // ========== ÉTAPE 1: PRÉDICTION ==========
    
    // Mise à jour de l'angle avec le gyro (modèle)
    rate = gyro_rate - bias;
    angle += dt * rate;
    
    // Mise à jour de la matrice de covariance P
    // P = A * P * A' + Q
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;
    
    // ========== ÉTAPE 2: MISE À JOUR (CORRECTION) ==========
    
    // Innovation (erreur entre mesure et prédiction)
    float y = angle_acc - angle;
    
    // Covariance de l'innovation
    float S = P[0][0] + R_measure;
    
    // Gain de Kalman
    float K[2];
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;
    
    // Mise à jour de l'état
    angle += K[0] * y;
    bias  += K[1] * y;
    
    // Mise à jour de la covariance
    float P00_temp = P[0][0];
    float P01_temp = P[0][1];
    
    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;
    
    return angle;
}