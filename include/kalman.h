#ifndef KALMAN_H
#define KALMAN_H

class Kalman {
public:
    Kalman();
    
    // Configure le filtre (optionnel, valeurs par défaut OK)
    void setAngle(float angle);
    void setQangle(float Q_angle);
    void setQbias(float Q_bias);
    void setRmeasure(float R_measure);
    
    // Fonction principale : retourne l'angle filtré
    // angle_acc = angle mesuré par accéléromètre (degrés)
    // gyro_rate = vitesse angulaire gyro (degrés/seconde)
    // dt = delta time en secondes
    float update(float angle_acc, float gyro_rate, float dt);
    
    // Getters
    float getAngle() const { return angle; }
    float getBias() const { return bias; }
    float getRate() const { return rate; }

private:
    // État estimé
    float angle;  // Angle estimé (degrés)
    float bias;   // Biais gyro estimé (degrés/s)
    float rate;   // Vitesse angulaire non biaisée (degrés/s)
    
    // Covariance d'erreur (matrice 2x2)
    float P[2][2];
    
    // Paramètres de bruit (à tuner selon ton IMU)
    float Q_angle;   // Variance du bruit de processus (angle)
    float Q_bias;    // Variance du bruit de processus (biais)
    float R_measure; // Variance du bruit de mesure (accéléromètre)
};

#endif