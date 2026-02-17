#include <Arduino.h>
#include <EEPROM.h>
#include "setup_wizard.h"
#include "radio.h" // Pour accéder à raw_channel_x
#include "config.h"

// Variables de stockage temporaire
int temp_center[5];
int temp_min[5];
int temp_max[5];
bool recording_minmax = false;

void setup_wizard_init() {
    // Valeurs par défaut sécurisées
    for(int i=1; i<=4; i++) {
        temp_center[i] = 1500;
        temp_min[i] = 1100;
        temp_max[i] = 1900;
    }
    recording_minmax = false;
    Serial.println(">>> Web Setup Initialisé");
}

void step_save_center() {
    // On capture l'instant T
    radio_read_raw();
    temp_center[1] = raw_channel_1;
    temp_center[2] = raw_channel_2;
    temp_center[3] = raw_channel_3;
    temp_center[4] = raw_channel_4;
    Serial.println("Web Setup: Centres capturés");
}

void step_set_recording_minmax(bool active) {
    recording_minmax = active;
    if(active) {
        // Reset des min/max pour commencer la capture
        for(int i=1; i<=4; i++) {
            temp_min[i] = temp_center[i]; 
            temp_max[i] = temp_center[i];
        }
        Serial.println("Web Setup: Enregistrement Min/Max STARTED");
    } else {
        Serial.println("Web Setup: Enregistrement Min/Max STOPPED");
    }
}

// Cette fonction doit tourner en permanence dans la loop() si MODE_SETUP
void setup_loop_monitor() {
    if(!recording_minmax) return;

    radio_read_raw(); // Lecture fraîche

    int channels[] = {0, raw_channel_1, raw_channel_2, raw_channel_3, raw_channel_4};

    for(int i=1; i<=4; i++) {
        if(channels[i] < temp_min[i]) temp_min[i] = channels[i];
        if(channels[i] > temp_max[i]) temp_max[i] = channels[i];
    }
}

void step_save_eeprom() {
    // Ordre des canaux S.BUS standard (AETR : 1=Roll, 2=Pitch, 3=Thr, 4=Yaw)
    // On force l'assignation standard pour simplifier le web setup
    byte assign_1 = 1; 
    byte assign_2 = 2; 
    byte assign_3 = 3; 
    byte assign_4 = 4;

    // Détection inversion automatique (si le centre n'est pas au milieu du min/max)
    // Ici on suppose que le S.BUS est propre. On garde les flags d'inversion à 0 par défaut.
    // Si besoin d'inversion, on peut l'ajouter dans l'interface web plus tard.
    
    Serial.println(F("Sauvegarde EEPROM..."));
    
    // Centres
    EEPROM.write(0, temp_center[1] & 0xFF); EEPROM.write(1, temp_center[1] >> 8);
    EEPROM.write(2, temp_center[2] & 0xFF); EEPROM.write(3, temp_center[2] >> 8);
    EEPROM.write(4, temp_center[3] & 0xFF); EEPROM.write(5, temp_center[3] >> 8);
    EEPROM.write(6, temp_center[4] & 0xFF); EEPROM.write(7, temp_center[4] >> 8);
    
    // Max
    EEPROM.write(8, temp_max[1] & 0xFF); EEPROM.write(9, temp_max[1] >> 8);
    EEPROM.write(10, temp_max[2] & 0xFF); EEPROM.write(11, temp_max[2] >> 8);
    EEPROM.write(12, temp_max[3] & 0xFF); EEPROM.write(13, temp_max[3] >> 8);
    EEPROM.write(14, temp_max[4] & 0xFF); EEPROM.write(15, temp_max[4] >> 8);
    
    // Min
    EEPROM.write(16, temp_min[1] & 0xFF); EEPROM.write(17, temp_min[1] >> 8);
    EEPROM.write(18, temp_min[2] & 0xFF); EEPROM.write(19, temp_min[2] >> 8);
    EEPROM.write(20, temp_min[3] & 0xFF); EEPROM.write(21, temp_min[3] >> 8);
    EEPROM.write(22, temp_min[4] & 0xFF); EEPROM.write(23, temp_min[4] >> 8);
    
    // Assignation (Hardcodée Standard)
    EEPROM.write(24, assign_1);
    EEPROM.write(25, assign_2);
    EEPROM.write(26, assign_3);
    EEPROM.write(27, assign_4);
    
    // Config Gyro (On garde tes valeurs MPU6050)
    EEPROM.write(28, 1); // Roll Axis
    EEPROM.write(29, 2); // Pitch Axis
    EEPROM.write(30, 3); // Yaw Axis
    EEPROM.write(31, 1); // Type
    EEPROM.write(32, GYRO_ADDRESS); 
    
    // Signature
    EEPROM.write(33, 'J'); EEPROM.write(34, 'M'); EEPROM.write(35, 'B');
    
    EEPROM.commit();
    Serial.println(F("EEPROM Sauvegardée !"));
}