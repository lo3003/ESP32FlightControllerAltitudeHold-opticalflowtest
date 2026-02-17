#include <Arduino.h>
#include "esc_calibrate.h"
#include "motors.h"
#include "config.h"

void esc_calibrate_init() {
    // Rien de spécial, juste un log
    Serial.println(">>> CALIBRATION ESC ACTIVE <<<");
    Serial.println("Le signal Throttle est envoyé directement aux moteurs.");
}

void esc_calibrate_loop(DroneState *drone) {
    // En calibration, on copie bêtement le signal Throttle (Ch3) sur tous les moteurs
    // C'est ce qui permet de faire Haut -> Batterie -> Bip -> Bas -> Bip
    int throt = drone->channel_3;
    
    // Sécurité : On force un range 1000-2000
    if(throt < 1000) throt = 1000;
    if(throt > 2000) throt = 2000;
    
    motors_write_direct(throt, throt, throt, throt);

    
    // Clignotement LED lent pour dire "Je suis en calib"
    if((millis() / 500) % 2) digitalWrite(PIN_LED, HIGH); 
    else digitalWrite(PIN_LED, LOW);
}