#ifndef SETUP_WIZARD_H
#define SETUP_WIZARD_H

// Appelé au démarrage ou quand on entre dans la page Setup
void setup_wizard_init();

// Appelé par le bouton "Sauver Centres"
void step_save_center();

// Active/Désactive l'enregistrement des Min/Max
void step_set_recording_minmax(bool active);

// À mettre dans la loop() pour scanner les sticks
void setup_loop_monitor();

// Appelé par le bouton "Terminer"
void step_save_eeprom();

#endif