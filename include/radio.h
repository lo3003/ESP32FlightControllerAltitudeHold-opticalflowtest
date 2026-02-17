#ifndef RADIO_H
#define RADIO_H
#include "types.h"

// Variables accessibles par le setup wizard
extern int raw_channel_1, raw_channel_2, raw_channel_3, raw_channel_4;
extern int raw_channel_5;  // AUX1 (Canal 5)
extern int raw_channel_6;  // AUX2 (Canal 6)

void radio_init();
void radio_start_task();              // <-- AJOUT: task RX indépendante
void radio_read_raw();
void radio_update(DroneState *drone); // (va devenir une simple copie)
int convert_receiver_channel(byte function);

#endif