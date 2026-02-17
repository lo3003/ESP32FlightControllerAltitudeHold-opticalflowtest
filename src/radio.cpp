#include <Arduino.h>
#include "radio.h"
#include "config.h"
#include "sbus.h"

// --- AJOUT FreeRTOS ---
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

int raw_channel_1 = 1500, raw_channel_2 = 1500, raw_channel_3 = 0, raw_channel_4 = 1500;
int raw_channel_5 = 1000;   // AUX1 (Switch Altitude Hold)
int raw_channel_6 = 1000;   // AUX2 (Switch Position Hold)

bfs::SbusRx sbus(&Serial2, PIN_SBUS_RX, -1, true);
bfs::SbusData data;

// --- AJOUT: protection + timestamp ---
static portMUX_TYPE radio_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile unsigned long radio_last_ok_ms = 0;

// --- AJOUT: task handle ---
static TaskHandle_t radio_task_handle = nullptr;

void radio_init() {
    sbus.Begin();
}

// Fonction magique qui "écrase" les bords pour garantir 1000 et 2000
// MODIFICATION SECURITE : Retourne -1 si l'entrée est invalide (glitch à 0)
int process_channel(int input_sbus, bool reverse) {
    // --- SECURITE ANTI-GLITCH ---
    // Si la valeur S.BUS est proche de 0 (ex: câble débranché ou glitch), on rejette !
    // Une valeur S.BUS valide est généralement > 172.
    if (input_sbus < 50) {
        return -1; // Code d'erreur
    }

    // VOS VALEURS REELLES : env. 310 à 1690
    // ON TRICHE : On dit que le min est 360 et le max 1640 pour être sûr d'atteindre les bouts.
    int min_safe = 360;
    int max_safe = 1640;
    
    // Tout ce qui est en dessous de 360 deviendra 1000 pile.
    // Tout ce qui est au dessus de 1640 deviendra 2000 pile.
    int val = map(input_sbus, min_safe, max_safe, 1000, 2000);
    
    if(reverse) val = 3000 - val; // Inversion (2000+1000 - val)

    // Sécurité bornes ultimes
    return constrain(val, 1000, 2000);
}

void radio_read_raw() {
    bool new_data = false;
    while (sbus.Read()) {
        if (sbus.data().failsafe || sbus.data().lost_frame) {
            continue;
        }
        data = sbus.data();
        new_data = true;
    }

    if (new_data) {
        int t_roll  = process_channel(data.ch[3], false);
        int t_pitch = process_channel(data.ch[1], true);
        int t_thr   = process_channel(data.ch[2], false);
        int t_yaw   = process_channel(data.ch[0], false);
        int t_aux1  = process_channel(data.ch[PIN_AUX1_CHANNEL - 1], false); // Canal 5 (index 4)
        int t_aux2  = process_channel(data.ch[PIN_AUX2_CHANNEL - 1], false); // Canal 6 (index 5)

        if (t_roll != -1 && t_pitch != -1 && t_thr != -1 && t_yaw != -1) {
            portENTER_CRITICAL(&radio_mux);
            raw_channel_1 = t_roll;
            raw_channel_2 = t_pitch;
            raw_channel_3 = t_thr;
            raw_channel_4 = t_yaw;
            if (t_aux1 != -1) raw_channel_5 = t_aux1;
            if (t_aux2 != -1) raw_channel_6 = t_aux2;
            radio_last_ok_ms = millis();
            portEXIT_CRITICAL(&radio_mux);
        }
    }
}

// --- AJOUT: task radio (tourne même si IMU lag) ---
static void radio_task(void *parameter) {
    (void)parameter;
    for (;;) {
        radio_read_raw();
        // 1 tick ~ 1ms (suffisant pour vider le buffer S.BUS)
        vTaskDelay(1);
    }
}

void radio_start_task() {
    if (radio_task_handle != nullptr) return;

    // Core 0 pour ne pas dépendre de la loop() (souvent sur core 1)
    xTaskCreatePinnedToCore(
        radio_task,
        "radio_rx",
        4096,
        nullptr,
        3,                 // priorité > loop
        &radio_task_handle,
        0                  // core 0
    );
}

void radio_update(DroneState *drone) {
    // IMPORTANT: on ne lit plus le S.BUS ici (c’est la task qui le fait)
    // On copie juste la dernière valeur valide.
    int c1, c2, c3, c4, c5, c6;
    unsigned long last_ok;

    portENTER_CRITICAL(&radio_mux);
    c1 = raw_channel_1; c2 = raw_channel_2; c3 = raw_channel_3; c4 = raw_channel_4;
    c5 = raw_channel_5;
    c6 = raw_channel_6;
    last_ok = radio_last_ok_ms;
    portEXIT_CRITICAL(&radio_mux);

    // Si jamais aucune trame valide n’a encore été reçue
    if (c3 == 0) {
        drone->channel_3 = 0;
        return;
    }

    // Optionnel: si la radio devient stale, vous pouvez décider quoi faire
    // (ici: on garde les dernières valeurs)
    (void)last_ok;

    drone->channel_1 = c1;
    drone->channel_2 = c2;
    drone->channel_3 = c3;
    drone->channel_4 = c4;
    drone->channel_5 = c5;
    drone->channel_6 = c6;
}

int convert_receiver_channel(byte function) { return 1500; }