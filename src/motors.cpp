#include <Arduino.h>
#include "motors.h"
#include "config.h"

int esc_1, esc_2, esc_3, esc_4;

#define PWM_CH1 0
#define PWM_CH2 1
#define PWM_CH3 2
#define PWM_CH4 3

void motors_init() {
    ledcSetup(PWM_CH1, ESC_FREQ, 14);
    ledcSetup(PWM_CH2, ESC_FREQ, 14);
    ledcSetup(PWM_CH3, ESC_FREQ, 14);
    ledcSetup(PWM_CH4, ESC_FREQ, 14);

    ledcAttachPin(PIN_MOTOR_1, PWM_CH1);
    ledcAttachPin(PIN_MOTOR_2, PWM_CH2);
    ledcAttachPin(PIN_MOTOR_3, PWM_CH3);
    ledcAttachPin(PIN_MOTOR_4, PWM_CH4);

    motors_stop();
}

void motors_stop() {
    esc_1 = MOTOR_OFF; 
    esc_2 = MOTOR_OFF; 
    esc_3 = MOTOR_OFF; 
    esc_4 = MOTOR_OFF;
    motors_write(); 
}

void motors_mix(DroneState *drone) {
    int throttle;

    if (drone->current_mode == MODE_ALT_HOLD || drone->current_mode == MODE_AUTO_POS) {
        // --- MODE ALT HOLD : utiliser throttle_command calculé par le PID altitude ---
        int raw = drone->throttle_command;
        if (raw < 1000) raw = 1000;
        if (raw > 2000) raw = 2000;
        throttle = map(raw, 1000, 2000, MIN_THROTTLE_IDLE, MAX_THROTTLE_FLIGHT);
    } else {
        // --- MODE MANUEL : utiliser le stick gaz (channel 3) ---
        int raw_throttle = drone->channel_3;
        if (raw_throttle < 1000) raw_throttle = 1000;
        if (raw_throttle > 2000) raw_throttle = 2000;
        throttle = map(raw_throttle, 1000, 2000, MIN_THROTTLE_IDLE, MAX_THROTTLE_FLIGHT);
    }

    // Mixage PID selon configuration:
    // M1 (IO27) = Avant-Droit CCW  : PITCH(-) ROLL(+) YAW(-)
    // M2 (IO13) = Arrière-Droit CW : PITCH(+) ROLL(+) YAW(+)
    // M3 (IO25) = Arrière-Gauche CCW : PITCH(+) ROLL(-) YAW(-)
    // M4 (IO26) = Avant-Gauche CW  : PITCH(-) ROLL(-) YAW(+)
    
    int esc_1_calc = throttle - drone->pid_output_pitch + drone->pid_output_roll - drone->pid_output_yaw; 
    int esc_2_calc = throttle + drone->pid_output_pitch + drone->pid_output_roll + drone->pid_output_yaw; 
    int esc_3_calc = throttle + drone->pid_output_pitch - drone->pid_output_roll - drone->pid_output_yaw; 
    int esc_4_calc = throttle - drone->pid_output_pitch - drone->pid_output_roll + drone->pid_output_yaw; 

    // Saturation (Pour ne pas dépasser le max possible)
    int max_val = esc_1_calc;
    if(esc_2_calc > max_val) max_val = esc_2_calc;
    if(esc_3_calc > max_val) max_val = esc_3_calc;
    if(esc_4_calc > max_val) max_val = esc_4_calc;

    if(max_val > MAX_THROTTLE_FLIGHT) {
        int diff = max_val - MAX_THROTTLE_FLIGHT;
        esc_1_calc -= diff; esc_2_calc -= diff;
        esc_3_calc -= diff; esc_4_calc -= diff;
    }

    // Application des bornes finales
    esc_1 = constrain(esc_1_calc, MIN_THROTTLE_IDLE, MAX_THROTTLE_FLIGHT);
    esc_2 = constrain(esc_2_calc, MIN_THROTTLE_IDLE, MAX_THROTTLE_FLIGHT);
    esc_3 = constrain(esc_3_calc, MIN_THROTTLE_IDLE, MAX_THROTTLE_FLIGHT);
    esc_4 = constrain(esc_4_calc, MIN_THROTTLE_IDLE, MAX_THROTTLE_FLIGHT);
}

void motors_write() {
    ledcWrite(PWM_CH1, (esc_1 * 16383) / 4000);
    ledcWrite(PWM_CH2, (esc_2 * 16383) / 4000);
    ledcWrite(PWM_CH3, (esc_3 * 16383) / 4000);
    ledcWrite(PWM_CH4, (esc_4 * 16383) / 4000);
}

void motors_write_direct(int m1, int m2, int m3, int m4) {
    esc_1 = m1; esc_2 = m2; esc_3 = m3; esc_4 = m4;
    motors_write();
}