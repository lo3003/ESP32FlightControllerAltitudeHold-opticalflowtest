#ifndef YAW_FUSION_H
#define YAW_FUSION_H

#include "types.h"

/**
 * Filtre complémentaire pour fusion Yaw
 * Combine le gyroscope (court terme, réactif) et le magnétomètre (long terme, stable)
 *
 * Coefficient alpha = 0.98 : 98% gyro, 2% magnétomètre
 */

// Initialisation du filtre de fusion (appelé une fois au démarrage)
void yaw_fusion_init();

// Mise à jour du filtre complémentaire
// - drone: pointeur vers l'état du drone (lit gyro_yaw_input et alt_angle_yaw, écrit angle_yaw)
// - dt_s: delta time en secondes
void yaw_fusion_update(DroneState *drone, float dt_s);

// Reset de l'angle fusionné (réinitialise avec la valeur du magnétomètre)
void yaw_fusion_reset(DroneState *drone);

#endif
