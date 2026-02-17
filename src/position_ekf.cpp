#include "position_ekf.h"
#include "config.h"
#include <math.h>

// --- EKF 2D Position+Velocity ---
// State: [pos_x, pos_y, vel_x, vel_y] (cm, cm/s)

static float state[4] = {0};
static float P[4][4];
static float Q_accel = EKF_Q_ACCEL;    // Process noise (cm/s^2)^2
static float R_base  = EKF_R_BASE;     // Measurement noise base (cm/s)^2

static void init_covariance() {
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            P[i][j] = 0.0f;
    P[0][0] = 100.0f;  // pos_x uncertainty
    P[1][1] = 100.0f;  // pos_y uncertainty
    P[2][2] = 50.0f;   // vel_x uncertainty
    P[3][3] = 50.0f;   // vel_y uncertainty
}

void ekf_init() {
    for (int i = 0; i < 4; i++) state[i] = 0.0f;
    init_covariance();
}

void ekf_reset() {
    for (int i = 0; i < 4; i++) state[i] = 0.0f;
    init_covariance();
}

void ekf_predict(float accel_world_x, float accel_world_y, float dt) {
    float dt2 = dt * dt;

    // Accel Deadband : sous le seuil de bruit, forcer à zéro
    // Evite d'injecter un biais constant dans la position au repos
    const float ACCEL_DEADBAND = 4.0f;  // cm/s^2 (~0.004 G)
    if (fabsf(accel_world_x) < ACCEL_DEADBAND) accel_world_x = 0.0f;
    if (fabsf(accel_world_y) < ACCEL_DEADBAND) accel_world_y = 0.0f;

    // Friction de position : si la vitesse estimée est très faible,
    // ne pas laisser l'accélération résiduelle déplacer la position
    const float VEL_FRICTION_THRESH = 2.0f;  // cm/s
    bool vel_near_zero = (fabsf(state[2]) < VEL_FRICTION_THRESH) &&
                         (fabsf(state[3]) < VEL_FRICTION_THRESH);

    // 1. State prediction
    if (vel_near_zero && accel_world_x == 0.0f && accel_world_y == 0.0f) {
        // Vitesse et accélération quasi-nulles : geler la position
        // Seule la covariance évolue (l'EKF garde sa capacité de correction)
    } else {
        state[0] += state[2] * dt + 0.5f * accel_world_x * dt2;  // pos_x
        state[1] += state[3] * dt + 0.5f * accel_world_y * dt2;  // pos_y
    }
    state[2] += accel_world_x * dt;                            // vel_x
    state[3] += accel_world_y * dt;                            // vel_y

    // 2. Covariance prediction (simplified)
    // Propagate cross-terms first (before adding noise)
    P[0][0] += 2.0f * P[0][2] * dt + P[2][2] * dt2;
    P[1][1] += 2.0f * P[1][3] * dt + P[3][3] * dt2;
    P[0][2] += P[2][2] * dt;
    P[1][3] += P[3][3] * dt;
    P[2][0] = P[0][2];
    P[3][1] = P[1][3];

    // Add process noise
    float dt4_4 = Q_accel * dt2 * dt2 / 4.0f;
    float dt2_q = Q_accel * dt2;
    P[0][0] += dt4_4;
    P[1][1] += dt4_4;
    P[2][2] += dt2_q;
    P[3][3] += dt2_q;
}

void ekf_correct(float flow_vel_x, float flow_vel_y, int quality) {
    // 1. R dynamique based on quality
    float r = R_base;
    if (quality > 0) {
        r = R_base * (200.0f / (float)quality);
    }
    if (quality < FLOW_QUALITY_MIN) {
        r = 1e6f;  // Bad quality -> ignore measurement
    }

    // 2. Innovation
    float innov_x = flow_vel_x - state[2];
    float innov_y = flow_vel_y - state[3];

    // 3. Kalman gain (simplified, ignoring cross-axis terms)
    // S = H * P * H' + R -> S00 = P[2][2] + r, S11 = P[3][3] + r
    float S00 = P[2][2] + r;
    float S11 = P[3][3] + r;

    // Avoid division by zero
    if (S00 < 1e-6f) S00 = 1e-6f;
    if (S11 < 1e-6f) S11 = 1e-6f;

    // K = P * H' * S^-1
    float K00 = P[0][2] / S00;  // pos_x correction from vel_x
    float K10 = P[1][3] / S11;  // pos_y correction from vel_y
    float K20 = P[2][2] / S00;  // vel_x correction from vel_x
    float K31 = P[3][3] / S11;  // vel_y correction from vel_y

    // 4. State correction
    state[0] += K00 * innov_x;
    state[1] += K10 * innov_y;
    state[2] += K20 * innov_x;
    state[3] += K31 * innov_y;

    // 5. Covariance correction
    P[0][0] -= K00 * P[2][0];
    P[0][2] -= K00 * P[2][2];
    P[2][0] = P[0][2];
    P[2][2] -= K20 * P[2][2];

    P[1][1] -= K10 * P[3][1];
    P[1][3] -= K10 * P[3][3];
    P[3][1] = P[1][3];
    P[3][3] -= K31 * P[3][3];
}

void ekf_get_state(float *pos_x, float *pos_y, float *vel_x, float *vel_y) {
    *pos_x = state[0];
    *pos_y = state[1];
    *vel_x = state[2];
    *vel_y = state[3];
}
