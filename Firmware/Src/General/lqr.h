#ifndef LQR_H
#define LQR_H

#include "controller_api.h"

/*
 * Minimal LQR controller (single-output):
 *   state = [ position - target_position, positionD, angle, angleD ]^T
 *   Q = -K · state, clipped to [-1, 1].
 */

typedef struct {
    float K0;
    float K1;
    float K2;
    float K3;
} LqrGains;

float lqr_evaluate(float p, float pd, float a, float ad, float tp, const LqrGains* gains);

extern const LqrGains LQR_DefaultGains;
extern const ControllerOps LQR_Ops;

#endif /* LQR_H */
