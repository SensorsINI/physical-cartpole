#ifndef SECLOC_H
#define SECLOC_H

#include <stdint.h>

/*
 * Sparse Event-Based Closed-Loop Control (SecLoc) gate.
 *
 * Decides whether a fresh controller computation is due, or whether the
 * previous control value should be held. Matches the Python SeclocGate logic
 * with ref_period = 0 (no timing input on the on-chip Controller API):
 *   - angle and position are checked independently; either can fire,
 *   - the angle shift is measured from the active target equilibrium
 *     (|angle| for target up, pi - |angle| for target down),
 *   - on update only the reference of the axis (or axes) that fired
 *     is refreshed.
 */

typedef struct {
    float log_base;
    float ang_dead_band;
    float pos_dead_band;
} SeclocConfig;

typedef struct {
    float ang_last_shift;
    float pos_last_shift;
    float last_Q;
    uint8_t has_init;
} SeclocState;

void secloc_reset(SeclocState* state);

/* Angle distance from the active target equilibrium (te > 0: up, te < 0: down). */
float secloc_angle_shift_from_target(float a, float te);

/* Returns 1 when a new control sample should be computed, 0 to hold last_Q. */
int secloc_should_sample(
    SeclocState* state,
    const SeclocConfig* config,
    float p,
    float pd,
    float a,
    float ad,
    float tp,
    float te
);

#endif /* SECLOC_H */
