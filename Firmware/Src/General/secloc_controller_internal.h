#ifndef SECLOC_CONTROLLER_INTERNAL_H
#define SECLOC_CONTROLLER_INTERNAL_H

/*
 * Internal interface between the SecLoc controller core
 * (secloc_controller.c) and its PL backend module (secloc_controller_pl.c).
 * Not for platform or application code: use secloc_controller.h /
 * secloc_controller_pl.h instead.
 */

#include <stdint.h>

#include "secloc_logic.h"

/* --- implemented by secloc_controller.c ---------------------------------- */

/* Current gate config; the PL module pushes it to freshly registered ops. */
const SeclocConfig* secloc_controller_config(void);

/* --- implemented by secloc_controller_pl.c ------------------------------- */

/* 1 when a PL backend is selected, i.e. the SW path must not produce the
 * control signal this step. */
int secloc_pl_active(void);

/* One SecLoc step on the PL backend. Returns 1 on success (Q/fired/
 * gate_evaluated valid; in PL_SHADOW mode the SW gate in `state` was stepped
 * and compared). Returns 0 on a PL fault (block absent or transaction
 * failed), after incrementing the fault counter; the caller must output zero
 * force and flag the step, never compute it in SW. */
int secloc_pl_step(
    SeclocState* state,
    const SeclocConfig* config,
    float p, float pd, float a, float ad, float tp, float te, float time,
    float* Q, uint8_t* fired, uint8_t* gate_evaluated
);

/* Core -> PL notifications: gate config changed / controller (re)initialized.
 * Init also clears the shadow-mismatch and fault counters. */
void secloc_pl_notify_config(const SeclocConfig* config);
void secloc_pl_notify_init(const SeclocConfig* config);

#endif /* SECLOC_CONTROLLER_INTERNAL_H */
