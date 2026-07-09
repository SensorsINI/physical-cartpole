#ifndef SECLOC_CONTROLLER_H
#define SECLOC_CONTROLLER_H

#include "controller_api.h"
#include "secloc.h"
#include "secloc_inner.h"

extern const ControllerOps SECLOC_Ops;

/* Override the gate parameters baked into secloc_controller.c (defaults:
 * log_base 1.05, dead bands 0). Call before or after SECLOC_Ops.init. */
void secloc_controller_set_config(float log_base, float ang_dead_band, float pos_dead_band);

/* Read-only view of the gate state (last shifts, last_Q) for tests/telemetry. */
const SeclocState* secloc_controller_get_state(void);

#endif /* SECLOC_CONTROLLER_H */
