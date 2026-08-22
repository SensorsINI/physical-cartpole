#ifndef SECLOC_DEFAULTS_H
#define SECLOC_DEFAULTS_H

#include "secloc_inner.h"

/*
 * Default SecLoc gate profile for on-chip control.
 * Keep in sync with Driver/CartPoleSimulation/Control_Toolkit_ASF/config_secloc.yml
 * profile "default". When the PC driver is connected it overwrites gate values
 * on the chip (CMD_SET_SECLOC_CONFIG) with the current yaml contents, so these
 * defaults only matter when the chip runs standalone.
 *
 * Inner controller: mirrors globals.CONTROLLER_NAME on the PC (e.g. neural-imitator
 * -> SECLOC_INNER_NNC). Change here to swap the wrapped controller on Zynq.
 */
#define SECLOC_DEFAULT_LOG_BASE         1.05f
/* Throttle in control loop iterations (POLLING_PERIOD_MS each): after an
 * accepted update the gate is next consulted this many iterations later.
 * 0 or 1 = gate checked every iteration. 4 x 5 ms = 20 ms. */
#define SECLOC_DEFAULT_REF_PERIOD_TICKS 4
#define SECLOC_DEFAULT_DEAD_ANG         0.001f
#define SECLOC_DEFAULT_DEAD_POS         0.001f

/* Wrapped inner controller when OnChipController_SECLOC is active. */
#define SECLOC_DEFAULT_INNER_CONTROLLER SECLOC_INNER_NNC

/* Tick size of the gate's ref_period throttle. Must equal the control loop
 * period; control.c overrides it from POLLING_PERIOD_MS at init and whenever
 * the PC reconfigures the loop period. */
#define SECLOC_DEFAULT_TIME_QUANTUM_S 0.005f

/* Requested execution backend, applied from boot. This is a request, not a
 * capability probe: selecting a PL backend on a chip where the PL SecLoc
 * chain (secloc_shell + secloc_gate + nn_marshal) is absent or broken makes
 * every step a fault - the controller outputs ZERO force (dead cart, obvious
 * even standalone) and counts it (secloc_pl_fault_count, telemetry flag
 * bit 3). There is no fallback to the SW path. Set SECLOC_BACKEND_SW here
 * for builds that must run without the PL chain.
 *   SECLOC_BACKEND_SW        - gate + inner controller on the CPU
 *   SECLOC_BACKEND_PL        - gate + NN fused in the PL
 *   SECLOC_BACKEND_PL_SHADOW - control from the PL, SW gate stepped in
 *                              parallel and decisions compared
 * (secloc_shadow_mismatch_count). */
#define SECLOC_DEFAULT_BACKEND SECLOC_BACKEND_SW

#endif /* SECLOC_DEFAULTS_H */
