#ifndef SECLOC_CONTROLLER_H
#define SECLOC_CONTROLLER_H

#include "controller_api.h"
#include "secloc_logic.h"
#include "secloc_inner.h"

/*
 * SecLoc controller core: gate config/state, inner controller dispatch and
 * the SW (PS) evaluate path. The optional PL backend (registration, backend
 * selection, shadow-mismatch counter) is a separate module; see
 * secloc_controller_pl.h. This keeps secloc_controller.c free of platform
 * includes and of any PL knowledge beyond the internal hook points.
 */

extern const ControllerOps SECLOC_Ops;

/* Override the gate parameters baked into secloc_defaults.h. Call any time;
 * the PC driver pushes its config_secloc.yml values through this at startup
 * and whenever the yaml changes (CMD_SET_SECLOC_CONFIG). ref_period_ticks is
 * the throttle in control loop iterations (0 or 1 = gate checked every
 * iteration). */
void secloc_controller_set_config(
    float log_base,
    int32_t ref_period_ticks,
    float ang_dead_band,
    float pos_dead_band
);

/* Tick size of the ref_period throttle; must equal the control loop period.
 * control.c wires this from POLLING_PERIOD_MS. */
void secloc_controller_set_time_quantum(float time_quantum_s);

/* Read-only view of the gate state (last shifts, last_Q) for tests/telemetry. */
const SeclocState* secloc_controller_get_state(void);

/* Gate decision of the most recent SECLOC_Evaluate, packed for the state
 * packet: bit 0 = secloc_skipped_update, bit 1 = secloc_gate_skipped
 * (matches the Python CSV columns), bit 2 = the step was computed by the PL
 * backend (secloc_shell + gate + marshal chain in the FPGA fabric). */
uint8_t secloc_controller_telemetry_flags(void);

#endif /* SECLOC_CONTROLLER_H */
