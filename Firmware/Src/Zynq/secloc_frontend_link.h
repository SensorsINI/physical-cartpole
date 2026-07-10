#ifndef SECLOC_FRONTEND_LINK_H
#define SECLOC_FRONTEND_LINK_H

/*
 * MMIO driver for the SecLoc frontend HLS IP
 * (FPGA/CustomIPs/secloc_shell_hls): AXI-Lite shell for the three-block SecLoc
 * chain (shell + gate + marshal), talking to the hls4ml network over AXIS.
 *
 * All functions are safe to call on bitstreams without the IP (they turn
 * into no-ops returning 0 / not-present).
 */

#include <stdint.h>

/* Probe the IP (gate-reset transaction, checks the status version field).
 * Returns 1 when present and answering. Call once at boot before use. */
int SeclocFrontendLink_Init(void);
int SeclocFrontendLink_Present(void);

/* Plain neural-imitator mode: run the NN through the frontend with the gate
 * bypassed (force-compute; gate state untouched). nn_inputs holds the raw
 * (unnormalized) floats in wire order:
 *   angleD, angle_cos, angle_sin, position, positionD,
 *   target_equilibrium, target_position
 * Returns 1 on success, 0 on transaction timeout or network timeout. */
int SeclocFrontendLink_PlainEvaluate(const float nn_inputs[7], float* Q);

/* PL clock cycles the frontend spent waiting for the network during the most
 * recent transaction (0 when the gate skipped); isolates NN latency from the
 * gate + marshalling cost. */
uint32_t SeclocFrontendLink_LastNnWaitCycles(void);

/* update_count register after the most recent transaction (number of NN
 * evaluations since the last gate reset). */
uint32_t SeclocFrontendLink_LastUpdateCount(void);

/* Register the PL backend with secloc_controller (no-op when absent). */
void SeclocFrontendLink_RegisterSeclocBackend(void);

#endif /* SECLOC_FRONTEND_LINK_H */
