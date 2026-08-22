#include "secloc_frontend_link.h"
#include "hw_platform_config.h"

#if HW_HAS_SECLOC_FRONTEND

#include <math.h>
#include <string.h>

#include "xil_io.h"
/* Resolves via the flattened Vitis src tree (src/secloc_controller_pl.h ->
 * Firmware/Src/General/secloc_controller_pl.h), same as ../controller_api.h
 * in neural_imitator.c. */
#include "../secloc_controller_pl.h"

/* Register map: mirrors the Vitis-generated xsecloc_shell_hw.h
 * (FPGA/CustomIPs/secloc_shell_hls/.../drivers/secloc_shell_v1_0/src).
 * Regenerate after changing the HLS top-function signature. */
#define SFE_BASE                 HW_SECLOC_FRONTEND_BASEADDR
#define SFE_AP_CTRL              0x00  /* bit0 ap_start (COH), bit1 ap_done (COR), bit2 ap_idle */
#define SFE_ANGLED               0x10
#define SFE_ANGLE_COS            0x18
#define SFE_ANGLE_SIN            0x20
#define SFE_POSITION             0x28
#define SFE_POSITIOND            0x30
#define SFE_TARGET_EQUILIBRIUM   0x38
#define SFE_TARGET_POSITION      0x40
#define SFE_ANGLE                0x48
#define SFE_TICK                 0x50
#define SFE_LOG_BASE             0x58
#define SFE_REF_PERIOD_TICKS     0x60
#define SFE_DEAD_ANG             0x68
#define SFE_DEAD_POS             0x70
#define SFE_CONTROL_FLAGS        0x78
#define SFE_Q                    0x80
#define SFE_STATUS               0x90
#define SFE_UPDATE_COUNT         0xa0
#define SFE_NN_WAIT_CYCLES       0xb0

/* Mirrors FPGA/CustomIPs/secloc_shell_hls/secloc_shell.h and
 * FPGA/CustomIPs/secloc_stream_protocol.h */
#define SFE_CTRL_GATE_RESET      0x1u
#define SFE_CTRL_FORCE_COMPUTE   0x2u
#define SFE_STATUS_FIRED         0x1u
#define SFE_STATUS_GATE_EVAL     0x2u
#define SFE_STATUS_NN_TIMEOUT    0x8u
#define SFE_EXPECTED_VERSION     2u

/* Worst case is the NN-timeout path: ~4230 PL cycles at 10 MHz = ~0.42 ms.
 * At a ~667 MHz CPU with each MMIO poll taking >= tens of cycles this bound
 * is generous without risking a stuck control loop. */
#define SFE_POLL_LIMIT           2000000u

static int      sfe_present = 0;
static uint32_t sfe_last_status = 0;
static uint32_t sfe_last_nn_wait_cycles = 0;
static uint32_t sfe_last_update_count = 0;

static void sfe_write_float(uint32_t offset, float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    Xil_Out32(SFE_BASE + offset, bits);
}

static float sfe_read_float(uint32_t offset)
{
    uint32_t bits = Xil_In32(SFE_BASE + offset);
    float value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

/* Start one transaction and wait for ap_done (clear-on-read). */
static int sfe_transact(void)
{
    Xil_Out32(SFE_BASE + SFE_AP_CTRL, 0x1u);
    for (uint32_t i = 0; i < SFE_POLL_LIMIT; ++i) {
        if (Xil_In32(SFE_BASE + SFE_AP_CTRL) & 0x2u) {
            sfe_last_status         = Xil_In32(SFE_BASE + SFE_STATUS);
            sfe_last_update_count   = Xil_In32(SFE_BASE + SFE_UPDATE_COUNT);
            sfe_last_nn_wait_cycles = Xil_In32(SFE_BASE + SFE_NN_WAIT_CYCLES);
            return 1;
        }
    }
    return 0;
}

static int sfe_gate_reset(void)
{
    Xil_Out32(SFE_BASE + SFE_CONTROL_FLAGS, SFE_CTRL_GATE_RESET);
    int ok = sfe_transact();
    Xil_Out32(SFE_BASE + SFE_CONTROL_FLAGS, 0u);
    return ok;
}

int SeclocFrontendLink_Init(void)
{
    sfe_present = 0;
    if (sfe_gate_reset() && ((sfe_last_status >> 24) == SFE_EXPECTED_VERSION)) {
        sfe_present = 1;
    }
    return sfe_present;
}

int SeclocFrontendLink_Present(void)
{
    return sfe_present;
}

uint32_t SeclocFrontendLink_LastNnWaitCycles(void)
{
    return sfe_last_nn_wait_cycles;
}

uint32_t SeclocFrontendLink_LastUpdateCount(void)
{
    return sfe_last_update_count;
}

int SeclocFrontendLink_PlainEvaluate(const float nn_inputs[7], float* Q)
{
    if (!sfe_present) {
        return 0;
    }

    sfe_write_float(SFE_ANGLED,             nn_inputs[0]);
    sfe_write_float(SFE_ANGLE_COS,          nn_inputs[1]);
    sfe_write_float(SFE_ANGLE_SIN,          nn_inputs[2]);
    sfe_write_float(SFE_POSITION,           nn_inputs[3]);
    sfe_write_float(SFE_POSITIOND,          nn_inputs[4]);
    sfe_write_float(SFE_TARGET_EQUILIBRIUM, nn_inputs[5]);
    sfe_write_float(SFE_TARGET_POSITION,    nn_inputs[6]);
    /* angle/tick unused when the gate is bypassed */
    Xil_Out32(SFE_BASE + SFE_CONTROL_FLAGS, SFE_CTRL_FORCE_COMPUTE);

    int ok = sfe_transact();
    Xil_Out32(SFE_BASE + SFE_CONTROL_FLAGS, 0u);

    if (!ok || (sfe_last_status & SFE_STATUS_NN_TIMEOUT)) {
        return 0;
    }
    *Q = sfe_read_float(SFE_Q);
    return 1;
}

/* ---- SecLoc PL backend ops (secloc_controller_pl.h) -------------------- */

static int sfe_backend_evaluate(
    float p, float pd, float a, float ad, float tp, float te,
    int32_t tick,
    float* Q, uint8_t* fired, uint8_t* gate_evaluated
)
{
    if (!sfe_present) {
        return 0;
    }

    sfe_write_float(SFE_ANGLED,             ad);
    sfe_write_float(SFE_ANGLE_COS,          cosf(a));
    sfe_write_float(SFE_ANGLE_SIN,          sinf(a));
    sfe_write_float(SFE_POSITION,           p);
    sfe_write_float(SFE_POSITIOND,          pd);
    sfe_write_float(SFE_TARGET_EQUILIBRIUM, te);
    sfe_write_float(SFE_TARGET_POSITION,    tp);
    sfe_write_float(SFE_ANGLE,              a);
    Xil_Out32(SFE_BASE + SFE_TICK,          (uint32_t)tick);
    Xil_Out32(SFE_BASE + SFE_CONTROL_FLAGS, 0u);

    if (!sfe_transact()) {
        return 0;
    }

    *Q              = sfe_read_float(SFE_Q);
    *fired          = (sfe_last_status & SFE_STATUS_FIRED) ? 1u : 0u;
    *gate_evaluated = (sfe_last_status & SFE_STATUS_GATE_EVAL) ? 1u : 0u;
    return 1;
}

static void sfe_backend_set_params(
    float log_base, int32_t ref_period_ticks,
    float ang_dead_band, float pos_dead_band
)
{
    if (!sfe_present) {
        return;
    }
    sfe_write_float(SFE_LOG_BASE, log_base);
    Xil_Out32(SFE_BASE + SFE_REF_PERIOD_TICKS, (uint32_t)ref_period_ticks);
    sfe_write_float(SFE_DEAD_ANG, ang_dead_band);
    sfe_write_float(SFE_DEAD_POS, pos_dead_band);
}

static void sfe_backend_reset_gate(void)
{
    if (sfe_present) {
        (void)sfe_gate_reset();
    }
}

static const SeclocPlBackendOps sfe_backend_ops = {
    .evaluate       = sfe_backend_evaluate,
    .set_params     = sfe_backend_set_params,
    .reset_gate     = sfe_backend_reset_gate,
    .update_count   = SeclocFrontendLink_LastUpdateCount,
    .nn_wait_cycles = SeclocFrontendLink_LastNnWaitCycles,
};

void SeclocFrontendLink_RegisterSeclocBackend(void)
{
    if (sfe_present) {
        secloc_register_pl_backend(&sfe_backend_ops);
    }
}

#else  /* !HW_HAS_SECLOC_FRONTEND: bitstream without the IP */

int SeclocFrontendLink_Init(void) { return 0; }
int SeclocFrontendLink_Present(void) { return 0; }
int SeclocFrontendLink_PlainEvaluate(const float nn_inputs[7], float* Q)
{
    (void)nn_inputs;
    (void)Q;
    return 0;
}
uint32_t SeclocFrontendLink_LastNnWaitCycles(void) { return 0; }
uint32_t SeclocFrontendLink_LastUpdateCount(void) { return 0; }
void SeclocFrontendLink_RegisterSeclocBackend(void) { }

#endif /* HW_HAS_SECLOC_FRONTEND */
