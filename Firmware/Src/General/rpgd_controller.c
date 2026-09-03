/*
 * On-chip RPGD-C controller (Zynq PS / bare metal).
 * Timing/maps come from controller_profiles.c; this module only creates the
 * solver and talks to the CPU1 worker when -DRPGD_DUAL_CORE.
 */

#include "rpgd_controller.h"
#include "rpgd_config_defaults.h"
#include "rpgd_c/rpgd_cartpole.h"
#include "parameters.h"
#include "hardware_bridge.h"
#include "angle_processing.h"

#ifdef RPGD_DUAL_CORE
#include "rpgd_zynq_30ms_config.h"
#include "rpgd_amp_dispatch.h"
#endif

#include <math.h>
#include <stdint.h>
#include <string.h>

static const char* const RPGD_InputNames[] = {
    "angle",
    "angleD",
    "angle_cos",
    "angle_sin",
    "position",
    "positionD",
    "target_position",
    "target_equilibrium",
};

static const ControllerSpec RPGD_Spec = {
    .version   = 1,
    .n_inputs  = 8,
    .n_outputs = 1,
    .names     = RPGD_InputNames
};

static RpgdSolver* g_rpgd_solver;
static unsigned int g_rpgd_phase;
static unsigned int g_rpgd_stride = 1u;
static float g_rpgd_last_output;
static int g_rpgd_last_status = RPGD_STATUS_INVALID_ARGUMENT;
static int g_rpgd_fault_latched;
static int g_rpgd_deadline_grace;

static const ControllerSpec* RPGD_GetSpec(void) { return &RPGD_Spec; }

static uint32_t rpgd_config_fingerprint(const RpgdConfig* cfg)
{
    const unsigned char* p = (const unsigned char*)cfg;
    uint32_t h = 2166136261u;
    for (size_t i = 0; i < sizeof(*cfg); ++i) {
        h ^= p[i];
        h *= 16777619u;
    }
    return h;
}

static int rpgd_bind_timing(const RpgdConfig* cfg)
{
    const unsigned int control_period_us = (unsigned int)POLLING_PERIOD_MS * 1000u;
    const unsigned int model_period_us = (unsigned int)(cfg->mpc_timestep * 1000000.0f + 0.5f);
    if (control_period_us == 0u) return RPGD_STATUS_INVALID_CONFIG;
#ifdef RPGD_DUAL_CORE
    g_rpgd_stride = 1u;
    if (model_period_us != control_period_us) return RPGD_STATUS_INVALID_CONFIG;
#ifdef RPGD_CONTROL_PERIOD_MS
    if (POLLING_PERIOD_MS != (unsigned short)RPGD_CONTROL_PERIOD_MS) {
        return RPGD_STATUS_INVALID_CONFIG;
    }
#endif
    return RPGD_STATUS_OK;
#else
    g_rpgd_stride = (model_period_us + control_period_us / 2u) / control_period_us;
    if (g_rpgd_stride == 0u) g_rpgd_stride = 1u;
    const unsigned int effective_period_us = g_rpgd_stride * control_period_us;
    const unsigned int period_error_us = effective_period_us > model_period_us
        ? effective_period_us - model_period_us
        : model_period_us - effective_period_us;
    if (period_error_us > control_period_us / 4u) return RPGD_STATUS_INVALID_CONFIG;
    return RPGD_STATUS_OK;
#endif
}

static void RPGD_Init(void)
{
    Motor_Stop();
#ifdef RPGD_DUAL_CORE
    RpgdConfig cfg = RPGD_30MS_CONFIG;
#else
    RpgdConfig cfg = RPGD_DEFAULT_CONFIG;
#endif
    if (g_rpgd_solver) {
        rpgd_destroy(g_rpgd_solver);
        g_rpgd_solver = 0;
    }
    g_rpgd_phase = 0u;
    g_rpgd_last_output = 0.0f;
    g_rpgd_fault_latched = 0;
    g_rpgd_deadline_grace = 1;
    g_rpgd_last_status = rpgd_validate_config(&cfg);
    if (g_rpgd_last_status != RPGD_STATUS_OK) return;

    g_rpgd_last_status = rpgd_bind_timing(&cfg);
    if (g_rpgd_last_status != RPGD_STATUS_OK) {
        return;
    }

    g_rpgd_solver = rpgd_create(&cfg);
    if (!g_rpgd_solver) {
        g_rpgd_last_status = RPGD_STATUS_WORKSPACE_FAILURE;
        return;
    }

#ifdef RPGD_DUAL_CORE
    if (rpgd_amp_init(g_rpgd_solver, rpgd_config_fingerprint(&cfg)) != RPGD_STATUS_OK) {
        rpgd_destroy(g_rpgd_solver);
        g_rpgd_solver = 0;
        g_rpgd_last_status = RPGD_CONTROLLER_STATUS_AMP_UNAVAILABLE;
        g_rpgd_fault_latched = 1;
        Motor_Stop();
    }
#endif
}

static void RPGD_Release(void)
{
    Motor_Stop();
#ifdef RPGD_DUAL_CORE
    rpgd_amp_park();
#endif
    if (g_rpgd_solver) {
        rpgd_destroy(g_rpgd_solver);
        g_rpgd_solver = 0;
    }
    g_rpgd_last_output = 0.0f;
    g_rpgd_last_status = RPGD_STATUS_INVALID_ARGUMENT;
    g_rpgd_fault_latched = 0;
    g_rpgd_deadline_grace = 0;
}

static void RPGD_Evaluate(const float* in, float* out)
{
    if (!g_rpgd_solver || g_rpgd_fault_latched) {
        out[0] = 0.0f;
        return;
    }
    if (g_rpgd_phase != 0u) {
        g_rpgd_phase += 1u;
        if (g_rpgd_phase >= g_rpgd_stride) g_rpgd_phase = 0u;
        out[0] = g_rpgd_last_output;
        return;
    }
    float state6[6];
    state6[0] = in[0];
    state6[1] = in[1];
    state6[2] = in[2];
    state6[3] = in[3];
    state6[4] = in[4];
    state6[5] = in[5];
    RpgdRuntime runtime;
    runtime.target_position = in[6];
    runtime.target_equilibrium = in[7];
    runtime.L = 0.0f;
    runtime.m_pole = 0.0f;
    for (int i = 0; i < 8; ++i) {
        if (!isfinite(in[i])) {
            g_rpgd_last_status = RPGD_STATUS_INVALID_ARGUMENT;
            g_rpgd_last_output = 0.0f;
            g_rpgd_fault_latched = 1;
            out[0] = 0.0f;
            return;
        }
    }
#ifdef RPGD_DUAL_CORE
    const float prev_u = g_rpgd_last_output;
    const int amp_rc = rpgd_amp_step(g_rpgd_solver, state6, &runtime, &g_rpgd_last_output);
    if (amp_rc == RPGD_CONTROLLER_STATUS_AMP_TIMEOUT) {
        g_rpgd_last_output = prev_u;
        g_rpgd_last_status = RPGD_STATUS_OK;
        out[0] = prev_u;
        g_rpgd_phase = g_rpgd_stride > 1u ? 1u : 0u;
        return;
    }
    g_rpgd_last_status = amp_rc;
#else
    g_rpgd_last_output = rpgd_step(g_rpgd_solver, state6, &runtime);
    g_rpgd_last_status = rpgd_get_last_status(g_rpgd_solver);
#endif
    if (!isfinite(g_rpgd_last_output) || g_rpgd_last_status != RPGD_STATUS_OK) {
        g_rpgd_last_output = 0.0f;
        g_rpgd_fault_latched = 1;
        Motor_Stop();
    }
    g_rpgd_phase = g_rpgd_stride > 1u ? 1u : 0u;
    out[0] = g_rpgd_last_output;
}

int rpgd_controller_last_status(void)
{
    return g_rpgd_last_status;
}

unsigned int rpgd_controller_stride(void)
{
    return g_rpgd_stride;
}

int rpgd_controller_owns_timing(void)
{
    return 0;
}

int rpgd_controller_take_deadline_grace(void)
{
    if (!g_rpgd_deadline_grace) return 0;
    g_rpgd_deadline_grace = 0;
    return 1;
}

void rpgd_controller_latch_fault(int status)
{
    g_rpgd_fault_latched = 1;
    g_rpgd_last_status = status;
    g_rpgd_last_output = 0.0f;
    g_rpgd_phase = 0u;
    Motor_Stop();
}

const ControllerOps RPGD_Ops = {
    .spec     = RPGD_GetSpec,
    .init     = RPGD_Init,
    .evaluate = RPGD_Evaluate,
    .release  = RPGD_Release
};
