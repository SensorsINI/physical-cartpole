#include "rpgd_on_target_test.h"

#ifndef RPGD_ON_TARGET_TEST

void rpgd_on_target_test_run(void)
{
}

#else

#include "hardware_bridge.h"
#include "parameters.h"
#include "rpgd_on_target_config.h"
#include "rpgd_golden_vectors.h"
#include "rpgd_config_defaults.h"
#include "rpgd_controller.h"
#include "rpgd_c/rpgd_cartpole.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

#ifdef ZYNQ
#include "xil_printf.h"
#endif

#define RPGD_TEST_MAX_SAMPLES 32

_Static_assert(RPGD_TEST_PARITY_STEPS == RPGD_GOLDEN_N_STEPS,
               "Parity-step configuration does not match generated goldens");
_Static_assert(RPGD_TEST_STEADY_ITERS <= RPGD_TEST_MAX_SAMPLES,
               "Increase RPGD_TEST_MAX_SAMPLES for steady timing");
_Static_assert(RPGD_TEST_WARMUP_ITERS <= RPGD_TEST_MAX_SAMPLES,
               "Increase RPGD_TEST_MAX_SAMPLES for warmup timing");

static void print_u32(const char* label, unsigned int value)
{
#ifdef ZYNQ
    xil_printf("%s%u\r\n", label, value);
#else
    (void)label;
    (void)value;
#endif
}

static void print_i32(const char* label, int value)
{
#ifdef ZYNQ
    xil_printf("%s%d\r\n", label, value);
#else
    (void)label;
    (void)value;
#endif
}

static unsigned int us_from_ticks(unsigned long long ticks)
{
    return (unsigned int)((ticks * 1000000ULL) / (unsigned long long)CLOCK_FREQ);
}

static void sort_u64(unsigned long long* values, int n)
{
    for (int i = 1; i < n; ++i) {
        unsigned long long key = values[i];
        int j = i - 1;
        while (j >= 0 && values[j] > key) {
            values[j + 1] = values[j];
            --j;
        }
        values[j + 1] = key;
    }
}

static float absf_local(float x)
{
    return x < 0.0f ? -x : x;
}

static int within_tol(float a, float b, float tol)
{
    return isfinite(a) && isfinite(b) && isfinite(tol) && absf_local(a - b) <= tol;
}

static void print_timing(const char* title, unsigned long long* samples, int n)
{
    unsigned long long copy[RPGD_TEST_MAX_SAMPLES];
    if (n > RPGD_TEST_MAX_SAMPLES) n = RPGD_TEST_MAX_SAMPLES;
    memcpy(copy, samples, (size_t)n * sizeof(unsigned long long));
    sort_u64(copy, n);
    unsigned long long min_ticks = copy[0];
    unsigned long long max_ticks = copy[n - 1];
    unsigned long long sum = 0;
    for (int i = 0; i < n; ++i) sum += copy[i];
    unsigned long long median_ticks = copy[n / 2];
    int p95_index = (95 * n + 99) / 100 - 1;
    if (p95_index < 0) p95_index = 0;
    unsigned long long p95_ticks = copy[p95_index];
#ifdef ZYNQ
    xil_printf("%s n=%d\r\n", title, n);
#endif
    print_u32("  min_us=", us_from_ticks(min_ticks));
    print_u32("  median_us=", us_from_ticks(median_ticks));
    print_u32("  p95_us=", us_from_ticks(p95_ticks));
    print_u32("  max_us=", us_from_ticks(max_ticks));
    print_u32("  mean_us=", us_from_ticks(sum / (unsigned long long)n));
    print_u32("  min_ticks=", (unsigned int)min_ticks);
    print_u32("  median_ticks=", (unsigned int)median_ticks);
}

static int run_parity(void)
{
    RpgdConfig cfg = RPGD_DEFAULT_CONFIG;
    RpgdRuntime runtime;
    runtime.target_position = 0.0f;
    runtime.target_equilibrium = 1.0f;
    runtime.L = 0.0f;
    runtime.m_pole = 0.0f;

    float state6[6];
    memcpy(state6, RPGD_GOLDEN_STATE, sizeof(state6));

    RpgdSolver* solver = rpgd_create(&cfg);
    if (!solver) {
#ifdef ZYNQ
        xil_printf("RPGD_PARITY FAIL create\r\n");
#endif
        return 0;
    }

    float grad[RPGD_GOLDEN_HORIZON];
    rpgd_debug_gradient_adjoint(&cfg, &runtime, state6, RPGD_GOLDEN_Q_KERNEL, grad);
    int pass = 1;
    for (int h = 0; h < RPGD_GOLDEN_HORIZON; ++h) {
        if (!within_tol(grad[h], RPGD_GOLDEN_GRAD[h], RPGD_GOLDEN_GRAD_ABS_TOL)) pass = 0;
    }

    rpgd_debug_set_q(solver, RPGD_GOLDEN_Q_INIT);
    float costs[RPGD_GOLDEN_N_COSTS];
    int indices[RPGD_GOLDEN_N_COSTS];
    for (int i = 0; i < RPGD_TEST_PARITY_STEPS; ++i) {
        float u = rpgd_step(solver, state6, &runtime);
        if (rpgd_get_last_status(solver) != RPGD_STATUS_OK) pass = 0;
        if (!within_tol(u, RPGD_GOLDEN_U_STEPS[i], RPGD_GOLDEN_U_ABS_TOL)) pass = 0;
        print_i32("parity_u_err_u=", (int)((u - RPGD_GOLDEN_U_STEPS[i]) * 1000000.0f));
        if (i == 0) {
            rpgd_debug_get_costs(solver, costs);
            rpgd_debug_get_indices(solver, indices);
            if (indices[0] != RPGD_GOLDEN_BEST_INDEX) pass = 0;
            for (int n = 0; n < RPGD_GOLDEN_N_COSTS; ++n) {
                if (!within_tol(costs[n], RPGD_GOLDEN_COSTS[n], RPGD_GOLDEN_COST_ABS_TOL)) pass = 0;
                if (indices[n] != RPGD_GOLDEN_INDICES[n]) pass = 0;
            }
            print_i32("parity_best=", indices[0]);
        }
    }
    print_u32("workspace_bytes=", (unsigned int)rpgd_get_workspace_bytes(solver));
    print_u32("static_workspace_bytes=", (unsigned int)rpgd_get_static_workspace_bytes());
    print_i32("baremetal=", rpgd_is_baremetal());
    if (!rpgd_is_baremetal()
        || rpgd_get_workspace_bytes(solver) != rpgd_get_static_workspace_bytes()) pass = 0;
    rpgd_destroy(solver);
#ifdef ZYNQ
    xil_printf(pass ? "RPGD_PARITY PASS\r\n" : "RPGD_PARITY FAIL\r\n");
#endif
    return pass;
}

static void fill_runtime_and_state(RpgdRuntime* runtime, float* state6)
{
    memcpy(state6, RPGD_GOLDEN_STATE, 6u * sizeof(float));
    runtime->target_position = 0.0f;
    runtime->target_equilibrium = 1.0f;
    runtime->L = 0.0f;
    runtime->m_pole = 0.0f;
}

void rpgd_on_target_test_run(void)
{
#ifdef ZYNQ
    xil_printf("RPGD_ON_TARGET start\r\n");
#endif
    print_u32("timer_tick_hz=", CLOCK_FREQ);
    const int parity_pass = run_parity();
    print_i32("parity=", parity_pass);
    if (!parity_pass) {
#ifdef ZYNQ
        xil_printf("RPGD_ON_TARGET timing skipped: parity failed\r\n");
#endif
        return;
    }

    RpgdConfig cfg = RPGD_DEFAULT_CONFIG;
    unsigned long long t0 = GetTimeNowHighRes();
    RpgdSolver* solver = rpgd_create(&cfg);
    unsigned long long t1 = GetTimeNowHighRes();
    print_u32("init_us=", us_from_ticks(t1 - t0));
    if (!solver) {
#ifdef ZYNQ
        xil_printf("RPGD_ON_TARGET timing skipped: create failed\r\n");
#endif
        return;
    }

    RpgdRuntime runtime;
    float state6[6];
    fill_runtime_and_state(&runtime, state6);

    unsigned long long warmup[RPGD_TEST_WARMUP_ITERS];
    for (int i = 0; i < RPGD_TEST_WARMUP_ITERS; ++i) {
        unsigned long long s = GetTimeNowHighRes();
        (void)rpgd_step(solver, state6, &runtime);
        warmup[i] = GetTimeNowHighRes() - s;
    }
    print_timing("warmup", warmup, RPGD_TEST_WARMUP_ITERS);

    unsigned long long steady[RPGD_TEST_STEADY_ITERS];
    int timing_status = RPGD_STATUS_OK;
    for (int i = 0; i < RPGD_TEST_STEADY_ITERS; ++i) {
        unsigned long long s = GetTimeNowHighRes();
        const float out = rpgd_step(solver, state6, &runtime);
        steady[i] = GetTimeNowHighRes() - s;
        if (!isfinite(out) || rpgd_get_last_status(solver) != RPGD_STATUS_OK) {
            timing_status = rpgd_get_last_status(solver);
        }
    }
    print_timing("steady", steady, RPGD_TEST_STEADY_ITERS);

    sort_u64(steady, RPGD_TEST_STEADY_ITERS);
    unsigned int median_us = us_from_ticks(steady[RPGD_TEST_STEADY_ITERS / 2]);
    unsigned int max_us = us_from_ticks(steady[RPGD_TEST_STEADY_ITERS - 1]);
    const unsigned int period_us = (unsigned int)POLLING_PERIOD_MS * 1000u;
    const unsigned int mpc_timestep_us =
        (unsigned int)(cfg.mpc_timestep * 1000000.0f + 0.5f);
    print_u32("budget_period_us=", period_us);
    print_u32("budget_mpc_dt_us=", mpc_timestep_us);
    print_u32("median_vs_period_pct=", (median_us * 100u) / period_us);
    print_u32("median_vs_mpc_dt_pct=", (median_us * 100u) / mpc_timestep_us);
    print_i32("timing_status=", timing_status);
    print_i32("deadline_pass=", timing_status == RPGD_STATUS_OK && max_us < period_us);
    rpgd_destroy(solver);

    RPGD_Ops.init();
    print_u32("controller_stride=", rpgd_controller_stride());
    print_i32("controller_init_status=", rpgd_controller_last_status());
    RPGD_Ops.release();
#ifdef ZYNQ
    xil_printf("RPGD_ON_TARGET idle\r\n");
#endif
}

#endif /* RPGD_ON_TARGET_TEST */
