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
#include "rpgd_c/rpgd_worker.h"

#ifdef RPGD_DUAL_CORE
#include "rpgd_amp_dispatch.h"
#endif

#include <math.h>
#include <stdint.h>
#include <string.h>

#ifdef ZYNQ
#include "xil_printf.h"
#endif

_Static_assert(RPGD_TEST_PARITY_STEPS == RPGD_GOLDEN_N_STEPS,
               "Parity-step configuration does not match generated goldens");
_Static_assert(RPGD_TEST_STEADY_ITERS <= RPGD_TEST_MAX_SAMPLES,
               "Increase RPGD_TEST_MAX_SAMPLES for steady timing");
_Static_assert(RPGD_TEST_WARMUP_ITERS <= RPGD_TEST_MAX_SAMPLES,
               "Increase RPGD_TEST_MAX_SAMPLES for warmup timing");
_Static_assert(RPGD_TEST_LONG_ITERS <= RPGD_TEST_MAX_SAMPLES,
               "Increase RPGD_TEST_MAX_SAMPLES for long timing");

static unsigned long long g_sort_buf[RPGD_TEST_MAX_SAMPLES];
static unsigned long long g_warmup[RPGD_TEST_WARMUP_ITERS];
static unsigned long long g_steady[RPGD_TEST_MAX_SAMPLES];
static RpgdWorkerScratch g_test_scratch;

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
    if (n > RPGD_TEST_MAX_SAMPLES) n = RPGD_TEST_MAX_SAMPLES;
    memcpy(g_sort_buf, samples, (size_t)n * sizeof(unsigned long long));
    sort_u64(g_sort_buf, n);
    unsigned long long min_ticks = g_sort_buf[0];
    unsigned long long max_ticks = g_sort_buf[n - 1];
    unsigned long long sum = 0;
    for (int i = 0; i < n; ++i) sum += g_sort_buf[i];
    unsigned long long median_ticks = g_sort_buf[n / 2];
    int p95_index = (95 * n + 99) / 100 - 1;
    if (p95_index < 0) p95_index = 0;
    unsigned long long p95_ticks = g_sort_buf[p95_index];
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

static int run_split_parity(void)
{
    RpgdConfig cfg = RPGD_DEFAULT_CONFIG;
    RpgdRuntime runtime;
    float state6[6];
    fill_runtime_and_state(&runtime, state6);
    RpgdSolver* mono = rpgd_create(&cfg);
    if (!mono) return 0;
    rpgd_debug_set_q(mono, RPGD_GOLDEN_Q_INIT);
    float u_mono[RPGD_TEST_PARITY_STEPS];
    float costs_mono[RPGD_GOLDEN_N_COSTS];
    int idx_mono[RPGD_GOLDEN_N_COSTS];
    for (int i = 0; i < RPGD_TEST_PARITY_STEPS; ++i) {
        u_mono[i] = rpgd_step(mono, state6, &runtime);
        if (i == 0) {
            rpgd_debug_get_costs(mono, costs_mono);
            rpgd_debug_get_indices(mono, idx_mono);
        }
    }
    rpgd_destroy(mono);

    RpgdSolver* split = rpgd_create(&cfg);
    if (!split) return 0;
    rpgd_debug_set_q(split, RPGD_GOLDEN_Q_INIT);
    int pass = 1;
    for (int i = 0; i < RPGD_TEST_PARITY_STEPS; ++i) {
        RpgdStepPlan plan;
        if (rpgd_step_prepare(split, state6, &runtime, &plan) != RPGD_STATUS_OK) pass = 0;
        if (rpgd_step_optimize_range(split, &plan, 0, 8, &g_test_scratch) != RPGD_STATUS_OK) pass = 0;
        if (rpgd_step_optimize_range(split, &plan, 8, cfg.num_rollouts, &g_test_scratch) != RPGD_STATUS_OK) pass = 0;
        const float u = rpgd_step_finalize(split, &plan);
        if (u != u_mono[i]) pass = 0;
        if (i == 0) {
            float costs[RPGD_GOLDEN_N_COSTS];
            int idx[RPGD_GOLDEN_N_COSTS];
            rpgd_debug_get_costs(split, costs);
            rpgd_debug_get_indices(split, idx);
            if (memcmp(costs, costs_mono, sizeof(costs_mono)) != 0) pass = 0;
            if (memcmp(idx, idx_mono, sizeof(idx_mono)) != 0) pass = 0;
        }
    }
    rpgd_destroy(split);
#ifdef ZYNQ
    xil_printf(pass ? "RPGD_SPLIT PASS\r\n" : "RPGD_SPLIT FAIL\r\n");
#endif
    return pass;
}

#ifdef RPGD_DUAL_CORE
static int run_dual_core_parity_and_timing(void)
{
    RpgdConfig cfg = RPGD_DEFAULT_CONFIG;
    RpgdRuntime runtime;
    float state6[6];
    fill_runtime_and_state(&runtime, state6);

    RpgdSolver* serial = rpgd_create(&cfg);
    if (!serial) return 0;
    rpgd_debug_set_q(serial, RPGD_GOLDEN_Q_INIT);
    float u_serial[8];
    float q_serial[RPGD_GOLDEN_Q_INIT_LEN];
    for (int i = 0; i < 8; ++i) u_serial[i] = rpgd_step(serial, state6, &runtime);
    rpgd_debug_get_q(serial, q_serial);
    rpgd_destroy(serial);

    RpgdSolver* dual = rpgd_create(&cfg);
    if (!dual) return 0;
    if (rpgd_amp_init(dual, 0u) != RPGD_STATUS_OK) {
#ifdef ZYNQ
        xil_printf("RPGD_DUAL FAIL amp_init status=%d state=%u\r\n",
                   rpgd_amp_last_status(), rpgd_amp_worker_state());
#endif
        rpgd_destroy(dual);
        return 0;
    }
    rpgd_debug_set_q(dual, RPGD_GOLDEN_Q_INIT);
    int pass = 1;
    for (int i = 0; i < 8; ++i) {
        float u = 0.0f;
        const int rc = rpgd_amp_step(dual, state6, &runtime, &u);
        if (rc != RPGD_STATUS_OK || u != u_serial[i]) pass = 0;
        if (!rpgd_amp_ready()) pass = 0;
        const RpgdAmpTiming* t = rpgd_amp_last_timing();
        print_u32("dual_prepare_us=", t->prepare_us);
        print_u32("dual_dispatch_us=", t->dispatch_us);
        print_u32("dual_cpu0_us=", t->cpu0_range_us);
        print_u32("dual_cpu1_us=", t->cpu1_range_us);
        print_u32("dual_barrier_us=", t->barrier_us);
        print_u32("dual_finalize_us=", t->finalize_us);
        print_u32("dual_total_us=", t->total_us);
    }
    float q_dual[RPGD_GOLDEN_Q_INIT_LEN];
    rpgd_debug_get_q(dual, q_dual);
    if (memcmp(q_serial, q_dual, sizeof(q_serial)) != 0) pass = 0;

    int n = RPGD_TEST_STEADY_ITERS;
    if (n > RPGD_TEST_MAX_SAMPLES) n = RPGD_TEST_MAX_SAMPLES;
    int timing_status = RPGD_STATUS_OK;
    unsigned int over_30ms = 0;
    for (int i = 0; i < n; ++i) {
        unsigned long long s = GetTimeNowHighRes();
        float u = 0.0f;
        const int rc = rpgd_amp_step(dual, state6, &runtime, &u);
        g_steady[i] = GetTimeNowHighRes() - s;
        if (rc != RPGD_STATUS_OK || !isfinite(u)) timing_status = rc;
        if (us_from_ticks(g_steady[i]) >= 30000u) ++over_30ms;
        if (!rpgd_amp_ready()) {
            timing_status = rpgd_amp_last_status();
            n = i + 1;
            break;
        }
    }
    if (n < 1) {
#ifdef ZYNQ
        xil_printf("dual_steady skipped\r\n");
#endif
        rpgd_amp_park();
        rpgd_destroy(dual);
        return 0;
    }
    print_timing("dual_steady", g_steady, n);
    memcpy(g_sort_buf, g_steady, (size_t)n * sizeof(unsigned long long));
    sort_u64(g_sort_buf, n);
    const unsigned int median_us = us_from_ticks(g_sort_buf[n / 2]);
    const unsigned int p95_us = us_from_ticks(g_sort_buf[(95 * n + 99) / 100 - 1]);
    const unsigned int max_us = us_from_ticks(g_sort_buf[n - 1]);
    const int gate = pass && timing_status == RPGD_STATUS_OK
        && median_us <= 28000u && p95_us < 29000u && over_30ms == 0u;
    print_u32("dual_timeouts=", rpgd_amp_timeout_count());
    print_u32("dual_epoch=", rpgd_amp_epoch());
    print_u32("dual_worker_state=", rpgd_amp_worker_state());
    print_u32("dual_over_30ms=", over_30ms);
    print_u32("dual_max_us=", max_us);
    print_i32("dual_gate=", gate);
    rpgd_amp_park();
    rpgd_destroy(dual);
#ifdef ZYNQ
    xil_printf(pass ? "RPGD_DUAL_PARITY PASS\r\n" : "RPGD_DUAL_PARITY FAIL\r\n");
#endif
    return gate;
}
#endif

void rpgd_on_target_test_run(void)
{
#ifdef ZYNQ
    xil_printf("RPGD_ON_TARGET start\r\n");
#endif
    print_u32("timer_tick_hz=", CLOCK_FREQ);
    const int parity_pass = run_parity();
    print_i32("parity=", parity_pass);
    const int split_pass = run_split_parity();
    print_i32("split_parity=", split_pass);
    if (!parity_pass || !split_pass) {
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

    unsigned long long* warmup = g_warmup;
    for (int i = 0; i < RPGD_TEST_WARMUP_ITERS; ++i) {
        unsigned long long s = GetTimeNowHighRes();
        (void)rpgd_step(solver, state6, &runtime);
        warmup[i] = GetTimeNowHighRes() - s;
    }
    print_timing("warmup", warmup, RPGD_TEST_WARMUP_ITERS);

    unsigned long long* steady = g_steady;
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

#ifdef RPGD_DUAL_CORE
    print_i32("dual_gate=", run_dual_core_parity_and_timing());
    /* Dual-core RPGD_Init programs the control timer through the GIC.
     * This harness never calls Interrupt_Init, so skip controller bind here
     * (Stage B uses the normal firmware image). */
    print_i32("controller_init_skipped=", 1);
#else
    RPGD_Ops.init();
    print_u32("controller_stride=", rpgd_controller_stride());
    print_i32("controller_init_status=", rpgd_controller_last_status());
    RPGD_Ops.release();
#endif
#ifdef ZYNQ
    xil_printf("RPGD_ON_TARGET idle\r\n");
#endif
}

#endif /* RPGD_ON_TARGET_TEST */
