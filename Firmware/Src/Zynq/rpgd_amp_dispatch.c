#include "rpgd_amp_dispatch.h"

#ifdef RPGD_DUAL_CORE

#include "amp_ipc.h"
#include "hardware_bridge.h"
#include "parameters.h"
#include "rpgd_c/rpgd_platform.h"
#include "sleep.h"

#include <string.h>

static RpgdStepPlan g_shared_plan __attribute__((section(".amp_shared"), aligned(64)));
static RpgdWorkerScratch g_cpu0_scratch RPGD_ALIGN64;
static RpgdAmpTiming g_timing;
static int g_ready;
static int g_last_status = RPGD_CONTROLLER_STATUS_AMP_UNAVAILABLE;
static unsigned int g_timeout_count;
static uint32_t g_epoch;

static unsigned int us_from_ticks(unsigned long long ticks)
{
    return (unsigned int)((ticks * 1000000ULL) / (unsigned long long)CLOCK_FREQ);
}

static unsigned long long now_ticks(void)
{
    return GetTimeNowHighRes();
}

static unsigned long long ticks_from_us(unsigned int timeout_us)
{
    return ((unsigned long long)timeout_us * (unsigned long long)CLOCK_FREQ) / 1000000ULL;
}

static int worker_idle_state(uint32_t state)
{
    return state == AMP_WORKER_READY
        || state == AMP_WORKER_DONE
        || state == AMP_WORKER_OFFLINE
        || state == AMP_WORKER_FAULT;
}

static int wait_until(int (*pred)(const AmpMailbox*), unsigned int timeout_us)
{
    AmpMailbox* mb = amp_ipc_mailbox();
    const unsigned long long start = now_ticks();
    const unsigned long long budget = ticks_from_us(timeout_us);
    unsigned long long last_sev = start;
    const unsigned long long sev_every = ticks_from_us(1000u);
    /* Timed waits must not WFE: a missed SEV deadlocks both cores and
     * the timeout never runs. Poll and re-SEV about once per millisecond. */
    while ((now_ticks() - start) <= budget) {
        amp_ipc_refresh(mb);
        if (pred(mb)) return 1;
        if ((now_ticks() - last_sev) >= sev_every) {
            amp_ipc_sev();
            last_sev = now_ticks();
        }
    }
    amp_ipc_refresh(mb);
    return pred(mb);
}

static uint32_t g_sync_heartbeat;

static int mb_synced(const AmpMailbox* mb)
{
    return amp_ipc_mailbox_valid(mb)
        && mb->worker_state == AMP_WORKER_READY
        && mb->heartbeat != g_sync_heartbeat
        && mb->completed_epoch == 0u;
}

typedef struct {
    uint32_t epoch;
} DrainTarget;

static DrainTarget g_drain;

static int mb_drained(const AmpMailbox* mb)
{
    if (mb->worker_state == AMP_WORKER_FAULT) return 1;
    if (mb->completed_epoch == g_drain.epoch && worker_idle_state(mb->worker_state)) return 1;
    if (mb->worker_state == AMP_WORKER_OFFLINE) return 1;
    if (mb->worker_state == AMP_WORKER_READY && mb->completed_epoch >= g_drain.epoch) return 1;
    return 0;
}

static int mb_is_offline(const AmpMailbox* mb)
{
    return mb->worker_state == AMP_WORKER_OFFLINE;
}

static void flush_cb(const void* addr, size_t bytes) { amp_ipc_flush(addr, bytes); }
static void inv_cb(const void* addr, size_t bytes) { amp_ipc_invalidate(addr, bytes); }

static void flush_shared(RpgdSolver* solver, const RpgdStepPlan* plan)
{
    rpgd_cache_visit_solver(solver, flush_cb);
    rpgd_cache_visit_rollout_slice(solver, 0, rpgd_get_num_rollouts(solver), flush_cb);
    if (plan) amp_ipc_flush(plan, sizeof(*plan));
}

static void invalidate_cpu1_slices(RpgdSolver* solver)
{
    const int n = rpgd_get_num_rollouts(solver);
    rpgd_cache_visit_rollout_slice(solver, n / 2, n, inv_cb);
}

static void drain_worker(uint32_t epoch)
{
    g_drain.epoch = epoch;
    (void)wait_until(mb_drained, RPGD_AMP_DRAIN_TIMEOUT_US);
}

static void publish_cpu0_result(int status, float action)
{
    AmpMailbox* mb = amp_ipc_mailbox();
    mb->result_status = status;
    memcpy(&mb->reserved[0], &action, sizeof(action));
    mb->reserved[1] = g_timing.total_us;
    mb->reserved[2] = g_timing.prepare_us;
    mb->reserved[3] = g_timing.dispatch_us;
    mb->reserved[4] = g_timing.cpu0_range_us;
    mb->reserved[5] = g_timing.cpu1_range_us;
    mb->reserved[6] = g_timing.barrier_us;
    mb->reserved[7] = g_timing.finalize_us;
    amp_ipc_commit(mb);
}

static int worker_usable(void)
{
    AmpMailbox* mb = amp_ipc_mailbox();
    amp_ipc_refresh(mb);
    return amp_ipc_mailbox_valid(mb)
        && mb->worker_state != AMP_WORKER_FAULT
        && mb->worker_state != AMP_WORKER_BUSY
        && mb->worker_state != AMP_WORKER_OFFLINE;
}

static void fail_step(RpgdSolver* solver, int status, uint32_t epoch, int is_timeout)
{
    drain_worker(epoch);
    rpgd_step_abort(solver, RPGD_STATUS_WORKER_FAILURE);
    if (is_timeout) {
        ++g_timeout_count;
        /* UART/IRQ load can push a step past 24.5 ms. Keep the worker; the
         * caller holds the last action instead of latching a fatal fault. */
    } else {
        Motor_Stop();
    }
    g_ready = worker_usable();
    g_last_status = status;
    publish_cpu0_result(status, 0.0f);
}

int rpgd_amp_init(RpgdSolver* solver, uint32_t config_fingerprint)
{
    g_ready = 0;
    g_epoch = 0;
    g_timeout_count = 0;
    g_last_status = RPGD_CONTROLLER_STATUS_AMP_UNAVAILABLE;
    memset(&g_timing, 0, sizeof(g_timing));
    memset(&g_shared_plan, 0, sizeof(g_shared_plan));
    if (!solver) return g_last_status;

    AmpMailbox* mb = amp_ipc_mailbox();
    amp_ipc_refresh(mb);
    g_sync_heartbeat = mb->heartbeat;
    amp_ipc_init_cpu0(
        config_fingerprint,
        (uint32_t)rpgd_get_solver_size(),
        (uint32_t)sizeof(RpgdStepPlan),
        (uint32_t)sizeof(RpgdWorkerScratch));
    amp_ipc_sev();
    usleep(200);
    amp_ipc_sev();

    if (!wait_until(mb_synced, RPGD_AMP_READY_TIMEOUT_US)) {
        return g_last_status;
    }
    amp_ipc_refresh(mb);
    if (!amp_ipc_mailbox_valid(mb)) return g_last_status;
    if (mb->solver_sizeof && mb->solver_sizeof != (uint32_t)rpgd_get_solver_size()) {
        return g_last_status;
    }
    if (mb->plan_sizeof && mb->plan_sizeof != (uint32_t)sizeof(RpgdStepPlan)) {
        return g_last_status;
    }
    if (mb->scratch_sizeof && mb->scratch_sizeof != (uint32_t)sizeof(RpgdWorkerScratch)) {
        return g_last_status;
    }
    if (mb->config_fingerprint != config_fingerprint) {
        return g_last_status;
    }
    if (mb->heartbeat == 0u) {
        return g_last_status;
    }
    mb->command = AMP_CMD_NOP;
    amp_ipc_commit(mb);
    g_ready = 1;
    g_last_status = RPGD_STATUS_OK;
    return RPGD_STATUS_OK;
}

void rpgd_amp_park(void)
{
    AmpMailbox* mb = amp_ipc_mailbox();
    amp_ipc_refresh(mb);
    mb->command = AMP_CMD_STOP;
    amp_ipc_commit(mb);
    amp_ipc_sev();
    (void)wait_until(mb_is_offline, RPGD_AMP_READY_TIMEOUT_US);
    g_ready = 0;
}

int rpgd_amp_ready(void) { return g_ready; }
int rpgd_amp_last_status(void) { return g_last_status; }
unsigned int rpgd_amp_epoch(void) { return g_epoch; }
unsigned int rpgd_amp_timeout_count(void) { return g_timeout_count; }
unsigned int rpgd_amp_worker_state(void)
{
    AmpMailbox* mb = amp_ipc_mailbox();
    amp_ipc_refresh(mb);
    return mb->worker_state;
}
const RpgdAmpTiming* rpgd_amp_last_timing(void) { return &g_timing; }

int rpgd_amp_step(RpgdSolver* solver, const float* state6, const RpgdRuntime* runtime, float* u)
{
    memset(&g_timing, 0, sizeof(g_timing));
    if (u) *u = 0.0f;
    if (!g_ready || !solver || !worker_usable()) {
        g_ready = 0;
        g_last_status = RPGD_CONTROLLER_STATUS_AMP_UNAVAILABLE;
        return g_last_status;
    }

    const unsigned long long t_all = now_ticks();
    AmpMailbox* mb = amp_ipc_mailbox();
    unsigned long long t0 = t_all;
    const int prep = rpgd_step_prepare(solver, state6, runtime, &g_shared_plan);
    g_timing.prepare_us = us_from_ticks(now_ticks() - t0);
    if (prep != RPGD_STATUS_OK) {
        g_last_status = prep;
        return prep;
    }

    uint32_t epoch = g_epoch + 1u;
    if (epoch == 0u) epoch = 1u;
    const int N = rpgd_get_num_rollouts(solver);
    int cpu0_first = 0;
    int cpu0_last = N / 2;
    int cpu1_first = N / 2;
    int cpu1_last = N;

    t0 = now_ticks();
    const unsigned long long t_dispatch = t0;
    mb->range_first = cpu1_first;
    mb->range_last = cpu1_last;
    mb->solver_addr = (uint32_t)(uintptr_t)solver;
    mb->plan_addr = (uint32_t)(uintptr_t)&g_shared_plan;
    mb->result_status = RPGD_STATUS_OK;
    mb->start_ticks = 0u;
    mb->end_ticks = 0u;
    mb->command = AMP_CMD_OPTIMIZE_RANGE;
    mb->job_epoch = epoch;
    flush_shared(solver, &g_shared_plan);
    amp_ipc_commit(mb);
    amp_ipc_sev();
    g_epoch = epoch;
    g_timing.dispatch_us = us_from_ticks(now_ticks() - t0);

    t0 = now_ticks();
    const int local_rc = rpgd_step_optimize_range(
        solver, &g_shared_plan, cpu0_first, cpu0_last, &g_cpu0_scratch);
    g_timing.cpu0_range_us = us_from_ticks(now_ticks() - t0);
    if (local_rc != RPGD_STATUS_OK) {
        fail_step(solver, local_rc, epoch, 0);
        return g_last_status;
    }

    const unsigned long long deadline = t_dispatch + ticks_from_us(RPGD_AMP_TIMEOUT_US);
    int completed = 0;
    t0 = now_ticks();
    while (now_ticks() <= deadline) {
        amp_ipc_refresh(mb);
        if (mb->completed_epoch == epoch
            && (mb->worker_state == AMP_WORKER_DONE || mb->worker_state == AMP_WORKER_READY)
            && mb->result_status == RPGD_STATUS_OK) {
            completed = 1;
            break;
        }
        if (mb->worker_state == AMP_WORKER_FAULT) break;
    }
    g_timing.barrier_us = us_from_ticks(now_ticks() - t0);
    amp_ipc_refresh(mb);
    g_timing.cpu1_range_us = us_from_ticks(
        (unsigned long long)(uint32_t)(mb->end_ticks - mb->start_ticks));

    if (!completed || mb->completed_epoch != epoch || mb->result_status != RPGD_STATUS_OK) {
        const int status = (mb->worker_state == AMP_WORKER_FAULT)
            ? RPGD_CONTROLLER_STATUS_AMP_FAULT
            : RPGD_CONTROLLER_STATUS_AMP_TIMEOUT;
        fail_step(solver, status, epoch, status == RPGD_CONTROLLER_STATUS_AMP_TIMEOUT);
        return g_last_status;
    }

    rpgd_cache_visit_rollout_slice(solver, cpu0_first, cpu0_last, flush_cb);
    invalidate_cpu1_slices(solver);
    t0 = now_ticks();
    const float action = rpgd_step_finalize(solver, &g_shared_plan);
    g_timing.finalize_us = us_from_ticks(now_ticks() - t0);
    g_timing.total_us = us_from_ticks(now_ticks() - t_all);
    g_last_status = rpgd_get_last_status(solver);
    /* Persist the complete CPU0 result in uncached OCM. The worker owns
     * result_status while a job runs; CPU0 owns it after completion. */
    publish_cpu0_result(g_last_status, action);
    if (u) *u = action;
    return g_last_status;
}

#else

int rpgd_amp_init(RpgdSolver* solver, uint32_t config_fingerprint)
{
    (void)solver;
    (void)config_fingerprint;
    return RPGD_CONTROLLER_STATUS_AMP_UNAVAILABLE;
}
void rpgd_amp_park(void) {}
int rpgd_amp_ready(void) { return 0; }
int rpgd_amp_last_status(void) { return RPGD_CONTROLLER_STATUS_AMP_UNAVAILABLE; }
unsigned int rpgd_amp_epoch(void) { return 0; }
unsigned int rpgd_amp_timeout_count(void) { return 0; }
unsigned int rpgd_amp_worker_state(void) { return 0; }
static const RpgdAmpTiming g_zero_timing;
const RpgdAmpTiming* rpgd_amp_last_timing(void) { return &g_zero_timing; }
int rpgd_amp_step(RpgdSolver* solver, const float* state6, const RpgdRuntime* runtime, float* u)
{
    (void)solver;
    (void)state6;
    (void)runtime;
    if (u) *u = 0.0f;
    return RPGD_CONTROLLER_STATUS_AMP_UNAVAILABLE;
}

#endif
