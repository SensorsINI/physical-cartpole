#include "amp_ipc.h"
#include "rpgd_c/rpgd_cartpole.h"
#include "rpgd_c/rpgd_platform.h"
#include "rpgd_c/rpgd_worker.h"

#include "xil_cache.h"
#include "xtime_l.h"

#include <math.h>
#include <stdint.h>

static RpgdWorkerScratch g_cpu1_scratch RPGD_ALIGN64;

static void flush_cb(const void* addr, size_t bytes) { amp_ipc_flush(addr, bytes); }
static void inv_cb(const void* addr, size_t bytes) { amp_ipc_invalidate(addr, bytes); }

static void enable_vfp(void)
{
    unsigned int cpacr;
    __asm__ volatile("mrc p15, 0, %0, c1, c0, 2" : "=r"(cpacr));
    cpacr |= (0xFu << 20);
    __asm__ volatile("mcr p15, 0, %0, c1, c0, 2" :: "r"(cpacr));
    __asm__ volatile("isb" ::: "memory");
    unsigned int fpexc = 0x40000000u;
    __asm__ volatile("vmsr fpexc, %0" :: "r"(fpexc) : "memory");
}

static uint32_t now_ticks_u32(void)
{
    XTime t = 0;
    XTime_GetTime(&t);
    return (uint32_t)t;
}

static int finite_range_costs(RpgdSolver* solver, int first, int last)
{
    float costs[RPGD_MAX_NUM_ROLLOUTS];
    const int n = rpgd_get_num_rollouts(solver);
    if (n <= 0 || n > RPGD_MAX_NUM_ROLLOUTS) return 0;
    rpgd_debug_get_costs(solver, costs);
    for (int i = first; i < last && i < n; ++i) {
        if (!isfinite(costs[i]) && costs[i] != INFINITY) return 0;
    }
    return 1;
}

static void publish_mailbox(AmpMailbox* mb)
{
    amp_ipc_commit(mb);
    amp_ipc_sev();
}

static void enter_fault(AmpMailbox* mb, int status)
{
    mb->result_status = status;
    mb->worker_state = AMP_WORKER_FAULT;
    publish_mailbox(mb);
}

static void publish_ready(AmpMailbox* mb)
{
    mb->solver_sizeof = (uint32_t)rpgd_get_solver_size();
    mb->plan_sizeof = (uint32_t)sizeof(RpgdStepPlan);
    mb->scratch_sizeof = (uint32_t)sizeof(RpgdWorkerScratch);
    mb->result_status = RPGD_STATUS_OK;
    mb->heartbeat += 1u;
    mb->worker_state = AMP_WORKER_READY;
    publish_mailbox(mb);
}

static void wait_for_valid_mailbox(AmpMailbox* mb)
{
    for (;;) {
        amp_ipc_refresh(mb);
        if (amp_ipc_mailbox_valid(mb)) return;
        amp_ipc_wfe();
    }
}

int main(void)
{
    enable_vfp();
    Xil_ICacheEnable();
    Xil_DCacheEnable();

    AmpMailbox* mb = amp_ipc_mailbox();
    wait_for_valid_mailbox(mb);

    mb->heartbeat = 1u;
    publish_ready(mb);

    uint32_t seen_epoch = 0;
    int parked = 0;
    for (;;) {
        amp_ipc_wfe();
        amp_ipc_refresh(mb);
        if (!amp_ipc_mailbox_valid(mb)) continue;

        const uint32_t command = mb->command;
        if (command == AMP_CMD_SYNC) {
            parked = 0;
            seen_epoch = 0;
            mb->completed_epoch = 0u;
            mb->job_epoch = 0u;
            mb->command = AMP_CMD_NOP;
            publish_ready(mb);
            continue;
        }
        if (command == AMP_CMD_STOP) {
            parked = 1;
            mb->worker_state = AMP_WORKER_OFFLINE;
            mb->heartbeat += 1u;
            publish_mailbox(mb);
            continue;
        }
        if (parked) continue;
        if (command != AMP_CMD_OPTIMIZE_RANGE) continue;

        const uint32_t epoch = mb->job_epoch;
        if (epoch == 0u || epoch <= seen_epoch) continue;
        if (epoch != seen_epoch + 1u) {
            enter_fault(mb, RPGD_STATUS_WORKER_FAILURE);
            continue;
        }

        mb->worker_state = AMP_WORKER_BUSY;
        mb->heartbeat += 1u;
        publish_mailbox(mb);

        if (!amp_ipc_validate_ddr_ptr(mb->solver_addr, rpgd_get_solver_size())
            || !amp_ipc_validate_ddr_ptr(mb->plan_addr, sizeof(RpgdStepPlan))) {
            enter_fault(mb, RPGD_STATUS_WORKER_FAILURE);
            continue;
        }

        RpgdSolver* solver = (RpgdSolver*)(uintptr_t)mb->solver_addr;
        RpgdStepPlan* plan = (RpgdStepPlan*)(uintptr_t)mb->plan_addr;
        amp_ipc_invalidate(solver, rpgd_get_solver_size());
        rpgd_cache_visit_solver(solver, inv_cb);
        const int first = mb->range_first;
        const int last = mb->range_last;
        rpgd_cache_visit_rollout_slice(solver, first, last, inv_cb);
        amp_ipc_invalidate(plan, sizeof(*plan));

        mb->start_ticks = now_ticks_u32();
        const int rc = rpgd_step_optimize_range(solver, plan, first, last, &g_cpu1_scratch);
        mb->end_ticks = now_ticks_u32();
        if (rc != RPGD_STATUS_OK || !finite_range_costs(solver, first, last)) {
            enter_fault(mb, rc != RPGD_STATUS_OK ? rc : RPGD_STATUS_NUMERICAL_FAILURE);
            continue;
        }

        rpgd_cache_visit_rollout_slice(solver, first, last, flush_cb);
        seen_epoch = epoch;
        mb->result_status = RPGD_STATUS_OK;
        mb->completed_epoch = epoch;
        mb->worker_state = AMP_WORKER_READY;
        mb->heartbeat += 1u;
        publish_mailbox(mb);
    }
}
