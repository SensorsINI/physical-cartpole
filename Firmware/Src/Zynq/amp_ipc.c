#include "amp_ipc.h"

#include <string.h>

#ifdef __arm__
#include "xil_cache.h"
#include "xil_io.h"
#include "xpseudo_asm.h"
#endif

AmpMailbox* amp_ipc_mailbox(void)
{
    return (AmpMailbox*)(uintptr_t)AMP_IPC_MAILBOX_ADDR;
}

void amp_ipc_dmb(void)
{
#ifdef __arm__
    dmb();
#else
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
#endif
}

void amp_ipc_sev(void)
{
#ifdef __arm__
    __asm__ volatile("sev" ::: "memory");
#endif
}

void amp_ipc_wfe(void)
{
#ifdef __arm__
    __asm__ volatile("wfe" ::: "memory");
#endif
}

void amp_ipc_flush(const void* addr, size_t bytes)
{
#ifdef __arm__
    if (addr && bytes) Xil_DCacheFlushRange((INTPTR)addr, bytes);
#else
    (void)addr;
    (void)bytes;
#endif
}

void amp_ipc_invalidate(const void* addr, size_t bytes)
{
#ifdef __arm__
    if (addr && bytes) Xil_DCacheInvalidateRange((INTPTR)addr, bytes);
#else
    (void)addr;
    (void)bytes;
#endif
}

void amp_ipc_commit(AmpMailbox* mb)
{
    if (!mb) return;
    amp_ipc_dmb();
    amp_ipc_flush(mb, sizeof(*mb));
    amp_ipc_dmb();
}

void amp_ipc_refresh(AmpMailbox* mb)
{
    if (!mb) return;
    amp_ipc_invalidate(mb, sizeof(*mb));
    amp_ipc_dmb();
}

uint32_t amp_ipc_load_acquire_u32(const uint32_t* addr)
{
    uint32_t value = __atomic_load_n(addr, __ATOMIC_ACQUIRE);
    amp_ipc_dmb();
    return value;
}

void amp_ipc_store_release_u32(uint32_t* addr, uint32_t value)
{
    amp_ipc_dmb();
    __atomic_store_n(addr, value, __ATOMIC_RELEASE);
}

int amp_ipc_validate_ddr_ptr(uint32_t addr, size_t bytes)
{
    const uint32_t ddr_lo = 0x00100000u;
    const uint32_t ddr_hi = 0x40000000u;
    if (addr < ddr_lo) return 0;
    if (bytes > (size_t)(ddr_hi - addr)) return 0;
    if ((addr & 3u) != 0u) return 0;
    return 1;
}

int amp_ipc_mailbox_valid(const AmpMailbox* mb)
{
    return mb
        && mb->magic == AMP_IPC_MAGIC
        && mb->abi_version == AMP_IPC_ABI_VERSION;
}

void amp_ipc_init_cpu0(uint32_t config_fingerprint, uint32_t solver_sizeof,
                       uint32_t plan_sizeof, uint32_t scratch_sizeof)
{
    AmpMailbox* mb = amp_ipc_mailbox();
    amp_ipc_refresh(mb);
    const uint32_t state = mb->worker_state;
    const int recoverable = amp_ipc_mailbox_valid(mb)
        && state != AMP_WORKER_FAULT;
    if (!recoverable) {
        memset(mb, 0, sizeof(*mb));
        mb->magic = AMP_IPC_MAGIC;
        mb->abi_version = AMP_IPC_ABI_VERSION;
        mb->worker_state = AMP_WORKER_STARTING;
    }
    mb->command = AMP_CMD_SYNC;
    mb->job_epoch = 0u;
    mb->completed_epoch = 0u;
    mb->result_status = 0;
    mb->config_fingerprint = config_fingerprint;
    mb->solver_sizeof = solver_sizeof;
    mb->plan_sizeof = plan_sizeof;
    mb->scratch_sizeof = scratch_sizeof;
    amp_ipc_commit(mb);
    amp_ipc_sev();
}

void amp_ipc_start_cpu1(uint32_t entry_addr)
{
#ifdef __arm__
    Xil_Out32(AMP_IPC_CPU1_VECTOR, entry_addr);
    amp_ipc_dmb();
    amp_ipc_sev();
#else
    (void)entry_addr;
#endif
}
