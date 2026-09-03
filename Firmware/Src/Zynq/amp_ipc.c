#include "amp_ipc.h"

#include <string.h>

#ifdef __arm__
#include "xil_cache.h"
#include "xil_io.h"
#include "xil_mmu.h"
#include "xil_printf.h"
#include "xpseudo_asm.h"
#include "sleep.h"
#endif

#if defined(RPGD_DUAL_CORE) && !defined(RPGD_WORKER_ONLY)
#include "cpu1_blob_meta.h"

extern const uint8_t cpu1_blob_start[];
extern const uint8_t cpu1_blob_end[];
#endif

AmpMailbox* amp_ipc_mailbox(void)
{
    return (AmpMailbox*)(uintptr_t)AMP_IPC_MAILBOX_ADDR;
}

void amp_ipc_configure_regions(void)
{
#ifdef __arm__
    /*
     * CPU0's BSP maintains L1+L2 while the USE_AMP CPU1 BSP maintains only
     * L1. A write-back shared window therefore lets CPU0 discard CPU1's dirty
     * L2 lines during invalidate. Use write-through for the AMP exchange
     * window so reads remain cached but every core's writes reach DDR.
     */
    Xil_DCacheFlush();
    Xil_SetTlbAttributes((INTPTR)AMP_IPC_SHARED_ADDR, NORM_WT_CACHE);
    Xil_SetTlbAttributes((INTPTR)0xFFF00000u, STRONG_ORDERED);
    dsb();
#endif
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
    /* FlushRange walks lines with u32 += 32. A range that touches
     * 0xFFFFFFE0 wraps to 0 and flushes the whole address space. */
    uint32_t a;
    if (!addr || !bytes) return;
    a = (uint32_t)(uintptr_t)addr;
    if (a >= 0xFFFFFFE0u || bytes > (0xFFFFFFFFu - a)) {
        Xil_DCacheFlush();
        return;
    }
    Xil_DCacheFlushRange((INTPTR)addr, bytes);
#else
    (void)addr;
    (void)bytes;
#endif
}

void amp_ipc_invalidate(const void* addr, size_t bytes)
{
#ifdef __arm__
    if (addr && bytes) {
        Xil_DCacheInvalidateRange((INTPTR)addr, bytes);
    }
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
    /*
     * Default standalone maps high OCM inner-WB non-shareable. A cached
     * store to 0xFFFFFFF0 never becomes visible to CPU1 BootROM (JTAG DAP
     * still reads the 0xFFFFFF2C sentinel). FlushRange(0xFFFFFFF0) also
     * wraps and hangs CPU0. Map the 1 MB page uncached, then UG585+SEV.
     */
    amp_ipc_configure_regions();
    Xil_Out32(AMP_IPC_CPU1_VECTOR, entry_addr);
    dsb();
    amp_ipc_sev();
    usleep(100);
    amp_ipc_sev();
    {
        uint32_t rst;
        Xil_Out32(0xF8000008u, 0x0000DF0Du);
        rst = Xil_In32(0xF8000244u);
        rst &= ~(0x2u | 0x20u);
        Xil_Out32(0xF8000244u, rst);
        Xil_Out32(0xF8000004u, 0x0000767Bu);
    }
    dsb();
    amp_ipc_sev();
#else
    (void)entry_addr;
#endif
}

#if defined(RPGD_DUAL_CORE) && !defined(RPGD_WORKER_ONLY)
#ifdef __arm__
static int cpu1_worker_alive(AmpMailbox* mb)
{
    unsigned i;
    uint32_t hb;

    amp_ipc_refresh(mb);
    if (!amp_ipc_mailbox_valid(mb)) return 0;
    if (mb->worker_state != AMP_WORKER_READY
        && mb->worker_state != AMP_WORKER_DONE) {
        return 0;
    }
    /* Leftover OCM after rst-system can look READY with CPU1 dead. */
    hb = mb->heartbeat;
    mb->command = AMP_CMD_SYNC;
    amp_ipc_commit(mb);
    amp_ipc_sev();
    for (i = 0; i < 100u; ++i) {
        usleep(1000);
        amp_ipc_refresh(mb);
        if (amp_ipc_mailbox_valid(mb)
            && mb->worker_state == AMP_WORKER_READY
            && mb->heartbeat != hb) {
            return 1;
        }
        amp_ipc_sev();
    }
    return 0;
}
#endif

int amp_ipc_load_and_start_cpu1(void)
{
    AmpMailbox* mb = amp_ipc_mailbox();
    unsigned i;

#ifdef __arm__
    amp_ipc_configure_regions();
    if (cpu1_worker_alive(mb)) {
        xil_printf("CPU1 READY\r\n");
        return 0;
    }
#else
    amp_ipc_refresh(mb);
    if (amp_ipc_mailbox_valid(mb) && mb->worker_state == AMP_WORKER_READY) {
        return 0;
    }
#endif

    {
        const size_t n = (size_t)(cpu1_blob_end - cpu1_blob_start);
        if (n == 0u || n > AMP_IPC_CPU1_BLOB_MAX
            || (CPU1_BLOB_SIZE != 0u && n != (size_t)CPU1_BLOB_SIZE)) {
#ifdef __arm__
            xil_printf("CPU1 start FAIL\r\n");
#endif
            return -1;
        }
#ifdef __arm__
        memcpy((void*)(uintptr_t)AMP_IPC_CPU1_LOAD_ADDR, cpu1_blob_start, n);
        amp_ipc_flush((void*)(uintptr_t)AMP_IPC_CPU1_LOAD_ADDR, n);
        Xil_ICacheInvalidateRange((INTPTR)AMP_IPC_CPU1_LOAD_ADDR, n);
#else
        (void)n;
        return -1;
#endif
    }

    memset(mb, 0, sizeof(*mb));
    mb->magic = AMP_IPC_MAGIC;
    mb->abi_version = AMP_IPC_ABI_VERSION;
    mb->worker_state = AMP_WORKER_STARTING;
    amp_ipc_commit(mb);
    amp_ipc_start_cpu1(CPU1_ENTRY_ADDR);

#ifdef __arm__
    for (i = 0; i < (AMP_IPC_CPU1_READY_TIMEOUT_US / 1000u); ++i) {
        amp_ipc_refresh(mb);
        if (amp_ipc_mailbox_valid(mb) && mb->worker_state == AMP_WORKER_READY) {
            xil_printf("CPU1 READY\r\n");
            return 0;
        }
        amp_ipc_sev();
        usleep(1000);
    }
    xil_printf("CPU1 start FAIL\r\n");
#else
    (void)i;
#endif
    return -1;
}
#endif
