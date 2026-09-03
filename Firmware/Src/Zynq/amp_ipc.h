#ifndef AMP_IPC_H
#define AMP_IPC_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AMP_IPC_MAGIC          0x52504744u /* 'RPGD' */
#define AMP_IPC_ABI_VERSION    1u
#define AMP_IPC_MAILBOX_ADDR   0xFFFF0000u
#define AMP_IPC_CPU1_VECTOR    0xFFFFFFF0u
#define AMP_IPC_CPU1_LOAD_ADDR 0x10000000u
#define AMP_IPC_CPU1_BLOB_MAX  (64u * 1024u)
#define AMP_IPC_CPU1_READY_TIMEOUT_US 2000000u
#define AMP_IPC_SHARED_ADDR    0x20000000u
#define AMP_IPC_SHARED_SIZE    0x00100000u

enum AmpWorkerState {
    AMP_WORKER_OFFLINE  = 0,
    AMP_WORKER_STARTING = 1,
    AMP_WORKER_READY    = 2,
    AMP_WORKER_BUSY     = 3,
    AMP_WORKER_DONE     = 4,
    AMP_WORKER_FAULT    = 5
};

enum AmpCommand {
    AMP_CMD_NOP            = 0,
    AMP_CMD_OPTIMIZE_RANGE = 1,
    AMP_CMD_STOP           = 2,
    AMP_CMD_SYNC           = 3
};

/*
 * High OCM is inner-writeback and non-shareable on the Zynq A9 default MMU
 * table. Every mailbox publish must flush the whole struct; every consume
 * must invalidate the whole struct. 4-byte field flushes are not enough.
 */
typedef struct AmpMailbox {
    uint32_t magic;
    uint32_t abi_version;
    uint32_t worker_state;
    uint32_t command;
    uint32_t job_epoch;
    uint32_t completed_epoch;
    int32_t  range_first;
    int32_t  range_last;
    uint32_t solver_addr;
    uint32_t plan_addr;
    int32_t  result_status;
    uint32_t heartbeat;
    uint32_t start_ticks;
    uint32_t end_ticks;
    uint32_t config_fingerprint;
    uint32_t solver_sizeof;
    uint32_t plan_sizeof;
    uint32_t scratch_sizeof;
    uint32_t reserved[8];
} AmpMailbox;

_Static_assert(sizeof(AmpMailbox) % 4u == 0u, "AmpMailbox must be 32-bit sized");
_Static_assert(offsetof(AmpMailbox, magic) == 0u, "AmpMailbox.magic offset");
_Static_assert(offsetof(AmpMailbox, abi_version) == 4u, "AmpMailbox.abi_version offset");
_Static_assert(offsetof(AmpMailbox, worker_state) == 8u, "AmpMailbox.worker_state offset");
_Static_assert(offsetof(AmpMailbox, command) == 12u, "AmpMailbox.command offset");
_Static_assert(offsetof(AmpMailbox, job_epoch) == 16u, "AmpMailbox.job_epoch offset");
_Static_assert(offsetof(AmpMailbox, completed_epoch) == 20u, "AmpMailbox.completed_epoch offset");
_Static_assert(offsetof(AmpMailbox, range_first) == 24u, "AmpMailbox.range_first offset");
_Static_assert(offsetof(AmpMailbox, range_last) == 28u, "AmpMailbox.range_last offset");
_Static_assert(offsetof(AmpMailbox, solver_addr) == 32u, "AmpMailbox.solver_addr offset");
_Static_assert(offsetof(AmpMailbox, plan_addr) == 36u, "AmpMailbox.plan_addr offset");
_Static_assert(offsetof(AmpMailbox, result_status) == 40u, "AmpMailbox.result_status offset");
_Static_assert(offsetof(AmpMailbox, heartbeat) == 44u, "AmpMailbox.heartbeat offset");
_Static_assert(offsetof(AmpMailbox, reserved) == 72u, "AmpMailbox.reserved offset");
_Static_assert(sizeof(AmpMailbox) == 104u, "AmpMailbox size must stay versioned");
_Static_assert(AMP_IPC_MAILBOX_ADDR + sizeof(AmpMailbox) < AMP_IPC_CPU1_VECTOR,
               "mailbox must stay below the CPU1 release vector");

AmpMailbox* amp_ipc_mailbox(void);
void amp_ipc_configure_regions(void);
void amp_ipc_dmb(void);
void amp_ipc_sev(void);
void amp_ipc_wfe(void);
void amp_ipc_flush(const void* addr, size_t bytes);
void amp_ipc_invalidate(const void* addr, size_t bytes);
void amp_ipc_commit(AmpMailbox* mb);
void amp_ipc_refresh(AmpMailbox* mb);
uint32_t amp_ipc_load_acquire_u32(const uint32_t* addr);
void amp_ipc_store_release_u32(uint32_t* addr, uint32_t value);
int amp_ipc_validate_ddr_ptr(uint32_t addr, size_t bytes);
int amp_ipc_mailbox_valid(const AmpMailbox* mb);
void amp_ipc_init_cpu0(uint32_t config_fingerprint, uint32_t solver_sizeof,
                       uint32_t plan_sizeof, uint32_t scratch_sizeof);
void amp_ipc_start_cpu1(uint32_t entry_addr);
#if defined(RPGD_DUAL_CORE) && !defined(RPGD_WORKER_ONLY)
int amp_ipc_load_and_start_cpu1(void);
#endif

#ifdef __cplusplus
}
#endif

#endif
