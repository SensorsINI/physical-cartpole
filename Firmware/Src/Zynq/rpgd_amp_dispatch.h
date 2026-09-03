#ifndef RPGD_AMP_DISPATCH_H
#define RPGD_AMP_DISPATCH_H

#include "rpgd_c/rpgd_cartpole.h"
#include "rpgd_c/rpgd_worker.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RPGD_AMP_CPU0_FIRST           0
#define RPGD_AMP_CPU0_LAST            4
#define RPGD_AMP_CPU1_FIRST           4
#define RPGD_AMP_CPU1_LAST            8
#define RPGD_AMP_TIMEOUT_US           19000u
#define RPGD_AMP_DRAIN_TIMEOUT_US     50000u
#define RPGD_AMP_READY_TIMEOUT_US     2000000u
#ifndef RPGD_CONTROLLER_STATUS_AMP_UNAVAILABLE
#define RPGD_CONTROLLER_STATUS_AMP_UNAVAILABLE (-101)
#endif
#define RPGD_CONTROLLER_STATUS_AMP_TIMEOUT     (-102)
#define RPGD_CONTROLLER_STATUS_AMP_FAULT       (-103)

typedef struct RpgdAmpTiming {
    unsigned int prepare_us;
    unsigned int dispatch_us;
    unsigned int cpu0_range_us;
    unsigned int cpu1_range_us;
    unsigned int barrier_us;
    unsigned int finalize_us;
    unsigned int total_us;
} RpgdAmpTiming;

int rpgd_amp_init(RpgdSolver* solver, uint32_t config_fingerprint);
void rpgd_amp_park(void);
int rpgd_amp_ready(void);
int rpgd_amp_last_status(void);
unsigned int rpgd_amp_epoch(void);
unsigned int rpgd_amp_timeout_count(void);
unsigned int rpgd_amp_worker_state(void);
const RpgdAmpTiming* rpgd_amp_last_timing(void);

int rpgd_amp_step(RpgdSolver* solver, const float* state6, const RpgdRuntime* runtime, float* u);

#ifdef __cplusplus
}
#endif

#endif
