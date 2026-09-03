#ifndef RPGD_CONTROLLER_H
#define RPGD_CONTROLLER_H

#include "controller_api.h"

#define RPGD_CONTROLLER_STATUS_DEADLINE_MISSED (-100)
#define RPGD_CONTROLLER_STATUS_AMP_UNAVAILABLE (-101)

extern const ControllerOps RPGD_Ops;

int rpgd_controller_last_status(void);
int rpgd_controller_is_ready(void);
unsigned int rpgd_controller_stride(void);
int rpgd_controller_owns_timing(void);
int rpgd_controller_take_deadline_grace(void);
void rpgd_controller_latch_fault(int status);

#endif
