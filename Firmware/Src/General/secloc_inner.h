#ifndef SECLOC_INNER_H
#define SECLOC_INNER_H

/*
 * Select which controller SecLoc wraps on Zynq.
 * Default: SECLOC_DEFAULT_INNER_CONTROLLER in secloc_defaults.h (override at
 * runtime with secloc_set_inner_controller() if needed).
 */

typedef enum {
    SECLOC_INNER_LQR = 0,
    SECLOC_INNER_PID = 1,
    SECLOC_INNER_NNC = 2,  /* pure-C neural imitator (NNC_Ops, NC_C/network.c) */
} SeclocInnerController;

extern SeclocInnerController secloc_inner_controller;

void secloc_set_inner_controller(SeclocInnerController inner);
SeclocInnerController secloc_get_inner_controller(void);

#endif /* SECLOC_INNER_H */
