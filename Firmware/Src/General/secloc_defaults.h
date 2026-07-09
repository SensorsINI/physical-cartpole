#ifndef SECLOC_DEFAULTS_H
#define SECLOC_DEFAULTS_H

/*
 * Default SecLoc gate profile for on-chip control.
 * Keep in sync with Driver/CartPoleSimulation/Control_Toolkit_ASF/config_secloc.yml
 * profile "default". When the PC driver is connected it overwrites these values
 * on the chip (CMD_SET_SECLOC_CONFIG) with the current yaml contents, so the
 * defaults only matter when the chip runs standalone.
 */
#define SECLOC_DEFAULT_LOG_BASE         1.05f
/* Throttle in control loop iterations (POLLING_PERIOD_MS each): after an
 * accepted update the gate is next consulted this many iterations later.
 * 0 or 1 = gate checked every iteration. 4 x 5 ms = 20 ms. */
#define SECLOC_DEFAULT_REF_PERIOD_TICKS 4
#define SECLOC_DEFAULT_DEAD_ANG         0.001f
#define SECLOC_DEFAULT_DEAD_POS         0.001f

/* Tick size of the gate's ref_period throttle. Must equal the control loop
 * period; control.c overrides it from POLLING_PERIOD_MS at init and whenever
 * the PC reconfigures the loop period. */
#define SECLOC_DEFAULT_TIME_QUANTUM_S 0.005f

#endif /* SECLOC_DEFAULTS_H */
