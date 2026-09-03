#ifndef CONTROLLER_PROFILES_H_
#define CONTROLLER_PROFILES_H_

#include <stdbool.h>
#include "onchip_controllers.h"

/*
 * Show mux: Zybo SW0–SW3 pick the on-chip controller. Timing / PWM / motor
 * map / slider / derivative median / FPGA dead-zone live here, not in
 * IROS_SHORT_POLE_PROFILE.
 *
 * SW0 RPGD, SW1 Dense-8 C, SW2 LSTM, SW3 short-pole PL. One-hot only.
 */

#define SHOW_SWITCH_MUX_MASK 0x0Fu

bool show_switch_mux_enabled(void);

/* Decode SW0–SW3. Returns an OnChipController_* id, or SHOW_MUX_IDLE. */
int show_mux_controller_from_switches(unsigned int switch_bits);

/*
 * Same decode, but the reading must repeat for two consecutive polls
 * before it is committed. Returns SHOW_MUX_UNSTABLE while waiting.
 */
int show_mux_debounced_controller(unsigned int switch_bits);

/* Apply plant/timing for that controller (SHOW_MUX_IDLE → long-pole default). */
void apply_show_profile_for_controller(int controller_id);

#endif
