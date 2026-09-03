#ifndef ONCHIP_CONTROLLERS_H_
#define ONCHIP_CONTROLLERS_H_

/*
 * Shared on-chip controller ids. control.c and controller_profiles.c must
 * agree: the show mux returns these values.
 */
#define OnChipController_PID 0
#define OnChipController_NeuralImitator 1
#define OnChipController_PID_position 2
#define OnChipController_LQR 3
#define OnChipController_neural_controller_C 4
#define OnChipController_SECLOC 5
#define OnChipController_SECLOC_LQR 6
#define OnChipController_neural_controller_LSTM_C 7
#define OnChipController_RPGD 8
#define OnChipController_NONE 0xFFFF

/* Decoder sentinels (signed). Not stored in current_controller. */
#define SHOW_MUX_IDLE     (-1)
#define SHOW_MUX_UNSTABLE (-2)

#endif
