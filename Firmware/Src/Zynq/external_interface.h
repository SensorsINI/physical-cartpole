#ifndef EXTERNAL_INTERFACE_H_
#define EXTERNAL_INTERFACE_H_

#include "xscugic.h"

extern XScuGic XScuGicInstance; // The Instance of the Interrupt Controller Driver

#ifdef XPAR_EQUILIBRIUM_SWITCH_GPIO_DEVICE_ID
void ExternalInterfaceInit();
u32 get_external_button_state();
int get_target_equilibrium_from_external_button();
#endif

#ifdef XPAR_PMODAD1_BASEADDR
/* Decoded PmodAD1 ch1/D1: (raw >> 17) & 0xFFF. */
u32 get_slider_state();
/* −1…+1 for CartPoleFirmware: left rail −1, electrical mid 0, right rail +1.
 * Affine ADC rails in external_interface.c; keep tools/slider_pmod in sync. */
float get_normed_slider_state();
#endif

#endif /* EXTERNAL_INTERFACE_H_ */

