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
u32 get_slider_state();
float get_normed_slider_state();
#endif

#endif /* EXTERNAL_INTERFACE_H_ */

