#include "xscugic.h"
#include "xgpio.h"
#include "external_interface.h"
#include <stdio.h>

XGpio ExternalSwitchGpio;

u32 ButtonState = 0;
u32 NewButtonState = 0;
int TargetEquilibrium = 0;

void GPIO_external_button_interrupt(void *InstancePtr);
int ExternalButtonInit();


void ExternalInterfaceInit()
{
#ifdef XPAR_EQUILIBRIUM_SWITCH_GPIO_DEVICE_ID
	ExternalButtonInit();
#endif
}

#ifdef XPAR_EQUILIBRIUM_SWITCH_GPIO_DEVICE_ID
void GPIO_external_button_interrupt(void *InstancePtr) {

    XGpio_InterruptClear(&ExternalSwitchGpio, 1);
    NewButtonState = XGpio_DiscreteRead(&ExternalSwitchGpio, 1);

    if ((NewButtonState != 0) && (NewButtonState != 3) && (NewButtonState != ButtonState)){
    	// The case with NewButtonState == 3 should not happen (both sides of the button pressed at the same time)
    	// But we do observe it, also sometimes observe wring side active which probably indicates that button is faulty.
    	ButtonState = NewButtonState;
    	if (ButtonState == 2){
    		TargetEquilibrium = 1;
    	} else if (ButtonState == 1){
    		TargetEquilibrium = -1;
    	}
    }
}


int ExternalButtonInit() {

    // Initialize GPIO device
    XGpio_Initialize(&ExternalSwitchGpio, XPAR_EQUILIBRIUM_SWITCH_GPIO_DEVICE_ID);


    // Set the direction for all signals to be inputs
    XGpio_SetDataDirection(&ExternalSwitchGpio, 1, 0xFFFFFFFF);

    // Enable GPIO interrupts
    XGpio_InterruptEnable(&ExternalSwitchGpio, 3); // 3 is just a binary 11, which enables the two bits of GPIO to raise interrupt
    XGpio_InterruptGlobalEnable(&ExternalSwitchGpio);

    // Set up GPIO interrupt system
    XScuGic_Connect(&XScuGicInstance, XPAR_FABRIC_EQUILIBRIUM_SWITCH_GPIO_IP2INTC_IRPT_INTR,
                             (Xil_ExceptionHandler)GPIO_external_button_interrupt, (void *)&ExternalSwitchGpio);

    // Enable GPIO interrupts in the controller
    XScuGic_Enable(&XScuGicInstance, XPAR_FABRIC_EQUILIBRIUM_SWITCH_GPIO_IP2INTC_IRPT_INTR);

    return XST_SUCCESS;
}
#endif

int get_target_equilibrium_from_external_button(){
	return TargetEquilibrium;
}

u32 get_external_button_state(){
	return ButtonState;
}


#define AD1_DATA_MASK 0xFFF
u32 slider_value = 2048;
u32 get_slider_state(){
#ifdef XPAR_PMODAD1_BASEADDR
	u32 ADC_data;
	ADC_data = Xil_In32(XPAR_PMODAD1_BASEADDR);
//	slider_value = ADC_data & AD1_DATA_MASK;
	slider_value = (ADC_data >> 1) & AD1_DATA_MASK;
//	slider_value = (data >> 17) & AD1_DATA_MASK; // to use second available ADC
#endif
	return slider_value;

}

float get_normed_slider_state(){
	float normed_slider_state = get_slider_state();
	normed_slider_state = -1.0*((normed_slider_state/2048.0)-1.0);
	return normed_slider_state;
}

