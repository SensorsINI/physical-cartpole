#include "buttons_and_switches.h"

XGpioPs Gpio; // The Instance of the GPIO Driver

ActionHandler btn4_action_handler = NULL;
ActionHandler btn5_action_handler = NULL;


static void Btn_Intr_Handler(void *CallBackRef);

void Buttons_And_Switches_Init(){
    XGpioPs_Config *GPIOConfigPtr;

    // GPIO driver initialization
    GPIOConfigPtr = XGpioPs_LookupConfig(GPIO_DEVICE_ID);
    XGpioPs_CfgInitialize(&Gpio, GPIOConfigPtr, GPIOConfigPtr->BaseAddr);

    // Register GPIO interrupt handler
    XScuGic_Connect(&XScuGicInstance, GPIO_INTERRUPT_ID,
					(Xil_InterruptHandler)Btn_Intr_Handler,
					(void *)&Gpio);

    // Enable GPIO interrupts in the controller
    XScuGic_Enable(&XScuGicInstance, GPIO_INTERRUPT_ID);

    // Set direction for button pins
    XGpioPs_SetDirectionPin(&Gpio, PS_BTN_4, 0);
    XGpioPs_SetDirectionPin(&Gpio, PS_BTN_5, 0);

}



// This function will be called every time a button interrupt rises
static void Btn_Intr_Handler(void *CallBackRef)
{
    XGpioPs *GpioInstancePtr = (XGpioPs *)CallBackRef;

    // Check which button caused the interrupt and call the corresponding function via the function pointer
    if(XGpioPs_IntrGetStatusPin(GpioInstancePtr, PS_BTN_4)){
        XGpioPs_IntrClearPin(GpioInstancePtr, PS_BTN_4); // Clear the interrupt for BTN 4
        if (btn4_action_handler != NULL) {
            btn4_action_handler(); // Call the function pointed to by btn4_action_handler
        }
    }
    if(XGpioPs_IntrGetStatusPin(GpioInstancePtr, PS_BTN_5)){
        XGpioPs_IntrClearPin(GpioInstancePtr, PS_BTN_5); // Clear the interrupt for BTN 5
        if (btn5_action_handler != NULL) {
            btn5_action_handler(); // Call the function pointed to by btn5_action_handler
        }
    }
}



void Button_SetAction(unsigned int key, ActionHandler action){
	if (key == PS_BTN_4){
	    // Enable button interrupts
	    XGpioPs_SetIntrTypePin(&Gpio, PS_BTN_4, XGPIOPS_IRQ_TYPE_EDGE_RISING);
	    XGpioPs_IntrEnablePin(&Gpio, PS_BTN_4);
		XGpioPs_IntrClearPin(&Gpio, PS_BTN_4);
	    btn4_action_handler = action;
	} else if (key == PS_BTN_5){
	    XGpioPs_SetIntrTypePin(&Gpio, PS_BTN_5, XGPIOPS_IRQ_TYPE_EDGE_RISING);
	    XGpioPs_IntrEnablePin(&Gpio, PS_BTN_5);
		XGpioPs_IntrClearPin(&Gpio, PS_BTN_5);
		btn5_action_handler = action;
	} else {
		xil_printf("Unrecognized button, no action set");
	}

}
