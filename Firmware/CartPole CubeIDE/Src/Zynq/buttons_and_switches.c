#include "buttons_and_switches.h"

XGpioPs GpioPS; // The Instance of the GPIO Driver
XGpio Gpio;

ActionHandler btn4_action_handler = NULL;
ActionHandler btn5_action_handler = NULL;


static void Btn_Intr_Handler(void *CallBackRef);

void Buttons_And_Switches_Init(){
    XGpioPs_Config *GPIOConfigPtr;

    // GPIO driver initialization
    GPIOConfigPtr = XGpioPs_LookupConfig(GPIO_DEVICE_ID);
    XGpioPs_CfgInitialize(&GpioPS, GPIOConfigPtr, GPIOConfigPtr->BaseAddr);

    // Register GPIO interrupt handler
    XScuGic_Connect(&XScuGicInstance, GPIO_INTERRUPT_ID,
					(Xil_InterruptHandler)Btn_Intr_Handler,
					(void *)&GpioPS);

    // Enable GPIO interrupts in the controller
    XScuGic_Enable(&XScuGicInstance, GPIO_INTERRUPT_ID);

    // Set direction for button pins
    XGpioPs_SetDirectionPin(&GpioPS, PS_BTN_4, 0);
    XGpioPs_SetDirectionPin(&GpioPS, PS_BTN_5, 0);

    // GPIO for Switches on PL
    XGpio_Initialize(&Gpio, XPAR_GPIO_0_DEVICE_ID);
    XGpio_SetDataDirection(&Gpio, 1, 1);
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
	    XGpioPs_SetIntrTypePin(&GpioPS, PS_BTN_4, XGPIOPS_IRQ_TYPE_EDGE_RISING);
	    XGpioPs_IntrEnablePin(&GpioPS, PS_BTN_4);
		XGpioPs_IntrClearPin(&GpioPS, PS_BTN_4);
	    btn4_action_handler = action;
	} else if (key == PS_BTN_5){
	    XGpioPs_SetIntrTypePin(&GpioPS, PS_BTN_5, XGPIOPS_IRQ_TYPE_EDGE_RISING);
	    XGpioPs_IntrEnablePin(&GpioPS, PS_BTN_5);
		XGpioPs_IntrClearPin(&GpioPS, PS_BTN_5);
		btn5_action_handler = action;
	} else {
		xil_printf("Unrecognized button, no action set");
	}

}

u32 Switches_GetState()
{
	return XGpio_DiscreteRead(&Gpio, 1);
}

// TODO: This should be rather implemented as an interrupt!
u32 Switch_GetState(u32 switch_number)
{
	u32 switch_mask = 1 << switch_number;
	return ((Switches_GetState() & switch_mask) >> switch_number);
}


