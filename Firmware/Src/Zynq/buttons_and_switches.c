#include "buttons_and_switches.h"
#include "timer_interrupt.h"
#include "xil_printf.h"
#include "xgpio_l.h"
#include "hw_platform_config.h"
#include <stdint.h>

XGpioPs GpioPS; // The Instance of the GPIO Driver
XGpio Gpio;
#if HW_HAS_PL_BUTTONS
static XGpio GpioPlButtons;
#endif

ActionHandler btn4_action_handler = NULL;
ActionHandler btn5_action_handler = NULL;

#define DEBOUNCE_TIME_MS 100  // Debounce time in milliseconds

static uint32_t last_interrupt_time_btn4 = 0;
static uint32_t last_interrupt_time_btn5 = 0;

static void Btn_Intr_Handler(void *CallBackRef);

#if HW_HAS_PL_BUTTONS
static ActionHandler pl_btn_action_handler[4] = {NULL, NULL, NULL, NULL};
static uint32_t last_interrupt_time_pl_btn[4] = {0, 0, 0, 0};
static u32 pl_buttons_prev_state = 0;
static u32 pl_buttons_irq_id = 0;
static int pl_buttons_irq_enabled = 0;
static int pl_buttons_present = 0;
static void PlBtn_Intr_Handler(void *CallBackRef);

static int PlButtons_Init(void)
{
	int status;

	status = XGpio_Initialize(&GpioPlButtons, HW_PL_BUTTONS_GPIO_DEVICE_ID);
	if (status != XST_SUCCESS) {
		return status;
	}
	pl_buttons_irq_id = HW_PL_BUTTONS_IRQ;

	XGpio_SetDataDirection(&GpioPlButtons, 1, 0xF);
	pl_buttons_prev_state = XGpio_DiscreteRead(&GpioPlButtons, 1) & 0xF;

	XScuGic_SetPriorityTriggerType(&XScuGicInstance, pl_buttons_irq_id, 0xA0, 0x3);

	status = XScuGic_Connect(&XScuGicInstance, pl_buttons_irq_id,
				 (Xil_InterruptHandler)PlBtn_Intr_Handler,
				 (void *)&GpioPlButtons);
	if (status != XST_SUCCESS) {
		return status;
	}

	XGpio_InterruptDisable(&GpioPlButtons, XGPIO_IR_CH1_MASK);
	XGpio_InterruptClear(&GpioPlButtons, XGPIO_IR_CH1_MASK);
	pl_buttons_present = 1;
	return XST_SUCCESS;
}
#endif

void Buttons_And_Switches_Init(){
    XGpioPs_Config *GPIOConfigPtr;

    // GPIO driver initialization
    GPIOConfigPtr = XGpioPs_LookupConfig(XPAR_XGPIOPS_0_DEVICE_ID);
    XGpioPs_CfgInitialize(&GpioPS, GPIOConfigPtr, GPIOConfigPtr->BaseAddr);

    // Register GPIO interrupt handler
    XScuGic_Connect(&XScuGicInstance, XPAR_XGPIOPS_0_INTR,
					(Xil_InterruptHandler)Btn_Intr_Handler,
					(void *)&GpioPS);

    // Enable GPIO interrupts in the controller
    XScuGic_Enable(&XScuGicInstance, XPAR_XGPIOPS_0_INTR);

    // Set direction for button pins
    XGpioPs_SetDirectionPin(&GpioPS, PS_BTN_4, 0);
    XGpioPs_SetDirectionPin(&GpioPS, PS_BTN_5, 0);

    // GPIO for Switches on PL
    XGpio_Initialize(&Gpio, XPAR_SWITCHES_AND_LEDS_GPIO_DEVICE_ID);
    XGpio_SetDataDirection(&Gpio, 1, 1);

#if HW_HAS_PL_BUTTONS
    if (PlButtons_Init() != XST_SUCCESS) {
	    xil_printf("PL buttons GPIO init failed\r\n");
    }
#else
    xil_printf("PL buttons skipped: refresh Vitis platform from the PL-buttons XSA\r\n");
#endif
}


// This function will be called every time a button interrupt rises
static void Btn_Intr_Handler(void *CallBackRef)
{
    XGpioPs *GpioInstancePtr = (XGpioPs *)CallBackRef;
    uint32_t current_time = TIMER1_getSystemTime_Us()/1000;

    // Check which button caused the interrupt and call the corresponding function via the function pointer
    if(XGpioPs_IntrGetStatusPin(GpioInstancePtr, PS_BTN_4)){
        if ((current_time - last_interrupt_time_btn4) > DEBOUNCE_TIME_MS) {
            last_interrupt_time_btn4 = current_time;
            XGpioPs_IntrClearPin(GpioInstancePtr, PS_BTN_4); // Clear the interrupt for BTN 4
            if (btn4_action_handler != NULL) {
                btn4_action_handler(); // Call the function pointed to by btn4_action_handler
            }
        } else {
            XGpioPs_IntrClearPin(GpioInstancePtr, PS_BTN_4); // Clear the interrupt to prevent re-triggering
        }
    }
    if(XGpioPs_IntrGetStatusPin(GpioInstancePtr, PS_BTN_5)){
        if ((current_time - last_interrupt_time_btn5) > DEBOUNCE_TIME_MS) {
            last_interrupt_time_btn5 = current_time;
            XGpioPs_IntrClearPin(GpioInstancePtr, PS_BTN_5); // Clear the interrupt for BTN 5
            if (btn5_action_handler != NULL) {
                btn5_action_handler(); // Call the function pointed to by btn5_action_handler
            }
        } else {
            XGpioPs_IntrClearPin(GpioInstancePtr, PS_BTN_5); // Clear the interrupt to prevent re-triggering
        }
    }
}

#if HW_HAS_PL_BUTTONS
static void PlBtn_Intr_Handler(void *CallBackRef)
{
	XGpio *gpio = (XGpio *)CallBackRef;
	u32 now_ms = TIMER1_getSystemTime_Us() / 1000;
	u32 state = XGpio_DiscreteRead(gpio, 1) & 0xF;
	u32 rising = (state ^ pl_buttons_prev_state) & state;
	unsigned int i;

	XGpio_InterruptClear(gpio, XGPIO_IR_CH1_MASK);
	pl_buttons_prev_state = state;

	for (i = 0; i < 4; ++i) {
		if ((rising & (1u << i)) == 0) {
			continue;
		}
		if ((now_ms - last_interrupt_time_pl_btn[i]) <= DEBOUNCE_TIME_MS) {
			continue;
		}
		last_interrupt_time_pl_btn[i] = now_ms;
		if (pl_btn_action_handler[i] != NULL) {
			pl_btn_action_handler[i]();
		}
	}
}
#endif


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
	} else if (key <= PL_BTN_3) {
		/* PL_BTN_0 is Zybo BTN0 (BUTTON_3 in hardware_bridge), not a third PS button. */
#if HW_HAS_PL_BUTTONS
		if (!pl_buttons_present) {
			xil_printf("PL buttons not available, no action set\r\n");
			return;
		}
		pl_btn_action_handler[key] = action;
		if (!pl_buttons_irq_enabled) {
			XGpio_InterruptClear(&GpioPlButtons, XGPIO_IR_CH1_MASK);
			pl_buttons_prev_state = XGpio_DiscreteRead(&GpioPlButtons, 1) & 0xF;
			XGpio_InterruptEnable(&GpioPlButtons, XGPIO_IR_CH1_MASK);
			XGpio_InterruptGlobalEnable(&GpioPlButtons);
			XScuGic_Enable(&XScuGicInstance, pl_buttons_irq_id);
			pl_buttons_irq_enabled = 1;
		}
#else
		xil_printf("PL buttons not available, no action set\r\n");
#endif
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
