/*
 * This application runs a neural network controller on Zynq.
 * It expects to receive input to the network via UART and returns output of the network.
 *
 * To create application for this project in Vitis, create an empty application first.
 * Than in src folder create symbolic link to this folder, hardware bridge and Zynq folder
 * To create symbolic links use native OS tools (e.g. in Linux Ctrl+Shift drag).
 * Do not do it from within Vitis, its tool to create symbolic links does not work fine for compiler.
 *
 * If you encounter problems with UART, while running it with a new platform,
 * you need to add extern declaration XUartPs_SendBuffer or equivalent for UART Lite or 550 IP to the Xilinx Driver
 * e.g. for UART PS add in xuartps.h
 * extern u32  XUartPs_SendBuffer(XUartPs *InstancePtr);
 * extern u32  XUartPs_ReceiveBuffer(XUartPs *InstancePtr);
 *
 */


#include "xparameters.h"

#include "hardware_bridge.h"

#include <stdio.h>
#include "xil_printf.h"
#include "xil_types.h"
#include "xtime_l.h"
#include "math.h"
#include <stdint.h>

#include "controller_manager.h"     /* NEW: handshake + data bridge */
#include "Zynq/neural_imitator.h"   /* exposes NeuralImitator_Ops */
#include "neural_controller_C.h"
#include "lqr.h"

/******************** Constant Definitions **********************************/

/* Choose the active controller. Later: swap to PID/MPC ops here. */
static const ControllerOps* select_controller(void)
{
    /* Example: keep NN for both switch positions for now. */
//    return &NeuralImitator_Ops;
    return &LQR_Ops;
//    return &NNC_Ops;
}

int main() {

	General_Init();
	PC_Connection_Init();
	Buttons_And_Switches_Init();
	Led_Init();

    /* --- Controller selection and init (generic) --- */
    CR_SetActive(select_controller());
    if (CR_GetActive() && CR_GetActive()->init) CR_GetActive()->init();

    /* --- Control-plane: tell PC what inputs we need (names/order/counts). --- */
    /* Note: Handshake will be called when PC sends GET_SPEC command */

	while (1) {

        /* OPTIONAL: on boards with switches, pick desired controller here */
        const ControllerOps* desired = select_controller();
        CR_RequestSwitch(desired);   /* no-op if unchanged */

        /* Process messages from PC using unified message handling */
        /* This handles both state data and control commands (spec requests, pings) */
        while (CR_ProcessMessage()) {
            /* Keep processing messages until no more are available */
            /* State data messages will automatically trigger controller evaluation and output */
        }

        /* If a switch is pending, announce it to PC */
        if (CR_SendSpecCookieIfPending()) {
            /* PC will send a new spec request, which will be handled by CR_ProcessMessage() */
        }

		Leds_over_switches_Update(Switches_GetState());
	}

    if (CR_GetActive() && CR_GetActive()->release) CR_GetActive()->release();
	return XST_SUCCESS;
}