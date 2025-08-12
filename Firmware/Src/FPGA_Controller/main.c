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
    return &NeuralImitator_Ops;
//    return &LQR_Ops;
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
    CR_HandshakeOnce();

    /* Allocate once with generous bounds; we only use IN_BYTES/OUT_BYTES each cycle. */
    static unsigned char rx_uart_buffer[MAX_INPUTS  * 4];
    static unsigned char tx_uart_buffer[MAX_OUTPUTS * 4];

	while (1) {

        /* OPTIONAL: on boards with switches, pick desired controller here */
        const ControllerOps* desired = select_controller();
        CR_RequestSwitch(desired);   /* no-op if unchanged */

        /* Sizes may change after a handshake; query each cycle */
        const uint8_t nin  = CR_ActiveNumInputs();
        const uint8_t nout = CR_ActiveNumOutputs();
        const unsigned IN_BYTES  = (unsigned)nin  * 4u;
        const unsigned OUT_BYTES = (unsigned)nout * 4u;

        /* Accumulate exactly IN_BYTES */
        static unsigned int have = 0;
        while (have < IN_BYTES){
            int newDataCount = Message_GetFromPC(&rx_uart_buffer[have]);
            have += (unsigned int)newDataCount;
        }
        have -= IN_BYTES;

        /* Compute control using the active controller (works for NN/PID/MPC). */
        CR_EvaluateBytes(rx_uart_buffer, tx_uart_buffer);

        /* If a switch is pending, announce & complete the re-handshake now.
           This sends a 4B cookie BEFORE outputs, then blocks for GET_SPEC,
           finalizes the switch, and replies with the new spec. */
        if (CR_SendSpecCookieIfPending()) {
            CR_HandshakeOnce();
            /* After this point, active controller may differ; sizes will be
               picked up next loop via CR_ActiveNumInputs/Outputs(). */
        }

        /* Send outputs for THIS cycle (from the controller active before switch). */
		Message_SendToPC(tx_uart_buffer, OUT_BYTES);

		Leds_over_switches_Update(Switches_GetState());
	}

    if (CR_GetActive() && CR_GetActive()->release) CR_GetActive()->release();
	return XST_SUCCESS;
}
