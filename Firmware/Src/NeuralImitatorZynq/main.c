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

#include "Zynq/neural_imitator.h"

/******************** Constant Definitions **********************************/



int main() {

	General_Init();
	PC_Connection_Init();
	Buttons_And_Switches_Init();
	Led_Init();
	Neural_Imitator_Init();

	while (1) {

		static unsigned char rx_uart_buffer[NETWORK_INPUT_SIZE_IN_BYTES];
		static unsigned char tx_uart_buffer[NETWORK_OUTPUT_SIZE_IN_BYTES];

		static unsigned int  uart_received_Cnt = 0;
		while(uart_received_Cnt < NETWORK_INPUT_SIZE_IN_BYTES){
			int newDataCount = Message_GetFromPC(&rx_uart_buffer[uart_received_Cnt]);
			uart_received_Cnt += newDataCount;
		}
		uart_received_Cnt -= NETWORK_INPUT_SIZE_IN_BYTES;

//		float angleD = *((float          *)&rx_uart_buffer[ 0]);
//		float angle_cos = *((float          *)&rx_uart_buffer[ 4]);
//		float angle_sin = *((float          *)&rx_uart_buffer[ 8]);
//		float position = *((float          *)&rx_uart_buffer[ 12]);
//		float positionD = *((float          *)&rx_uart_buffer[ 16]);
//		float target_position = *((float          *)&rx_uart_buffer[ 24]);
//		float Q = neural_imitator_cartpole_step(0.0, angleD, angle_cos, angle_sin, position, positionD, target_position, 0.0);
//        *((float *)&tx_uart_buffer[0]) = Q;

		Neural_Imitator_Evaluate(rx_uart_buffer, tx_uart_buffer);

		Message_SendToPC(tx_uart_buffer, NETWORK_OUTPUT_SIZE_IN_BYTES);

		Leds_over_switches_Update(Switches_GetState());
	}

	Neural_Imitator_ReleaseResources();

	return XST_SUCCESS;
}
