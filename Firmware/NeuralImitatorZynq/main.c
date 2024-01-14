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

/******************** Constant Definitions **********************************/

#define NETWORKS_SWITCH_NUMBER	3

#define MEM_BASE_ADDR		0x01000000

#define TX_BUFFER_BASE		(MEM_BASE_ADDR + 0x00100000)
#define RX_BUFFER_BASE		(MEM_BASE_ADDR + 0x00300000)

#define MLP_ACTIVATION_NEURONS		7 // 4bytes each
#define MLP_PREDICTION_NEURONS		1 // 4bytes each
#define DATA_WORD_BYTES				4

#define NETWORK_INPUT_SIZE_IN_BYTES		(MLP_ACTIVATION_NEURONS * DATA_WORD_BYTES)
#define NETWORK_OUTPUT_SIZE_IN_BYTES		(MLP_PREDICTION_NEURONS * DATA_WORD_BYTES)

// EdgeDRNN input buffer
short* edgedrnn_stim; //[] = {131,256,8,36,2,256,14,0};

int main() {

	General_Init();
	PC_Connection_Init();
	Buttons_And_Switches_Init();
	Led_Init();
	HLS4ML_Network_Init();
	EdgeDRNN_Network_Init();

	int32_t *TxBufferPtr;
	int32_t *RxBufferPtr;

	TxBufferPtr = (int32_t *) TX_BUFFER_BASE;
	RxBufferPtr = (int32_t *) RX_BUFFER_BASE;

	// Loop to receive the data from the cartpole simulator, feed the MLP, get the prediction and send it back to the
	// cartpole simulator
	float actv_floating_point, predic_floating_point;
	int32_t actv_fixed_point_32;
	short actv_fixed_point_16;
	int32_t predic_fixed_point_32;

	while (1) {

		static unsigned char rx_uart_buffer[NETWORK_INPUT_SIZE_IN_BYTES];
		static unsigned char tx_uart_buffer[NETWORK_OUTPUT_SIZE_IN_BYTES];

		static unsigned int  uart_received_Cnt = 0;
		while(uart_received_Cnt < NETWORK_INPUT_SIZE_IN_BYTES){
			int newDataCount = Message_GetFromPC(&rx_uart_buffer[uart_received_Cnt]);
			uart_received_Cnt += newDataCount;
		}
		uart_received_Cnt -= NETWORK_INPUT_SIZE_IN_BYTES;



		if(Switch_GetState(NETWORKS_SWITCH_NUMBER))
		{
			// Use EdgeDRNN accelerator

			for (int neuron_idx = 0; neuron_idx < MLP_ACTIVATION_NEURONS;	neuron_idx++)
			{
				actv_floating_point = *((float *)&rx_uart_buffer[neuron_idx*DATA_WORD_BYTES]);
				actv_fixed_point_16 = float_to_fixed_16(actv_floating_point, 8);
				edgedrnn_stim[neuron_idx] = actv_fixed_point_16;
			}
			edgedrnn_stim[7] = 0; // FIXME: This is probably just setting the target equilibrium or position to 0

			// FIXME: I want to change the interface here - it should be like for HLS4ML so that it can be used for car.
			predic_floating_point = EdgeDRNN_Network_Evaluate((short*) (edgedrnn_stim));

			for (int neuron_idx = 0; neuron_idx < MLP_PREDICTION_NEURONS;	neuron_idx++)
			{
				*((float          *)&tx_uart_buffer[neuron_idx*DATA_WORD_BYTES]) = predic_floating_point;
			}


		}
		else
		{
			// Use MLP accelerator

			for (int neuron_idx = 0; neuron_idx < MLP_ACTIVATION_NEURONS;	neuron_idx++)
			{
				actv_floating_point = *((float *)&rx_uart_buffer[neuron_idx*DATA_WORD_BYTES]);
				actv_fixed_point_32 = float_to_fixed_32(actv_floating_point, 14);
				TxBufferPtr[neuron_idx] = actv_fixed_point_32;
			}

			HLS4ML_Network_Evaluate((UINTPTR) TxBufferPtr, NETWORK_INPUT_SIZE_IN_BYTES,
									(UINTPTR) RxBufferPtr, NETWORK_OUTPUT_SIZE_IN_BYTES);


			for (int neuron_idx = 0; neuron_idx < MLP_PREDICTION_NEURONS;	neuron_idx++)
			{
//				predic_fixed_point_32 = extend_sign(RxBufferPtr[neuron_idx]);
//				predic_floating_point = fixed_to_float(predic_fixed_point_32);
				predic_fixed_point_32 = extend_sign_32(RxBufferPtr[neuron_idx], 19);
				predic_floating_point = fixed_to_float_32(predic_fixed_point_32, 14);
				*((float          *)&tx_uart_buffer[neuron_idx*DATA_WORD_BYTES]) = predic_floating_point;
			}

		}


		Message_SendToPC(tx_uart_buffer, NETWORK_OUTPUT_SIZE_IN_BYTES);

		Leds_over_switches_Update(Switches_GetState());
	}

	EdgeDRNN_Network_ReleaseResources();

	return XST_SUCCESS;
}
