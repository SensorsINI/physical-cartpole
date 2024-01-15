#include "neural_imitator.h"

#include "xparameters.h"

#include "buttons_and_switches.h"


#include "fixed_point.hpp"

#include "EdgeDRNN/EdgeDRNN_Network.h"

#include "HLS4ML/HLS4ML_Network.h"


#define NETWORKS_SWITCH_NUMBER	3

#define MEM_BASE_ADDR		0x01000000

#define TX_BUFFER_BASE		(MEM_BASE_ADDR + 0x00100000)
#define RX_BUFFER_BASE		(MEM_BASE_ADDR + 0x00300000)

float hls_normalize_a[] = {0.06736747,    1.0,    1.0,    5.050505,    0.92031026,    1.0,    5.050505};
float hls_normalize_b[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float hls_denormalize_A[] = {1.0};
float hls_denormalize_B[] = {0.0};

// EdgeDRNN input buffer
short* edgedrnn_stim; //[] = {131,256,8,36,2,256,14,0};


void Neural_Imitator_Init()
{
	HLS4ML_Network_Init();
	EdgeDRNN_Network_Init();

}


void Neural_Imitator_Evaluate(unsigned char * network_input_buffer, unsigned char * network_output_buffer)
{

	int32_t *TxBufferPtr;
	int32_t *RxBufferPtr;

	TxBufferPtr = (int32_t *) TX_BUFFER_BASE;
	RxBufferPtr = (int32_t *) RX_BUFFER_BASE;

	float actv_floating_point, predic_floating_point;
	int32_t actv_fixed_point_32;
	short actv_fixed_point_16;
	int32_t predic_fixed_point_32;


	if(Switch_GetState(NETWORKS_SWITCH_NUMBER))
	{
		// Use EdgeDRNN accelerator

		for (int neuron_idx = 0; neuron_idx < MLP_ACTIVATION_NEURONS;	neuron_idx++)
		{
			actv_floating_point = *((float *)&network_input_buffer[neuron_idx*DATA_WORD_BYTES]);
			actv_fixed_point_16 = float_to_fixed_16(actv_floating_point, 8);
			edgedrnn_stim[neuron_idx] = actv_fixed_point_16;
		}
		edgedrnn_stim[7] = 0; // FIXME: This is probably just setting the target equilibrium or position to 0

		// FIXME: I want to change the interface here - it should be like for HLS4ML so that it can be used for car.
		predic_floating_point = EdgeDRNN_Network_Evaluate((short*) (edgedrnn_stim));

		for (int neuron_idx = 0; neuron_idx < MLP_PREDICTION_NEURONS;	neuron_idx++)
		{
			*((float          *)&network_output_buffer[neuron_idx*DATA_WORD_BYTES]) = predic_floating_point;
		}


	}
	else
	{
		// Use MLP accelerator

		for (int neuron_idx = 0; neuron_idx < MLP_ACTIVATION_NEURONS;	neuron_idx++)
		{
			actv_floating_point = *((float *)&network_input_buffer[neuron_idx*DATA_WORD_BYTES]);
			actv_floating_point = hls_normalize_a[neuron_idx]*actv_floating_point + hls_normalize_b[neuron_idx];
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
			predic_floating_point = hls_denormalize_A[neuron_idx]*predic_floating_point + hls_denormalize_B[neuron_idx];
			*((float          *)&network_output_buffer[neuron_idx*DATA_WORD_BYTES]) = predic_floating_point;
		}

	}
}


void Neural_Imitator_ReleaseResources(){
	EdgeDRNN_Network_ReleaseResources();
}

float neural_imitator_cartpole_step(float angle, float angleD, float angle_cos, float angle_sin, float position, float positionD, float target_position, float time)
{
	// Define input and output buffers
	// Load proper values into input buffer
	static unsigned char network_input_buff[NETWORK_INPUT_SIZE_IN_BYTES];
	static unsigned char network_output_buff[NETWORK_OUTPUT_SIZE_IN_BYTES];

	float target_equilibrium = 1.0;
//	float network_input_buff[] = {angleD, angle_cos, angle_sin, position, positionD, target_equilibrium, target_position};

	*((float *)&network_input_buff[0]) = angleD;
	*((float *)&network_input_buff[4]) = angle_cos;
	*((float *)&network_input_buff[8]) = angle_sin;
	*((float *)&network_input_buff[12]) = position;
	*((float *)&network_input_buff[16]) = positionD;
	*((float *)&network_input_buff[20]) = target_equilibrium;
	*((float *)&network_input_buff[24]) = target_position;

	// Evaluate network value
//	Neural_Imitator_Evaluate((unsigned char*)network_input_buff, network_output_buff);
	Neural_Imitator_Evaluate(network_input_buff, network_output_buff);

	// Output buffer for cartpole has just 1 element and this is the requested control input
	float Q = *((float          *)&network_output_buff[ 0]);
	return Q;
}

