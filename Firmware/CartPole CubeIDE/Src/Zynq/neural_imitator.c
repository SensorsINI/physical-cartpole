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

// Normalization Cartpole
float hls_normalize_a[] = {0.06736747,    1.0,    1.0,    5.050505,    0.92031026,    1.0,    5.050505};
float hls_normalize_b[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float hls_denormalize_A[] = {1.0};
float hls_denormalize_B[] = {0.0};

// Normalization f1t
//float hls_normalize_a[] = {3.78787875,3.10077524,2.74725246,2.03045702,1.44092214,1.03896105,0.79207927,0.62952471,0.51719677,0.43591982,0.37432155,0.32637075,0.28901735,0.25913447,0.23482446,0.21468441,0.19776526,0.18336849,0.17095478,0.16015373,2.44798040,1.39860129,0.92336106,0.68027210,0.53533190,0.44179368,0.37707391,0.32976091,0.29342723,0.26413101,0.24064493,0.22121446,0.20464544,0.19166268,0.17882690,0.16701461,0.15673982,0.14775415,0.13991883,0.13302295,0.14385384,0.14385384,0.14385384,0.14385384,0.14385384,0.14385384,0.14385384,0.14385384,0.14385384,0.14385384,0.14385384,0.14385384,0.14385384,0.14385384,0.14385384,0.14385384,0.14385384,0.14385384,0.14385384,0.14385384,0.12087514,0.14053826,1.26502204,2.23214269
//};
//float hls_normalize_b[] = {-1.36742425,-2.16899228,-2.89835143,-2.69137049,-2.18876076,-1.71376622,-1.38495052,-1.14667928,-0.96948540,-0.83347863,-0.72375071,-0.63315928,-0.56127167,-0.50323915,-0.45614654,-0.41713184,-0.38455451,-0.35711008,-0.33361828,-0.31342089,0.06242347,0.06713283,0.04801476,0.03877544,0.03104925,0.02628672,0.02149332,0.01665294,0.00938964,0.00449026,0.00421131,0.00940156,0.01074386,0.01102066,0.01287556,0.01244247,0.01159871,0.01019502,0.00755560,0.00405717,-1.24843562,-1.24843562,-1.24843562,-1.24843562,-1.24843562,-1.24843562,-1.24843562,-1.24843562,-1.24843562,-1.24843562,-1.24843562,-1.24843562,-1.24843562,-1.24843562,-1.24843562,-1.24843562,-1.24843562,-1.24843562,-1.24843562,-1.24843562,0.03046048,-1.00000000,-0.05249846,0.00000000
//};
//float hls_denormalize_A[] = {0.73350000,8.11649990
//};
//float hls_denormalize_B[] = {-0.06550002,7.25649977
//};

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
			actv_fixed_point_32 = float_to_fixed_32(actv_floating_point, MLP_TOTAL_BITS_PER_VARIABLE-MLP_INTEGER_PLUS_SIGN_BITS_PER_VARIABLE);
			TxBufferPtr[neuron_idx] = actv_fixed_point_32;
		}

		HLS4ML_Network_Evaluate((UINTPTR) TxBufferPtr, NETWORK_INPUT_SIZE_IN_BYTES,
								(UINTPTR) RxBufferPtr, NETWORK_OUTPUT_SIZE_IN_BYTES);


		for (int neuron_idx = 0; neuron_idx < MLP_PREDICTION_NEURONS;	neuron_idx++)
		{
//				predic_fixed_point_32 = extend_sign(RxBufferPtr[neuron_idx]);
//				predic_floating_point = fixed_to_float(predic_fixed_point_32);
			predic_fixed_point_32 = extend_sign_32(RxBufferPtr[neuron_idx], MLP_TOTAL_BITS_PER_VARIABLE-1);
			predic_floating_point = fixed_to_float_32(predic_fixed_point_32, MLP_TOTAL_BITS_PER_VARIABLE-MLP_INTEGER_PLUS_SIGN_BITS_PER_VARIABLE);
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

