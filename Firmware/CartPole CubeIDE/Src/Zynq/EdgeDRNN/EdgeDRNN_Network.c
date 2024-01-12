#include "edgedrnn.h"
#include "edgedrnn_test.h"
#include "dma.h"
#include "drnn_data.h"

dma_t* p_dma_obj;
edgedrnn_t* p_edgedrnn_obj = NULL;;

short* p_snd_buf = NULL;
short* p_rcv_buf = NULL;

void EdgeDRNN_Network_Init()
{
	//----------------------------------------------
	// Create EdgeDRNN DMA Object
	//----------------------------------------------
	p_dma_obj = dma_create(XPAR_AXI_DMA_1_DEVICE_ID,
							AQI,
							AQF,
							XPAR_AXI_DMA_1_M_AXI_MM2S_DATA_WIDTH / 8,
							INP_SIZE,
							RNN_SIZE);

	p_snd_buf = dma_set_snd_buf(p_dma_obj, p_snd_buf);
	p_rcv_buf = dma_set_rcv_buf(p_dma_obj, p_rcv_buf);

	//----------------------------------------------
	// Create Edgedrnn Object
	//----------------------------------------------
	p_edgedrnn_obj = edgedrnn_create(XPAR_EDGEDRNN_WRAPPER_0_BASEADDR, (u32) rnn_param,
									NUM_PE,
									THX,		  // num_pe
									THH,		  // a_qi
									AQI,		  // a_qf
									AQF,		  // w_qi
									WQI,		  // w_qf
									WQF,
									XPAR_AXI_DMA_1_M_AXI_MM2S_DATA_WIDTH / 8,	// hp_size
									RNN_LAYERS,                // rnn_num_layers
									INP_SIZE,	                // rnn_inp_size
									RNN_SIZE,	                // rnn_hid_size
									RNN_PARAM_SIZE                   // rnn_mat_size
									);
}


float EdgeDRNN_Network_Evaluate(short* edgedrnn_stim)
{
	dma_set_snd_buf_addr(p_dma_obj,	(short*) (edgedrnn_stim));
	dma_snd(p_dma_obj); // Kick-off MM2S Transfer
	dma_rcv(p_dma_obj); // Kick-off S2MM Transfer
	float predic_floating_point = classification_layer(p_rcv_buf);
	return predic_floating_point;
}

void EdgeDRNN_Network_ReleaseResources()
{
	free(p_snd_buf);
	free(p_rcv_buf);
	free(p_dma_obj);
	free(p_edgedrnn_obj);
}
