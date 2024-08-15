#include "dma.h"


dma_t* dma_create(
	int dma_id,
	u32 a_qi,
	u32 a_qf,
	u32 hp_bytes,
	u32 rnn_inp_size,
	u32 rnn_hid_size
){
	// Local variables
	dma_t* p_dma_obj;
	u32 a_bw = 0;
	u32 a_word_size = 0;

	// Allocate memory for dma object
	p_dma_obj = (dma_t*) malloc(sizeof(dma_t));
	if (p_dma_obj == NULL)
	{
		xil_printf("ERROR! Failed to allocate memory for dma object.\n\r");
		return NULL;
	}
	/************************************************
	 * Set DMA HW
	 ************************************************/
	// Activation
	a_bw           = a_qi + a_qf;
	a_word_size    = (u32)ceil((double)a_bw/8.0);

	/************************************************
	 * Configure DMA
	 ************************************************/
	// Set DMA parameters
	p_dma_obj->mm2s_bytes_per_beat = hp_bytes;
	p_dma_obj->mm2s_burst_length   = rnn_inp_size/(hp_bytes/a_word_size);
	p_dma_obj->s2mm_bytes_per_beat = hp_bytes;
	p_dma_obj->s2mm_burst_length   = rnn_hid_size/(hp_bytes/a_word_size);

	// Create DMA Passthrough object to be used by dma
	p_dma_obj->p_dma_passthrough_inst = dma_passthrough_create(dma_id);
	if (p_dma_obj->p_dma_passthrough_inst == NULL)
	{
		xil_printf("ERROR! Failed to create DMA Passthrough object for use by the DAC.\n\r");
		return NULL;
	}

	// Beat size = HP port size
	dma_passthrough_set_snd_sample_size_bytes(p_dma_obj->p_dma_passthrough_inst, p_dma_obj->mm2s_bytes_per_beat);
	dma_passthrough_set_rcv_sample_size_bytes(p_dma_obj->p_dma_passthrough_inst, p_dma_obj->s2mm_bytes_per_beat);
	dma_passthrough_set_snd_buf_length(p_dma_obj->p_dma_passthrough_inst, p_dma_obj->mm2s_burst_length);
	dma_passthrough_set_rcv_buf_length(p_dma_obj->p_dma_passthrough_inst, p_dma_obj->s2mm_burst_length);

	// Enable DMA
	dma_passthrough_reset(p_dma_obj->p_dma_passthrough_inst); // Reset DMA to flush those 4 extra samples that are accepted before DMA configuration
	return p_dma_obj;
}


short* dma_set_snd_buf(dma_t* p_dma_obj, short* p_snd_buf)
{
    //Assign memory space to send buffer
	p_snd_buf = (short*) calloc(p_dma_obj->mm2s_burst_length, p_dma_obj->mm2s_bytes_per_beat);
    if (p_snd_buf == NULL)
	{
		xil_printf("ERROR! Failed to allocate memory for send buffer.\n\r");
		return NULL;
	}
    dma_passthrough_set_snd_buf(p_dma_obj->p_dma_passthrough_inst, p_snd_buf);

    return p_snd_buf;
}

short* dma_set_snd_buf_addr(dma_t* p_dma_obj, short* p_snd_buf)
{
    //Assign memory space to send buffer
    dma_passthrough_set_snd_buf(p_dma_obj->p_dma_passthrough_inst, p_snd_buf);

    return p_snd_buf;
}

short* dma_set_rcv_buf(dma_t* p_dma_obj, short* p_rcv_buf)
{
    //Assign memory space to receive buffer
	p_rcv_buf = (short*) calloc(p_dma_obj->s2mm_burst_length, p_dma_obj->s2mm_bytes_per_beat);
    if (p_rcv_buf == NULL)
	{
		xil_printf("ERROR! Failed to allocate memory for receive buffer.\n\r");
		return NULL;
	}
    dma_passthrough_set_rcv_buf(p_dma_obj->p_dma_passthrough_inst, p_rcv_buf);

    return p_rcv_buf;
}

int dma_snd(dma_t* p_dma_obj)
{
	// Local variables
	int status;

	status = dma_passthrough_snd(p_dma_obj->p_dma_passthrough_inst);
	if (status != DMA_PASSTHROUGH_SUCCESS)
	{
		xil_printf("ERROR! DMA Passthrough error occurred when trying to send data.\n\r");
		return DMA_FAIL;
	}

	return DMA_SUCCESS;
}

// Non-blocking
int dma_snd_nb(dma_t* p_dma_obj)
{
	// Local variables
	int status;

	status = dma_passthrough_snd_nb(p_dma_obj->p_dma_passthrough_inst);
	if (status != DMA_PASSTHROUGH_SUCCESS)
	{
		xil_printf("ERROR! DMA Passthrough error occurred when trying to send data.\n\r");
		return DMA_FAIL;
	}

	return DMA_SUCCESS;
}

u32 dma_get_snd_status(dma_t* p_dma_obj)
{
    return dma_passthrough_snd_busy(p_dma_obj->p_dma_passthrough_inst);
}

int dma_rcv(dma_t* p_dma_obj)
{
	// Local variables
	int status;

	status = dma_passthrough_rcv(p_dma_obj->p_dma_passthrough_inst);
	if (status != DMA_PASSTHROUGH_SUCCESS)
	{
		xil_printf("ERROR! DMA Passthrough error occurred when trying to receive data.\n\r");
		return DMA_FAIL;
	}

	return DMA_SUCCESS;
}

// Non-blocking
int dma_rcv_nb(dma_t* p_dma_obj)
{
	// Local variables
	int status;

	status = dma_passthrough_rcv_nb(p_dma_obj->p_dma_passthrough_inst);
	if (status != DMA_PASSTHROUGH_SUCCESS)
	{
		xil_printf("ERROR! DMA Passthrough error occurred when trying to receive data.\n\r");
		return DMA_FAIL;
	}

	return DMA_SUCCESS;
}

u32 dma_get_rcv_status(dma_t* p_dma_obj)
{
    return dma_passthrough_rcv_busy(p_dma_obj->p_dma_passthrough_inst);
}

u32 dma_get_snd_beat_size(dma_t* p_dma_obj)
{
    return p_dma_obj->mm2s_bytes_per_beat;
}

u32 dma_get_rcv_beat_size(dma_t* p_dma_obj)
{
    return p_dma_obj->s2mm_bytes_per_beat;
}

u32 dma_get_snd_burst_length(dma_t* p_dma_obj)
{
    return p_dma_obj->mm2s_burst_length;
}

u32 dma_get_rcv_burst_length(dma_t* p_dma_obj)
{
    return p_dma_obj->s2mm_burst_length;
}

u32 dma_get_snd_buf_length(dma_t* p_dma_obj)
{
    return p_dma_obj->mm2s_bytes_per_beat * p_dma_obj->mm2s_burst_length / 2;
}

u32 dma_get_rcv_buf_length(dma_t* p_dma_obj)
{
    return p_dma_obj->s2mm_bytes_per_beat * p_dma_obj->s2mm_burst_length / 2;
}
