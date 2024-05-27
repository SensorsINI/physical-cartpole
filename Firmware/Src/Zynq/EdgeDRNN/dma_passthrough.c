
// Includes
#include "dma_passthrough.h"

#include <stdlib.h>
#include "xaxidma.h"

// Private data
typedef struct dma_passthrough_periphs
{
	XAxiDma dma_inst;
} dma_passthrough_periphs_t;

typedef struct dma_passthrough
{
	dma_passthrough_periphs_t periphs;
	void*                     p_rcv_buf;
	void*                     p_snd_buf;
	int                       rcv_buf_length;
	int                       snd_buf_length;
	int                       rcv_sample_size_bytes;
	int                       snd_sample_size_bytes;
} dma_passthrough_t;

// Private functions
static int init_dma(XAxiDma* p_dma_inst, int dma_device_id)
{

	// Local variables
	int             status = 0;
	XAxiDma_Config* cfg_ptr;

	// Look up hardware configuration for device
	cfg_ptr = XAxiDma_LookupConfig(dma_device_id);
	if (!cfg_ptr)
	{
		xil_printf("ERROR! No hardware configuration found for AXI DMA with device id %d.\r\n", dma_device_id);
		return DMA_PASSTHROUGH_DMA_INIT_FAIL;
	}

	// Initialize driver
	status = XAxiDma_CfgInitialize(p_dma_inst, cfg_ptr);
	if (status != XST_SUCCESS)
	{
		xil_printf("ERROR! Initialization of AXI DMA failed with %d\r\n", status);
		return DMA_PASSTHROUGH_DMA_INIT_FAIL;
	}

	// Test for Scatter Gather
	if (XAxiDma_HasSg(p_dma_inst))
	{
		xil_printf("ERROR! Device configured as SG mode.\r\n");
		return DMA_PASSTHROUGH_DMA_INIT_FAIL;
	}

	// Disable interrupts for both channels
	XAxiDma_IntrDisable(p_dma_inst, XAXIDMA_IRQ_ALL_MASK, XAXIDMA_DEVICE_TO_DMA);
	XAxiDma_IntrDisable(p_dma_inst, XAXIDMA_IRQ_ALL_MASK, XAXIDMA_DMA_TO_DEVICE);

	// Reset DMA
	XAxiDma_Reset(p_dma_inst);
	while (!XAxiDma_ResetIsDone(p_dma_inst)) {}

	return DMA_PASSTHROUGH_SUCCESS;

}

// Public functions
dma_passthrough_t* dma_passthrough_create(int dma_device_id)
{

	// Local variables
	dma_passthrough_t* p_obj;
	int                status;

	// Allocate memory for DMA Passthrough object
	p_obj = (dma_passthrough_t*) malloc(sizeof(dma_passthrough_t));
	if (p_obj == NULL)
	{
		xil_printf("ERROR! Failed to allocate memory for DMA Passthrough object.\n\r");
		return NULL;
	}

	// Register and initialize peripherals
	status = init_dma(&p_obj->periphs.dma_inst, dma_device_id);
	if (status != DMA_PASSTHROUGH_SUCCESS)
	{
		xil_printf("ERROR! Failed to initialize AXI DMA.\n\r");
		dma_passthrough_destroy(p_obj);
		return NULL;
	}

	// Initialize buffer pointers
	dma_passthrough_set_rcv_buf(p_obj, NULL);
	dma_passthrough_set_snd_buf(p_obj, NULL);

	// Initialize buffer length
	dma_passthrough_set_rcv_buf_length(p_obj, 1024);
	dma_passthrough_set_snd_buf_length(p_obj, 1024);

	// Initialize sample size (64-bit beat in default)
	dma_passthrough_set_rcv_sample_size_bytes(p_obj, 8);
	dma_passthrough_set_snd_sample_size_bytes(p_obj, 8);

	return p_obj;
}

void dma_passthrough_destroy(dma_passthrough_t* p_dma_passthrough_inst)
{
	free(p_dma_passthrough_inst);
}

void dma_passthrough_reset(dma_passthrough_t* p_dma_passthrough_inst)
{
	XAxiDma_Reset(&p_dma_passthrough_inst->periphs.dma_inst);
}

void dma_passthrough_set_rcv_buf(dma_passthrough_t* p_dma_passthrough_inst, void* p_rcv_buf)
{
	p_dma_passthrough_inst->p_rcv_buf = p_rcv_buf;
}

void* dma_passthrough_get_rcv_buf(dma_passthrough_t* p_dma_passthrough_inst)
{
	return (p_dma_passthrough_inst->p_rcv_buf);
}

void dma_passthrough_set_snd_buf(dma_passthrough_t* p_dma_passthrough_inst, void* p_snd_buf)
{
	p_dma_passthrough_inst->p_snd_buf = p_snd_buf;
}

void* dma_passthrough_get_snd_buf(dma_passthrough_t* p_dma_passthrough_inst)
{
	return (p_dma_passthrough_inst->p_snd_buf);
}

void dma_passthrough_set_rcv_buf_length(dma_passthrough_t* p_dma_passthrough_inst, int buf_length)
{
	p_dma_passthrough_inst->rcv_buf_length = buf_length;
}

void dma_passthrough_set_snd_buf_length(dma_passthrough_t* p_dma_passthrough_inst, int buf_length)
{
	p_dma_passthrough_inst->snd_buf_length = buf_length;
}

int dma_passthrough_get_rcv_buf_length(dma_passthrough_t* p_dma_passthrough_inst)
{
	return (p_dma_passthrough_inst->rcv_buf_length);
}

int dma_passthrough_get_snd_buf_length(dma_passthrough_t* p_dma_passthrough_inst)
{
	return (p_dma_passthrough_inst->snd_buf_length);
}

void dma_passthrough_set_rcv_sample_size_bytes(dma_passthrough_t* p_dma_passthrough_inst, int sample_size_bytes)
{
	p_dma_passthrough_inst->rcv_sample_size_bytes = sample_size_bytes;
}

void dma_passthrough_set_snd_sample_size_bytes(dma_passthrough_t* p_dma_passthrough_inst, int sample_size_bytes)
{
	p_dma_passthrough_inst->snd_sample_size_bytes = sample_size_bytes;
}

int dma_passthrough_get_rcv_sample_size_bytes(dma_passthrough_t* p_dma_passthrough_inst)
{
	return (p_dma_passthrough_inst->rcv_sample_size_bytes);
}

int dma_passthrough_get_snd_sample_size_bytes(dma_passthrough_t* p_dma_passthrough_inst)
{
	return (p_dma_passthrough_inst->snd_sample_size_bytes);
}

int dma_passthrough_rcv(dma_passthrough_t* p_dma_passthrough_inst)
{
		//printf("Status_before: %d\n", XAxiDma_Busy(&p_dma_passthrough_inst->periphs.dma_inst, XAXIDMA_DEVICE_TO_DMA));
		// Local variables
		int       status    = 0;
		const int num_bytes = p_dma_passthrough_inst->rcv_buf_length*p_dma_passthrough_inst->rcv_sample_size_bytes;

		// Flush cache
		#if (!DMA_PASSTHROUGH_IS_CACHE_COHERENT)
			Xil_DCacheFlushRange((int)p_dma_passthrough_inst->p_rcv_buf, num_bytes);
		#endif

		//printf("Status_after: %d\n", XAxiDma_Busy(&p_dma_passthrough_inst->periphs.dma_inst, XAXIDMA_DEVICE_TO_DMA));

		// Kick off S2MM transfer
		status = XAxiDma_SimpleTransfer
		(
			&p_dma_passthrough_inst->periphs.dma_inst,
			(int)p_dma_passthrough_inst->p_rcv_buf,
			num_bytes,
			XAXIDMA_DEVICE_TO_DMA
		);
		if (status != XST_SUCCESS)
		{
			xil_printf("ERROR! Failed to kick off S2MM transfer!\n\r");
			return DMA_PASSTHROUGH_XFER_FAIL;
		}

		// Wait for transfer to complete
		while (XAxiDma_Busy(&p_dma_passthrough_inst->periphs.dma_inst, XAXIDMA_DEVICE_TO_DMA)) {}

		// Check DMA for errors
		if ((XAxiDma_ReadReg(p_dma_passthrough_inst->periphs.dma_inst.RegBase, XAXIDMA_RX_OFFSET+XAXIDMA_SR_OFFSET) & XAXIDMA_IRQ_ERROR_MASK) != 0)
		{
			xil_printf("ERROR! AXI DMA returned an error during the S2MM transfer.\n\r");
			return DMA_PASSTHROUGH_XFER_FAIL;
		}

		return DMA_PASSTHROUGH_SUCCESS;

}

int dma_passthrough_rcv_nb(dma_passthrough_t* p_dma_passthrough_inst)
{
		//printf("Status_before: %d\n", XAxiDma_Busy(&p_dma_passthrough_inst->periphs.dma_inst, XAXIDMA_DEVICE_TO_DMA));
		// Local variables
		int       status    = 0;
		const int num_bytes = p_dma_passthrough_inst->rcv_buf_length*p_dma_passthrough_inst->rcv_sample_size_bytes;

		// Flush cache
		#if (!DMA_PASSTHROUGH_IS_CACHE_COHERENT)
			Xil_DCacheFlushRange((int)p_dma_passthrough_inst->p_rcv_buf, num_bytes);
		#endif

		//printf("Status_after: %d\n", XAxiDma_Busy(&p_dma_passthrough_inst->periphs.dma_inst, XAXIDMA_DEVICE_TO_DMA));

		// Kick off S2MM transfer
		status = XAxiDma_SimpleTransfer
		(
			&p_dma_passthrough_inst->periphs.dma_inst,
			(int)p_dma_passthrough_inst->p_rcv_buf,
			num_bytes,
			XAXIDMA_DEVICE_TO_DMA
		);
		if (status != XST_SUCCESS)
		{
			xil_printf("ERROR! Failed to kick off S2MM transfer!\n\r");
			return DMA_PASSTHROUGH_XFER_FAIL;
		}

		// Check DMA for errors
		if ((XAxiDma_ReadReg(p_dma_passthrough_inst->periphs.dma_inst.RegBase, XAXIDMA_RX_OFFSET+XAXIDMA_SR_OFFSET) & XAXIDMA_IRQ_ERROR_MASK) != 0)
		{
			xil_printf("ERROR! AXI DMA returned an error during the S2MM transfer.\n\r");
			return DMA_PASSTHROUGH_XFER_FAIL;
		}

		return DMA_PASSTHROUGH_SUCCESS;

}

u32 dma_passthrough_rcv_busy(dma_passthrough_t* p_dma_passthrough_inst)
{

	return (XAxiDma_Busy(&p_dma_passthrough_inst->periphs.dma_inst, XAXIDMA_DEVICE_TO_DMA));

}

int dma_passthrough_snd(dma_passthrough_t* p_dma_passthrough_inst)
{

	// Local variables
	int       status    = 0;
	const int num_bytes = p_dma_passthrough_inst->snd_buf_length*p_dma_passthrough_inst->snd_sample_size_bytes;

	//printf("Status_before: %d\n", XAxiDma_Busy(&p_dma_passthrough_inst->periphs.dma_inst, XAXIDMA_DMA_TO_DEVICE));

	// Flush cache
	#if (!DMA_PASSTHROUGH_IS_CACHE_COHERENT)
		Xil_DCacheFlushRange((int)p_dma_passthrough_inst->p_snd_buf, num_bytes);
	#endif

	//printf("Status_after: %d\n", XAxiDma_Busy(&p_dma_passthrough_inst->periphs.dma_inst, XAXIDMA_DMA_TO_DEVICE));

	// Kick off MM2S transfer
	status = XAxiDma_SimpleTransfer
	(
		&p_dma_passthrough_inst->periphs.dma_inst,
		(int)p_dma_passthrough_inst->p_snd_buf,
		num_bytes,
		XAXIDMA_DMA_TO_DEVICE
	);
	if (status != XST_SUCCESS)
	{
		xil_printf("ERROR! Failed to kick off MM2S transfer!\n\r");
		return DMA_PASSTHROUGH_XFER_FAIL;
	}

	// Wait for transfer to complete
	while (XAxiDma_Busy(&p_dma_passthrough_inst->periphs.dma_inst, XAXIDMA_DMA_TO_DEVICE)) {}

	// Check DMA for errors
	if ((XAxiDma_ReadReg(p_dma_passthrough_inst->periphs.dma_inst.RegBase, XAXIDMA_TX_OFFSET+XAXIDMA_SR_OFFSET) & XAXIDMA_IRQ_ERROR_MASK) != 0)
	{
		xil_printf("ERROR! AXI DMA returned an error during the MM2S transfer.\n\r");
		return DMA_PASSTHROUGH_XFER_FAIL;
	}

	return DMA_PASSTHROUGH_SUCCESS;
}

int dma_passthrough_snd_nb(dma_passthrough_t* p_dma_passthrough_inst)
{

	// Local variables
	int       status    = 0;
	const int num_bytes = p_dma_passthrough_inst->snd_buf_length*p_dma_passthrough_inst->snd_sample_size_bytes;

	//printf("Status_before: %d\n", XAxiDma_Busy(&p_dma_passthrough_inst->periphs.dma_inst, XAXIDMA_DMA_TO_DEVICE));

	// Flush cache
	#if (!DMA_PASSTHROUGH_IS_CACHE_COHERENT)
		Xil_DCacheFlushRange((int)p_dma_passthrough_inst->p_snd_buf, num_bytes);
	#endif

	//printf("Status_after: %d\n", XAxiDma_Busy(&p_dma_passthrough_inst->periphs.dma_inst, XAXIDMA_DMA_TO_DEVICE));

	// Kick off MM2S transfer
	status = XAxiDma_SimpleTransfer
	(
		&p_dma_passthrough_inst->periphs.dma_inst,
		(int)p_dma_passthrough_inst->p_snd_buf,
		num_bytes,
		XAXIDMA_DMA_TO_DEVICE
	);
	if (status != XST_SUCCESS)
	{
		xil_printf("ERROR! Failed to kick off MM2S transfer!\n\r");
		return DMA_PASSTHROUGH_XFER_FAIL;
	}

	// Check DMA for errors
	if ((XAxiDma_ReadReg(p_dma_passthrough_inst->periphs.dma_inst.RegBase, XAXIDMA_TX_OFFSET+XAXIDMA_SR_OFFSET) & XAXIDMA_IRQ_ERROR_MASK) != 0)
	{
		xil_printf("ERROR! AXI DMA returned an error during the MM2S transfer.\n\r");
		return DMA_PASSTHROUGH_XFER_FAIL;
	}

	return DMA_PASSTHROUGH_SUCCESS;
}

u32 dma_passthrough_snd_busy(dma_passthrough_t* p_dma_passthrough_inst)
{

	return (XAxiDma_Busy(&p_dma_passthrough_inst->periphs.dma_inst, XAXIDMA_DMA_TO_DEVICE));

}
