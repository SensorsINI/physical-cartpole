#include "xparameters.h"

#include "xaxidma.h"

#include "HLS4ML_Network.h"

#define DMA_DEV_ID		XPAR_AXIDMA_0_DEVICE_ID

u32 HLS4ML_Network_ListenToOutput(UINTPTR BuffAddr, u32 Length);
u32 HLS4ML_Network_WriteToInput(UINTPTR BuffAddr, u32 Length);
void HLS4ML_Network_WaitToReturn();

XAxiDma AxiDmaHLS4ML;

u32 HLS4ML_Network_Init()
{
	XAxiDma_Config *CfgPtr;
	/* Initialize the XAxiDma device.
	 */
	CfgPtr = XAxiDma_LookupConfig(DMA_DEV_ID);
	if (!CfgPtr) {
		return XST_FAILURE;
	}

	u32 Status = XAxiDma_CfgInitialize(&AxiDmaHLS4ML, CfgPtr);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	if (XAxiDma_HasSg(&AxiDmaHLS4ML)) {
		return XST_FAILURE;
	}

	/* Disable interrupts, we use polling mode
	 */
	XAxiDma_IntrDisable(&AxiDmaHLS4ML, XAXIDMA_IRQ_ALL_MASK, XAXIDMA_DEVICE_TO_DMA);
	XAxiDma_IntrDisable(&AxiDmaHLS4ML, XAXIDMA_IRQ_ALL_MASK, XAXIDMA_DMA_TO_DEVICE);

	return XST_SUCCESS;
}


void HLS4ML_Network_Evaluate(UINTPTR NetworkInputBuffAddr, u32 network_input_length_in_bytes,
							 UINTPTR NetworkOutputBuffAddr, u32 network_output_length_in_bytes)
{
	/* Flush the buffers before the DMA transfer, in case the Data Cache
	 * is enabled
	 */
	Xil_DCacheFlushRange(NetworkInputBuffAddr, network_input_length_in_bytes);
	Xil_DCacheFlushRange(NetworkOutputBuffAddr, network_output_length_in_bytes);

	u32 Status;
	Status = HLS4ML_Network_ListenToOutput(NetworkOutputBuffAddr, network_output_length_in_bytes);
	Status = HLS4ML_Network_WriteToInput(NetworkInputBuffAddr, network_input_length_in_bytes);

	HLS4ML_Network_WaitToReturn();
}


u32 HLS4ML_Network_ListenToOutput(UINTPTR BuffAddr, u32 Length)
{
	// Subscribe the read transaction
	u32 Status = XAxiDma_SimpleTransfer(&AxiDmaHLS4ML, BuffAddr,
			Length, XAXIDMA_DEVICE_TO_DMA);

	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}
	return XST_SUCCESS;
}


u32 HLS4ML_Network_WriteToInput(UINTPTR BuffAddr, u32 Length)
{
	// Subscribe the write transaction
	u32 Status = XAxiDma_SimpleTransfer(&AxiDmaHLS4ML, BuffAddr,
	Length, XAXIDMA_DMA_TO_DEVICE);

	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}
	return XST_SUCCESS;
}


void HLS4ML_Network_WaitToReturn()
{
	// Wait until both transaction are completed
	while ((XAxiDma_Busy(&AxiDmaHLS4ML, XAXIDMA_DEVICE_TO_DMA))
			|| (XAxiDma_Busy(&AxiDmaHLS4ML, XAXIDMA_DMA_TO_DEVICE))) {
		/* Wait */
	}
}
