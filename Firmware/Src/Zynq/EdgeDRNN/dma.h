#ifndef DMA_H_
#define DMA_H_

#include "xparameters.h"
#include "stdlib.h"
#include "math.h"
#include "xil_printf.h"
#include "dma_passthrough.h"

#define DMA_SUCCESS 0
#define DMA_FAIL    -1

typedef struct dma
{
	dma_passthrough_t* p_dma_passthrough_inst;
	/*************************************
	 * Memory Mapped to Stream (PS to PL)
	 *************************************/
	u32                mm2s_burst_length;
	u32                mm2s_bytes_per_beat;
	/*************************************
	 * Stream to Memory Mapped (PL to PS)
	 *************************************/
	u32                s2mm_burst_length;
	u32                s2mm_bytes_per_beat;
} dma_t;

extern dma_t dma_inst;             // AXI Direct Memory Access

dma_t* dma_create(
	int dma_id,
	u32 a_qi,
	u32 a_qf,
	u32 hp_size,
	u32 rnn_inp_size,
	u32 rnn_hid_size
);

short* dma_set_snd_buf(dma_t* p_dma_obj, short* p_snd_buf);

short* dma_set_rcv_buf(dma_t* p_dma_obj, short* p_rcv_buf);

short* dma_set_snd_buf_addr(dma_t* p_dma_obj, short* p_snd_buf);

int dma_snd(dma_t* dma_inst);

int dma_snd_nb(dma_t* dma_inst);

u32 dma_get_snd_status(dma_t* p_dma_obj);

int dma_rcv(dma_t* dma_inst);

int dma_rcv_nb(dma_t* p_dma_obj);

u32 dma_get_rcv_status(dma_t* p_dma_obj);

u32 dma_get_snd_beat_size(dma_t* p_dma_obj);

u32 dma_get_rcv_beat_size(dma_t* p_dma_obj);

u32 dma_get_snd_burst_length(dma_t* p_dma_obj);

u32 dma_get_rcv_burst_length(dma_t* p_dma_obj);

u32 dma_get_snd_buf_length(dma_t* p_dma_obj);

u32 dma_get_rcv_buf_length(dma_t* p_dma_obj);
#endif
