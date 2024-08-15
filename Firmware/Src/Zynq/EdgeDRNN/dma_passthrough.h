
//////////////////////////////////////////////////////////////////////////////////
//
// Company:        Xilinx
// Engineer:       bwiec
// Create Date:    30 June 2015, 10:57:11 PM
// Library Name:   AXI DMA Passthrough
// File Name:      dma_passthrough.h
// Target Devices: Zynq
// Tool Versions:  2015.1
// Description:    Middleware API for a DMA passthrough using the AXI DMA.
// Dependencies:
//   - xaxidma.h - Driver version v8.1
// Revision History:
//   - v1.0
//     * Initial release
//     * Tested on ZC702 and Zedboard
// Additional Comments:
//   - This library is intended to be used as a layer between the base xaxidma
//     driver and application software. The programming model is a buffer received
//     from the outside world which can be operated on by the CPU. When it's done,
//     it copies the data to a second buffer which is sent back out to the world.
//   - While this library is designed targeting Zynq, since it is a soft IP, it
//     could be easily extended to support Microblaze as well.
//   - Source and destination buffers must be contiguous. Scatter gather is not
//     supported.
//   - S2MM and MM2S tdata widths on the AXI DMA must be the same.
//
//////////////////////////////////////////////////////////////////////////////////
#include "xil_types.h"
#ifndef DMA_PASSTHROUGH_H_
#define DMA_PASSTHROUGH_H_

// Hardware-specific parameters
#define DMA_PASSTHROUGH_IS_CACHE_COHERENT  0 // Set to 1 to avoid overhead of software cache flushes if going through the ACP

// Return types
#define DMA_PASSTHROUGH_SUCCESS            0
#define DMA_PASSTHROUGH_DMA_INIT_FAIL     -1
#define DMA_PASSTHROUGH_XFER_FAIL         -2

// Object forward declaration
typedef struct dma_passthrough dma_passthrough_t;

// Public functions

//
// dma_passthrough_create - Create a DMA Passthrough object.
//
//  Arguments
//    - dma_device_id:      Device ID of the DMA instance to use.
//    - sample_size_bytes:  Number of bytes per sample (i.e. number of bytes in tdata bus).
//
//  Return
//    - dma_passthrough_t*: Non-NULL pointer to dma_passthrough_t object on success.
//    - NULL:               NULL if something failed.
//
dma_passthrough_t* dma_passthrough_create(int dma_device_id);

//
// dma_passthrough_destroy - Destroy DMA Passthrough object.
//
//  Arguments
//    - p_dma_passthrough_inst: Pointer to dma_passthrough_t object to be deallocated.
//
void dma_passthrough_destroy(dma_passthrough_t* p_dma_passthrough_inst);

//
// dma_passthrough_reset - Reset the DMA engine.
//
//  Arguments
//    - p_dma_passthrough_inst: Pointer to dma_passthrough_t object.
//
void dma_passthrough_reset(dma_passthrough_t* p_dma_passthrough_inst);

//
// dma_passthrough_set_rcv_buf - Set pointer to receive buffer to be used.
//
//  Arguments
//    - p_dma_passthrough_inst: Pointer to dma_passthrough_t object.
//    - p_stim_buf:             Pointer to receive buffer to be used by the DMA.
//
void dma_passthrough_set_rcv_buf(dma_passthrough_t* p_dma_passthrough_inst, void* p_rcv_buf);

//
// dma_passthrough_get_rcv_buf - Get a pointer to the receive buffer.
//.
//  Arguments
//    - p_dma_passthrough_inst: Pointer to dma_passthrough_t object.
//
//  Return
//    - void*:                  Pointer to the receive buffer to be used by the DMA.
//
void* dma_passthrough_get_rcv_buf(dma_passthrough_t* p_dma_passthrough_inst);

//
// dma_passthrough_set_snd_buf - Set pointer to send buffer to be used.
//
//  Arguments
//    - p_dma_passthrough_inst: Pointer to dma_passthrough_t object.
//    - p_stim_buf:             Pointer to send buffer to be used by the DMA.
//
void dma_passthrough_set_snd_buf(dma_passthrough_t* p_dma_passthrough_inst, void* p_snd_buf);

//
// dma_passthrough_get_snd_buf - Get a pointer to the send buffer.
//.
//  Arguments
//    - p_dma_passthrough_inst: Pointer to dma_passthrough_t object.
//
//  Return
//    - void*:                  Pointer to the send buffer to be used by the DMA.
//
void* dma_passthrough_get_snd_buf(dma_passthrough_t* p_dma_passthrough_inst);

//
// dma_passthrough_set_rcv_buf_length - Set the receive buffer length (in samples) to use for DMA transfers.
//
//    - p_dma_passthrough_inst: Pointer to dma_passthrough_t object.
//    - buf_length:             Buffer length (in samples) to use for DMA transfers.
//
void dma_passthrough_set_rcv_buf_length(dma_passthrough_t* p_dma_passthrough_inst, int buf_length);

//
// dma_passthrough_set_snd_buf_length - Set the send buffer length (in samples) to use for DMA transfers.
//
//    - p_dma_passthrough_inst: Pointer to dma_passthrough_t object.
//    - buf_length:             Buffer length (in samples) to use for DMA transfers.
//
void dma_passthrough_set_snd_buf_length(dma_passthrough_t* p_dma_passthrough_inst, int buf_length);

//
// dma_passthrough_get_rcv_buf_length - Get the receive buffer length (in samples) to be used for DMA transfers.
//
//  Arguments
//    - p_dma_passthrough_inst: Pointer to dma_passthrough_t object.
//
//  Return
//    - int:                    Buffer length (in samples) to be transferred by the DMA.
//
int dma_passthrough_get_rcv_buf_length(dma_passthrough_t* p_dma_passthrough_inst);

//
// dma_passthrough_get_snd_buf_length - Get the send buffer length (in samples) to be used for DMA transfers.
//
//  Arguments
//    - p_dma_passthrough_inst: Pointer to dma_passthrough_t object.
//
//  Return
//    - int:                    Buffer length (in samples) to be transferred by the DMA.
//
int dma_passthrough_get_snd_buf_length(dma_passthrough_t* p_dma_passthrough_inst);

//
// dma_passthrough_set_rcv_sample_size_bytes - Set the size (in bytes) of each sample (i.e. number
//                                   of bytes in tdata bus).
//
//  Arguments
//    - p_dma_passthrough_inst:  Pointer to dma_passthrough_t object.
//    - sample_size_bytes:       Number of bytes per sample.
//
void dma_passthrough_set_rcv_sample_size_bytes(dma_passthrough_t* p_dma_passthrough_inst, int sample_size_bytes);

//
// dma_passthrough_set_snd_sample_size_bytes - Set the size (in bytes) of each sample (i.e. number
//                                   of bytes in tdata bus).
//
//  Arguments
//    - p_dma_passthrough_inst:  Pointer to dma_passthrough_t object.
//    - sample_size_bytes:       Number of bytes per sample.
//
void dma_passthrough_set_snd_sample_size_bytes(dma_passthrough_t* p_dma_passthrough_inst, int sample_size_bytes);

//
// dma_passthrough_get_rcv_sample_size_bytes - Get the size (in bytes) of each sample (i.e. number
//                                   of bytes in tdata bus).
//
//  Arguments
//    - p_dma_passthrough_inst: Pointer to dma_passthrough_t object.
//
//  Return
//    - int:                    Number of bytes per sample.
//
int dma_passthrough_get_rcv_sample_size_bytes(dma_passthrough_t* p_dma_passthrough_inst);

//
// dma_passthrough_get_snd_sample_size_bytes - Get the size (in bytes) of each sample (i.e. number
//                                   of bytes in tdata bus).
//
//  Arguments
//    - p_dma_passthrough_inst: Pointer to dma_passthrough_t object.
//
//  Return
//    - int:                    Number of bytes per sample.
//
int dma_passthrough_get_snd_sample_size_bytes(dma_passthrough_t* p_dma_passthrough_inst);

//
// dma_passthrough_rcv - Receive a data buffer via DMA.
//
//  Arguments
//    - p_dma_passthrough_inst:    Pointer to dma_passthrough_t object.
//
//  Return
//    - DMA_SUCCESS:               No errors occurred during any DMA transfers.
//    - DMA_PASSTHROUGH_XFER_FAIL: Some error occurred during one of the DMA transfers.
//
int dma_passthrough_rcv(dma_passthrough_t* p_dma_passthrough_inst);
int dma_passthrough_rcv_nb(dma_passthrough_t* p_dma_passthrough_inst);
u32 dma_passthrough_rcv_busy(dma_passthrough_t* p_dma_passthrough_inst);

//
// dma_passthrough_snd - Send a data buffer via DMA.
//
//  Arguments
//    - p_dma_passthrough_inst:    Pointer to dma_passthrough_t object.
//
//  Return
//    - DMA_SUCCESS:               No errors occurred during any DMA transfers.
//    - DMA_PASSTHROUGH_XFER_FAIL: Some error occurred during one of the DMA transfers.
//
int dma_passthrough_snd(dma_passthrough_t* p_dma_passthrough_inst);
int dma_passthrough_snd_nb(dma_passthrough_t* p_dma_passthrough_inst);
u32 dma_passthrough_snd_busy(dma_passthrough_t* p_dma_passthrough_inst);

//
// dma_passthrough_create - Create a DMA Passthrough object.
//
//  Arguments
//    - dma_device_id:      Device ID of the DMA instance to use.
//    - sample_size_bytes:  Number of bytes per sample (i.e. number of bytes in tdata bus).
//
//  Return
//    - dma_passthrough_t*: Non-NULL pointer to dma_passthrough_t object on success.
//    - NULL:               NULL if something failed.
//
dma_passthrough_t* dma_passthrough_create_without_itr(int dma_device_id, int sample_size_bytes);

int dma_passthrough_rcv_mic(dma_passthrough_t* p_dma_passthrough_inst);
#endif /* DMA_PASSTHROUGH_H_ */
