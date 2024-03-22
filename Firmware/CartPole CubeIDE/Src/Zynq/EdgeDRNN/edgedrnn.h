#ifndef edgedrnn_H
#define edgedrnn_H

#include "xil_io.h"
#include "xil_types.h"
#include "xparameters.h"
#include "xil_cache.h"
#include "stdlib.h"
#include "math.h"

#include "edgedrnn_params.h"

// Return types
#define edgedrnn_SUCCESS   0
#define edgedrnn_FAIL     -1

typedef struct edgedrnn
{

	/*******************
	 * Activation
	 *******************/
	u32 a_qi;		 // Number of integer bits before the decimal point - Activation
	u32 a_qf;		 // Number of integer bits after the decimal point - Activation
	u32 a_bw;        // Bit width - Activation
	u32 a_word_size; // Word size in byte - Activation
	/*******************
	 * Weight
	 *******************/
	u32 w_BaseAddr;  // Weight base address
	u32 w_qi;		 // Number of integer bits after the decimal point - Weight
	u32 w_qf;		 // Number of integer bits after the decimal point - Weight
	u32 w_bw;        // Bit width - Weight
	u32 w_word_size; // Word size in byte - Weight
	/*******************
	 * Network
	 *******************/
	u32 rnn_num_layers;
	u32 rnn_inp_size;
	u32 rnn_hid_size;
    u32 rnn_mat_size;
	/*******************
	 * Hardware
	 *******************/
	u32 num_pe;                 // Number of process element
	u32 hp_size;                // AXI-HP port size in bytes
	u32 io_size;                // I/O AXIS port size in bytes
	u32 wp_size;                // Weight input port size in bytes
	u32 col_addr_size;          // Column address output port size in bytes
} edgedrnn_t;

edgedrnn_t* edgedrnn_create(
	int edgedrnn_id,
    u32 w_BaseAddr,
	u32 num_pe,
	u32 th_x,
	u32 th_h,
	u32 a_qi, 
	u32 a_qf, 
	u32 w_qi, 
	u32 w_qf, 
	u32 hp_size,
	u32 rnn_num_layers,
	u32 rnn_inp_size,
	u32 rnn_hid_size,
    u32 rnn_mat_size
);



/******************************************************************************/
/** cast_fp_to_fxp
*
* @param x: input floating point number
* @param m: number of integer bits before the decimal point
* @param n: number of fraction bits after the decimal point
*
* @return	output fixed point number
*
* @note		Converts a floating point number into a fixed point number in uint32
*
*******************************************************************************/
u16 cast_fp_to_fxp(double x, int m, int n);


/******************************************************************************/
/** cast_fxp_to_fp
*
* @param x: input fixed point number
* @param n: number of fraction bits after the decimal point
*
* @return	output floating point number
*
* @note		Converts a floating point number into a fixed point number in uint32
*
*******************************************************************************/
double cast_fxp_to_fp(u16 x, int n);

/******************************************************************************/
/** clamp
*
* @param x:   input number
* @param min: lower threshold
* @param max: higher threshold
*
* @return	output clipped number
*
* @note		Clip function that limits input in the interval of [min, max]
*
*******************************************************************************/
double clamp (double x, double min, double max);

void write_cfg_reg (int edgedrnn_id, int reg_idx, int reg_val);
float classification_layer(short* p_rcv_buf);
float classification_layer_fxp(short* p_rcv_buf);
#endif
