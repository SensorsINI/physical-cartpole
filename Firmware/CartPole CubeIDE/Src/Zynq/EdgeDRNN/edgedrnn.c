#include "edgedrnn.h"

edgedrnn_t* edgedrnn_create(
	int edgedrnn_id,
	u32 w_BaseAddr,
	u32 num_pe,
	u32 thx,
	u32 thh,
	u32 a_qi, 
	u32 a_qf, 
	u32 w_qi, 
	u32 w_qf, 
	u32 hp_size,
	u32 rnn_num_layers,
	u32 rnn_inp_size,
	u32 rnn_hid_size,
    u32 rnn_mat_size
){
	// Local variables
	edgedrnn_t* p_edgedrnn_obj;

	// Allocate memory for edgedrnn object
	p_edgedrnn_obj = (edgedrnn_t*) malloc(sizeof(edgedrnn_t));
	if (p_edgedrnn_obj == NULL)
	{
		xil_printf("ERROR! Failed to allocate memory for edgedrnn object.\n\r");
		return NULL;
	}
	/************************************************
	 * Set edgedrnn HW & Network Parameters
	 ************************************************/
	// Activation
	p_edgedrnn_obj->a_qi           = a_qi;
	p_edgedrnn_obj->a_qf           = a_qf;
	p_edgedrnn_obj->a_bw           = a_qi + a_qf;
	p_edgedrnn_obj->a_word_size    = (u32)ceil((double)p_edgedrnn_obj->a_bw/8.0);
	// Weight
    p_edgedrnn_obj->w_BaseAddr     = w_BaseAddr;
	p_edgedrnn_obj->w_qi           = w_qi;
	p_edgedrnn_obj->w_qf           = w_qf;
	p_edgedrnn_obj->w_bw           = w_qi + w_qf;
	p_edgedrnn_obj->w_word_size    = (u32)ceil((double)p_edgedrnn_obj->w_bw/8.0);
	// Network
	p_edgedrnn_obj->rnn_num_layers = rnn_num_layers;
	p_edgedrnn_obj->rnn_inp_size   = rnn_inp_size;
	p_edgedrnn_obj->rnn_hid_size   = rnn_hid_size;
    p_edgedrnn_obj->rnn_mat_size   = rnn_mat_size;
	// Hardware
	p_edgedrnn_obj->num_pe         = num_pe;
	p_edgedrnn_obj->hp_size        = hp_size;
	p_edgedrnn_obj->io_size        = p_edgedrnn_obj->a_word_size*num_pe;
	p_edgedrnn_obj->wp_size        = p_edgedrnn_obj->w_word_size*num_pe;
	p_edgedrnn_obj->col_addr_size  = 2;  // 16-bit column address

	/************************************************
	 * Write edgedrnn config registers through AXI-GP
	 ************************************************/
	// Config network size
	write_cfg_reg (edgedrnn_id, 0, w_BaseAddr);     // Base Address of Weights
	write_cfg_reg (edgedrnn_id, 1, thx);            // Delta Threshold X
	write_cfg_reg (edgedrnn_id, 2, thh);            // Delta Threshold X
	write_cfg_reg (edgedrnn_id, 3, rnn_num_layers); // Number of layers
	write_cfg_reg (edgedrnn_id, 4, rnn_inp_size);   // Input layer Size
	write_cfg_reg (edgedrnn_id, 5, rnn_hid_size);   // Hidden layer 1 Size
	write_cfg_reg (edgedrnn_id, 6, rnn_hid_size);   // Hidden layer 2 Size
	write_cfg_reg (edgedrnn_id, 7, 0);              // Hidden layer 3 Size
	write_cfg_reg (edgedrnn_id, 8, 0);              // Hidden layer 4 Size


	/************************************************
	 * Write RNN Parameters to DRAM in PS 
	 ************************************************/
	Xil_DCacheFlushRange(w_BaseAddr, rnn_mat_size);

	return p_edgedrnn_obj;
}



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
u16 cast_fp_to_fxp(double x, int m, int n)
{

    double max_val = 32767; // 2^(m+n-1)-1
    double min_val = -32768;                        // -2^(m+n-1)
	double value = round(x * 256);
    value = clamp(value, min_val, max_val);
    return (uint16_t)((int) value);
}

//u16 cast_fp_to_fxp(double x, int m, int n)
//{
//
//	double power = pow(2.0, (double)n);
//    double max_val = pow(2.0, (double)(m + n - 1)) - 1; // 2^(m+n-1)-1
//    double min_val = -max_val-1;                        // -2^(m+n-1)
//	double value = round(x * power);
//    value = clamp(value, min_val, max_val);
//    return (uint16_t)((int) value);
//}

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
double cast_fxp_to_fp(u16 x, int n)
{
	double power = pow(2.0, (double)n);
	double value = ((double) (short)x) / power;
    return value;
}

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
double clamp (double x, double min, double max)
{
	double t = x < min ? min : x;
	return t > max ? max : t;
}

/******************************************************************************/
/** write_cfg_reg\
*
* @param edgedrnn_id: base address of EdgeDRNN
* @param reg_idx:     index of the config register
* @param reg_val:     register value
*
* @return
*
* @note		Assign a value to the EdgeDRNN config register
*
*******************************************************************************/
void write_cfg_reg (int edgedrnn_id, int reg_idx, int reg_val)
{
	Xil_Out32(edgedrnn_id + 4*reg_idx, reg_val);
}


float classification_layer(short* p_rcv_buf)
{
	float cl_out = 0;
	float inp_curr = 0;
	float weight_curr = 0;
	for (int i = 0; i < RNN_SIZE; i++)
	{
		inp_curr = (float) p_rcv_buf[i]/256.0;
		weight_curr = cl_weight_fp[i];
		cl_out += inp_curr * weight_curr;
	}
	cl_out += cl_bias_fp[0];

	return cl_out;
}

float classification_layer_fxp(short* p_rcv_buf)
{
	int cl_out = 0;
	int inp_curr = 0;
	int weight_curr = 0;
	float cl_temp = 0.;
	float cl_out_fl = 0.;
	for (int i = 0; i < RNN_SIZE; i++)
	{
		inp_curr = (int)p_rcv_buf[i];
		weight_curr = (int)cl_weight[i];
		cl_out += inp_curr * weight_curr;
	}
	cl_out += (int) cl_bias[0] * 256;

	cl_temp = (float) cl_out;

	cl_out = round(cl_temp / 128.0);

//	cl_out_fl = (float) cl_out / 32768.0;

	return cl_out;
}
