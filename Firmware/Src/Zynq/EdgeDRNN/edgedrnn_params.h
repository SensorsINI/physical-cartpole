#ifndef EDGEDRNN_PARAMS_H
#define EDGEDRNN_PARAMS_H
/*
 * Var Type: NN Parameter Matrix 
 * Var Name: rnn_param
 * Bit Width: 8
 * Dimension: (384, 394)
 */
#define RNN_PARAM_SIZE 151296
extern const signed char rnn_param[RNN_PARAM_SIZE] __attribute__ ((aligned (8)));
/*
 * Var Type: NN Parameter Matrix 
 * Var Name: cl_bias
 * Bit Width: 8
 * Dimension: (1, 1)
 */
#define CL_BIAS_SIZE 1
extern const signed char cl_bias[CL_BIAS_SIZE] __attribute__ ((aligned (8)));
/*
 * Var Type: General Matrix 
 * Var Name: cl_bias_fp
 * Bit Width: 0
 * Dimension: (1, 1)
 */
#define CL_BIAS_FP_NUM_ROWS 1
#define CL_BIAS_FP_NUM_COLS 1
#define CL_BIAS_FP_MAT_SIZE 1
extern const float cl_bias_fp[CL_BIAS_FP_MAT_SIZE];
/*
 * Var Type: NN Parameter Matrix 
 * Var Name: cl_weight
 * Bit Width: 8
 * Dimension: (1, 128)
 */
#define CL_WEIGHT_SIZE 128
extern const signed char cl_weight[CL_WEIGHT_SIZE] __attribute__ ((aligned (8)));
/*
 * Var Type: General Matrix 
 * Var Name: cl_weight_fp
 * Bit Width: 0
 * Dimension: (1, 128)
 */
#define CL_WEIGHT_FP_NUM_ROWS 1
#define CL_WEIGHT_FP_NUM_COLS 128
#define CL_WEIGHT_FP_MAT_SIZE 128
extern const float cl_weight_fp[CL_WEIGHT_FP_MAT_SIZE];
#define RNN_LAYERS 2
#define INP_SIZE 8
#define RNN_SIZE 128
#define THX 0
#define THH 64
#define NUM_PE 8
#define AQI 8
#define AQF 8
#define WQI 1
#define WQF 7
#endif