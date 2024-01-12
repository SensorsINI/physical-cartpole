#ifndef EDGEDRNN_TEST_H
#define EDGEDRNN_TEST_H
/*
 * Var Type: General Matrix 
 * Var Name: edgedrnn_test_stim
 * Bit Width: 16
 * Dimension: (8, 1000)
 */
#define EDGEDRNN_TEST_STIM_NUM_ROWS 8
#define EDGEDRNN_TEST_STIM_NUM_COLS 1000
#define EDGEDRNN_TEST_STIM_MAT_SIZE 8000
extern const short edgedrnn_test_stim[EDGEDRNN_TEST_STIM_MAT_SIZE];
/*
 * Var Type: General Matrix 
 * Var Name: edgedrnn_test_gold_fc
 * Bit Width: 16
 * Dimension: (1, 1000)
 */
#define EDGEDRNN_TEST_GOLD_FC_NUM_ROWS 1
#define EDGEDRNN_TEST_GOLD_FC_NUM_COLS 1000
#define EDGEDRNN_TEST_GOLD_FC_MAT_SIZE 1000
extern const short edgedrnn_test_gold_fc[EDGEDRNN_TEST_GOLD_FC_MAT_SIZE];
/*
 * Var Type: General Matrix 
 * Var Name: edgedrnn_test_stim_float
 * Bit Width: 0
 * Dimension: (8, 1000)
 */
#define EDGEDRNN_TEST_STIM_FLOAT_NUM_ROWS 8
#define EDGEDRNN_TEST_STIM_FLOAT_NUM_COLS 1000
#define EDGEDRNN_TEST_STIM_FLOAT_MAT_SIZE 8000
extern const float edgedrnn_test_stim_float[EDGEDRNN_TEST_STIM_FLOAT_MAT_SIZE];
/*
 * Var Type: General Matrix 
 * Var Name: edgedrnn_test_gold_fc_float
 * Bit Width: 0
 * Dimension: (1, 1000)
 */
#define EDGEDRNN_TEST_GOLD_FC_FLOAT_NUM_ROWS 1
#define EDGEDRNN_TEST_GOLD_FC_FLOAT_NUM_COLS 1000
#define EDGEDRNN_TEST_GOLD_FC_FLOAT_MAT_SIZE 1000
extern const float edgedrnn_test_gold_fc_float[EDGEDRNN_TEST_GOLD_FC_FLOAT_MAT_SIZE];
/*
 * Var Type: General Matrix 
 * Var Name: edgedrnn_test_gold_rnn
 * Bit Width: 16
 * Dimension: (128, 1000)
 */
#define EDGEDRNN_TEST_GOLD_RNN_NUM_ROWS 128
#define EDGEDRNN_TEST_GOLD_RNN_NUM_COLS 1000
#define EDGEDRNN_TEST_GOLD_RNN_MAT_SIZE 128000
extern const short edgedrnn_test_gold_rnn[EDGEDRNN_TEST_GOLD_RNN_MAT_SIZE];
/*
 * Var Type: General Matrix 
 * Var Name: edgedrnn_test_gold_rnn_float
 * Bit Width: 0
 * Dimension: (128, 1000)
 */
#define EDGEDRNN_TEST_GOLD_RNN_FLOAT_NUM_ROWS 128
#define EDGEDRNN_TEST_GOLD_RNN_FLOAT_NUM_COLS 1000
#define EDGEDRNN_TEST_GOLD_RNN_FLOAT_MAT_SIZE 128000
extern const float edgedrnn_test_gold_rnn_float[EDGEDRNN_TEST_GOLD_RNN_FLOAT_MAT_SIZE];
#endif