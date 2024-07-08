#include "angle_processing.h"
#include "parameters.h"
#include "median_filter.h"
#include <stdlib.h>
#include <limits.h> // For SHRT_MAX
#include "math.h"



// Averaging derivatives with median filter
#define ANGLE_D_BUFFER_SIZE 1 // Median filter for pole's angular velocity
#define POSITION_D_BUFFER_SIZE 1 // Median filter for cart's velocity
#define timesteps_for_derivatives 5 // Based on how many timesteps should the derivative be calculated, This is also used to determine invalid steps so you need it even if you don't use derivative from chip!



const int ADC_RANGE = 4096;

int angle_raw = 0, angle_raw_prev = -1, angle_raw_stable = -1, angle_raw_sensor;
float angleD_raw = 0, angleD_raw_stable = -1, angleD_raw_sensor;
int frozen = 0;

float angleDBuffer[ANGLE_D_BUFFER_SIZE]; // Buffer for angle derivatives, using int
float positionDBuffer[POSITION_D_BUFFER_SIZE]; // Buffer for position derivatives, also using int for processing

// Initialize buffer indices
unsigned short angleDBufferIndex = 0;
unsigned short positionDBufferIndex = 0;

// Helper function to update circular buffers for int values
void updateCircularBuffer(int* buffer, unsigned short* index, unsigned short size, int newValue) {
    buffer[*index] = newValue;
    *index = (*index + 1) % size; // Update index in a circular manner
}

void updateCircularBuffer_float(float* buffer, unsigned short* index, unsigned short size, float newValue) {
    buffer[*index] = newValue;
    *index = (*index + 1) % size; // Update index in a circular manner
}

// Function to average derivatives
void average_derivatives(float* angleDPtr, float* positionDPtr){
    // Update angleD buffer with current value
    updateCircularBuffer_float(angleDBuffer, &angleDBufferIndex, ANGLE_D_BUFFER_SIZE, *angleDPtr);

    updateCircularBuffer_float(positionDBuffer, &positionDBufferIndex, POSITION_D_BUFFER_SIZE, *positionDPtr);

    // Calculate medians using the updated buffers
    float angleDMedian = ClassicMedianFilter_float(angleDBuffer, ANGLE_D_BUFFER_SIZE); // Adjust casting if necessary
    float positionDMedian = ClassicMedianFilter_float(positionDBuffer, POSITION_D_BUFFER_SIZE); // Adjust casting if necessary

    // Update pointers with median values
    *angleDPtr = angleDMedian;
    *positionDPtr = positionDMedian; // Convert int back to short
}

void process_angle(int angleSamples[], unsigned short angleSampIndex, unsigned short angle_averageLen, int* anglePtr, int* angle_raw_Ptr, float* angleDPtr, int* invalid_stepPtr){
		int angle = ClassicMedianFilter(angleSamples, angle_averageLen);
		*anglePtr = angle;
		*angle_raw_Ptr = angle;

		int invalid_step = anomaly_detection(angleSamples, angleSampIndex, angle_averageLen);
		*invalid_stepPtr = invalid_step;

		treat_deadangle_with_derivative(anglePtr, invalid_step);

		*angleDPtr = angleD_raw;
}

// Anomaly Detection: count invalid buffer steps
int anomaly_detection(int* angleSamples, unsigned short angleSampIndex, unsigned short angle_averageLen){
	int invalid_step = 0;
	if(angle_averageLen > 1) {
		for (int i = 0; i < angle_averageLen; i++) {
			// start at oldest value (since angleSampIndex is not yet overwritten)
			int curr = angleSamples[(angleSampIndex + i) % angle_averageLen];
			int prev = angleSamples[(angleSampIndex + i + angle_averageLen - 1) % angle_averageLen];

			// previous value for oldest value not existing
			if(i != 0 && abs(wrapLocal(curr-prev)) > MAX_ADC_STEP)
				invalid_step++;
		}
	}
	return invalid_step;
}



#define ANGLE_HISTORY_BUFFER_SIZE (timesteps_for_derivatives+1)
int angle_history[ANGLE_HISTORY_BUFFER_SIZE]; // Buffer to store past angles
int frozen_history[ANGLE_HISTORY_BUFFER_SIZE]; // Buffer to store past angles
int idx_for_derivative_calculation_angle = 0; // Current index in the buffer
int angle_history_initialised = 0;

// Initialize the angle history buffer to -1
void init_angle_history() {
    for (int i = 0; i < ANGLE_HISTORY_BUFFER_SIZE; ++i) {
        angle_history[i] = -1;
        frozen_history[i] = 0;
    }
}


void treat_deadangle_with_derivative(int* anglePtr, int invalid_step) {

	if (angle_history_initialised == 0)
	{
		init_angle_history();
		angle_history_initialised = 1;
	}


    // Calculate the index for the k-th past angle
    int kth_past_index = (idx_for_derivative_calculation_angle - timesteps_for_derivatives + ANGLE_HISTORY_BUFFER_SIZE) % ANGLE_HISTORY_BUFFER_SIZE;
    int kth_past_angle = angle_history[kth_past_index];
    int kth_past_frozen = frozen_history[kth_past_index];

    // Anomaly Detection: unstable buffer (invalid steps) or unstable angle_raw (jump in angle_raw), only inside region close to 0
    if (kth_past_angle != -1 &&
       ((invalid_step > 5 && abs(wrapLocal(kth_past_angle)) < ADC_RANGE/20) ||
       (abs(wrapLocal(*anglePtr - kth_past_angle)) > ADC_RANGE/8 && kth_past_frozen < 3))) {

        frozen++;
        *anglePtr = angle_raw_stable != -1 ? angle_raw_stable : 0;
        angleD_raw = angleD_raw_stable != -1 ? angleD_raw_stable : 0;
    } else {
        int not_normed_angleD_raw = angle_raw_stable != -1 ? wrapLocal(*anglePtr - kth_past_angle) :0;
        angleD_raw =  (float)(not_normed_angleD_raw)/ ((timesteps_for_derivatives-1) + kth_past_frozen + 1);
        angle_raw_stable = *anglePtr;
        angleD_raw_stable = angleD_raw;
        frozen = 0;
    }

    angle_raw_sensor = *anglePtr;
    angleD_raw_sensor = angleD_raw;

    // Save previous values
    angle_raw_prev = *anglePtr;

    // Save current angle in the history buffer and update index
    angle_history[idx_for_derivative_calculation_angle] = *anglePtr;
    frozen_history[idx_for_derivative_calculation_angle] = frozen;
    idx_for_derivative_calculation_angle = (idx_for_derivative_calculation_angle + 1) % ANGLE_HISTORY_BUFFER_SIZE; // Move to next index, wrap around if necessary
}


#define POSITION_HISTORY_BUFFER_SIZE (timesteps_for_derivatives+1)

short position_history[POSITION_HISTORY_BUFFER_SIZE]; // Buffer to store past positions

int idx_for_derivative_calculation_position = 0; // Current index in the buffer
int position_history_initialised = 0;

// Initialize the angle history buffer to -1
void init_position_history() {
    for (int i = 0; i < POSITION_HISTORY_BUFFER_SIZE; ++i) {
        position_history[i] = SHRT_MAX;
        frozen_history[i] = 0;
    }
}


void calculate_position_difference_per_timestep(short* positionPtr, float* positionDPtr) {

	if (position_history_initialised == 0)
	{
		init_position_history();
		position_history_initialised = 1;
	}

    // Calculate the index for the k-th past angle
    int kth_past_index = (idx_for_derivative_calculation_position - timesteps_for_derivatives + POSITION_HISTORY_BUFFER_SIZE) % POSITION_HISTORY_BUFFER_SIZE;
    int kth_past_position = position_history[kth_past_index];

    short not_normed_positionD_raw = kth_past_position != SHRT_MAX ? (*positionPtr - kth_past_position) :0;
    *positionDPtr =  (float)(not_normed_positionD_raw)/ timesteps_for_derivatives;
    position_history[idx_for_derivative_calculation_position] = *positionPtr;

    idx_for_derivative_calculation_position = (idx_for_derivative_calculation_position + 1) % POSITION_HISTORY_BUFFER_SIZE; // Move to next index, wrap around if necessary
}


int wrapLocal(int angle) {
    if (angle > ADC_RANGE/2)
		return angle - ADC_RANGE;
	if (angle <= -ADC_RANGE/2)
		return angle + ADC_RANGE;
	else
		return angle;
}


float wrapLocal_float(float angle) {
    if (angle > ADC_RANGE/2)
		return angle - ADC_RANGE;
	if (angle <= -ADC_RANGE/2)
		return angle + ADC_RANGE;
	else
		return angle;
}


float wrapLocal_rad(float angle) {
    if (angle > M_PI)
		return angle - 2*M_PI;
	if (angle <= -M_PI)
		return angle + 2*M_PI;
	else
		return angle;
}


int unwrapLocal(int previous, int current) {
	int diff = current-previous;

	if (diff > ADC_RANGE/2)
		return current - ADC_RANGE;
	if (diff < -ADC_RANGE/2)
		return current + ADC_RANGE;
	else
		return current;
}

int wrap(int current) {
	if(current > 0)
		return current - ADC_RANGE * (current / ADC_RANGE);
	else
		return current + ADC_RANGE * (current / ADC_RANGE + 1);
}

int unwrap(int previous, int current) {
    int diff = previous-current;
	if (diff>0)
    	return current + ADC_RANGE * (((2 * diff) / ADC_RANGE + 1) / 2);
	else
    	return current + ADC_RANGE * (((2 * diff) / ADC_RANGE - 1) / 2);
}
