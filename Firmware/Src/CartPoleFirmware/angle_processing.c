#include "angle_processing.h"
#include "parameters.h"
#include "median_filter.h"
#include <stdlib.h>
#include <limits.h> // For SHRT_MAX
#include "math.h"



// Averaging derivatives with median filter on Firmware
#define ANGLE_D_BUFFER_SIZE 1 // Median filter for pole's angular velocity
#define POSITION_D_BUFFER_SIZE 1 // Median filter for cart's velocity

#define MAX_TIMESTEPS_FOR_DERIVATIVE 20


int angle_raw = 0, angle_raw_prev = -1, angle_raw_stable = -1;
float angleD_raw = 0, angleD_raw_stable = -1;
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

int angle_history[MAX_TIMESTEPS_FOR_DERIVATIVE+1]; // Buffer to store past angles
int frozen_history[MAX_TIMESTEPS_FOR_DERIVATIVE+1]; // Buffer to store past angles
int idx_for_derivative_calculation_angle = 0; // Current index in the buffer
int angle_history_initialised = 0;

// Initialize the angle history buffer to -1
void init_angle_history() {

    for (int i = 0; i < MAX_TIMESTEPS_FOR_DERIVATIVE+1; ++i) {
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
    int kth_past_index = (idx_for_derivative_calculation_angle + 1) % (TIMESTEPS_FOR_DERIVATIVE+1);;
    int kth_past_angle = angle_history[kth_past_index];
    int kth_past_frozen = frozen_history[kth_past_index];

    int not_normed_angleD_raw = kth_past_angle == -1 ? 0 : wrapLocal(*anglePtr - kth_past_angle);
    float current_difference = (float)(not_normed_angleD_raw)/ (TIMESTEPS_FOR_DERIVATIVE + kth_past_frozen);

    // Anomaly Detection: unstable buffer (invalid steps) or unstable angle_raw (jump in angle_raw), only inside region close to 0
    if (kth_past_angle != -1 &&
       ((invalid_step > 5 && abs(wrapLocal(kth_past_angle)) < ANGLE_360_DEG_IN_ADC_UNITS/20) ||
       (abs(current_difference) > TIMESTEPS_FOR_DERIVATIVE * ANGLE_360_DEG_IN_ADC_UNITS/8 && kth_past_frozen < 3))) {

        frozen++;
        *anglePtr = angle_raw_stable != -1 ? angle_raw_stable : 0;
        angleD_raw = angleD_raw_stable != -1 ? angleD_raw_stable : 0;
    } else {
        angleD_raw = current_difference;
        angle_raw_stable = *anglePtr;
        angleD_raw_stable = angleD_raw;
        frozen = 0;
    }

    // Save previous values
    angle_raw_prev = *anglePtr;

    // Save current angle in the history buffer and update index
    angle_history[idx_for_derivative_calculation_angle] = *anglePtr;
    frozen_history[idx_for_derivative_calculation_angle] = frozen;
    idx_for_derivative_calculation_angle = (idx_for_derivative_calculation_angle + 1) % (TIMESTEPS_FOR_DERIVATIVE+1); // Move to next index, wrap around if necessary
}

short position_history[MAX_TIMESTEPS_FOR_DERIVATIVE+1]; // Buffer to store past positions

int idx_for_derivative_calculation_position = 0; // Current index in the buffer
int position_history_initialised = 0;

// Initialize the angle history buffer to -1
void init_position_history() {

    for (int i = 0; i < MAX_TIMESTEPS_FOR_DERIVATIVE+1; ++i) {
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
    int kth_past_index = (idx_for_derivative_calculation_position +1) % (TIMESTEPS_FOR_DERIVATIVE+1);
    int kth_past_position = position_history[kth_past_index];

    short not_normed_positionD_raw = kth_past_position != SHRT_MAX ? (*positionPtr - kth_past_position) :0;
    *positionDPtr =  (float)(not_normed_positionD_raw)/ TIMESTEPS_FOR_DERIVATIVE;
    position_history[idx_for_derivative_calculation_position] = *positionPtr;

    idx_for_derivative_calculation_position = (idx_for_derivative_calculation_position + 1) % (TIMESTEPS_FOR_DERIVATIVE+1); // Move to next index, wrap around if necessary
}


int wrapLocal(int angle) {
    if (angle > ANGLE_360_DEG_IN_ADC_UNITS/2)
		return angle - ANGLE_360_DEG_IN_ADC_UNITS;
	if (angle <= -ANGLE_360_DEG_IN_ADC_UNITS/2)
		return angle + ANGLE_360_DEG_IN_ADC_UNITS;
	else
		return angle;
}


float wrapLocal_float(float angle) {
    if (angle > ANGLE_360_DEG_IN_ADC_UNITS/2)
		return angle - ANGLE_360_DEG_IN_ADC_UNITS;
	if (angle <= -ANGLE_360_DEG_IN_ADC_UNITS/2)
		return angle + ANGLE_360_DEG_IN_ADC_UNITS;
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

	if (diff > ANGLE_360_DEG_IN_ADC_UNITS/2)
		return current - ANGLE_360_DEG_IN_ADC_UNITS;
	if (diff < -ANGLE_360_DEG_IN_ADC_UNITS/2)
		return current + ANGLE_360_DEG_IN_ADC_UNITS;
	else
		return current;
}

int wrap(int current) {
	if(current > 0)
		return current - ANGLE_360_DEG_IN_ADC_UNITS * (current / ANGLE_360_DEG_IN_ADC_UNITS);
	else
		return current + ANGLE_360_DEG_IN_ADC_UNITS * (current / ANGLE_360_DEG_IN_ADC_UNITS + 1);
}

int unwrap(int previous, int current) {
    int diff = previous-current;
	if (diff>0)
    	return current + ANGLE_360_DEG_IN_ADC_UNITS * (((2 * diff) / ANGLE_360_DEG_IN_ADC_UNITS + 1) / 2);
	else
    	return current + ANGLE_360_DEG_IN_ADC_UNITS * (((2 * diff) / ANGLE_360_DEG_IN_ADC_UNITS - 1) / 2);
}
