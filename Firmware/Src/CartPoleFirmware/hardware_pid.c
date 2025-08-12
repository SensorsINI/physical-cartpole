/*
 * hardware_pid.c
 *
 *  Created on: 19 Dec 2023
 *      Author: marcinpaluch
 */

#include "hardware_bridge.h"
#include "communication_with_PC.h"
#include "hardware_pid.h"
#include <stdio.h>
#include <math.h>

float ANGLE_KP      =          18.0f;
float ANGLE_KI      =          38.0f	;
float ANGLE_KD      =          4.0f;

float POSITION_KP   =          22.0f;
float POSITION_KI   =          1.0f;
float POSITION_KD   =          12.0f;

float POSITION_ONLY_KP   =     4.0f;
float POSITION_ONLY_KI   =     0.0f;
float POSITION_ONLY_KD   =     0.0f;

float sensitivity_pP_gain = 1.0;
float sensitivity_pI_gain = 1.0;
float sensitivity_pD_gain = 0.01;

float sensitivity_aP_gain = 1.0;
float sensitivity_aI_gain = 1.0;
float sensitivity_aD_gain = 0.01;

// Error and previous error variables
float position_error = 0.0;

// Last time variable
float time_last = -1.0;

PIDState pid_state_angle = {0.0, 0.0};
PIDState pid_state_position = {0.0, 0.0};
PIDState pid_position_state_position = {0.0, 0.0};

// PID step function
float pid_step(float angle, float angleD, float position, float positionD, float target_position, float time) {

	float time_difference;
    float Q_position, Q_angle, Q;
    // Time difference calculation
    if (time_last < 0.0) {
        time_difference = 0.0;
    } else {
        time_difference = time - time_last;
    }

    if (time_difference > 0.1) {
        time_difference = 0.0;
    }

    time_last = time;

    // Position PID
    position_error = position - target_position;
    float I_clip = 1.0/POSITION_KI;
    Q_position = pid_core(&pid_state_position, position_error, time_difference, POSITION_KP, POSITION_KI, POSITION_KD, sensitivity_pP_gain, sensitivity_pI_gain, sensitivity_pD_gain, I_clip);

    // Angle PID
    I_clip = 1.0/ANGLE_KI;
    Q_angle = pid_core(&pid_state_angle, angle, time_difference, -ANGLE_KP, -ANGLE_KI, -ANGLE_KD, sensitivity_aP_gain, sensitivity_aI_gain, sensitivity_aD_gain, I_clip);

    Q = Q_angle + Q_position;

    return Q;
}


float pid_position_step(float angle, float angleD, float position, float positionD, float target_position, float time) {

	float time_difference;
    float Q;
    // Time difference calculation
    if (time_last < 0.0) {
        time_difference = 0.0;
    } else {
        time_difference = time - time_last;
    }

    if (time_difference > 0.1) {
        time_difference = 0.0;
    }

    time_last = time;

    // Position PID
    position_error = position - target_position;

    float I_clip = 0.0005;
    Q = pid_core(&pid_position_state_position, position_error, time_difference, -POSITION_ONLY_KP, -POSITION_ONLY_KI, -POSITION_ONLY_KD, sensitivity_pP_gain, sensitivity_pI_gain, sensitivity_pD_gain, I_clip);

    return Q;
}


float pid_core(PIDState *pid_state, float error, float time_difference,
		float KP, float KI, float KD,
		float sensitivity_P_gain, float sensitivity_I_gain, float sensitivity_D_gain,
		float I_clip
)
{
    float error_diff = 0.0;

    if (time_difference > 0.0001) {
        error_diff = (error - pid_state->error_previous) / time_difference;
    } else {
        error_diff = 0.0;
    }

    pid_state->error_previous = error;

	pid_state->error_integral += error * time_difference;
	// Clipping; dividing with KI gain prevents error_integral becoming big and destabilizing the system
	if (KI != 0) {
	    pid_state->error_integral = fmax(fmin(pid_state->error_integral, fabs(I_clip)), -fabs(I_clip));
	} else {
	    pid_state->error_integral = 0; // or handle the zero case as needed
	}

    float aP = KP * error * sensitivity_P_gain;
    float aI = KI * pid_state->error_integral * sensitivity_I_gain;
    float aD = KD * error_diff * sensitivity_D_gain;

    float control_signal = aP + aI + aD;

    return control_signal;
}


void cmd_SetPIDConfig(const unsigned char * config)
{
	disable_irq();

	POSITION_KP         = *((float          *)&config[ 0]);
	POSITION_KI         = *((float          *)&config[ 4]);
	POSITION_KD         = *((float          *)&config[8]);

	ANGLE_KP            = *((float          *)&config[ 12]);
	ANGLE_KI            = *((float          *)&config[ 16]);
	ANGLE_KD            = *((float          *)&config[20]);

	enable_irq();
}


void cmd_GetPIDConfig(unsigned char * txBuffer)
{
	prepare_message_to_PC_config_PID(txBuffer, POSITION_KP, POSITION_KI, POSITION_KD, ANGLE_KP, ANGLE_KI, ANGLE_KD);

	disable_irq();
	Message_SendToPC(txBuffer, 34);
	enable_irq();
}

/* =======================================================================
 * New controller API wrappers (non-invasive):
 * - Expose two ControllerOps instances while preserving original code.
 * - PID_Ops (angle+position): inputs = angle, angleD, position, positionD, target_position, time
 * - PIDPos_Ops (position-only): inputs = position, positionD, target_position, time
 * - evaluate() forwards to existing pid_step / pid_position_step.
 * ======================================================================= */

static const char* const PID_InputNames[] = {
    "angle", "angleD", "position", "positionD", "target_position", "time"
};

static const ControllerSpec PID_Spec = {
    .version   = 1,
    .n_inputs  = 6,
    .n_outputs = 1,
    .names     = PID_InputNames
};

static void PID_Init(void)
{
    /* Reset internal state used by the legacy implementation. */
    pid_state_angle.error_previous = 0.0f;
    pid_state_angle.error_integral = 0.0f;
    pid_state_position.error_previous = 0.0f;
    pid_state_position.error_integral = 0.0f;
    pid_position_state_position.error_previous = 0.0f;
    pid_position_state_position.error_integral = 0.0f;
    position_error = 0.0f;
    time_last = -1.0f;
}

static void PID_Release(void)
{
    /* nothing */
}

static void PID_Evaluate(const float* in, float* out)
{
    /* Forward to the existing function; 'time' is provided by the caller. */
    out[0] = pid_step(in[0], in[1], in[2], in[3], in[4], in[5]);
}

static const ControllerSpec* PID_GetSpec(void)
{
    return &PID_Spec;
}

const ControllerOps PID_Ops = {
    .spec     = PID_GetSpec,
    .init     = PID_Init,
    .evaluate = PID_Evaluate,
    .release  = PID_Release
};


/* -------- Position-only controller -------- */

static const char* const PIDPos_InputNames[] = {
    "position", "positionD", "target_position", "time"
};

static const ControllerSpec PIDPos_Spec = {
    .version   = 1,
    .n_inputs  = 4,
    .n_outputs = 1,
    .names     = PIDPos_InputNames
};

static void PIDPos_Init(void)
{
    PID_Init();
}

static void PIDPos_Release(void)
{
    /* nothing */
}

static void PIDPos_Evaluate(const float* in, float* out)
{
    /* Existing pid_position_step expects angle,angleD as well; pass zeros. */
    out[0] = pid_position_step(0.0f, 0.0f, in[0], in[1], in[2], in[3]);
}

static const ControllerSpec* PIDPos_GetSpec(void)
{
    return &PIDPos_Spec;
}

const ControllerOps PIDPos_Ops = {
    .spec     = PIDPos_GetSpec,
    .init     = PIDPos_Init,
    .evaluate = PIDPos_Evaluate,
    .release  = PIDPos_Release
};
