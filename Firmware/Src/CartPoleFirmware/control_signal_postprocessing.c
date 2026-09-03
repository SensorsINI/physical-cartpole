/*
 * control_signal_postprocessing.c
 *
 *  Created on: Dec 19, 2023
 *      Author: marcinpaluch
 */

#include "hardware_bridge.h"
#include "parameters.h"
#include <limits.h>
#include <math.h>
#include <stdlib.h>

#define POSITION_LIMIT_BRAKE_MARGIN_COUNTS 300
#define MOTOR_STALL_TIMEOUT_MS 100u
#define MOTOR_STALL_MIN_TRAVEL_COUNTS 2

static int stall_guard_initialized;
static int stall_guard_latched;
static int stall_guard_last_position;
static unsigned int stall_guard_stationary_ms;

int control_signal_to_motor_command(float Q, float positionD, bool correct_motor_dynamics) {
	if (!isfinite(Q) || !isfinite(positionD)) {
		return 0;
	}
	float actualMotorCmd_float = Q;
    int actualMotorCmd = 0;

    if (correct_motor_dynamics) {

    	actualMotorCmd_float = actualMotorCmd_float * MOTOR_CORRECTION[0];
        if (actualMotorCmd_float != 0) {
            if (positionD > 0) {
            	actualMotorCmd_float += MOTOR_CORRECTION[1];
            } else if (positionD < 0) {
            	actualMotorCmd_float -= MOTOR_CORRECTION[2];
            }
        }
    }

    actualMotorCmd_float = actualMotorCmd_float * (float)MOTOR_PWM_PERIOD_IN_CLOCK_CYCLES;
    if (!isfinite(actualMotorCmd_float)) {
		return 0;
    }

    actualMotorCmd = (int)actualMotorCmd_float;

    return actualMotorCmd;
}

void motor_command_safety_check(int* motor_command_ptr){
    // Check if motor power in safe boundaries
    if (*motor_command_ptr > MOTOR_FULL_SCALE_SAFE) *motor_command_ptr = MOTOR_FULL_SCALE_SAFE;
    else if (*motor_command_ptr < -MOTOR_FULL_SCALE_SAFE) *motor_command_ptr = -MOTOR_FULL_SCALE_SAFE;
}


void safety_switch_off(int* motor_command_ptr, int positionLimitLeft, int positionLimitRight){
	int position = Encoder_Read();
    // Leave braking distance before either mechanical stop.
	if ((*motor_command_ptr < 0)
		&& (position < (positionLimitLeft + POSITION_LIMIT_BRAKE_MARGIN_COUNTS)))
	{
		*motor_command_ptr = 0;
	}
	else if ((*motor_command_ptr > 0)
		&& (position > (positionLimitRight - POSITION_LIMIT_BRAKE_MARGIN_COUNTS)))
	{
		*motor_command_ptr = 0;
	}
}

void motor_stall_safety_reset(void)
{
	stall_guard_initialized = 0;
	stall_guard_latched = 0;
	stall_guard_last_position = 0;
	stall_guard_stationary_ms = 0u;
}

int motor_stall_safety_check(int* motor_command_ptr, int position, unsigned short period_ms)
{
	if (!motor_command_ptr) return 1;
	if (!stall_guard_initialized) {
		stall_guard_initialized = 1;
		stall_guard_last_position = position;
	}
	if (stall_guard_latched) {
		*motor_command_ptr = 0;
		return 1;
	}

	/* A substantial command must produce encoder motion. This catches a dead
	 * encoder, disconnected drive, or a cart pushing against a hard stop. */
	if (abs(*motor_command_ptr) < MOTOR_FULL_SCALE_SAFE / 4) {
		stall_guard_stationary_ms = 0u;
		stall_guard_last_position = position;
		return 0;
	}
	if (abs(position - stall_guard_last_position) >= MOTOR_STALL_MIN_TRAVEL_COUNTS) {
		stall_guard_stationary_ms = 0u;
	} else {
		unsigned int next = stall_guard_stationary_ms + (unsigned int)period_ms;
		stall_guard_stationary_ms = next < stall_guard_stationary_ms ? UINT_MAX : next;
	}
	stall_guard_last_position = position;

	if (stall_guard_stationary_ms >= MOTOR_STALL_TIMEOUT_MS) {
		stall_guard_latched = 1;
		*motor_command_ptr = 0;
		return 1;
	}
	return 0;
}
