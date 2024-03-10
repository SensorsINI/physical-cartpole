#include <stdio.h>
#include <stdlib.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <stdbool.h>
#include "parameters.h"
#include "hardware_bridge.h"

#define TARGET_POSITION_0            0.0
#define TARGET_POSITION_SWING_UP     0.0
#define TARGET_POSITION_1            0.09
#define TARGET_POSITION_2            -0.09

#define TIME_FOR_SWINGUP             0.01
#define TIME_FOR_TARGET_1            4.02
#define TIME_OF_EXPERIMENT           10.03

#define UPDATE_INTERVAL              2

int current_experiment_phase = -1;
float time_start_stable_down;

bool ControlOnChip_Enabled_var_original;
bool USE_TARGET_SWITCHES_var_original;

// Enum for experiment phases
typedef enum {
    RESET,
    SWINGUP,
    GO_TO_TARGET_1,
    GO_TO_TARGET_2
} ExperimentPhase;

void action_reset(float position, float angle, float time, float* target_position, float* target_equilibrium);
void action_swing_up(float time, float* target_position);
void action_go_to_target_1(float time, float* target_position);
void action_go_to_target_2(
		float time,
		int* run_hardware_experiment, int* save_to_offline_buffers,
		bool* ControlOnChip_Enabled_var, int* motor_command, bool* USE_TARGET_SWITCHES_var);

void HardwareExperimentProtocol(
		float position, float angle, float time,
		float* target_position, float* target_equilibrium,
		int* run_hardware_experiment, int* save_to_offline_buffers,
		bool* ControlOnChip_Enabled_var, int* motor_command, bool* USE_TARGET_SWITCHES_var){

    if(*run_hardware_experiment == 1){
        if(current_experiment_phase == -1){
            ControlOnChip_Enabled_var_original = *ControlOnChip_Enabled_var;
            USE_TARGET_SWITCHES_var_original = *USE_TARGET_SWITCHES_var;
            SetControlUpdatePeriod(UPDATE_INTERVAL);
            *ControlOnChip_Enabled_var = true;
            *USE_TARGET_SWITCHES_var = false;

            current_experiment_phase = RESET;
        }
        switch(current_experiment_phase) {
            case RESET:
                action_reset(position, angle, time, target_position, target_equilibrium);
                break;
            case SWINGUP:
            	*save_to_offline_buffers = 1;
                action_swing_up(time, target_position);
                break;
            case GO_TO_TARGET_1:
                action_go_to_target_1(time, target_position);
                break;
            case GO_TO_TARGET_2:
                action_go_to_target_2(
                		time,
                		run_hardware_experiment, save_to_offline_buffers,
						ControlOnChip_Enabled_var, motor_command, USE_TARGET_SWITCHES_var);
                break;
            default:
                printf("Unknown experiment phase: %d\n", current_experiment_phase);
                exit(1);
        }
    }
}


void action_reset(float position, float angle, float time, float* target_position, float* target_equilibrium) {
    *target_position = TARGET_POSITION_0;
    *target_equilibrium = -1.0;
    if (fabs(position - *target_position) < 0.01 && fabs(angle) > M_PI - 0.1) {
        *target_position = TARGET_POSITION_1;
        *target_equilibrium = 1.0;
        time_start_stable_down = time;
        current_experiment_phase = SWINGUP;
    }
}

void action_swing_up(float time, float* target_position) {
    if (time - time_start_stable_down >= TIME_FOR_SWINGUP) {
        *target_position = TARGET_POSITION_1;
        current_experiment_phase = GO_TO_TARGET_1;
    }
}

void action_go_to_target_1(float time, float* target_position) {
    if (time - time_start_stable_down > TIME_FOR_TARGET_1) {
        *target_position = TARGET_POSITION_2;
        current_experiment_phase = GO_TO_TARGET_2;
    }
}

void action_go_to_target_2(
		float time,
		int* run_hardware_experiment, int* save_to_offline_buffers,
		bool* ControlOnChip_Enabled_var, int* motor_command, bool* USE_TARGET_SWITCHES_var) {
    if (time - time_start_stable_down > TIME_OF_EXPERIMENT) {
        current_experiment_phase = -1;
        *save_to_offline_buffers = 0;
        *ControlOnChip_Enabled_var = ControlOnChip_Enabled_var_original;
        if (ControlOnChip_Enabled_var_original)
        *motor_command = 0;
        *USE_TARGET_SWITCHES_var = USE_TARGET_SWITCHES_var_original;
        SetControlUpdatePeriod(CONTROL_LOOP_PERIOD_MS);
        *run_hardware_experiment = 2;
    }
}
