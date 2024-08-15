#include "hardware_bridge.h"
#include "parameters.h"
#include "control.h"
#include "angle_processing.h"
#include "communication_with_PC.h"
#include <stdlib.h>
#include "math.h"
#include "hardware_pid.h"
#include "control_signal_postprocessing.h"
#include "experiment_protocol.h"
#include "offline_data_manager.h"

#define OnChipController_PID 0
#define OnChipController_NeuralImitator 1
#define OnChipController_PID_position 2

// The 3 variables below only matter on Zynq
#define	CONTROLLERS_SWITCH_NUMBER		0
#define	POSITION_JUMPS_SWITCH_NUMBER	1
#define	EQUILIBRIUM_SWITCH_NUMBER		2

unsigned short current_controller = OnChipController_NeuralImitator;

bool correct_motor_dynamics = true;


bool            streamEnable        = false;
bool 			interrupt_occurred	= false;

float			ANGLE_HANGING;
float  			ANGLE_DEVIATION;

volatile bool	HardwareConfigSetFromPC;
volatile bool 	ControlOnChip_Enabled;

int 			motor_command 		= 0;

bool            isCalibrated 		= true;
unsigned short  ledPeriod;

int				positionCentre;
int				positionLimitLeft;
int				positionLimitRight;
unsigned long	time_measurement_done = 0, time_motor_command_obtained = 0, latency = 0;

unsigned long 	time_current_measurement = 0;
unsigned long	time_last_measurement = 0;

bool			new_motor_command_obtained			= true;

unsigned short 	angleSampIndex		= 0;
int *angleSamples;

short 			position_short;

unsigned short	latency_violation = 0;

static unsigned char rxBuffer[SERIAL_MAX_PKT_LENGTH];
static unsigned char txBuffer[200];

void 			cmd_Ping(const unsigned char * buff, unsigned int len);
void            cmd_StreamOutput(bool en);
void 			cmd_ControlMode(bool en);
void			cmd_SetControlConfig(const unsigned char * config);
void 			cmd_GetControlConfig(void);
void			cmd_CollectRawAngle(const unsigned short, const unsigned short);
void			cmd_RunHardwareExperiment(void);
void 			cmd_transfer_buffers(void);

float angle_deviation_update(float new_angle_hanging);
float angle_deviation_update(float new_angle_hanging){
	float angle_deviation;
    // update angle deviation according to ANGLE_HANGING update
    if (new_angle_hanging < ANGLE_360_DEG_IN_ADC_UNITS / 2){
        angle_deviation = - new_angle_hanging - ANGLE_360_DEG_IN_ADC_UNITS / 2.0;  // moves upright to 0 and hanging to -pi
    } else {
        angle_deviation = - new_angle_hanging + ANGLE_360_DEG_IN_ADC_UNITS / 2.0;  // moves upright to 0 and hanging to pi
    }
    return angle_deviation;
}

void CONTROL_Init(void)
{
	ControlOnChip_Enabled		= false;
	HardwareConfigSetFromPC = false;
    isCalibrated        = false;
    ledPeriod           = 500/CONTROL_LOOP_PERIOD_MS;

    positionCentre      = (short)Encoder_Read(); // assume starting position is near center
    positionLimitLeft   = positionCentre - 2400;
    positionLimitRight  = positionCentre + 2400; // guess defaults based on 7000-8000 counts at limits

    if (MOTOR == MOTOR_ORIGINAL){
    	ANGLE_HANGING = ANGLE_HANGING_ORIGINAL;
    } else if (MOTOR == MOTOR_POLOLU){
    	ANGLE_HANGING = ANGLE_HANGING_POLOLU;
    }

    ANGLE_DEVIATION = angle_deviation_update(ANGLE_HANGING);

    angleSamples = malloc(ANGLE_AVERAGE_LEN_MAX * sizeof(int));
    if (angleSamples == NULL) {
        // Handle memory allocation failure
        exit(1); // or another appropriate error handling
    }

#ifdef USE_EXTERNAL_INTERFACE
    USE_TARGET_SWITCHES = false;
#endif

    correct_motor_dynamics = (current_controller == OnChipController_PID) ? false : true;
}

void CONTROL_ToggleState(void)
{
	cmd_ControlMode(!ControlOnChip_Enabled);
}

bool calibrate = false;
void CONTROL_ToggleCalibration(void)
{
    disable_irq();
    calibrate = true;
	enable_irq();
}


int clip(int value, int min, int max) {
	if (value > max)
		return max;
	if (value < min)
		return min;
	return value;
}



// Called from Timer interrupt every CONTROL_LOOP_PERIOD_MS ms
void CONTROL_Loop(void)
{

	interrupt_occurred = true;

}
float Q;
float target_equilibrium = 1.0;
float target_position = 0.0;

// The 4 variable below only matter on Zynq if USE_TARGET_SWITCHES==TRUE
int position_period  = 1000;  // In a unit of control updates
float position_jumps_target = 0.09;
int position_jumps_enabled = 0;
int position_jumps_interval_counter = 0;

int run_hardware_experiment = 0;
int save_to_offline_buffers = 0;

float time = 0.0;

float angle = 0.0;
float position = 0.0;
float angleD = 0.0;
float angleD_unprocessed = 0.0;
float positionD = 0.0;

void CONTROL_BackgroundTask(void)
{

	if(interrupt_occurred)
	{
		interrupt_occurred = false;

		if(CONTROL_SYNC) {
			Motor_SetPower(motor_command, MOTOR_PWM_PERIOD_IN_CLOCK_CYCLES);
		}

		static unsigned char	buffer[30];

		static unsigned short 	ledPeriodCnt	= 0;
		static bool				ledState 		= false;

		int   					motor_command_from_chip;

		int angle_int = 0;
		int angle_raw_int = 0;
		int invalid_step = 0;



		time_last_measurement = time_current_measurement;
		time_current_measurement = GetTimeNow();

		position_short = Encoder_Read();
		position_short = position_short - positionCentre;
		process_angle(angleSamples, angleSampIndex, ANGLE_AVERAGE_LEN, &angle_int, &angle_raw_int, &angleD, &invalid_step);
        angleD_unprocessed = angleD;

		unsigned long time_difference_between_measurement = time_current_measurement-time_last_measurement;

		calculate_position_difference_per_timestep(&position_short, &positionD);

	    average_derivatives(&angleD, &positionD);

	    float angle_cos, angle_sin;

	    float time_difference_between_measurement_s = time_difference_between_measurement/1000000.0;
		angle = wrapLocal_rad(((angle_int) + ANGLE_DEVIATION) * (ANGLE_NORMALIZATION_FACTOR));
	    position = position_short * POSITION_NORMALIZATION_FACTOR;

	    angle_cos = cos(angle);
	    angle_sin = sin(angle);
	    angleD = (angleD*(ANGLE_NORMALIZATION_FACTOR)/time_difference_between_measurement_s);
	    positionD = (positionD*POSITION_NORMALIZATION_FACTOR/time_difference_between_measurement_s);

        time = time_current_measurement/1000000.0;

		// Microcontroller Control Routine
		if (ControlOnChip_Enabled)	{

#ifdef ZYNQ
#ifndef USE_EXTERNAL_INTERFACE
			if(USE_TARGET_SWITCHES && position_jumps_enabled){

				if (position_jumps_interval_counter >= position_period)
				{
					position_jumps_target = -position_jumps_target;
					target_position = position_jumps_target;
					position_jumps_interval_counter = 0;
				} else {
					++position_jumps_interval_counter;
				}


			}
#endif
#endif
			switch (current_controller){
			case OnChipController_PID:
			{
				Q = pid_step(angle, angleD, position, positionD, target_position, time);
				break;
			}
			case OnChipController_NeuralImitator:
			{
				Q = neural_imitator_cartpole_step(angle, angleD, angle_cos, angle_sin, position, positionD, target_equilibrium, target_position, time);
				break;
			}
			case OnChipController_PID_position:
			{
				Q = pid_position_step(angle, angleD, position, positionD, target_position, time);
				break;
			}
			default:
			{
				Q = 0.0;
				break;
			}

			}

	        motor_command_from_chip = control_signal_to_motor_command(Q, positionD, correct_motor_dynamics);
	        motor_command_safety_check(&motor_command_from_chip);
	        safety_switch_off(&motor_command_from_chip, positionLimitLeft, positionLimitRight);

			time_motor_command_obtained = GetTimeNow();
			new_motor_command_obtained = true;
            time_measurement_done = time_current_measurement;

            motor_command = motor_command_from_chip;

	        if(!CONTROL_SYNC)
	        {
	        	Motor_SetPower(motor_command, MOTOR_PWM_PERIOD_IN_CLOCK_CYCLES);
	        }
		}

        HardwareExperimentProtocol(position, angle, time,
        		&target_position, &target_equilibrium,
				&run_hardware_experiment, &save_to_offline_buffers,
				&ControlOnChip_Enabled, &motor_command, &USE_TARGET_SWITCHES);

        if(save_to_offline_buffers){
            fill_data_buffers(
                    time,
                    angle,
                    position,
                    angleD,
                    positionD,
                    target_equilibrium,
                    target_position,
                    Q
            );
        }

		if (run_hardware_experiment==2){
			unsigned short experiment_length = get_buffers_index();
		    send_information_experiment_done(buffer, experiment_length);
		    Message_SendToPC(buffer, 6);
		    run_hardware_experiment=0;
		}


		// Send latest state to the PC
		static int slowdown = 0;
	    if (streamEnable && ++slowdown>=CONTROL_SLOWDOWN && run_hardware_experiment==0)
	    {
	        slowdown = 0;

	    	if(time_motor_command_obtained > 0 && time_measurement_done > 0 && new_motor_command_obtained) {
	        	latency = time_motor_command_obtained - time_measurement_done;
	        	latency_violation = 0;
	    	} else{
	    		latency_violation = 1;
	    		latency = CONTROL_LOOP_PERIOD_MS*1000;
	    	}

	    	prepare_message_to_PC_state(
	    			buffer,
					31,
					angle_int,
					angleD_unprocessed,
					position_short,
					target_position,
					motor_command,
					invalid_step,
					time_difference_between_measurement,
					time_current_measurement,
					latency,
					latency_violation
					);

	    	Message_SendToPC(buffer, 31);

	        if(new_motor_command_obtained) {
	        	time_measurement_done = time_current_measurement;
	        	time_motor_command_obtained = 0;
	        	new_motor_command_obtained = false;
	        }
	    }

		// Flash LED every second (500 ms on, 500 ms off)
		ledPeriodCnt++;
		if (ledPeriodCnt >= ledPeriod)
		{
			ledPeriodCnt	= 0;
			ledState 		= !ledState;
			Led_Switch(ledState);
		}

		interrupt_occurred = false;

	}

	static unsigned int 	uart_received_Cnt			= 0;
	static unsigned long    lastRead = 0;

	///////////////////////////////////////////////////
	// Collect samples of angular displacement
	///////////////////////////////////////////////////
	unsigned long now = GetTimeNow();

	// int-overflow after 1h
	if (now < lastRead) {
		lastRead = now;
	}
	// read every ca. 100us
	else if (now > lastRead + ANGLE_MEASUREMENT_INTERVAL_US) {
		// conversion takes 18us
		angleSamples[angleSampIndex] = Goniometer_Read();
		angleSampIndex = (++angleSampIndex >= ANGLE_AVERAGE_LEN ? 0 : angleSampIndex);

		lastRead = now;
	}

#ifdef ZYNQ
	if(Switch_GetState(CONTROLLERS_SWITCH_NUMBER)){
		current_controller = OnChipController_PID;
	} else{
		current_controller = OnChipController_NeuralImitator;
	}

#ifndef USE_EXTERNAL_INTERFACE
	if (USE_TARGET_SWITCHES)
	{
		if(Switch_GetState(POSITION_JUMPS_SWITCH_NUMBER)){
			position_jumps_enabled = 1;
		} else{
			target_position = 0.0;
			position_jumps_interval_counter = 0;
			position_jumps_enabled = 0;
		}

		if(Switch_GetState(EQUILIBRIUM_SWITCH_NUMBER)){
			target_equilibrium = 1.0;
		} else{
			target_equilibrium = -1.0;
		}
	}
#else

	if(Switch_GetState(EQUILIBRIUM_SWITCH_NUMBER)){ // Reuse the switch to enable position PID
			current_controller = OnChipController_PID_position;
		}

	target_position = get_normed_slider_state()*2*position_jumps_target;

	int target_equilibrium_from_external_button = get_target_equilibrium_from_external_button();
	if (target_equilibrium_from_external_button != 0){
		target_equilibrium = target_equilibrium_from_external_button;
	}
#endif
	Leds_over_switches_Update(Switches_GetState());
	indicate_target_position_with_leds(&target_position);
#endif

	///////////////////////////////////////////////////
	// Process Commands from PC
	///////////////////////////////////////////////////
	if(run_hardware_experiment==0)
	{
	    int newDataCount = Message_GetFromPC(&rxBuffer[uart_received_Cnt]);
        uart_received_Cnt += newDataCount;
	} else
	{
	    uart_received_Cnt = 0;
	 }


	int current_command = get_command_from_PC_message(rxBuffer, &uart_received_Cnt);

	switch (current_command){
		case CMD_PING:
		{
			unsigned int pktLen = rxBuffer[2];
			cmd_Ping(rxBuffer, pktLen);
			break;
		}
		case CMD_STREAM_ON:
		{
			cmd_StreamOutput(rxBuffer[3] != 0);
			break;
		}
		case CMD_CALIBRATE:
		{
			calibrate=true;
			break;
		}

		case CMD_RUN_HARDWARE_EXPERIMENT:
        {
        	
            cmd_RunHardwareExperiment();
            break;
        }

        case CMD_TRANSFER_BUFFERS:
        {
            cmd_transfer_buffers();
            break;
        }

		case CMD_CONTROL_MODE:
		{
			cmd_ControlMode(rxBuffer[3] != 0);
			break;
		}
		case CMD_SET_PID_CONFIG:
		{
			cmd_SetPIDConfig(&rxBuffer[3]);
			break;
		}
		case CMD_GET_PID_CONFIG:
		{
			cmd_GetPIDConfig(txBuffer);
			break;
		}
		case CMD_SET_CONTROL_CONFIG:
		{
			cmd_SetControlConfig(&rxBuffer[3]);
			break;
		}
		case CMD_GET_CONTROL_CONFIG:
		{
			cmd_GetControlConfig();
			break;
		}
		case CMD_SET_MOTOR:
		{
			int motor_command_from_PC = *((int *)&rxBuffer[3]);
			safety_switch_off(&motor_command_from_PC, positionLimitLeft, positionLimitRight);

			time_motor_command_obtained = GetTimeNow();
            if(new_motor_command_obtained){
                time_measurement_done = time_current_measurement;
            }
			new_motor_command_obtained = true;

			motor_command = motor_command_from_PC;

			if(!CONTROL_SYNC)
			{
				Motor_SetPower(motor_command_from_PC, MOTOR_PWM_PERIOD_IN_CLOCK_CYCLES);
			}
			break;
		}
		case CMD_SET_TARGET_POSITION:
		{
			target_position = *((float *)&rxBuffer[3]);
#ifdef ZYNQ
			USE_TARGET_SWITCHES = false;
#endif
			break;
		}
		case CMD_SET_TARGET_EQUILIBRIUM:
		{
			target_equilibrium = *((float *)&rxBuffer[3]);
			break;
		}
		case CMD_COLLECT_RAW_ANGLE:
		{
			unsigned short length 	   = 256 * (unsigned short)rxBuffer[4] + (unsigned short)rxBuffer[3];
			unsigned short interval_us = 256 * (unsigned short)rxBuffer[6] + (unsigned short)rxBuffer[5];
			cmd_CollectRawAngle(length, interval_us);
			break;
		}
		default:
		{
			break;
		}
	}

	if (calibrate){
		cmd_Calibrate();
	}

}

void cmd_Ping(const unsigned char * buff, unsigned int len)
{
	disable_irq();
	Message_SendToPC(buff, len);
	enable_irq();
}

void cmd_StreamOutput(bool en)
{
	disable_irq();
	streamEnable = en;
	enable_irq();
}

void cmd_RunHardwareExperiment(void)
{
	streamEnable = false;
    run_hardware_experiment = 1;
}

void cmd_transfer_buffers(void)
{
    send_buffers();
}

void cmd_Calibrate(void)
{
	unsigned short SPEED_CALIBRATION = (float)(MOTOR_PWM_PERIOD_IN_CLOCK_CYCLES) * 0.3;
	int pos;
	int diff;
	float fDiff;
	static unsigned char	buffer[30];
	short encoderDirection	= 1;

	Encoder_Set_Direction(encoderDirection);

	disable_irq();
	Motor_Stop();
	Led_Switch(true);

	// Get left limit
	Sleep_ms(100);
	positionLimitRight = Encoder_Read();
	Motor_SetPower(SPEED_CALIBRATION, MOTOR_PWM_PERIOD_IN_CLOCK_CYCLES);

	do {
		Sleep_ms(100);
		pos  = Encoder_Read();
		diff = pos - positionLimitRight;
		positionLimitRight = pos;

		// if we don't move enough, must have hit limit
	} while(abs(diff) > 15);

	Motor_Stop();
	Led_Switch(false);

	// Get right limit
	Sleep_ms(100);
	positionLimitLeft = Encoder_Read();
	Motor_SetPower(-SPEED_CALIBRATION, MOTOR_PWM_PERIOD_IN_CLOCK_CYCLES);

	do {
		Sleep_ms(100);
		pos  = Encoder_Read();
		diff = pos - positionLimitLeft;
		positionLimitLeft = pos;

		// if we don't move enough, must have hit limit
	} while(abs(diff) > 15);

	Motor_Stop();

	// Move pendulum to the centre (roughly)
	Led_Switch(true);
	Sleep_ms(200);
	// invert reading for original motor
	if (positionLimitLeft > positionLimitRight) {
		positionLimitLeft *= -1;
		positionLimitRight *= -1;
		encoderDirection = -1;
		Encoder_Set_Direction(encoderDirection);
	} else {
		encoderDirection = 1;
	}

	positionCentre = (positionLimitLeft + positionLimitRight) / 2;			// average limits

	// Slower to get back to middle
	Motor_SetPower(SPEED_CALIBRATION, MOTOR_PWM_PERIOD_IN_CLOCK_CYCLES);
	do {
		fDiff = 2.0 * abs(Encoder_Read() - positionCentre) / abs(positionLimitRight - positionLimitLeft);
		// Slow Down even more to get more accurately to the middle
		if(fDiff < 1e-1) {
			Motor_SetPower(SPEED_CALIBRATION/2, MOTOR_PWM_PERIOD_IN_CLOCK_CYCLES);
		}
	} while(fDiff > 5e-4);
	Motor_Stop();

	if(!HardwareConfigSetFromPC)
	{
		MOTOR = encoderDirection==1 ? MOTOR_POLOLU : MOTOR_ORIGINAL;
		ANGLE_HANGING = MOTOR==MOTOR_POLOLU ? ANGLE_HANGING_POLOLU : ANGLE_HANGING_ORIGINAL;

		ANGLE_DEVIATION = angle_deviation_update(ANGLE_HANGING);
	}

	Sleep_ms(100);
	prepare_message_to_PC_calibration(buffer, encoderDirection);
    Message_SendToPC(buffer, 5);

	isCalibrated = true;
	calibrate = false;
	Led_Switch(false);
	enable_irq();
}

void cmd_ControlMode(bool en)
{
    disable_irq();
	if (en && !ControlOnChip_Enabled)
	{
        ledPeriod           = 100/CONTROL_LOOP_PERIOD_MS;
	}
	else if (!en && ControlOnChip_Enabled)
	{
		Motor_Stop();
		motor_command = 0;
        ledPeriod           = 500/CONTROL_LOOP_PERIOD_MS;
	}

	ControlOnChip_Enabled = en;
	enable_irq();
}


void cmd_SetControlConfig(const unsigned char * config)
{
	disable_irq();

	CONTROL_LOOP_PERIOD_MS = *((unsigned short *)&config[0]);
    CONTROL_SYNC			= *((bool	        *)&config[2]);
    ANGLE_HANGING      = *((float          *)&config[ 3]);
    ANGLE_AVERAGE_LEN    = *((unsigned short *)&config[ 7]);
    correct_motor_dynamics = *((bool	        *)&config[9]);

    SetControlUpdatePeriod(CONTROL_LOOP_PERIOD_MS);
    ANGLE_DEVIATION = angle_deviation_update(ANGLE_HANGING);

    HardwareConfigSetFromPC = true;

	enable_irq();
}


void cmd_GetControlConfig(void)
{
	prepare_message_to_PC_control_config(txBuffer, CONTROL_LOOP_PERIOD_MS, CONTROL_SYNC, ANGLE_HANGING, ANGLE_AVERAGE_LEN, correct_motor_dynamics);

	disable_irq();
	Message_SendToPC(txBuffer, 16);
	enable_irq();
}


void cmd_CollectRawAngle(unsigned short MEASURE_LENGTH, unsigned short INTERVAL_US)
{

	Interrupt_Unset();
	Motor_Stop();
	Led_Switch(true);

	txBuffer[ 0] = SERIAL_SOF;
	txBuffer[ 1] = CMD_COLLECT_RAW_ANGLE;
	txBuffer[ 2] = 4 + 2*MEASURE_LENGTH;

	unsigned int now = 0, lastRead = 0;

	unsigned int i;
	for(i=0; i<MEASURE_LENGTH;) {
		Led_Switch(i % 2);
		now = GetTimeNow();

		// int-overflow after 1h
		if (now < lastRead) {
			lastRead = now;
		}
		// read every ca. 100us
		else if (now > lastRead + INTERVAL_US) {
			// conversion takes 18us
			*((unsigned short *)&txBuffer[ 3 + 2*i]) = Goniometer_Read();
			lastRead = now;
			i++;
		}
	}
	Led_Switch(true);

	//txBuffer[3 + 2*MEASURE_LENGTH] = crc(txBuffer, 3 + 2*MEASURE_LENGTH);

	disable_irq();
	Message_SendToPC(txBuffer, 4 + 2*MEASURE_LENGTH);
	Interrupt_Set(CONTROL_Loop);
	enable_irq();
}
