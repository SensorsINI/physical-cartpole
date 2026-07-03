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
#include "controller_api_helper.h"
#include "neural_controller_C.h"
#include "lqr.h"
#include "secloc_lqr.h"


#define OnChipController_PID 0
#define OnChipController_NeuralImitator 1
#define OnChipController_PID_position 2
#define OnChipController_LQR 3
#define OnChipController_neural_controller_C 4
#define OnChipController_SECLOC_LQR 5


// The 3 variables below only matter on Zynq
#define	CONTROLLERS_SWITCH_NUMBER		0
#define	POSITION_JUMPS_SWITCH_NUMBER	1
#define	EQUILIBRIUM_SWITCH_NUMBER		2

unsigned short current_controller = OnChipController_neural_controller_C;

bool correct_motor_dynamics = true;


bool            streamEnable        = false;
bool 			interrupt_occurred	= false;

float			ANGLE_HANGING;
float  			ANGLE_DEVIATION;

volatile bool	HardwareConfigSetFromPC;
volatile bool 	ControlOnChip_Enabled;
volatile bool	PCControl_Enabled;

int 			motor_command 		= 0;

bool            isCalibrated 		= true;
unsigned short  ledPeriod;

int				positionCentre;
int				positionLimitLeft;
int				positionLimitRight;
unsigned long	time_measurement_done = 0, time_motor_command_obtained = 0, latency = 0;

unsigned long 	time_current_measurement = 0;
unsigned long	time_last_measurement = 0;
// Monotonic firmware-owned experiment time; avoids exposing raw 32-bit timer wrap to the PC.
unsigned long long time_accumulated_us = 0;

bool			new_motor_command_obtained			= true;

unsigned short 	angleSampIndex		= 0;
int *angleSamples;

short 			position_short;

unsigned short	latency_violation = 0;

static unsigned char rxBuffer[SERIAL_MAX_PKT_LENGTH];
static unsigned char txBuffer[200];

// Global state variables for controller binding
float time = 0.0f;
float angle = 0.0f;
float angleD = 0.0f;
float angle_cos = 0.0f;
float angle_sin = 0.0f;
float position = 0.0f;
float positionD = 0.0f;
float target_equilibrium = 1.0f;
float target_position = 0.0f;
float angleD_unprocessed = 0.0f;

static SignalEntry g_signals[] = {
    { "angle",              &angle },
    { "angleD",             &angleD },
    { "angle_cos",          &angle_cos },
    { "angle_sin",          &angle_sin },
    { "position",           &position },
    { "positionD",          &positionD },
    { "target_equilibrium", &target_equilibrium },
    { "target_position",    &target_position },
    { "time",               &time }    /* simulated/MCU time in seconds */
};
#define G_SIGNALS_LEN  (sizeof(g_signals)/sizeof(g_signals[0]))

static ControllerBinding g_cb;

void 			cmd_Ping(const unsigned char * buff, unsigned int len);
void            cmd_StreamOutput(bool en);
void 			cmd_ControlMode(bool en);
void			cmd_PCControlMode(bool en);
void			cmd_SetControlConfig(const unsigned char * config);
void 			cmd_GetControlConfig(void);
void			cmd_CollectRawAngle(const unsigned short, const unsigned short);
void			cmd_RunHardwareExperiment(void);
void 			cmd_transfer_buffers(void);
void			CONTROL_CalibrationStep(void);

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
	PCControl_Enabled			= false;
	HardwareConfigSetFromPC = false;
    isCalibrated        = false;
    ledPeriod           = 500/POLLING_PERIOD_MS;

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

    CB_Init(&g_cb);
}

void CONTROL_ToggleState(void)
{
	cmd_ControlMode(!ControlOnChip_Enabled);
}

typedef enum {
	CalibrationState_Idle,
	CalibrationState_DriveToWall,
	CalibrationState_ReverseDetect,
	CalibrationState_DriveToCenter,
	CalibrationState_SettleAtCenter,
	CalibrationState_Finished
} CalibrationState;

bool calibrate = false;
static CalibrationState calibrationState = CalibrationState_Idle;
static int calibrationTicks = 0;
static int calibrationLastPosition = 0;
static int calibrationLowMotionSamples = 0;
static short calibrationEncoderDirection = 1;
static bool calibrationFailed = false;

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



// Called from Timer interrupt every POLLING_PERIOD_MS ms
void CONTROL_Loop(void)
{

	interrupt_occurred = true;

}
float Q;

// The 4 variable below only matter on Zynq if USE_TARGET_SWITCHES==TRUE
int position_period  = 1000;  // In a unit of control updates
float position_jumps_target = 0.09;
int position_jumps_enabled = 0;
int position_jumps_interval_counter = 0;

int run_hardware_experiment = 0;
int save_to_offline_buffers = 0;


void CONTROL_BackgroundTask(void)
{

	if(interrupt_occurred)
	{
		interrupt_occurred = false;

		if(CONTROL_SYNC) {
			Motor_SetPower(motor_command, MOTOR_PWM_PERIOD_IN_CLOCK_CYCLES);
		}

		static unsigned char	buffer[STATE_MESSAGE_LEN];

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
		if (time_difference_between_measurement == 0) {
			// Should not happen in normal operation; use the configured loop period to avoid a zero-division sample.
			time_difference_between_measurement = POLLING_PERIOD_MS * 1000;
		}
		time_accumulated_us += (unsigned long long)time_difference_between_measurement;

		calculate_position_difference_per_timestep(&position_short, &positionD);

	    average_derivatives(&angleD, &positionD);

	    float time_difference_between_measurement_s = time_difference_between_measurement/1000000.0;
		angle = wrapLocal_rad(((angle_int) + ANGLE_DEVIATION) * (ANGLE_NORMALIZATION_FACTOR));
	    position = position_short * POSITION_NORMALIZATION_FACTOR;

	    angle_cos = cos(angle);
	    angle_sin = sin(angle);
	    angleD = (angleD*(ANGLE_NORMALIZATION_FACTOR)/time_difference_between_measurement_s);
	    positionD = (positionD*POSITION_NORMALIZATION_FACTOR/time_difference_between_measurement_s);

        time = time_accumulated_us/1000000.0;

		if (calibrate) {
			CONTROL_CalibrationStep();
		} else {
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
	                CB_RebindOnChange(&g_cb, &PID_Ops, g_signals, (uint8_t)G_SIGNALS_LEN);
	                Q = CB_Eval(&g_cb);
					break;
				}
				case OnChipController_NeuralImitator:
				{
	                CB_RebindOnChange(&g_cb, &NeuralImitator_Ops, g_signals, (uint8_t)G_SIGNALS_LEN);
	                Q = CB_Eval(&g_cb);
					break;
				}
				case OnChipController_PID_position:
				{
	                CB_RebindOnChange(&g_cb, &PIDPos_Ops, g_signals, (uint8_t)G_SIGNALS_LEN);
	                Q = CB_Eval(&g_cb);
					break;
				}
				case OnChipController_LQR:
	            {
	                CB_RebindOnChange(&g_cb, &LQR_Ops, g_signals, (uint8_t)G_SIGNALS_LEN);
	                Q = CB_Eval(&g_cb);
	                break;
	            }
	            case OnChipController_neural_controller_C:
	            {
	                CB_RebindOnChange(&g_cb, &NNC_Ops, g_signals, (uint8_t)G_SIGNALS_LEN);
	                Q = CB_Eval(&g_cb);
	                break;
	            }
	            case OnChipController_SECLOC_LQR:
	            {
	                CB_RebindOnChange(&g_cb, &SECLOC_LQR_Ops, g_signals, (uint8_t)G_SIGNALS_LEN);
	                Q = CB_Eval(&g_cb);
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
		}

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
			} else if (PCControl_Enabled) {
				latency_violation = 1;
				latency = POLLING_PERIOD_MS*1000;
			} else {
				latency_violation = 0;
				latency = 0;
			}

	    	prepare_message_to_PC_state(
	    			buffer,
					STATE_MESSAGE_LEN,
					angle_int,
					angleD_unprocessed,
					position_short,
					target_position,
					motor_command,
					invalid_step,
					time_difference_between_measurement,
					time_accumulated_us,
					latency,
					latency_violation
					);

	    	Message_SendToPC(buffer, STATE_MESSAGE_LEN);

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
#ifdef USE_EXTERNAL_INTERFACE
	current_controller = OnChipController_neural_controller_C;

	target_position = get_normed_slider_state()*2*position_jumps_target;

	int target_equilibrium_from_external_button = get_target_equilibrium_from_external_button();
	if (target_equilibrium_from_external_button != 0){
		target_equilibrium = target_equilibrium_from_external_button;
	}
#else
	current_controller = OnChipController_neural_controller_C;

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
			cmd_Calibrate();
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
		case CMD_PC_CONTROL_MODE:
		{
			cmd_PCControlMode(rxBuffer[3] != 0);
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

	if (calibrate && calibrationState == CalibrationState_Idle){
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

	if (calibrationState != CalibrationState_Idle) {
		return;
	}

	ControlOnChip_Enabled = false;
	PCControl_Enabled = false;
	isCalibrated = false;
	calibrate = true;
	calibrationState = CalibrationState_DriveToWall;
	calibrationTicks = 0;
	calibrationLowMotionSamples = 0;
	calibrationEncoderDirection = 1;
	calibrationFailed = false;

	Encoder_Set_Direction(calibrationEncoderDirection);
	calibrationLastPosition = Encoder_Read();
	motor_command = SPEED_CALIBRATION;
	Motor_SetPower(motor_command, MOTOR_PWM_PERIOD_IN_CLOCK_CYCLES);
	Led_Switch(true);
}

void CONTROL_CalibrationStep(void)
{
	const unsigned short SPEED_CALIBRATION = (float)(MOTOR_PWM_PERIOD_IN_CLOCK_CYCLES) * 0.3;
	const int wallMotionThreshold = 15;
	static unsigned char	buffer[30];

	int sampleTicks = 100 / POLLING_PERIOD_MS;
	int initialDriveTicks = 500 / POLLING_PERIOD_MS;
	int reverseDetectTimeoutTicks = 5000 / POLLING_PERIOD_MS;
	int centerTimeoutTicks = 15000 / POLLING_PERIOD_MS;
	int centerSettleTicks = 300 / POLLING_PERIOD_MS;
	int wallTimeoutTicks = 15000 / POLLING_PERIOD_MS;

	if (sampleTicks < 1) sampleTicks = 1;
	if (initialDriveTicks < 1) initialDriveTicks = 1;
	if (reverseDetectTimeoutTicks < 1) reverseDetectTimeoutTicks = 1;
	if (centerTimeoutTicks < 1) centerTimeoutTicks = 1;
	if (centerSettleTicks < 1) centerSettleTicks = 1;
	if (wallTimeoutTicks < 1) wallTimeoutTicks = 1;

	calibrationTicks++;

	switch (calibrationState) {
	case CalibrationState_DriveToWall:
		motor_command = SPEED_CALIBRATION;

		if (calibrationTicks >= initialDriveTicks && calibrationTicks % sampleTicks == 0) {
			int currentPosition = Encoder_Read();
			int diff = currentPosition - calibrationLastPosition;
			calibrationLastPosition = currentPosition;

			if (abs(diff) <= wallMotionThreshold) {
				calibrationLowMotionSamples++;
			} else {
				calibrationLowMotionSamples = 0;
			}

			if (calibrationLowMotionSamples >= 2) {
				Motor_Stop();
				motor_command = 0;
				Encoder_Init();
				calibrationState = CalibrationState_ReverseDetect;
				calibrationTicks = 0;
				calibrationLowMotionSamples = 0;
				Led_Switch(false);
			}
		}

		if (calibrationTicks > wallTimeoutTicks) {
			calibrationFailed = true;
			Motor_Stop();
			motor_command = 0;
			calibrationState = CalibrationState_Finished;
		}
		break;

	case CalibrationState_ReverseDetect:
		motor_command = -SPEED_CALIBRATION;

		if (abs(Encoder_Read()) >= 30) {
			if (Encoder_Read() > 0) {
				calibrationEncoderDirection = -1;
			} else {
				calibrationEncoderDirection = 1;
			}
			Encoder_Set_Direction(calibrationEncoderDirection);
			Encoder_Init();
			calibrationState = CalibrationState_DriveToCenter;
			calibrationTicks = 0;
		} else if (calibrationTicks > reverseDetectTimeoutTicks) {
			calibrationFailed = true;
			Motor_Stop();
			motor_command = 0;
			calibrationState = CalibrationState_Finished;
		}
		break;

	case CalibrationState_DriveToCenter:
	{
		int target = -(int)(POSITION_ENCODER_RANGE / 2);
		int currentPosition = Encoder_Read();
		int error = currentPosition - target;
		float fDiff = 2.0 * abs(error) / POSITION_ENCODER_RANGE;

		if (fDiff <= 1e-2) {
			Motor_Stop();
			motor_command = 0;
			calibrationState = CalibrationState_SettleAtCenter;
			calibrationTicks = 0;
		} else if (calibrationTicks > centerTimeoutTicks) {
			calibrationFailed = true;
			Motor_Stop();
			motor_command = 0;
			calibrationState = CalibrationState_Finished;
		} else {
			int speed = (fDiff < 1e-1) ? SPEED_CALIBRATION / 3 : SPEED_CALIBRATION;
			int direction = (error > 0) ? -1 : 1;
			motor_command = direction * speed;
		}
		break;
	}

	case CalibrationState_SettleAtCenter:
		Motor_Stop();
		motor_command = 0;

		if (calibrationTicks >= centerSettleTicks) {
			Encoder_Init();
			int halfTrack = (int)(POSITION_ENCODER_RANGE / 2);
			positionCentre = 0;
			positionLimitRight = halfTrack;
			positionLimitLeft = -halfTrack;
			calibrationState = CalibrationState_Finished;
		}
		break;

	case CalibrationState_Finished:
		Motor_Stop();
		motor_command = 0;

		if(!HardwareConfigSetFromPC && !calibrationFailed)
		{
			MOTOR = calibrationEncoderDirection==1 ? MOTOR_POLOLU : MOTOR_ORIGINAL;
			ANGLE_HANGING = MOTOR==MOTOR_POLOLU ? ANGLE_HANGING_POLOLU : ANGLE_HANGING_ORIGINAL;

			ANGLE_DEVIATION = angle_deviation_update(ANGLE_HANGING);
		}

		prepare_message_to_PC_calibration(buffer, calibrationFailed ? 0 : calibrationEncoderDirection);
	    Message_SendToPC(buffer, 5);

		isCalibrated = !calibrationFailed;
		calibrate = false;
		calibrationState = CalibrationState_Idle;
		Led_Switch(false);
		break;

	case CalibrationState_Idle:
	default:
		motor_command = 0;
		break;
	}

	time_motor_command_obtained = GetTimeNow();
	new_motor_command_obtained = true;
	time_measurement_done = time_current_measurement;

	if(!CONTROL_SYNC)
	{
		Motor_SetPower(motor_command, MOTOR_PWM_PERIOD_IN_CLOCK_CYCLES);
	}
}

void cmd_ControlMode(bool en)
{
    disable_irq();
	if (en && !ControlOnChip_Enabled)
	{
		PCControl_Enabled = false;
        ledPeriod           = 100/POLLING_PERIOD_MS;
	}
	else if (!en && ControlOnChip_Enabled)
	{
		Motor_Stop();
		motor_command = 0;
		time_motor_command_obtained = 0;
		new_motor_command_obtained = false;
        ledPeriod           = 500/POLLING_PERIOD_MS;
	}

	ControlOnChip_Enabled = en;
	enable_irq();
}

void cmd_PCControlMode(bool en)
{
	disable_irq();
	if (en)
	{
		ControlOnChip_Enabled = false;
		time_motor_command_obtained = 0;
		new_motor_command_obtained = false;
		ledPeriod = 100/POLLING_PERIOD_MS;
	}
	else if (PCControl_Enabled)
	{
		Motor_Stop();
		motor_command = 0;
		time_motor_command_obtained = 0;
		new_motor_command_obtained = false;
		ledPeriod = 500/POLLING_PERIOD_MS;
	}

	PCControl_Enabled = en;
	enable_irq();
}


void cmd_SetControlConfig(const unsigned char * config)
{
	disable_irq();

	POLLING_PERIOD_MS = *((unsigned short *)&config[0]);
    CONTROL_SYNC			= *((bool	        *)&config[2]);
    ANGLE_HANGING      = *((float          *)&config[ 3]);
    ANGLE_AVERAGE_LEN    = *((unsigned short *)&config[ 7]);
    correct_motor_dynamics = *((bool	        *)&config[9]);

    SetControlUpdatePeriod(POLLING_PERIOD_MS);
    ANGLE_DEVIATION = angle_deviation_update(ANGLE_HANGING);

    HardwareConfigSetFromPC = true;

	enable_irq();
}


void cmd_GetControlConfig(void)
{
	prepare_message_to_PC_control_config(txBuffer, POLLING_PERIOD_MS, CONTROL_SYNC, ANGLE_HANGING, ANGLE_AVERAGE_LEN, correct_motor_dynamics);

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
