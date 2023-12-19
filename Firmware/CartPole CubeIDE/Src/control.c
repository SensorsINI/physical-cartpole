#include "hardware_bridge.h"
#include "control.h"
#include "angle_processing.h"
#include "communication_with_PC.h"
#include <stdlib.h>
#include "math.h"
#include "hardware_pid.h"

#define OnChipController_PID 0

unsigned short current_controller = OnChipController_PID;

bool            streamEnable        = false;
float  			ANGLE_DEVIATION		= CONTROL_ANGLE_SET_POINT_ORIGINAL;

unsigned short	controlLoopPeriodMs = CONTROL_LOOP_PERIOD_MS;
bool			controlSync 		= CONTROL_SYNC;				// apply motor command at next loop
int 			controlLatencyUs 	= CONTROL_LATENCY_US;	// used to simulate Latency

volatile bool	HardwareConfigSetFromPC;
volatile bool 	ControlOnChip_Enabled;
bool			controlLatencyEnable = false;
int	 			controlLatencyTimestampUs = 0;
int 			controlCommand 		= 0;

bool            isCalibrated 		= true;
unsigned short  ledPeriod;

int				positionCentre;
int				positionLimitLeft;
int				positionLimitRight;
int				encoderDirection	= -1;
unsigned long	timeSent = 0, timeReceived = 0, latency = 0;

unsigned long 	time_current_measurement = 0;
unsigned long	time_last_measurement = 0;

bool			newReceived			= true;
float 			angle_I = 0, position_I = 0;

short 			position_previous = -30000;

long  			angleSamplesTimestamp[CONTROL_ANGLE_AVERAGE_LEN];
unsigned short 	angleSampIndex		= 0;
int  angleSamples[CONTROL_ANGLE_AVERAGE_LEN];
unsigned short	angle_averageLen	= CONTROL_ANGLE_AVERAGE_LEN;

unsigned short	latency_violation = 0;

static unsigned char rxBuffer[SERIAL_MAX_PKT_LENGTH];
static unsigned char txBuffer[17000];

void 			cmd_Ping(const unsigned char * buff, unsigned int len);
void            cmd_StreamOutput(bool en);
void            cmd_Calibrate(const unsigned char * buff, unsigned int len);
void 			cmd_ControlMode(bool en);
void			cmd_SetControlConfig(const unsigned char * config);
void 			cmd_GetControlConfig(void);
void 			cmd_SetMotor(int motorCmd);
void			cmd_collectRawAngle(const unsigned short, const unsigned short);

void CONTROL_Init(void)
{
	ControlOnChip_Enabled		= false;
	HardwareConfigSetFromPC = false;
    isCalibrated        = false;
    ledPeriod           = 500/controlLoopPeriodMs;

    positionCentre      = (short)Encoder_Read(); // assume starting position is near center
    positionLimitLeft   = positionCentre + 2400;
    positionLimitRight  = positionCentre - 2400; // guess defaults based on 7000-8000 counts at limits
}

void CONTROL_ToggleState(void)
{
	cmd_ControlMode(!ControlOnChip_Enabled);
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
	static unsigned short 	ledPeriodCnt	= 0;
	static bool				ledState 		= false;

    static unsigned int     stopCnt         = 0;
	static unsigned char	buffer[30];

	#define lastAngleLength 5

	int   					command;

	short 			position_short;
	short 			positionD_short;
	int angle_int = 0;
	int angleD_int = 0;
	int invalid_step = 0;

	float angle = 0.0;
	float position = 0.0;
	float angleD = 0.0;
	float positionD = 0.0;


	time_last_measurement = time_current_measurement;
	time_current_measurement = GetTimeNow();

	process_angle(angleSamples, angleSampIndex, angle_averageLen, &angle_int, &angleD_int, &invalid_step);

	unsigned long time_difference_between_measurement = time_current_measurement-time_last_measurement;

	position_short = encoderDirection * ((short)Encoder_Read() - positionCentre);

    if (position_previous!=-30000){
    	positionD_short = position_short-position_previous;
    } else {
    	positionD_short = 0;
    }
    position_previous = position_short;


    float ANGLE_NORMALIZATION_FACTOR = 0.001471127442561364;
    float POSITION_NORMALIZATION_FACTOR = 0.00009510086455331;
    float angle_cos, angle_sin;

    float time_difference_between_measurement_s = time_difference_between_measurement/1000000.0;
	angle = wrapLocal_rad((angle_int + ANGLE_DEVIATION) * ANGLE_NORMALIZATION_FACTOR);
    position = position_short * POSITION_NORMALIZATION_FACTOR;

    angle_cos = cos(angle);
    angle_sin = sin(angle);
    angleD = (angleD_int*ANGLE_NORMALIZATION_FACTOR/time_difference_between_measurement_s);
    positionD = (positionD_short*POSITION_NORMALIZATION_FACTOR/time_difference_between_measurement_s);
    float target_position = 0.0;

	// Microcontroller Control Routine
	if (ControlOnChip_Enabled)	{
//
//		switch (current_controller){
//		case OnChipController_PID:
//		{
//			float command_float = pid_step(angle, angleD, position, positionD, target_position, time_current_measurement/1000000.0);
//			break;
//		}
//
//		}
        command = 0;



		if      (command >  CONTROL_MOTOR_MAX_SPEED) command =  CONTROL_MOTOR_MAX_SPEED;
		else if (command < -CONTROL_MOTOR_MAX_SPEED) command = -CONTROL_MOTOR_MAX_SPEED;

        // Disable motor if falls hard on either limit
        if ((command < 0) && (position < (positionLimitLeft + 20))) {
            command = 0;
            stopCnt++;
        } else if ((command > 0) && (position > (positionLimitRight - 20))) {
            command = 0;
            stopCnt++;
        } else {
            stopCnt = 0;
        }

        // Quit control if pendulum has continously been at the end for 500 ms
        if (stopCnt == 500/controlLoopPeriodMs) {
            cmd_ControlMode(false);
            Motor_SetPower(0, PWM_PERIOD_IN_CLOCK_CYCLES);
            stopCnt = 0;
        } else {
        	if(controlLatencyUs > 0) {
        		controlLatencyTimestampUs = GetTimeNow() + controlLatencyUs;
        		controlCommand = -command;
        		controlLatencyEnable = true;
        	}
        	else
        		Motor_SetPower(command, PWM_PERIOD_IN_CLOCK_CYCLES);
        }
	}
	else
	{
		if(controlSync) {
            Motor_SetPower(-controlCommand, PWM_PERIOD_IN_CLOCK_CYCLES);
		} else {
			command = 0;
	        stopCnt = 0;
		}
	}

	// Send latest state to the PC
	static int slowdown = 0;
    if (streamEnable && ++slowdown>=CONTROL_SLOWDOWN)
    {
        slowdown = 0;

    	if(timeReceived > 0 && timeSent > 0 && newReceived) {
        	latency = timeReceived - timeSent;
        	latency_violation = 0;
    	} else{
    		latency_violation = 1;
    		latency = controlLoopPeriodMs*1000;
    	}

    	prepare_message_to_PC_state(
    			buffer,
				27,
    			angle_int,
				angleD_int,
				position_short,
				positionD_short,
				command,
				invalid_step,
				time_difference_between_measurement,
				time_current_measurement,
				latency,
				latency_violation);

    	Message_SendToPC(buffer, 27);

        if(newReceived) {
        	timeSent = time_current_measurement;
        	timeReceived = 0;
        	newReceived = false;
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
}

void CONTROL_BackgroundTask(void)
{
	static unsigned int 	uart_received_Cnt			= 0;
	short					motorCmd;
	static unsigned long    lastRead = 0;
	int						read = 0;

	///////////////////////////////////////////////////
	// Collect samples of angular displacement
	///////////////////////////////////////////////////
	unsigned long now = GetTimeNow();

	// int-overflow after 1h
	if (now < lastRead) {
		lastRead = now;
	}
	// read every ca. 100us
	else if (now > lastRead + CONTROL_ANGLE_MEASUREMENT_INTERVAL_US) {
		// conversion takes 18us
		read = Goniometer_Read();
		angleSamples[angleSampIndex] = read;
		angleSampIndex = (++angleSampIndex >= angle_averageLen ? 0 : angleSampIndex);

		lastRead = now;
	}

	///////////////////////////////////////////////////
	// Apoply Delayed Control Command
	///////////////////////////////////////////////////
	if (controlLatencyEnable && controlLatencyTimestampUs >= GetTimeNow()) {
		Motor_SetPower(-controlCommand, PWM_PERIOD_IN_CLOCK_CYCLES);
		controlLatencyEnable = false;
	}

	///////////////////////////////////////////////////
	// Process Commands from PC
	///////////////////////////////////////////////////
	while (Message_GetFromPC(&rxBuffer[uart_received_Cnt]))
		uart_received_Cnt++;

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
			unsigned int pktLen = rxBuffer[2];
			cmd_Calibrate(rxBuffer, pktLen);
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
			motorCmd = (((short)rxBuffer[4])<<8) | ((short)rxBuffer[3]);
			timeReceived = GetTimeNow();

            if(newReceived){
                timeSent = time_current_measurement;
            }

			newReceived = true;

			if(controlSync) {
				controlCommand = motorCmd;
			} else {
				cmd_SetMotor(-motorCmd);
			}
			break;
		}
		case CMD_COLLECT_RAW_ANGLE_MODE_1:
		{
			unsigned short length 	   = 256 * (unsigned short)rxBuffer[4] + (unsigned short)rxBuffer[3];
			unsigned short interval_us = 256 * (unsigned short)rxBuffer[6] + (unsigned short)rxBuffer[5];
			cmd_CollectRawAngle(length, interval_us);
			break;
		}
		case CMD_COLLECT_RAW_ANGLE_MODE_2:
		{
			motorCmd =
			timeReceived = GetTimeNow();
			newReceived = true;

			if(controlSync) {
				controlCommand = motorCmd;
			} else {
				cmd_SetMotor(-motorCmd);
			}
			break;
		}
		default:
		{
			break;
		}
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

void cmd_Calibrate(const unsigned char * buff, unsigned int len)
{
	#define SPEED 3000
	int pos;
	int diff;
	float fDiff;
	static unsigned char	buffer[30];

	disable_irq();
	Motor_Stop();
	Led_Switch(true);

	// Get left limit
	Sleep_ms(100);
	positionLimitLeft = Encoder_Read();
	Motor_SetPower(SPEED, PWM_PERIOD_IN_CLOCK_CYCLES);

	do {
		Sleep_ms(100);
		pos  = Encoder_Read();
		diff = pos - positionLimitLeft;
		positionLimitLeft = pos;

		// if we don't move enough, must have hit limit
	} while(abs(diff) > 15);

	Motor_Stop();
	Led_Switch(false);

	// Get right limit
	Sleep_ms(100);
	positionLimitRight = Encoder_Read();
	Motor_SetPower(-SPEED, PWM_PERIOD_IN_CLOCK_CYCLES);

	do {
		Sleep_ms(100);
		pos  = Encoder_Read();
		diff = pos - positionLimitRight;
		positionLimitRight = pos;

		// if we don't move enough, must have hit limit
	} while(abs(diff) > 15);

	Motor_Stop();

	// Move pendulum to the centre (roughly)
	Led_Switch(true);
	Sleep_ms(200);
	// invert reading for original motor
	if (positionLimitRight < positionLimitLeft) {
		int temp = positionLimitRight;
		positionLimitRight = positionLimitLeft;
		positionLimitLeft = temp;
		encoderDirection = 1;
	} else
		encoderDirection = -1;
	positionCentre = (positionLimitRight + positionLimitLeft) / 2;			// average limits

	// Slower to get back to middle
	Motor_SetPower(SPEED, PWM_PERIOD_IN_CLOCK_CYCLES);
	do {
		fDiff = 2.0 * abs(Encoder_Read() - positionCentre) / abs(positionLimitRight - positionLimitLeft);
		// Slow Down even more to get more accurately to the middle
		if(fDiff < 1e-1) {
			Motor_SetPower(SPEED/2, PWM_PERIOD_IN_CLOCK_CYCLES);
		}
	} while(fDiff > 5e-4);
	Motor_Stop();

	if(!HardwareConfigSetFromPC)
	{
		ANGLE_DEVIATION = encoderDirection==-1 ? CONTROL_ANGLE_SET_POINT_POLULU : CONTROL_ANGLE_SET_POINT_ORIGINAL;
	}

	Sleep_ms(100);
	prepare_message_to_PC_calibration(buffer, encoderDirection);
    Message_SendToPC(buffer, 5);

	isCalibrated = true;
	Led_Switch(false);
	enable_irq();
}

void cmd_ControlMode(bool en)
{
    if(en && !isCalibrated) {
    	cmd_Calibrate(0, 0);
    }
    disable_irq();
	if (en && !ControlOnChip_Enabled)
	{
        ledPeriod           = 100/controlLoopPeriodMs;
	}
	else if (!en && ControlOnChip_Enabled)
	{
		Motor_Stop();
        ledPeriod           = 500/controlLoopPeriodMs;
	}

	ControlOnChip_Enabled = en;
	enable_irq();
}


void cmd_SetControlConfig(const unsigned char * config)
{
	disable_irq();

	controlLoopPeriodMs = *((unsigned short *)&config[0]);
    controlSync			= *((bool	        *)&config[2]);
    controlLatencyUs    = *((int            *)&config[3]);
    ANGLE_DEVIATION      = *((float          *)&config[ 7]);
    angle_averageLen    = *((unsigned short *)&config[ 11]);

    SetControlUpdatePeriod(controlLoopPeriodMs);
    HardwareConfigSetFromPC = true;

	enable_irq();
}


void cmd_GetControlConfig(void)
{
	prepare_message_to_PC_control_config(txBuffer, controlLoopPeriodMs, controlSync, controlLatencyUs, ANGLE_DEVIATION, angle_averageLen);

	disable_irq();
	Message_SendToPC(txBuffer, 15);
	enable_irq();
}


void cmd_SetMotor(int speed)
{

	int pwm_duty_cycle_in_clock_cycles = speed;
	int pwm_period_in_clock_cycles = PWM_PERIOD_IN_CLOCK_CYCLES;

	int position;

	//	Motor_SetPower(-speed);
	// Only command the motor if the on-board control routine is disabled
	if (!ControlOnChip_Enabled){
				position = Encoder_Read();

				// Disable motor if falls hard on either limit
				if ((pwm_duty_cycle_in_clock_cycles > 0) && (position < (positionLimitLeft + 10)))
				{
						Motor_Stop();
				}
				else if ((pwm_duty_cycle_in_clock_cycles < 0) && (position > (positionLimitRight - 10)))
				{
					Motor_Stop();
				}
		else{
				Motor_SetPower(pwm_duty_cycle_in_clock_cycles, pwm_period_in_clock_cycles);
		}
	}
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
