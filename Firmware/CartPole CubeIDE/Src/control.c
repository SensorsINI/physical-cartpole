#include "hardware_bridge.h"
#include "control.h"
#include "angle_processing.h"
#include "communication_with_PC.h"
#include <stdlib.h>


bool            streamEnable        = false;
short  			angle_setPoint		= CONTROL_ANGLE_SET_POINT_ORIGINAL;
float 			angle_smoothing		= CONTROL_ANGLE_SMOOTHING;
float 			angle_KP			= CONTROL_ANGLE_KP;
float 			angle_KI			= CONTROL_ANGLE_KI;
float 			angle_KD			= CONTROL_ANGLE_KD;

short  			position_setPoint	= CONTROL_POSITION_SET_POINT;
unsigned short	position_ctrlPeriod	= (CONTROL_POSITION_PERIOD_MS / CONTROL_LOOP_PERIOD_MS);
float 			position_smoothing	= CONTROL_POSITION_SMOOTHING;
float 			position_KP			= CONTROL_POSITION_KP;
float 			position_KI			= CONTROL_POSITION_KI;
float 			position_KD			= CONTROL_POSITION_KD;

unsigned short	controlLoopPeriodMs = CONTROL_LOOP_PERIOD_MS;
bool			controlSync 		= CONTROL_SYNC;				// apply motor command at next loop
int 			controlLatencyUs 	= CONTROL_LATENCY_US;	// used to simulate Latency

volatile bool 	controlEnabled;
bool			controlLatencyEnable = false;
int	 			controlLatencyTimestampUs = 0;
int 			controlCommand 		= 0;

bool            isCalibrated 		= true;
unsigned short  ledPeriod;
short			angleErrPrev;
short			positionErrPrev;
unsigned short 	positionPeriodCnt;
int				positionCentre;
int				positionLimitLeft;
int				positionLimitRight;
int				encoderDirection	= 1;
unsigned long	timeMeasured = 0, timeSent = 0, timeReceived = 0, latency = 0;

unsigned int 	time_current_measurement;
unsigned int	time_last_measurement;

bool			newReceived			= true;
float 			angle_I = 0, position_I = 0;

short 			position_previous = -30000;

long  			angleSamplesTimestamp[CONTROL_ANGLE_AVERAGE_LEN];
unsigned short 	angleSampIndex		= 0;
int  angleSamples[CONTROL_ANGLE_AVERAGE_LEN];
unsigned short	angle_averageLen	= CONTROL_ANGLE_AVERAGE_LEN;


static unsigned char rxBuffer[SERIAL_MAX_PKT_LENGTH];
static unsigned char txBuffer[17000];

void 			cmd_Ping(const unsigned char * buff, unsigned int len);
void            cmd_StreamOutput(bool en);
void            cmd_Calibrate(const unsigned char * buff, unsigned int len);
void 			cmd_ControlMode(bool en);
void 			cmd_SetAngleConfig(const unsigned char * config);
void 			cmd_GetAngleConfig(void);
void 			cmd_SetPositionConfig(const unsigned char * config);
void 			cmd_GetPositionConfig(void);
void 			cmd_SetMotor(int motorCmd);
void			cmd_SetControlConfig(const unsigned char * config);
void			cmd_collectRawAngle(const unsigned short, const unsigned short);

void CONTROL_Init(void)
{
	controlEnabled		= false;
    isCalibrated        = false;
    ledPeriod           = 500/controlLoopPeriodMs;
	angleErrPrev		= 0;
	positionErrPrev		= 0;
	positionPeriodCnt 	= position_ctrlPeriod - 1;
    positionCentre      = (short)Encoder_Read(); // assume starting position is near center
    positionLimitLeft   = positionCentre + 2400;
    positionLimitRight  = positionCentre - 2400; // guess defaults based on 7000-8000 counts at limits
}

void CONTROL_ToggleState(void)
{
	cmd_ControlMode(!controlEnabled);
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
    static int				angleCmd        = 0;
    static int              positionCmd     = 0;
    static unsigned char	packetCnt       = 0;
    static unsigned int     stopCnt         = 0;
	static unsigned char	buffer[30];
	static int 				prevAngle = 0, stableAngle = 0;
	static int 				pprevAngle = 0, ppprevAngle = 0;
	static int 				angleI = 0;
	#define lastAngleLength 5
	int 					angleErr;
	short 					positionRaw, positionErr;
	static short 			position_filtered, positionErrPrev, positionPrev;
	float 					angleErrDiff;
	float 					positionErrDiff;
	int   					command;

	short 			position;
	short 			positionD;
	int angle = 0;
	int angleD = 0;
	int invalid_step = 0;


	timeMeasured = GetTimeNow();

	time_last_measurement = time_current_measurement;
	time_current_measurement = GetTimeNow();

	process_angle(angleSamples, angleSampIndex, angle_averageLen, &angle, &angleD, &invalid_step);

	unsigned long time_difference_between_measurement = time_current_measurement-time_last_measurement;

	positionRaw = positionCentre + encoderDirection * ((short)Encoder_Read() - positionCentre);
    position = (positionRaw - positionCentre);

    if (position_previous!=-30000){
    	positionD = position-position_previous;
    } else {
    	positionD = 0;
    }

	// Microcontroller Control Routine
	if (controlEnabled)	{
		// Angle PID control
        angleErr = angle - angle_setPoint;
        angleErrDiff = angleD;
        angle_I += angleErr;
		angleCmd	 = (angle_KP*angleErr + angle_KI*angleI + angle_KD*angleErrDiff);

		// Position PID control
		if (++positionPeriodCnt >= position_ctrlPeriod) {
			positionPeriodCnt = 0;

			// IIR Filter for Position
			if(position_smoothing < 1.0)
				position_filtered = (position_smoothing*position) + (1.0 - position_smoothing)*position_filtered;

			// Position PID control
			positionErr = position_filtered - position_setPoint;
			positionErrDiff = positionErr - positionErrPrev;
			positionErrPrev = positionErr;
			position_I += positionErr;
			positionCmd = (position_KP*positionErr + position_KI*position_I + position_KD*positionErrDiff);
		}

		// Limit the motor speed
		command = - angleCmd - positionCmd;
		if      (command >  CONTROL_MOTOR_MAX_SPEED) command =  CONTROL_MOTOR_MAX_SPEED;
		else if (command < -CONTROL_MOTOR_MAX_SPEED) command = -CONTROL_MOTOR_MAX_SPEED;

        // Disable motor if falls hard on either limit
        if ((command < 0) && (positionRaw < (positionLimitLeft + 20))) {
            command = 0;
            stopCnt++;
        } else if ((command > 0) && (positionRaw > (positionLimitRight - 20))) {
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
        		Motor_SetPower(-command, PWM_PERIOD_IN_CLOCK_CYCLES);
        }
	}
	else
	{
		if(controlSync) {
            Motor_SetPower(controlCommand, PWM_PERIOD_IN_CLOCK_CYCLES);
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
    	}

    	prepare_message_to_PC_state(
    			buffer,
    			angle,
				angleD,
				position,
				positionD,
				command,
				invalid_step,
				time_difference_between_measurement,
				timeMeasured,
				latency);

    	Message_SendToPC(buffer, 25);

        if(newReceived) {
        	timeSent = timeMeasured;
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
	static unsigned int 	rxCnt			= 0;
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
		Motor_SetPower(controlCommand, PWM_PERIOD_IN_CLOCK_CYCLES);
		controlLatencyEnable = false;
	}

	///////////////////////////////////////////////////
	// Process Commands from PC
	///////////////////////////////////////////////////
	if (Message_GetFromPC(&rxBuffer[rxCnt]))
		rxCnt++;

	int current_command = get_command_from_PC_message(rxBuffer, &rxCnt);

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
		case CMD_SET_ANGLE_CONFIG:
		{
			cmd_SetAngleConfig(&rxBuffer[3]);
			break;
		}
		case CMD_GET_ANGLE_CONFIG:
		{
			cmd_GetAngleConfig();
			break;
		}
		case CMD_SET_POSITION_CONFIG:
		{
			cmd_SetPositionConfig(&rxBuffer[3]);
			break;
		}
		case CMD_GET_POSITION_CONFIG:
		{
			cmd_GetPositionConfig();
			break;
		}
		case CMD_SET_MOTOR:
		{
			motorCmd = (((short)rxBuffer[4])<<8) | ((short)rxBuffer[3]);
			timeReceived = GetTimeNow();
			newReceived = true;

			if(controlSync) {
				controlCommand = motorCmd;
			} else {
				cmd_SetMotor(motorCmd);
			}
			break;
		}
		case CMD_SET_CONTROL_CONFIG:
		{
			cmd_SetControlConfig(&rxBuffer[3]);
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
				cmd_SetMotor(motorCmd);
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
	Motor_SetPower(-SPEED, PWM_PERIOD_IN_CLOCK_CYCLES);

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
	Motor_SetPower(SPEED, PWM_PERIOD_IN_CLOCK_CYCLES);

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
		encoderDirection = -1;
	} else
		encoderDirection = 1;
	positionCentre = (positionLimitRight + positionLimitLeft) / 2;			// average limits

	// Slower to get back to middle
	Motor_SetPower(-SPEED, PWM_PERIOD_IN_CLOCK_CYCLES);
	do {
		fDiff = 2.0 * abs(Encoder_Read() - positionCentre) / abs(positionLimitRight - positionLimitLeft);
		// Slow Down even more to get more accurately to the middle
		if(fDiff < 1e-1) {
			Motor_SetPower(-SPEED/2, PWM_PERIOD_IN_CLOCK_CYCLES);
		}
	} while(fDiff > 5e-4);
	Motor_Stop();

	angle_setPoint = encoderDirection==1 ? CONTROL_ANGLE_SET_POINT_POLULU : CONTROL_ANGLE_SET_POINT_ORIGINAL;

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
	if (en && !controlEnabled)
	{
		angleErrPrev		= 0;
		positionErrPrev		= 0;
		positionPeriodCnt 	= position_ctrlPeriod - 1;
        ledPeriod           = 100/controlLoopPeriodMs;
	}
	else if (!en && controlEnabled)
	{
		Motor_Stop();
        ledPeriod           = 500/controlLoopPeriodMs;
	}

	controlEnabled = en;
	enable_irq();
}

void cmd_SetAngleConfig(const unsigned char * config)
{
	disable_irq();
    angle_setPoint      = *((short          *)&config[ 0]);
    angle_averageLen    = *((unsigned short *)&config[ 2]);
    angle_smoothing     = *((float          *)&config[ 4]);
    angle_KP            = *((float          *)&config[ 8]);
    angle_KI            = *((float          *)&config[12]);
    angle_KD            = *((float          *)&config[16]);
	angleErrPrev		= 0;
	enable_irq();
}

void cmd_GetAngleConfig(void)
{
	prepare_message_to_PC_angle_config(txBuffer, angle_setPoint, angle_averageLen, angle_smoothing, angle_KP, angle_KI, angle_KD, controlLatencyUs, controlSync);

	disable_irq();
	Message_SendToPC(txBuffer, 29);
	enable_irq();
}

void cmd_SetPositionConfig(const unsigned char * config)
{
	disable_irq();
    position_setPoint   = *((short          *)&config[ 0]);
    position_ctrlPeriod = *((unsigned short *)&config[ 2]);
    position_smoothing  = *((float          *)&config[ 4]);
    position_KP         = *((float          *)&config[ 8]);
    position_KD         = *((float          *)&config[12]);
	positionErrPrev		= 0;
	position_ctrlPeriod	= position_ctrlPeriod / controlLoopPeriodMs;
	positionPeriodCnt	= position_ctrlPeriod - 1;
	enable_irq();
}

void cmd_GetPositionConfig(void)
{
	unsigned short temp;

	temp = position_ctrlPeriod * controlLoopPeriodMs;
	prepare_message_to_PC_position_config(txBuffer, position_setPoint, temp, position_smoothing, position_KP, position_KD);

	disable_irq();
	Message_SendToPC(txBuffer, 20);
	enable_irq();
}

void cmd_SetMotor(int speed)
{

	int pwm_duty_cycle_in_clock_cycles = speed;
	int pwm_period_in_clock_cycles = PWM_PERIOD_IN_CLOCK_CYCLES;

	int position;

	//	Motor_SetPower(speed);
	// Only command the motor if the on-board control routine is disabled
	if (!controlEnabled){
				position = Encoder_Read();

				// Disable motor if falls hard on either limit
				if ((pwm_duty_cycle_in_clock_cycles < 0) && (position < (positionLimitLeft + 10)))
				{
						Motor_Stop();
				}
				else if ((pwm_duty_cycle_in_clock_cycles > 0) && (position > (positionLimitRight - 10)))
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

void cmd_SetControlConfig(const unsigned char * config)
{
	disable_irq();

	controlLoopPeriodMs = *((unsigned short *)&config[0]);
    controlSync			= *((bool	        *)&config[2]);
    controlLatencyUs    = *((int            *)&config[3]);

    SetControlUpdatePeriod(controlLoopPeriodMs);
	position_ctrlPeriod	= position_ctrlPeriod / controlLoopPeriodMs;

	enable_irq();
}
