#include "angle.h"
#include "encoder.h"
#include "motor.h"
#include "led.h"
#include "usart.h"
#include "control.h"
#include "timer.h"
#include "angle_processing.h"

#include <stdlib.h>

// Command set
#define SERIAL_MAX_PKT_LENGTH		32
#define SERIAL_SOF					0xAA
#define CMD_PING					0xC0
#define CMD_STREAM_ON               0xC1
#define CMD_CALIBRATE				0xC2
#define CMD_CONTROL_MODE			0xC3
#define CMD_SET_ANGLE_CONFIG		0xC4
#define CMD_GET_ANGLE_CONFIG		0xC5
#define CMD_SET_POSITION_CONFIG		0xC6
#define CMD_GET_POSITION_CONFIG		0xC7
#define CMD_SET_MOTOR				0xC8
#define CMD_SET_CONTROL_CONFIG		0xC9
#define CMD_COLLECT_RAW_ANGLE		0xCA
#define CMD_STATE					0xCC

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

unsigned char 	crc(const unsigned char * message, unsigned int len);
bool 			crcIsValid(const unsigned char * buff, unsigned int len, unsigned char crcVal);
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
    positionCentre      = (short)ENCODER_Read(); // assume starting position is near center
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


	timeMeasured = TIMER1_getSystemTime_Us();

	time_last_measurement = time_current_measurement;
	time_current_measurement = TIMER1_getSystemTime_Us();

	process_angle(angleSamples, angleSampIndex, angle_averageLen, &angle, &angleD, &invalid_step);

	unsigned long time_difference_between_measurement = time_current_measurement-time_last_measurement;

	positionRaw = positionCentre + encoderDirection * ((short)ENCODER_Read() - positionCentre);
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
            MOTOR_SetSpeed(0);
            stopCnt = 0;
        } else {
        	if(controlLatencyUs > 0) {
        		controlLatencyTimestampUs = TIMER1_getSystemTime_Us() + controlLatencyUs;
        		controlCommand = -command;
        		controlLatencyEnable = true;
        	}
        	else
        		MOTOR_SetSpeed(-command);
        }
	}
	else
	{
		if(controlSync) {
            MOTOR_SetSpeed(controlCommand);
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

        buffer[ 0] = SERIAL_SOF;
        buffer[ 1] = CMD_STATE;
        buffer[ 2] = 25;
        *((short *)&buffer[3]) = angle;
        *((short *)&buffer[5]) = angleD;
        *((short *)&buffer[7]) = position;
        *((short *)&buffer[9]) = positionD;
        *((short *)&buffer[11]) = command;
        *((unsigned char *)&buffer[13]) = invalid_step;
        *((unsigned int *)&buffer[14]) = (unsigned int)time_difference_between_measurement;
        *((unsigned int *)&buffer[18]) = (unsigned int)timeMeasured;
        *((unsigned short *)&buffer[22]) = (unsigned short)(latency / 10);
        // latency maximum: 10 * 65'535 Us = 653ms

    	Message_SendToPC(buffer, 27);

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
		Led_Enable(ledState);
	}
}

void CONTROL_BackgroundTask(void)
{
	static unsigned int 	rxCnt			= 0;
	unsigned int			i;
	unsigned int 			idx;
	unsigned int			pktLen;
	short					motorCmd;
	static unsigned long    lastRead = 0;
	int						read = 0;

	///////////////////////////////////////////////////
	// Collect samples of angular displacement
	///////////////////////////////////////////////////
	unsigned long now = TIMER1_getSystemTime_Us();

	// int-overflow after 1h
	if (now < lastRead) {
		lastRead = now;
	}
	// read every ca. 100us
	else if (now > lastRead + CONTROL_ANGLE_MEASUREMENT_INTERVAL_US) {
		// conversion takes 18us
		read = ANGLE_Read();
		angleSamples[angleSampIndex] = read;
		angleSampIndex = (++angleSampIndex >= angle_averageLen ? 0 : angleSampIndex);

		lastRead = now;
	}

	///////////////////////////////////////////////////
	// Apoply Delayed Control Command
	///////////////////////////////////////////////////
	if (controlLatencyEnable && controlLatencyTimestampUs >= TIMER1_getSystemTime_Us()) {
		MOTOR_SetSpeed(controlCommand);
		controlLatencyEnable = false;
	}

	///////////////////////////////////////////////////
	// Process Commands from PC
	///////////////////////////////////////////////////
	if (USART_ReceiveAsync(&rxBuffer[rxCnt]))
		rxCnt++;

	// Buffer should have at least 4 bytes
	if (rxCnt >= 4)
	{
		idx = 0;

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

            if(newReceived){
                timeSent = timeMeasured;
            }

			newReceived = true;

									if(controlSync) {
										controlCommand = motorCmd;
									} else {
										cmd_SetMotor(motorCmd);
									}
								}
								break;
							}

							case CMD_SET_CONTROL_CONFIG:
							{
								if (pktLen == 11)
								{
									cmd_SetControlConfig(&rxBuffer[3]);
								}
								break;
							}

							case CMD_COLLECT_RAW_ANGLE:
							{
								if (pktLen == 8)
								{
									unsigned short length 	   = 256 * (unsigned short)rxBuffer[4] + (unsigned short)rxBuffer[3];
									unsigned short interval_us = 256 * (unsigned short)rxBuffer[6] + (unsigned short)rxBuffer[5];
									cmd_CollectRawAngle(length, interval_us);
								}

								if (pktLen == 6)
								{
									motorCmd =
									timeReceived = TIMER1_getSystemTime_Us();
									newReceived = true;

									if(controlSync) {
										controlCommand = motorCmd;
									} else {
										cmd_SetMotor(motorCmd);
									}
								}

								break;
							}

							default:
							{
								break;
							}
						}

						idx = pktLen;   // Trim message
					}
					else
					{
						idx = 1;  // Trim SOF and start looking for the next packet
					}
				}
			}
			else
			{
				idx = 1;  // Trim SOF and start looking for the next packet
			}
		}
		else
		{
			idx = 1;  // Trim SOF and start looking for the next packet
		}

		// Shift buffer until first character is SOF
		if (idx != 0)
		{
			for (; idx < rxCnt; idx++)
			{
				if (rxBuffer[idx] == SERIAL_SOF)
				{
					break;
				}
			}

			rxCnt -= idx;
			for (i = 0; i < rxCnt; i++)
			{
				rxBuffer[i] = rxBuffer[idx+i];
			}
		}
	}
}

void cmd_Ping(const unsigned char * buff, unsigned int len)
{
	__disable_irq();
	USART_SendBuffer(buff, len);
	__enable_irq();
}

void cmd_StreamOutput(bool en)
{
	__disable_irq();
	streamEnable = en;
	__enable_irq();
}

void cmd_Calibrate(const unsigned char * buff, unsigned int len)
{
	#define SPEED 3000
	int pos;
	int diff;
	float fDiff;
	static unsigned char	buffer[30];

	__disable_irq();
	MOTOR_Stop();
	Led_Enable(true);

	// Get left limit
	SYS_DelayMS(100);
	positionLimitLeft = ENCODER_Read();
	MOTOR_SetSpeed(-SPEED);

	do {
		SYS_DelayMS(100);
		pos  = ENCODER_Read();
		diff = pos - positionLimitLeft;
		positionLimitLeft = pos;

		// if we don't move enough, must have hit limit
	} while(abs(diff) > 15);

	MOTOR_Stop();
	Led_Enable(false);

	// Get right limit
	SYS_DelayMS(100);
	positionLimitRight = ENCODER_Read();
	MOTOR_SetSpeed(SPEED);

	do {
		SYS_DelayMS(100);
		pos  = ENCODER_Read();
		diff = pos - positionLimitRight;
		positionLimitRight = pos;

		// if we don't move enough, must have hit limit
	} while(abs(diff) > 15);

	MOTOR_Stop();

	// Move pendulum to the centre (roughly)
	Led_Enable(true);
	SYS_DelayMS(200);
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
	MOTOR_SetSpeed(-SPEED);
	do {
		fDiff = 2.0 * abs(ENCODER_Read() - positionCentre) / abs(positionLimitRight - positionLimitLeft);
		// Slow Down even more to get more accurately to the middle
		if(fDiff < 1e-1) {
			MOTOR_SetSpeed(-SPEED/2);
		}
	} while(fDiff > 5e-4);
	MOTOR_Stop();

	angle_setPoint = encoderDirection==1 ? CONTROL_ANGLE_SET_POINT_POLULU : CONTROL_ANGLE_SET_POINT_ORIGINAL;

	SYS_DelayMS(100);

    buffer[ 0] = SERIAL_SOF;
    buffer[ 1] = CMD_CALIBRATE;
    buffer[ 2] = 5;
    *((signed char *)&buffer[3]) = (signed char)encoderDirection;
    buffer[4] = crc(buffer, 4);
    USART_SendBuffer(buffer, 5);

	isCalibrated = true;
	Led_Enable(false);
	__enable_irq();
}

void cmd_ControlMode(bool en)
{
    if(en && !isCalibrated) {
    	cmd_Calibrate(0, 0);
    }
    __disable_irq();
	if (en && !controlEnabled)
	{
		angleErrPrev		= 0;
		positionErrPrev		= 0;
		positionPeriodCnt 	= position_ctrlPeriod - 1;
        ledPeriod           = 100/controlLoopPeriodMs;
	}
	else if (!en && controlEnabled)
	{
		MOTOR_Stop();
        ledPeriod           = 500/controlLoopPeriodMs;
	}

	controlEnabled = en;
	__enable_irq();
}

void cmd_SetAngleConfig(const unsigned char * config)
{
	__disable_irq();
    angle_setPoint      = *((short          *)&config[ 0]);
    angle_averageLen    = *((unsigned short *)&config[ 2]);
    angle_smoothing     = *((float          *)&config[ 4]);
    angle_KP            = *((float          *)&config[ 8]);
    angle_KI            = *((float          *)&config[12]);
    angle_KD            = *((float          *)&config[16]);
	angleErrPrev		= 0;
	__enable_irq();
}

void cmd_GetAngleConfig(void)
{
	txBuffer[ 0] = SERIAL_SOF;
	txBuffer[ 1] = CMD_GET_ANGLE_CONFIG;
	txBuffer[ 2] = 28;
    *((short          *)&txBuffer[ 3]) = angle_setPoint;
	*((unsigned short *)&txBuffer[ 5]) = angle_averageLen;
    *((float          *)&txBuffer[ 7]) = angle_smoothing;
    *((float          *)&txBuffer[11]) = angle_KP;
    *((float          *)&txBuffer[15]) = angle_KI;
    *((float          *)&txBuffer[19]) = angle_KD;
    *((float          *)&txBuffer[23]) = controlLatencyUs;
    *((bool           *)&txBuffer[27]) = controlSync;
	txBuffer[28] = crc(txBuffer, 28);

	__disable_irq();
	USART_SendBuffer(txBuffer, 29);
	__enable_irq();
}

void cmd_SetPositionConfig(const unsigned char * config)
{
	__disable_irq();
    position_setPoint   = *((short          *)&config[ 0]);
    position_ctrlPeriod = *((unsigned short *)&config[ 2]);
    position_smoothing  = *((float          *)&config[ 4]);
    position_KP         = *((float          *)&config[ 8]);
    position_KD         = *((float          *)&config[12]);
	positionErrPrev		= 0;
	position_ctrlPeriod	= position_ctrlPeriod / controlLoopPeriodMs;
	positionPeriodCnt	= position_ctrlPeriod - 1;
	__enable_irq();
}

void cmd_GetPositionConfig(void)
{
	unsigned short temp;

	temp = position_ctrlPeriod * controlLoopPeriodMs;

	txBuffer[ 0] = SERIAL_SOF;
	txBuffer[ 1] = CMD_GET_POSITION_CONFIG;
	txBuffer[ 2] = 20;
    *((short          *)&txBuffer[ 3]) = position_setPoint;
	*((unsigned short *)&txBuffer[ 5]) = temp;
    *((float          *)&txBuffer[ 7]) = position_smoothing;
    *((float          *)&txBuffer[11]) = position_KP;
    *((float          *)&txBuffer[15]) = position_KD;
	txBuffer[19] = crc(txBuffer, 19);

	__disable_irq();
	USART_SendBuffer(txBuffer, 20);
	__enable_irq();
}

void cmd_SetMotor(int speed)
{
		int position;

	//	MOTOR_SetSpeed(speed);
	// Only command the motor if the on-board control routine is disabled
	if (!controlEnabled){
				position = ENCODER_Read();

				// Disable motor if falls hard on either limit
				if ((speed < 0) && (position < (positionLimitLeft + 10)))
				{
						MOTOR_Stop();
				}
				else if ((speed > 0) && (position > (positionLimitRight - 10)))
				{
						MOTOR_Stop();
				}
		else{
				MOTOR_SetSpeed(speed);
		}
	}
}

void cmd_CollectRawAngle(unsigned short MEASURE_LENGTH, unsigned short INTERVAL_US)
{

	TIMER1_SetCallback(0);
	MOTOR_Stop();
	Led_Enable(true);

	txBuffer[ 0] = SERIAL_SOF;
	txBuffer[ 1] = CMD_COLLECT_RAW_ANGLE;
	txBuffer[ 2] = 4 + 2*MEASURE_LENGTH;

	unsigned int now = 0, lastRead = 0;

	unsigned int i;
	for(i=0; i<MEASURE_LENGTH;) {
		Led_Enable(i % 2);
		now = TIMER1_getSystemTime_Us();

		// int-overflow after 1h
		if (now < lastRead) {
			lastRead = now;
		}
		// read every ca. 100us
		else if (now > lastRead + INTERVAL_US) {
			// conversion takes 18us
			*((unsigned short *)&txBuffer[ 3 + 2*i]) = ANGLE_Read();
			lastRead = now;
			i++;
		}
	}
	Led_Enable(true);

	//txBuffer[3 + 2*MEASURE_LENGTH] = crc(txBuffer, 3 + 2*MEASURE_LENGTH);

	__disable_irq();
	USART_SendBuffer(txBuffer, 4 + 2*MEASURE_LENGTH);
	TIMER1_SetCallback(CONTROL_Loop);
	__enable_irq();
}

void cmd_SetControlConfig(const unsigned char * config)
{
	__disable_irq();

	controlLoopPeriodMs = *((unsigned short *)&config[0]);
    controlSync			= *((bool	        *)&config[2]);
    controlLatencyUs    = *((int            *)&config[3]);

    TIMER1_ChangePeriod(controlLoopPeriodMs);
	position_ctrlPeriod	= position_ctrlPeriod / controlLoopPeriodMs;

	__enable_irq();
}

unsigned char crc(const unsigned char * buff, unsigned int len)
{
    unsigned char crc8 = 0x00;
	unsigned char val;
	unsigned char sum;
	unsigned int  i;

    while (len--)
    {
        val = *buff++;
        for (i = 0; i < 8; i++)
        {
            sum = (crc8 ^ val) & 0x01;
            crc8 >>= 1;
            if (sum > 0)
            {
                crc8 ^= 0x8C;
            }
            val >>= 1;
        }
    }
    return crc8;
}

bool crcIsValid(const unsigned char * buff, unsigned int len, unsigned char crcVal)
{
    return crcVal == crc(buff, len);
}
