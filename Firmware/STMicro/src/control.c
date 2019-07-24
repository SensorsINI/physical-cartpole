#include "angle.h"
#include "encoder.h"
#include "motor.h"
#include "led.h"
#include "usart.h"
#include "control.h"

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
#define CMD_STATE					0xCC

bool            streamEnable        = false;
short  			angle_setPoint		= CONTROL_ANGLE_SET_POINT;
unsigned short	angle_averageLen	= CONTROL_ANGLE_AVERAGE_LEN;
float 			angle_smoothing		= CONTROL_ANGLE_SMOOTHING;
float 			angle_KP			= CONTROL_ANGLE_KP;
float 			angle_KD			= CONTROL_ANGLE_KD;
short  			position_setPoint	= CONTROL_POSITION_SET_POINT;
unsigned short	position_ctrlPeriod	= (CONTROL_POSITION_PERIOD_MS / CONTROL_LOOP_PERIOD_MS);
float 			position_smoothing	= CONTROL_POSITION_SMOOTHING;
float 			position_KP			= CONTROL_POSITION_KP;
float 			position_KD			= CONTROL_POSITION_KD;

volatile bool 	controlEnabled;
bool            isCalibrated;
unsigned short  ledPeriod;
unsigned short  angleSamples[64];
short			angleErrPrev;
short			positionErrPrev;
unsigned short 	positionPeriodCnt;
short			positionCentre;
short			positionLimitLeft;
short			positionLimitRight;

static unsigned char rxBuffer[SERIAL_MAX_PKT_LENGTH];
static unsigned char txBuffer[32];

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

void CONTROL_Init(void)
{
	controlEnabled		= false;
    isCalibrated        = false;
    ledPeriod           = 500/CONTROL_LOOP_PERIOD_MS;
	angleErrPrev		= 0;
	positionErrPrev		= 0;
	positionPeriodCnt 	= position_ctrlPeriod - 1;
    positionCentre      = (short)ENCODER_Read();
    positionLimitLeft   = positionCentre - 1;
    positionLimitRight  = positionCentre + 1;
}

void CONTROL_ToggleState(void)
{
	cmd_ControlMode(!controlEnabled);
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
	static unsigned char	buffer[11];
	unsigned int 			angleAccum;
	unsigned short 			i;
	short 					angle, angleErr;
	short 					positionRaw, position, positionErr;
	float 					angleErrDiff;
	float 					positionErrDiff;
	int   					command;
    
	// Get latest angle and position
	angleAccum = 0;
	for (i = 0; i < angle_averageLen; i++)
	{
		angleAccum += angleSamples[i];
	}
	angle 		= (short)(angleAccum / angle_averageLen);
	positionRaw = (short)ENCODER_Read();
    position    = positionRaw - positionCentre;

	// Microcontroller Control Routine
	if (controlEnabled)
	{      
		// Angle PD control
        angleErr = angle - angle_setPoint;
		if (angle_smoothing < 1.0)
		{
			angleErrDiff = (angle_smoothing*angleErr) + ((1.0 - angle_smoothing)*angleErrPrev);
		}
		else
		{
			angleErrDiff = angleErr - angleErrPrev;
		}
		angleErrPrev = angleErr;
		angleCmd	 = -(angle_KP*angleErr + angle_KD*angleErrDiff);

		// Position PD control
		positionPeriodCnt++;
		if (positionPeriodCnt >= position_ctrlPeriod)
		{
			positionPeriodCnt = 0;
			positionErr = position - position_setPoint;
			if (position_smoothing < 1.0)
			{
				positionErrDiff = (position_smoothing*positionErr) + ((1.0 - position_smoothing)*positionErrPrev);
			}
			else
			{
				positionErrDiff = positionErr - positionErrPrev;
			}
			positionErrPrev	= positionErr;
			positionCmd		= position_KP*positionErr + position_KD*positionErrDiff;
		}

		// Limit the motor speed
		// This will reduce the numerical range of command from an int to a short
		command = angleCmd - positionCmd;
		if      (command >  CONTROL_MOTOR_MAX_SPEED) command =  CONTROL_MOTOR_MAX_SPEED;
		else if (command < -CONTROL_MOTOR_MAX_SPEED) command = -CONTROL_MOTOR_MAX_SPEED;
		
        // Disable motor if falls hard on either limit
        if ((command < 0) && (positionRaw < (positionLimitLeft + 20)))
        {
            command = 0;
            stopCnt++;
        }
        else if ((command > 0) && (positionRaw > (positionLimitRight - 20)))
        {
            command = 0;
            stopCnt++;
        }
        else
        {
            stopCnt = 0;
        }
        
        // Quit control if pendulum has continously been at the end for 500 ms
        if (stopCnt == 500/CONTROL_LOOP_PERIOD_MS)
        {
            cmd_ControlMode(false);
            stopCnt = 0;
        }
        else
        {
            MOTOR_SetSpeed(command);
        }
	}
	else
	{
		command = 0;
        stopCnt = 0;
	}

	// Send latest state to the PC
    if (streamEnable)
    {
        buffer[ 0] = SERIAL_SOF;
        buffer[ 1] = CMD_STATE;
        buffer[ 2] = 11;
        buffer[ 3] = packetCnt++;
        *((short *)&buffer[4]) = angle;
        *((short *)&buffer[6]) = position;
        *((short *)&buffer[8]) = (short)command;
        buffer[10] = crc(buffer, 10);
        USART_SendBuffer(buffer, 11);
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
	static unsigned short 	angleSampIndex 	= 0;
	static unsigned int 	rxCnt			= 0;
	unsigned int			i;
	unsigned int 			idx;
	unsigned int			pktLen;
	short					motorCmd;

	///////////////////////////////////////////////////
	// Collect samples of angular displacement
	///////////////////////////////////////////////////
	angleSamples[angleSampIndex++] = ANGLE_Read();
	if (angleSampIndex >= angle_averageLen)
	{
		angleSampIndex = 0;
	}

	///////////////////////////////////////////////////
	// Process Commands from PC
	///////////////////////////////////////////////////
	if (USART_ReceiveAsync(&rxBuffer[rxCnt]))
	{
		rxCnt++;
	}

	// Buffer should have at least 4 bytes
	if (rxCnt >= 4)
	{
		idx = 0;

		// Message must start with SOF character
    	if (rxBuffer[0] == SERIAL_SOF)
		{
			// Packet length must be less than the max
			pktLen = rxBuffer[2];
			if ((pktLen <= SERIAL_MAX_PKT_LENGTH) && (pktLen >= 4))
			{
				// Receive entire message packet (including CRC)
				if (rxCnt >= pktLen)
				{
					// Validate message integrity
					if (crcIsValid(rxBuffer, pktLen-1, rxBuffer[pktLen-1]))
					{
						// Process message
						switch (rxBuffer[1])
						{
							case CMD_PING:
							{
								cmd_Ping(rxBuffer, pktLen);
								break;
							}
                            
                            case CMD_STREAM_ON:
                            {
                                if (pktLen == 5)
								{
									cmd_StreamOutput(rxBuffer[3] != 0);
								}
                                break;
                            }
							
							case CMD_CALIBRATE:
							{
								if (pktLen == 4)
								{
									cmd_Calibrate(rxBuffer, pktLen);
								}
								break;
							}

							case CMD_CONTROL_MODE:
							{
								if (pktLen == 5)
								{
									cmd_ControlMode(rxBuffer[3] != 0);
								}
								break;
							}

							case CMD_SET_ANGLE_CONFIG:
							{
								if (pktLen == 20)
								{
									cmd_SetAngleConfig(&rxBuffer[3]);
								}
								break;
							}

							case CMD_GET_ANGLE_CONFIG:
							{
								if (pktLen == 4)
								{
									cmd_GetAngleConfig();
								}
								break;
							}

							case CMD_SET_POSITION_CONFIG:
							{
								if (pktLen == 20)
								{
									cmd_SetPositionConfig(&rxBuffer[3]);
								}
								break;
							}

							case CMD_GET_POSITION_CONFIG:
							{
								if (pktLen == 4)
								{
									cmd_GetPositionConfig();
								}
								break;
							}

							case CMD_SET_MOTOR:
							{
								if (pktLen == 6)
								{
									motorCmd = (((short)rxBuffer[4])<<8) | ((short)rxBuffer[3]);
									cmd_SetMotor(motorCmd);
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
	short pos;
	short diff;

    __disable_irq();
	MOTOR_Stop();
	Led_Enable(true);

	// Get left limit
	SYS_DelayMS(10);
	positionLimitLeft = ENCODER_Read();
	MOTOR_SetSpeed(-1500);
	
	while (true)
	{
		SYS_DelayMS(25);
		pos  = ENCODER_Read();
		diff = pos - positionLimitLeft;
		positionLimitLeft = pos;

		if ((diff < 1) && (diff > -1))
		{
			break;
		}
	}

	MOTOR_Stop();

	// Get right limit
	SYS_DelayMS(10);
	positionLimitRight = ENCODER_Read();
	MOTOR_SetSpeed(1500);
	
	while (true)
	{
		SYS_DelayMS(25);
		pos  = ENCODER_Read();
		diff = pos - positionLimitRight;
		positionLimitRight = pos;
		
		if ((diff < 1) && (diff > -1))
		{
			break;
		}
	}

	MOTOR_Stop();

	// Move pendulum to the centre (roughly)
	SYS_DelayMS(10);
	positionCentre = (positionLimitRight + positionLimitLeft) >> 1;
	MOTOR_SetSpeed(-1500);
	while (true)
	{
		pos = ENCODER_Read();
		diff = pos - positionCentre;
		if (diff < 1)
		{
			break;
		}
	}

	MOTOR_Stop();
    
    SYS_DelayMS(100);
    USART_SendBuffer(buff, len);
    isCalibrated = true;
    Led_Enable(false);
    __enable_irq();
}

void cmd_ControlMode(bool en)
{
    if (en && !isCalibrated)
    {
        cmd_Calibrate(0, 0);
    }
    
	__disable_irq();
	if (en && !controlEnabled)
	{
		angleErrPrev		= 0;
		positionErrPrev		= 0;
		positionPeriodCnt 	= position_ctrlPeriod - 1;
        ledPeriod           = 100/CONTROL_LOOP_PERIOD_MS;
	}
	else if (!en && controlEnabled)
	{
		MOTOR_Stop();
        ledPeriod           = 500/CONTROL_LOOP_PERIOD_MS;
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
    angle_KD            = *((float          *)&config[12]);
	angleErrPrev		= 0;
	__enable_irq();
}

void cmd_GetAngleConfig(void)
{
	txBuffer[ 0] = SERIAL_SOF;
	txBuffer[ 1] = CMD_GET_ANGLE_CONFIG;
	txBuffer[ 2] = 20;
    *((short          *)&txBuffer[ 3]) = angle_setPoint;
	*((unsigned short *)&txBuffer[ 5]) = angle_averageLen;
    *((float          *)&txBuffer[ 7]) = angle_smoothing;
    *((float          *)&txBuffer[11]) = angle_KP;
    *((float          *)&txBuffer[15]) = angle_KD;
	txBuffer[19] = crc(txBuffer, 19);

	__disable_irq();
	USART_SendBuffer(txBuffer, 20);
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
	position_ctrlPeriod	= position_ctrlPeriod / CONTROL_LOOP_PERIOD_MS;
	positionPeriodCnt	= position_ctrlPeriod - 1;
	__enable_irq();
}

void cmd_GetPositionConfig(void)
{
	unsigned short temp;

	temp = position_ctrlPeriod * CONTROL_LOOP_PERIOD_MS;

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
    short position;
    
	// Only command the motor if the on-board control routine is disabled
	if (!controlEnabled)
	{
        position = ENCODER_Read();
        
        // Disable motor if falls hard on either limit
        if ((speed < 0) && (position < (positionLimitLeft + 20)))
        {
            MOTOR_Stop();
        }
        else if ((speed > 0) && (position > (positionLimitRight - 20)))
        {
            MOTOR_Stop();
        }
		else
        {
            MOTOR_SetSpeed(speed);
		}
	}
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
