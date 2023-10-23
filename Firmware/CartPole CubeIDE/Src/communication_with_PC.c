#include "communication_with_PC.h"

unsigned char 	crc(const unsigned char * message, unsigned int len);
bool 			crcIsValid(const unsigned char * buff, unsigned int len, unsigned char crcVal);

int get_command_from_PC_message(unsigned char * rxBuffer, unsigned int rxCnt){
	unsigned int			i;
	unsigned int 			idx;
	unsigned int			pktLen;
	int current_command = CMD_DO_NOTHING;
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
								current_command = CMD_PING;
								break;
							}

                            case CMD_STREAM_ON:
                            {
                                if (pktLen == 5)
								{
                                	current_command = CMD_STREAM_ON;
								}
                                break;
                            }

							case CMD_CALIBRATE:
							{
								if (pktLen == 4)
								{
									current_command = CMD_CALIBRATE;
								}
								break;
							}

							case CMD_CONTROL_MODE:
							{
								if (pktLen == 5)
								{
									current_command = CMD_CONTROL_MODE;
								}
								break;
							}

							case CMD_SET_ANGLE_CONFIG:
							{
								if (pktLen == 24)
								{
									current_command = CMD_SET_ANGLE_CONFIG;
								}
								break;
							}

							case CMD_GET_ANGLE_CONFIG:
							{
								if (pktLen == 4)
								{
									current_command = CMD_GET_ANGLE_CONFIG;
								}
								break;
							}

							case CMD_SET_POSITION_CONFIG:
							{
								if (pktLen == 20)
								{
									current_command = CMD_SET_POSITION_CONFIG;
								}
								break;
							}

							case CMD_GET_POSITION_CONFIG:
							{
								if (pktLen == 4)
								{
									current_command = CMD_GET_POSITION_CONFIG;
								}
								break;
							}

							case CMD_SET_MOTOR:
							{
								if (pktLen == 6)
								{
									current_command = CMD_SET_MOTOR;
								}
								break;
							}

							case CMD_SET_CONTROL_CONFIG:
							{
								if (pktLen == 11)
								{
									current_command = CMD_SET_CONTROL_CONFIG;
								}
								break;
							}

							case CMD_COLLECT_RAW_ANGLE:
							{
								if (pktLen == 8)
								{
									current_command = CMD_COLLECT_RAW_ANGLE_MODE_1;
								}

								else if (pktLen == 6)
								{
									current_command = CMD_COLLECT_RAW_ANGLE_MODE_2;
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

	return current_command;
}

void prepare_message_to_PC_state(
		unsigned char * buffer,
		int angle,
		int angleD,
		int position,
		int positionD,
		int command,
		int invalid_step,
		unsigned long time_difference_between_measurement,
		unsigned long timeMeasured,
		unsigned long latency){

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
	buffer[24] = crc(buffer, 24);
}

void prepare_message_to_PC_calibration(unsigned char * buffer, int encoderDirection){
	buffer[ 0] = SERIAL_SOF;
	buffer[ 1] = CMD_CALIBRATE;
	buffer[ 2] = 5;
	*((signed char *)&buffer[3]) = (signed char)encoderDirection;
	buffer[4] = crc(buffer, 4);
}

void prepare_message_to_PC_angle_config(
		unsigned char * txBuffer,
		short angle_setPoint,
		unsigned short angle_averageLen,
		float angle_smoothing,
		float angle_KP,
		float angle_KI,
		float angle_KD,
		int controlLatencyUs,
		bool controlSync){

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

}

void prepare_message_to_PC_position_config(
		unsigned char * txBuffer,
		short position_setPoint,
		unsigned short temp,
		float position_smoothing,
		float position_KP,
		float position_KD
		){
	txBuffer[ 0] = SERIAL_SOF;
	txBuffer[ 1] = CMD_GET_POSITION_CONFIG;
	txBuffer[ 2] = 20;
	*((short          *)&txBuffer[ 3]) = position_setPoint;
	*((unsigned short *)&txBuffer[ 5]) = temp;
	*((float          *)&txBuffer[ 7]) = position_smoothing;
	*((float          *)&txBuffer[11]) = position_KP;
	*((float          *)&txBuffer[15]) = position_KD;
	txBuffer[19] = crc(txBuffer, 19);
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
