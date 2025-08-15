#include "communication_with_PC_general.h"


bool crcIsValid(const unsigned char * buff, unsigned int len, unsigned char crcVal)
{
    return crcVal == crc(buff, len);
}


void prepare_buffer_to_send_long(unsigned char* Buffer, unsigned char CMD, unsigned int message_length) {
    Buffer[0] = SERIAL_SOF; // Assuming SERIAL_SOF is defined elsewhere
    Buffer[1] = CMD;
    *((unsigned int*)&Buffer[2]) = message_length; // Correctly places message_length in the buffer

    // Assuming crc function exists and calculates CRC correctly
    // This assumes the CRC is to be placed at the end of the message and message_length includes the CRC byte
    Buffer[message_length - 1] = crc(Buffer, message_length - 1);
}


void prepare_message_to_PC_config_PID(
		unsigned char * txBuffer,
		float position_KP,
		float position_KI,
		float position_KD,
		float angle_KP,
		float angle_KI,
		float angle_KD
		){

	txBuffer[ 0] = SERIAL_SOF;
	txBuffer[ 1] = CMD_GET_PID_CONFIG;
	txBuffer[ 2] = 28;

	*((float          *)&txBuffer[3]) = position_KP;
	*((float          *)&txBuffer[7]) = position_KI;
	*((float          *)&txBuffer[11]) = position_KD;

	*((float          *)&txBuffer[15]) = angle_KP;
	*((float          *)&txBuffer[19]) = angle_KI;
	*((float          *)&txBuffer[23]) = angle_KD;

	txBuffer[27] = crc(txBuffer, 27);

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
