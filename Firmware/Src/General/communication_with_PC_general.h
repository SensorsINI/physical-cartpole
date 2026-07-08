#ifndef __COMMUNICATION_WITH_PC_GENERAL_H_
#define __COMMUNICATION_WITH_GENERAL_PC_H_

#include <stdbool.h>

// Command set for both programs
#define SERIAL_MAX_PKT_LENGTH		32
#define SERIAL_SOF					0xAA
#define CMD_PING					0xC0
#define CMD_STREAM_ON               0xC1
#define CMD_CALIBRATE				0xC2
#define CMD_CONTROL_MODE			0xC3
#define CMD_SET_PID_CONFIG		    0xC4
#define CMD_GET_PID_CONFIG			0xC5
#define CMD_SET_CONTROL_CONFIG		0xC6
#define CMD_GET_CONTROL_CONFIG		0xC7
#define CMD_SET_MOTOR				0xC8
#define CMD_SET_TARGET_POSITION		0xC9
#define CMD_COLLECT_RAW_ANGLE		0xCA
#define CMD_PC_CONTROL_MODE			0xCB
#define CMD_STATE					0xCC
#define CMD_SET_TARGET_EQUILIBRIUM  0xCD
#define CMD_RUN_HARDWARE_EXPERIMENT 0xCE
#define CMD_TRANSFER_BUFFERS        0xD1
#define CMD_SET_ANGLE_FILTER        0xD2
#define CMD_DO_NOTHING				0x00

unsigned char 	crc(const unsigned char * message, unsigned int len);
bool 			crcIsValid(const unsigned char * buff, unsigned int len, unsigned char crcVal);

void prepare_message_to_PC_config_PID(
		unsigned char * txBuffer,
		float position_KP,
		float position_KI,
		float position_KD,
		float angle_KP,
		float angle_KI,
		float angle_KD
		);

#endif /*__COMMUNICATION_WITH_PC_GENERAL_H_*/
