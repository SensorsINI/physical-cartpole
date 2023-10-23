#ifndef __COMMUNICATION_WITH_PC_H_
#define __COMMUNICATION_WITH_PC_H_

#include <stdbool.h>

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
#define CMD_COLLECT_RAW_ANGLE_MODE_1 0xB1
#define CMD_COLLECT_RAW_ANGLE_MODE_2 0xB2
#define CMD_STATE					0xCC
#define CMD_DO_NOTHING				0x00

int get_command_from_PC_message(unsigned char * rxBuffer, unsigned int rxCnt);
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
		unsigned long latency);
void prepare_message_to_PC_calibration(unsigned char * buffer, int encoderDirection);
void prepare_message_to_PC_angle_config(
		unsigned char * txBuffer,
		short angle_setPoint,
		unsigned short angle_averageLen,
		float angle_smoothing,
		float angle_KP,
		float angle_KI,
		float angle_KD,
		int controlLatencyUs,
		bool controlSync);
void prepare_message_to_PC_position_config(
		unsigned char * txBuffer,
		short position_setPoint,
		unsigned short temp,
		float position_smoothing,
		float position_KP,
		float position_KD
		);

#endif /*__COMMUNICATION_WITH_PC_H_*/
