#include "communication_with_PC.h"
#include <string.h>


int get_command_from_PC_message(unsigned char * rxBuffer, unsigned int* rxCnt){
	unsigned int			i;
	unsigned int 			idx;
	unsigned int			pktLen;
	int current_command = CMD_DO_NOTHING;
	// Buffer should have at least 4 bytes
	if (*rxCnt >= 4)
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
				if (*rxCnt >= pktLen)
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

							case CMD_RUN_HARDWARE_EXPERIMENT:
                            {
                                if (pktLen == 4)
                                {
                                    current_command = CMD_RUN_HARDWARE_EXPERIMENT;
                                }
                                break;
                            }


                            case CMD_TRANSFER_BUFFERS:
                            {
                                if (pktLen == 4)
                                {
                                    current_command = CMD_TRANSFER_BUFFERS;
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

							case CMD_PC_CONTROL_MODE:
							{
								if (pktLen == 5)
								{
									current_command = CMD_PC_CONTROL_MODE;
								}
								break;
							}

							case CMD_SET_PID_CONFIG:
							{
								if (pktLen == 34)
								{
									current_command = CMD_SET_PID_CONFIG;
								}
								break;
							}

							case CMD_GET_PID_CONFIG:
							{
								if (pktLen == 4)
								{
									current_command = CMD_GET_PID_CONFIG;
								}
								break;
							}

							case CMD_SET_CONTROL_CONFIG:
							{
								// 16 bytes: period/sync/hanging/avg/motor/timesteps
								// 17 bytes: same plus force_angle_hanging (apply hanging even if BTN0/QSPI locked it)
								if (pktLen == 16 || pktLen == 17)
								{
									current_command = CMD_SET_CONTROL_CONFIG;
								}
								break;
							}

							case CMD_GET_CONTROL_CONFIG:
							{
								if (pktLen == 4)
								{
									current_command = CMD_GET_CONTROL_CONFIG;
								}
								break;
							}

							case CMD_SET_MOTOR:
							{
								if (pktLen == 8)
								{
									current_command = CMD_SET_MOTOR;
								}
								break;
							}

							case CMD_SET_TARGET_POSITION:
							{
								if (pktLen == 8)
								{
									current_command = CMD_SET_TARGET_POSITION;
								}
								break;
							}

							case CMD_SET_TARGET_EQUILIBRIUM:
							{
								if (pktLen == 8)
								{
									current_command = CMD_SET_TARGET_EQUILIBRIUM;
								}
								break;
							}

							case CMD_COLLECT_RAW_ANGLE:
							{
								if (pktLen == 8)
								{
									current_command = CMD_COLLECT_RAW_ANGLE;
								}
								break;
							}

							case CMD_SET_ANGLE_FILTER:
							{
								if (pktLen == 8)
								{
									current_command = CMD_SET_ANGLE_FILTER;
								}
								break;
							}

							case CMD_SET_SECLOC_CONFIG:
							{
								// float log_base, int32 ref_period_ticks, float dead_ang, float dead_pos
								if (pktLen == 20)
								{
									current_command = CMD_SET_SECLOC_CONFIG;
								}
								break;
							}

							case CMD_GET_SECLOC_INFO:
							{
								if (pktLen == 4)
								{
									current_command = CMD_GET_SECLOC_INFO;
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
			for (; idx < *rxCnt; idx++)
			{
				if (rxBuffer[idx] == SERIAL_SOF)
				{
					break;
				}
			}

			*rxCnt -= idx;	// This is not correctly modified!
			for (i = 0; i < *rxCnt; i++)
			{
				rxBuffer[i] = rxBuffer[idx+i];
			}
		}
	}

	return current_command;
}

void prepare_message_to_PC_state(
		unsigned char * buffer,
		unsigned short message_len,
		int angle,
		float angleD_unprocessed,
		short position,
		float target_position,
		float target_equilibrium,
		int motor_command,
		int invalid_step,
		unsigned long time_difference_between_measurement,
		unsigned long long time_current_measurement,
		unsigned long latency,
		unsigned short	latency_violation,
		unsigned char secloc_flags
		){

	buffer[ 0] = SERIAL_SOF;
	buffer[ 1] = CMD_STATE;
	buffer[ 2] = message_len;
	serial_put_i16(&buffer[3], (short)angle);
	serial_put_f32(&buffer[5], angleD_unprocessed);
	serial_put_i16(&buffer[9], position);
	serial_put_f32(&buffer[11], target_position);
	serial_put_i16(&buffer[15], (short)motor_command);
	buffer[17] = (unsigned char)invalid_step;
	serial_put_u32(&buffer[18], (unsigned int)time_difference_between_measurement);
	memcpy(&buffer[22], &time_current_measurement, sizeof(time_current_measurement));
	serial_put_u16(&buffer[30], (unsigned short)(latency / 10));
	serial_put_u16(&buffer[32], latency_violation);
	buffer[34] = secloc_flags;
	serial_put_f32(&buffer[35], target_equilibrium);
	// latency maximum: 10 * 65'535 Us = 653ms
	buffer[message_len-1] = crc(buffer, message_len-1);
}

void prepare_message_to_PC_secloc_info(
		unsigned char * buffer,
		unsigned char backend,
		unsigned char pl_available,
		unsigned int shadow_mismatch_count,
		unsigned int pl_update_count,
		unsigned int pl_nn_wait_cycles,
		unsigned int pl_fault_count
		){

	buffer[ 0] = SERIAL_SOF;
	buffer[ 1] = CMD_GET_SECLOC_INFO;
	buffer[ 2] = 22;
	buffer[ 3] = backend;
	buffer[ 4] = pl_available;
	serial_put_u32(&buffer[ 5], shadow_mismatch_count);
	serial_put_u32(&buffer[ 9], pl_update_count);
	serial_put_u32(&buffer[13], pl_nn_wait_cycles);
	serial_put_u32(&buffer[17], pl_fault_count);
	buffer[21] = crc(buffer, 21);
}

void prepare_message_to_PC_calibration(unsigned char * buffer, int encoderDirection){
	buffer[ 0] = SERIAL_SOF;
	buffer[ 1] = CMD_CALIBRATE;
	buffer[ 2] = 5;
	*((signed char *)&buffer[3]) = (signed char)encoderDirection;
	buffer[4] = crc(buffer, 4);
}

void send_information_experiment_done(unsigned char * buffer, unsigned short experiment_length){

	buffer[ 0] = SERIAL_SOF;
	buffer[ 1] = CMD_RUN_HARDWARE_EXPERIMENT;
	buffer[ 2] = 6;
	*((unsigned short *)&buffer[3]) = (unsigned short)experiment_length;
	buffer[5] = crc(buffer, 5);
}

void prepare_message_to_PC_control_config(
		unsigned char * txBuffer,
		unsigned short polling_period,
		bool controlSync,
		float angle_hanging,
		unsigned short angle_averageLen,
		bool correct_motor_dynamics,
		unsigned short timesteps_for_derivative,
		bool hanging_set_on_chip
		){

	txBuffer[ 0] = SERIAL_SOF;
	txBuffer[ 1] = CMD_GET_CONTROL_CONFIG;
	txBuffer[ 2] = 17;
	serial_put_u16(&txBuffer[ 3], polling_period);
	txBuffer[ 5] = controlSync ? 1 : 0;
	serial_put_f32(&txBuffer[6], angle_hanging);
	serial_put_u16(&txBuffer[10], angle_averageLen);
	txBuffer[12] = correct_motor_dynamics ? 1 : 0;
	serial_put_u16(&txBuffer[13], timesteps_for_derivative);
	txBuffer[15] = hanging_set_on_chip ? 1 : 0;
	txBuffer[16] = crc(txBuffer, 16);

}
