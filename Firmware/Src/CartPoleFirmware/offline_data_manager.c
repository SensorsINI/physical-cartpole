#include "communication_with_PC.h"
#include "hardware_bridge.h"




#ifdef ZYNQ
#define OFFLINE_BUFFER_SIZE 20000 + 7 // 20s at 1kHz, *29 for total size of all buffers, that is 0.58MB; 5 bytess for SOF, command code, message length and CRC
#define OFFLINE_BUFFER_SIZE_FLOAT 4*(OFFLINE_BUFFER_SIZE-7) + 7
#elif defined(STM)
// TODO: Minimal buffer size. It must be smaller than Zynq as STM has less RAM.
// I don't need it so I am not searching what would be a reasonable value for STM
#define OFFLINE_BUFFER_SIZE 1 + 7
#define OFFLINE_BUFFER_SIZE_FLOAT 4*(OFFLINE_BUFFER_SIZE-7) + 7
#endif

//static float time_Buffer[OFFLINE_BUFFER_SIZE];
//
//static float angle_Buffer[OFFLINE_BUFFER_SIZE];
//static float position_Buffer[OFFLINE_BUFFER_SIZE];
//static float angleD_Buffer[OFFLINE_BUFFER_SIZE];
//static float positionD_Buffer[OFFLINE_BUFFER_SIZE];
//
//static signed char target_equilibrium_Buffer[OFFLINE_BUFFER_SIZE];
//static float target_position_Buffer[OFFLINE_BUFFER_SIZE];
//
//static float Q_Buffer[OFFLINE_BUFFER_SIZE]; // Corrected

static unsigned char time_Buffer[OFFLINE_BUFFER_SIZE_FLOAT];

static unsigned char angle_Buffer[OFFLINE_BUFFER_SIZE_FLOAT];
static unsigned char position_Buffer[OFFLINE_BUFFER_SIZE_FLOAT];
static unsigned char angleD_Buffer[OFFLINE_BUFFER_SIZE_FLOAT];
static unsigned char positionD_Buffer[OFFLINE_BUFFER_SIZE_FLOAT];

static unsigned char target_equilibrium_Buffer[OFFLINE_BUFFER_SIZE];
static unsigned char target_position_Buffer[OFFLINE_BUFFER_SIZE_FLOAT];

static unsigned char Q_Buffer[OFFLINE_BUFFER_SIZE_FLOAT];

unsigned short offline_buffers_index = 0;

void fill_data_buffers(
                float time,
                float angle,
                float position,
                float angleD,
                float positionD,
                float target_equilibrium,
                float target_position,
                float Q
				)
{

	int index_float = 4*offline_buffers_index;

	*((float *)&time_Buffer[index_float+6]) = time;

    *((float *)&angle_Buffer[index_float+6]) = angle;
    *((float *)&position_Buffer[index_float+6]) = position;
    *((float *)&angleD_Buffer[index_float+6]) = angleD;
    *((float *)&positionD_Buffer[index_float+6]) = positionD;

    *((float *)&target_position_Buffer[index_float+6]) = target_position;
    *((float *)&Q_Buffer[index_float+6]) = Q;

    signed char te_char = (signed char)(target_equilibrium);
    *((signed char *)&target_equilibrium_Buffer[offline_buffers_index+6]) = te_char;

    if (offline_buffers_index < OFFLINE_BUFFER_SIZE-1)
    {
        ++offline_buffers_index;
    }
}

unsigned short get_buffers_index()
{
    return offline_buffers_index;
}


void zero_buffers_index()
{
    offline_buffers_index = 0;
}

void send_buffer(unsigned char* Buffer, unsigned int message_length);

void send_buffers(void){
	static unsigned char txBuffer[200];

//    Message_SendToPC_blocking((unsigned char*)time_Buffer, (offline_buffers_index+1)*sizeof(float));  //float
//
//    Message_SendToPC_blocking((unsigned char*)angle_Buffer, (offline_buffers_index+1)*sizeof(float));  //float
//    Message_SendToPC_blocking((unsigned char*)position_Buffer, (offline_buffers_index+1)*sizeof(float));  //float
//    Message_SendToPC_blocking((unsigned char*)angleD_Buffer, (offline_buffers_index+1)*sizeof(float));  //float
//    Message_SendToPC_blocking((unsigned char*)positionD_Buffer, (offline_buffers_index+1)*sizeof(float));  //float
//
//    Message_SendToPC_blocking((unsigned char*)target_position_Buffer, (offline_buffers_index+1)*sizeof(float));  //float
//    Message_SendToPC_blocking((unsigned char*)Q_Buffer, (offline_buffers_index+1)*sizeof(float));  //float
//
//    Message_SendToPC_blocking((unsigned char*)target_equilibrium_Buffer, offline_buffers_index+1); //char
	unsigned int index_float = 4*(offline_buffers_index);
    unsigned int message_length = index_float+7;

    send_buffer(time_Buffer, message_length);

    send_buffer(angle_Buffer, message_length);
    send_buffer(angleD_Buffer, message_length);
    send_buffer(position_Buffer, message_length);
    send_buffer(positionD_Buffer, message_length);

    send_buffer(target_position_Buffer, message_length);
    send_buffer(Q_Buffer, message_length);

    message_length = offline_buffers_index+7;
    send_buffer(target_equilibrium_Buffer, message_length);

//    prepare_buffer_to_send_long(time_Buffer, CMD_TRANSFER_BUFFERS, message_length);
//
//    unsigned short sleep_between_transfers_ms = 100;
//    prepare_buffer_to_send_long(time_Buffer, CMD_TRANSFER_BUFFERS, message_length);
//    Message_SendToPC_blocking(time_Buffer, message_length);  //float
//    Sleep_ms(sleep_between_transfers_ms);  // Does not effect significantly user experience for these long transfers, but makes sure PC can prepare for the next one
//
//    Message_SendToPC_blocking(angle_Buffer, message_length);  //float
//    Sleep_ms(sleep_between_transfers_ms);
//    Message_SendToPC_blocking(angleD_Buffer, message_length);  //float
//    Sleep_ms(sleep_between_transfers_ms);
//    Message_SendToPC_blocking(position_Buffer, message_length);  //float
//    Sleep_ms(sleep_between_transfers_ms);
//    Message_SendToPC_blocking(positionD_Buffer, message_length);  //float
//    Sleep_ms(sleep_between_transfers_ms);
//
//    Message_SendToPC_blocking(target_position_Buffer, message_length);  //float
//    Sleep_ms(sleep_between_transfers_ms);
//    Message_SendToPC_blocking(Q_Buffer, message_length);  //float
//    Sleep_ms(sleep_between_transfers_ms);

//    Message_SendToPC_blocking(target_equilibrium_Buffer, offline_buffers_index); //char

    zero_buffers_index();
}


void send_buffer(unsigned char* Buffer, unsigned int message_length) {
    unsigned short sleep_between_transfers_ms = 100;  // Does not effect significantly user experience for these long transfers, but makes sure PC can prepare for the next one

    prepare_buffer_to_send_long(Buffer, CMD_TRANSFER_BUFFERS, message_length);
    Message_SendToPC_blocking(Buffer, message_length);
    Sleep_ms(sleep_between_transfers_ms);
}
