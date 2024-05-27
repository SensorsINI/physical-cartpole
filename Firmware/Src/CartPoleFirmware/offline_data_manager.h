#ifndef __OFFLINE_DATA_MANAGER_H_
#define __OFFLINE_DATA_MANAGER_H_

void fill_data_buffers(
                float time,
                float angle,
                float position,
                float angleD,
                float positionD,
                float target_equilibrium,
                float target_position,
                float Q);

unsigned short get_buffers_index();
void zero_buffers_index();

void send_buffers(void);

#endif /*__OFFLINE_DATA_MANAGER_H_*/
