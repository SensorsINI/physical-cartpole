#ifndef __ENCODER_H
#define __ENCODER_H
#include <sys.h>	 
  /**************************************************************************
Author��Minibalance
Our aliexpress��https://minibalance.aliexpress.com
**************************************************************************/
#define ENCODER_TIM_PERIOD (u16)(65535)   //���ɴ���65535 ��ΪF103�Ķ�ʱ����16λ�ġ�
void Encoder_Init_TIM2(void);
void Encoder_Init_TIM4(void);
int Read_Encoder(u8 TIMX);
void TIM4_IRQHandler(void);
void TIM2_IRQHandler(void);
#endif
