#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h"
 /**************************************************************************
���ߣ�ƽ��С��֮�� 
�Ա����̣�http://shop114407458.taobao.com/
**************************************************************************/
#define KEY5 PAin(5)   
#define KEY2 PAin(2)
#define KEY7 PAin(7)
#define KEY11 PAin(11)
#define KEY12 PAin(12)
void KEY_Init(void);          //������ʼ��
u8 click_N_Double (u8 time);  //��������ɨ���˫������ɨ��
u8 click(void);               //��������ɨ��
u8 Long_Press(void);
#endif 
