#ifndef __MOTOR_H
#define __MOTOR_H
#include <sys.h>	 
  /**************************************************************************
Author��Minibalance
Our aliexpress��https://minibalance.aliexpress.com
**************************************************************************/
#define PWMA   TIM3->CCR4    //�����ڽ�ʹ��һ·���� ʹ�õ�A·
#define AIN2   PBout(12)     //�����ڽ�ʹ��һ·���� ʹ�õ�A·
#define AIN1   PBout(13)     //�����ڽ�ʹ��һ·���� ʹ�õ�A·
#define BIN1   PBout(14)     //Ԥ�� ����
#define BIN2   PBout(15)     //Ԥ�� ����
#define PWMB   TIM3->CCR3    //Ԥ�� ����
void MiniBalance_PWM_Init(u16 arr,u16 psc);
void MiniBalance_Motor_Init(void);
#endif
