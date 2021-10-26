#ifndef __ADC_H
#define __ADC_H	
#include "sys.h"
 /**************************************************************************
Author£ºMinibalance
Our aliexpress£ºhttps://minibalance.aliexpress.com
**************************************************************************/
#define Battery_Ch 6
u16 Get_Adc(u8 ch);
int Get_battery_volt(void);   
void Angle_Adc_Init(void);   
void  Baterry_Adc_Init(void);
u16 Get_Adc_Average(u8 ch,u8 times);
#endif 















