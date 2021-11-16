#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h"
 /**************************************************************************
作者：平衡小车之家 
淘宝店铺：http://shop114407458.taobao.com/
**************************************************************************/
#define KEY5 PAin(5)   
#define KEY2 PAin(2)
#define KEY7 PAin(7)
#define KEY11 PAin(11)
#define KEY12 PAin(12)
void KEY_Init(void);          //按键初始化
u8 click_N_Double (u8 time);  //单击按键扫描和双击按键扫描
u8 click(void);               //单击按键扫描
u8 Long_Press(void);
#endif 
