#ifndef __LED_H
#define __LED_H
#include "sys.h"
 /**************************************************************************
Author��Minibalance
Our aliexpress��https://minibalance.aliexpress.com
**************************************************************************/
//LED port definition
#define LED PAout(4) // PA4
void LED_Init(void);  //initialization
void Led_Flash(u16 time);
#endif
