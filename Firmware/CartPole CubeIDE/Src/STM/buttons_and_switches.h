#ifndef __BUTTONS_AND_SWITCHES_H_
#define __BUTTONS_AND_SWITCHES_H_

#include "sys.h"

// All keys are on Port A
#define KEY_2           2
#define KEY_5           5
#define KEY_7           7
#define KEY_11          11
#define KEY_12          12

typedef void (*KEY_Callback)(void);

void Buttons_And_Switches_Init(void);
void Button_SetAction(unsigned int key, KEY_Callback cb);

#endif /*__BUTTONS_AND_SWITCHES_H_*/
