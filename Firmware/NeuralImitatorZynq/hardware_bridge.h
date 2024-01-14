/*
 * hardware_bridge.h
 *
 *  Created on: 20 Oct 2023
 *      Author: marcinpaluch
 */

#ifndef HARDWARE_BRIDGE_H_
#define HARDWARE_BRIDGE_H_

#include <stdbool.h>

#define CLOCK_FREQ 						333333343
#define PWM_PERIOD_IN_CLOCK_CYCLES      2500
#define MOTOR_FULL_SCALE				(PWM_PERIOD_IN_CLOCK_CYCLES-1)
#define MOTOR_FULL_SCALE_SAFE           ((int)(0.95 * MOTOR_FULL_SCALE + 0.5))


#define UART_BAUD 230400 	// 115200, 128000, 153600, 230400, 460800, 921600, 1500000, 2000000 // Not working for Zynq yet


#include "Zynq/led_zynq.h"

#define Led_Init			Led_Init
#define Led_Switch			Led_Switch


#include "xil_exception.h"

#define enable_irq			Xil_ExceptionEnable
#define disable_irq			Xil_ExceptionDisable


#include "Zynq/usart.h"

#define PC_Connection_Init()    PC_Connection_INIT(UART_BAUD);
#define Message_SendToPC		Message_SendToPC
#define Message_GetFromPC		Message_GetFromPC


#include "Zynq/sys.h"
#define General_Init		General_Init
#define Sleep_ms			Sleep_ms


#include "Zynq/buttons_and_switches.h"

#define Buttons_And_Switches_Init		Buttons_And_Switches_Init
#define BUTTON_1						PS_BTN_4
#define Button_SetAction				Button_SetAction


#include "Zynq/fixed_point.hpp"

#include "Zynq/EdgeDRNN/EdgeDRNN_Network.h"

#include "Zynq/HLS4ML/HLS4ML_Network.h"


//*/


// Prototypes of all functions. Just comment all above to check that everything goes indeed through hardware bridge


//void Led_Init(void);
//void Led_Switch(bool en);
//
//void enable_irq();
//void disable_irq();
//
//void PC_Connection_Init(void);
//void Message_SendToPC(const unsigned char * buff, unsigned int len);
//bool Message_GetFromPC(unsigned char * c);
//
//void General_Init();
//void Sleep_ms(unsigned int ms);
//
//void Buttons_And_Switches_Init(void);
//#define BUTTON_1	1
//typedef void (*KEY_Callback)(void);
//void Button_SetAction(unsigned int key, KEY_Callback cb);


#endif /* HARDWARE_BRIDGE_H_ */
