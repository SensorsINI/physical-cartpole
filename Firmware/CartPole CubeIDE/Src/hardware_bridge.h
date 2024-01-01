/*
 * hardware_bridge.h
 *
 *  Created on: 20 Oct 2023
 *      Author: marcinpaluch
 */

#ifndef HARDWARE_BRIDGE_H_
#define HARDWARE_BRIDGE_H_

#include <stdbool.h>

#define CLOCK_FREQ 						72000000
#define PWM_PERIOD_IN_CLOCK_CYCLES      7200
#define MOTOR_FULL_SCALE				PWM_PERIOD_IN_CLOCK_CYCLES-1
#define MOTOR_FULL_SCALE_SAFE           (int)(0.95 * MOTOR_FULL_SCALE + 0.5)

#define UART_BAUD 230400 	// 115200, 128000, 153600, 230400, 460800, 921600, 1500000, 2000000 // Not working for Zynq yet



#include "STM/goniometer_stm.h"

#define Goniometer_Init					Goniometer_Init
#define Goniometer_Read					Goniometer_Read


#include "STM/motor_stm.h"

#define Motor_Init() 		Motor_INIT(PWM_PERIOD_IN_CLOCK_CYCLES)
#define Motor_SetPwmPeriod  Motor_SetPwmPeriod
#define Motor_Stop			Motor_Stop
#define Motor_SetPower 		Motor_SetPower


#include "STM/encoder_stm.h"

#define Encoder_Init				Encoder_Init
#define Encoder_Read				Encoder_Read
#define Encoder_Set_Direction		Encoder_Set_Direction


#include "STM/led_stm.h"

#define Led_Init			Led_Init
#define Led_Switch			Led_Switch


#include "STM/core_cm3.h"

#define enable_irq			__enable_irq
#define disable_irq			__disable_irq


#include "STM/usart.h"

#define PC_Connection_Init()    USART_Init(UART_BAUD, true);
#define Message_SendToPC		USART_SendBuffer
#define Message_GetFromPC		USART_ReceiveAsync


#include "STM/timer.h"


#define Interruput_Init()   	TIMER1_Init(CONTROL_LOOP_PERIOD_MS)
#define GetTimeNow				TIMER1_getSystemTime_Us
#define Interrupt_Set    		TIMER1_SetCallback
#define Interrupt_Unset()	    TIMER1_SetCallback(0)
#define	SetControlUpdatePeriod  TIMER1_ChangePeriod


#include "STM/sys.h"

#define General_Init		General_Init
#define Sleep_ms			Sleep_ms


#include "STM/buttons_and_switches.h"

#define Buttons_And_Switches_Init		Buttons_And_Switches_Init
#define BUTTON_1						KEY_5
#define Button_SetAction				Button_SetAction

// Prototypes of all functions. Just comment all above to check that everything goes indeed through hardware bridge

//void Goniometer_Init(void);
//unsigned short Goniometer_Read(void);
//
//void Motor_Init(void);
//void Motor_SetPwmPeriod(int pwm_period_in_clock_cycles);
//void Motor_Stop(void);
//void Motor_SetPower(int pwm_duty_cycle_in_clock_cycles, int pwm_period_in_clock_cycles);
//
//void Encoder_Init(void);
//short Encoder_Read(void);
//
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
//void Interruput_Init(void);
//unsigned long GetTimeNow();
//typedef void (*TIMER1_Callback)(void);
//void Interrupt_Set(TIMER1_Callback cb);
//void Interrupt_Unset(void);
//void SetControlUpdatePeriod(unsigned int _periodMS);
//
//void General_Init();
//void Sleep_ms(unsigned int ms);
//
//void Buttons_And_Switches_Init(void);
//#define BUTTON_1	1
//typedef void (*KEY_Callback)(void);
//void Button_SetAction(unsigned int key, KEY_Callback cb);


#endif /* HARDWARE_BRIDGE_H_ */
