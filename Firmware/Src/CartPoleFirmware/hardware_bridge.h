#ifndef HARDWARE_BRIDGE_H_
#define HARDWARE_BRIDGE_H_








// Select between firmware for STM and firmware for Zynq
// UNcomment "#define STM" to use STM or UNcomment "#define Zynq" to use Zynq
#define STM
//#define ZYNQ

// The selection of FPGA board only matters if ZYNQ is defined
#ifdef ZYNQ
#define ZYBO_Z720
//#define ZEDBOARD
#endif


//configuration for Zynq
#define USE_EXTERNAL_INTERFACE






// Check that only one condition is defined
// Count the number of defined conditions
#define COUNT_DEFINED (defined(STM) + defined(ZYNQ))

// Check that only one condition is defined
#if COUNT_DEFINED > 1
#error "Both STM and Zynq firmware selected. Only one condition for firmware should be defined."
#endif

#undef COUNT_DEFINED // Optional: Undefine to keep the macro scope limited to this check

// See parameters.c to set values
extern const unsigned int UART_BAUD;
extern unsigned short CONTROL_LOOP_PERIOD_MS;
extern const int MOTOR_PWM_PERIOD_IN_CLOCK_CYCLES;

#ifdef STM

#include <stdbool.h>

#include "STM/goniometer_stm.h"

#define Goniometer_Init					Goniometer_Init
#define Goniometer_Read					Goniometer_Read


#include "STM/motor_stm.h"

#define Motor_Init() 		Motor_INIT(MOTOR_PWM_PERIOD_IN_CLOCK_CYCLES)
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

#define PC_Connection_Init()            PC_Connection_INIT(UART_BAUD);
#define Message_SendToPC		        Message_SendToPC
#define Message_SendToPC_blocking       Message_SendToPC_blocking
#define Message_GetFromPC		        Message_GetFromPC


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
#define BUTTON_2						KEY_X  // Not implemented yet
#define Button_SetAction				Button_SetAction

#include "STM/neural_imitator.h"  // Currently not implemented - just making the code compatible with Zynq
// In future this could be a neural network running on STM

#define Neural_Imitator_Init			Neural_Imitator_Init
#define neural_imitator_cartpole_step	neural_imitator_cartpole_step


#elif defined(ZYNQ)

#include "Zynq/goniometer_zynq.h"

#define Goniometer_Init					Goniometer_Init
#define Goniometer_Read					Goniometer_Read


#include "Zynq/motor_zynq.h"

#define Motor_Init() 		Motor_INIT(MOTOR_PWM_PERIOD_IN_CLOCK_CYCLES)
#define Motor_SetPwmPeriod  Motor_SetPwmPeriod
#define Motor_Stop			Motor_Stop
#define Motor_SetPower 		Motor_SetPower


#include "Zynq/encoder_zynq.h"

#define Encoder_Init				Encoder_Init
#define Encoder_Read				Encoder_Read
#define Encoder_Set_Direction		Encoder_Set_Direction


#include "Zynq/led_zynq.h"

#define Led_Init			Led_Init
#define Led_Switch			Led_Switch


#include "xil_exception.h"

#define enable_irq			Xil_ExceptionEnable
#define disable_irq			Xil_ExceptionDisable


#include "Zynq/usart.h"

#define PC_Connection_Init()    PC_Connection_INIT(UART_BAUD);
#define Message_SendToPC		Message_SendToPC
#define Message_SendToPC_blocking Message_SendToPC_blocking
#define Message_GetFromPC		Message_GetFromPC


#include "Zynq/timer_interrupt.h"


#define Interruput_Init()   	TIMER1_Init(CONTROL_LOOP_PERIOD_MS)
#define GetTimeNow				TIMER1_getSystemTime_Us  // OK
#define Interrupt_Set    		Interrupt_Set
#define Interrupt_Unset()	    Interrupt_Set(0)
#define	SetControlUpdatePeriod  SetControlUpdatePeriod


#include "Zynq/sys.h"
#define General_Init		General_Init
#define Sleep_ms			Sleep_ms


#include "Zynq/buttons_and_switches.h"

#define Buttons_And_Switches_Init		Buttons_And_Switches_Init
#define BUTTON_1						PS_BTN_4
#define BUTTON_2						PS_BTN_5
#define Button_SetAction				Button_SetAction


#include "Zynq/neural_imitator.h"

#define Neural_Imitator_Init			Neural_Imitator_Init
#define neural_imitator_cartpole_step	neural_imitator_cartpole_step

#ifdef USE_EXTERNAL_INTERFACE
#include "Zynq/external_interface.h"
#endif



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

#else
#error "No firmware - neither for STM nor for Zynq - defined, see hardware_bridge.h"
#endif

#endif /* HARDWARE_BRIDGE_H_ */
