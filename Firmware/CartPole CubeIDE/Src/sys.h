#ifndef __SYS_H_
#define __SYS_H_

#define STM32F10X_MD
#include "stm32f10x.h"
#include <stdbool.h>

#define SYS_JTAG_SWD_DISABLE    0x02
#define SYS_SWD_ENABLE          0x01
#define SYS_JTAG_SWD_ENABLE     0x00

#define SYS_GPIO_A              0
#define SYS_GPIO_B              1
#define SYS_GPIO_C              2
#define SYS_GPIO_D              3
#define SYS_GPIO_E              4
#define SYS_GPIO_F              5
#define SYS_GPIO_G              6 

#define SYS_TRIG_FALLING        1
#define SYS_TRIG_RISING         2
#define SYS_TRIG_RISE_FALL      3

#define GPIOA_ODR_ADDR          (GPIOA_BASE+12) // 0x4001080C 
#define GPIOB_ODR_ADDR          (GPIOB_BASE+12) // 0x40010C0C 
#define GPIOC_ODR_ADDR          (GPIOC_BASE+12) // 0x4001100C 
#define GPIOD_ODR_ADDR          (GPIOD_BASE+12) // 0x4001140C 
#define GPIOE_ODR_ADDR          (GPIOE_BASE+12) // 0x4001180C 
#define GPIOF_ODR_ADDR          (GPIOF_BASE+12) // 0x40011A0C    
#define GPIOG_ODR_ADDR          (GPIOG_BASE+12) // 0x40011E0C    

#define GPIOA_IDR_ADDR          (GPIOA_BASE+8) // 0x40010808 
#define GPIOB_IDR_ADDR          (GPIOB_BASE+8) // 0x40010C08 
#define GPIOC_IDR_ADDR          (GPIOC_BASE+8) // 0x40011008 
#define GPIOD_IDR_ADDR          (GPIOD_BASE+8) // 0x40011408 
#define GPIOE_IDR_ADDR          (GPIOE_BASE+8) // 0x40011808 
#define GPIOF_IDR_ADDR          (GPIOF_BASE+8) // 0x40011A08 
#define GPIOG_IDR_ADDR          (GPIOG_BASE+8) // 0x40011E08 
 
#define BITBAND(addr, bitnum)   ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)          *((volatile unsigned long *)(addr)) 
#define BIT_ADDR(addr, bitnum)  MEM_ADDR(BITBAND(addr, bitnum)) 

#define PA_OUT(n)               BIT_ADDR(GPIOA_ODR_ADDR,n) 
#define PA_IN(n)                BIT_ADDR(GPIOA_IDR_ADDR,n) 

#define PB_OUT(n)               BIT_ADDR(GPIOB_ODR_ADDR,n) 
#define PB_IN(n)                BIT_ADDR(GPIOB_IDR_ADDR,n) 

#define PC_OUT(n)               BIT_ADDR(GPIOC_ODR_ADDR,n) 
#define PC_IN(n)                BIT_ADDR(GPIOC_IDR_ADDR,n) 

#define PD_OUT(n)               BIT_ADDR(GPIOD_ODR_ADDR,n) 
#define PD_IN(n)                BIT_ADDR(GPIOD_IDR_ADDR,n) 

#define PE_OUT(n)               BIT_ADDR(GPIOE_ODR_ADDR,n) 
#define PE_IN(n)                BIT_ADDR(GPIOE_IDR_ADDR,n)

#define PF_OUT(n)               BIT_ADDR(GPIOF_ODR_ADDR,n)
#define PF_IN(n)                BIT_ADDR(GPIOF_IDR_ADDR,n)

#define PG_OUT(n)               BIT_ADDR(GPIOG_ODR_ADDR,n) 
#define PG_IN(n)                BIT_ADDR(GPIOG_IDR_ADDR,n)

void SYS_Init(void);
void SYS_JTAG_Set(unsigned int mode);
void SYS_NVIC_Init(unsigned int priority, unsigned int subPriority, unsigned int channel, unsigned int group);
void SYS_EXTI_Config(unsigned int port, unsigned int bit, unsigned int trig);
void SYS_DelayUS(unsigned int n);
void SYS_DelayMS(unsigned int ms);

#define POLOLU_MOTOR // define if using replacement Pololu motor with reversed polarity; see also encoder.c

#endif /*__SYS_H_*/
