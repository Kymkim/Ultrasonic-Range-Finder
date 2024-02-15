// SysTick.c
// Runs on TM4C123
// Implements an SysTick timer with Interrupt that triggers every 2Hz
// February 15, 2024
// Oliver Cabral

#include "tm4c123gh6pm.h"

void (*SysTickTask)(void);		//User defined function

//Initialize the SysTick timer with the given task
//
//Input: *task = Function to be executed when the SysTick timer triggers.
void SysTick_Init(void(*task)(void)){
	SysTickTask = task;
  NVIC_ST_CTRL_R 		= 0x00000000;                   
  NVIC_ST_RELOAD_R 	= (NVIC_ST_RELOAD_M/2)-1;  
  NVIC_ST_CURRENT_R = 0x00000000;          
  NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R&0x1FFFFFFF)|0x40000000; // priority 2
                              // enable SysTick with core clock and interrupts
  NVIC_ST_CTRL_R = 0x07;       // 111->7
}

// When SysTick triggers, flip the color of the LED
void SysTick_Handler(void){
	(*SysTickTask)();
	//GPIO_PORTF_DATA_R ^= 0x00000002;
}