// SysTickTestMain.c
// Runs on LM4F120/TM4C123
// Test the SysTick functions by activating the PLL, initializing the
// SysTick timer, and flashing an LED at a constant rate.
// Daniel Valvano
// September 12, 2013
// Modified by Min He and migrate to keil5

#include "SysTick.h"
#include "PLL.h"
#include "tm4c123gh6pm.h"

void PORTF_Init(void);

int main(void){
	PORTF_Init();							// PF2(Blue LED) is an output for debugging
  PLL_Init();               // set system clock to 50 MHz
  SysTick_Init();           // initialize SysTick timer
  while(1){
    GPIO_PORTF_DATA_R = GPIO_PORTF_DATA_R^0x04; // toggle PF2
//    SysTick_Wait(1);        // approximately 720 ns
//    SysTick_Wait(2);        // approximately 720 ns
//    SysTick_Wait(10000);    // approximately 0.2 ms
    SysTick_Wait10ms(10);    // approximately 20*10 ms
  }
}

void PORTF_Init(void)
{
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF; // activate port F
	while ((SYSCTL_RCGC2_R&SYSCTL_RCGC2_GPIOF)!=SYSCTL_RCGC2_GPIOF){};
	
	GPIO_PORTF_DIR_R |= 0x04; // make PF2 out (built-in blue LED)
  GPIO_PORTF_AFSEL_R &= ~0x04;// disable alt funct on PF2
  GPIO_PORTF_DEN_R |= 0x04; // enable digital I/O on PF2
                            // configure PF2 as GPIO
  GPIO_PORTF_PCTL_R = (GPIO_PORTF_PCTL_R&0xFFFFF0FF)+0x00000000;
  GPIO_PORTF_AMSEL_R |= 0x04;   // disable analog functionality on PF  
}
