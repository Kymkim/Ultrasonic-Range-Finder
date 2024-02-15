// Project1P1.c
// Runs on TM4C123
// Flashes a Red LED using SysTick timer every 2Hz (0.5s) running at 16Mhz clock
// Oliver Cabral
// February 15, 2023

#include "SysTick.h"
#include "PLL.h"
#include "tm4c123gh6pm.h"

extern void EnableInterrupts(void);
extern void WaitForInterrupt(void);

void PORTF_Init(void);
void ToggleLED(void);

int main(void){
	PORTF_Init();							
  PLL_Init();               
  SysTick_Init(&ToggleLED);   
	EnableInterrupts();
  while(1){
		WaitForInterrupt();
	}
}

void PORTF_Init(void)
{
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF; // activate port F
	while ((SYSCTL_RCGC2_R&SYSCTL_RCGC2_GPIOF)!=SYSCTL_RCGC2_GPIOF){};
	
	GPIO_PORTF_DIR_R |= 0x02; // make PF2 out (built-in blue LED)
  GPIO_PORTF_AFSEL_R &= ~0x02;// disable alt funct on PF2
  GPIO_PORTF_DEN_R |= 0x02; // enable digital I/O on PF2
                            // configure PF2 as GPIO
  GPIO_PORTF_PCTL_R = (GPIO_PORTF_PCTL_R&0xFFFFF0FF)+0x00000000;
  GPIO_PORTF_AMSEL_R |= 0x02;   // disable analog functionality on PF  
}

void ToggleLED(void){
	GPIO_PORTF_DATA_R ^= 0x00000002;
}	
