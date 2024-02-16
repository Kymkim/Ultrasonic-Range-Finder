// Ultrasonic_Sensor.c
// This program runs on TM4C123.
// This is an example program to show how to interface HC-SR04 Ultrasonic sensor.
// PB6 connects to echo pin to generate edge-triggered interrupt.
// PB7 connects to Ultrasonic sensor trigger pin.
// SysTick timer is used to generate the required timing for trigger pin and measure echo pulse width.
// Global variable "distance" is used to hold the distance in cemtimeter for the obstacles
// in front of the sensor. 
// By Dr. Min He
// December 10th, 2018

#include <stdint.h>
#include "SysTick.h"
#include "PLL.h"
#include "Timer1A.h"
#include "tm4c123gh6pm.h"

#define TRIGGER_PIN 		(*((volatile unsigned long *)0x40005200))  // PB7 is the trigger pin	
#define TRIGGER_VALUE 	0x80   // trigger at bit 7
#define ECHO_PIN 				(*((volatile unsigned long *)0x40005100))  // PB6 is the echo pin	
#define ECHO_VALUE 			0x40   // trigger at bit 6
#define MC_LEN 					0.0625 // length of one machine cycle in microsecond for 16MHz clock
#define SOUND_SPEED 		0.0343 // centimeter per micro-second

extern void EnableInterrupts(void);
extern void GPIOPortB_Handler(void);
void PortB_Init(void);
void Delay(void);

static volatile uint8_t done=0;
static volatile uint32_t distance=0;

int main(void){
	PLL_Init();
	PortB_Init();
  EnableInterrupts();
  while(1){
		done = 0;
		distance = 0;
		TRIGGER_PIN &= ~TRIGGER_VALUE; // send low to trigger
		SysTick_Wait1us(2);
		TRIGGER_PIN |= TRIGGER_VALUE; // send high to trigger
		SysTick_Wait1us(10);
		TRIGGER_PIN &= ~TRIGGER_VALUE; // send low to trigger
		
    while (!done);
  }
}

void PortB_Init(void){ 
  SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOB;           // 1) activate clock for Port A
	while ((SYSCTL_RCGC2_R&SYSCTL_RCGC2_GPIOB)!=SYSCTL_RCGC2_GPIOB){}; // wait for clock to start
                                    // 2) no need to unlock PA2
  GPIO_PORTB_PCTL_R &= ~0xFF000000; // 3) regular GPIO
  GPIO_PORTB_AMSEL_R &= (uint32_t)~0xC0;      // 4) disable analog function on PA2
  GPIO_PORTB_DIR_R &= ~0x40;        // 5) PB6:echo pin, input
  GPIO_PORTB_DIR_R |= 0x80;         // 5) PB7:trigger pin, output
  GPIO_PORTB_AFSEL_R &= ~0xC0;      // 6) regular port function
  GPIO_PORTB_DEN_R |= 0xC0;         // 7) enable digital port
  GPIO_PORTB_IS_R &= ~0x40;         // PB6 is edge-sensitive
  GPIO_PORTB_IBE_R |= 0x40;         // PB6 is both edges
  GPIO_PORTB_IEV_R &= ~0x40;        // PB6 both edge event
  GPIO_PORTB_ICR_R = 0x40;          // clear flag 6
  GPIO_PORTB_IM_R |= 0x40;          // arm interrupt on PB6
  NVIC_PRI0_R = (NVIC_PRI0_R&0xFFFF1FFF)|0x00006000; // (g) priority 3
  NVIC_EN0_R = 0x00000002;          // (h) enable Port B edge interrupt
}

void GPIOPortB_Handler(void){
	
	if (ECHO_PIN==ECHO_VALUE){  // echo pin rising edge is detected, start timing
		Timer1A_Init();	//Begin counting ticks
	}
	else { // echo pin falling edge is detected, end timing and calculate distance.
    // The following code is based on the fact that the HCSR04 ultrasonic sensor 
    // echo pin will always go low after a trigger with bouncing back
    // or after a timeout. The maximum distance can be detected is 400cm.
		// The speed of sound is approximately 340 meters per second, 
		// or  .0343 c/µS.
    // Distance = (echo pulse width * 0.0343)/2; = ((# of mc)*MC_LEN*SOUND_SPEED)/2
		Timer1A_Stop();
		distance = (uint32_t)(Timer1A_Get_MC_Elapsed()*MC_LEN*SOUND_SPEED)/2;		
		done = 1;
	}
  GPIO_PORTB_ICR_R = 0x40;      // acknowledge flag 6
}
