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
#include "PLL.h"
#include "Timer1A.h"
#include "tm4c123gh6pm.h"
#include "SysTick.h"

#define TRIGGER_PIN 		(*((volatile unsigned long *)0x40005200))  // PB7 is the trigger pin	
#define TRIGGER_VALUE 	0x80   // trigger at bit 7
#define ECHO_PIN 				(*((volatile unsigned long *)0x40005100))  // PB6 is the echo pin	
#define ECHO_VALUE 			0x40   // trigger at bit 6
#define MC_LEN 					0.0625 // length of one machine cycle in microsecond for 16MHz clock
#define SOUND_SPEED 		0.0343 // centimeter per micro-second
#define	PORTF_LED				0x07
#define PORTF_RED				0x02
#define PERIOD_PWM			16000000					
#define PERIOD_BLNK			4000000								//0.25ms
#define HALF_DUTY				PERIOD_PWM/2
#define LED							(*((volatile unsigned long *)0x40025038))
#define LED_RED					0x2
#define LED_BLUE				0x4
#define LED_GREEN				0x8
#define LED_ALL					0xE

extern void EnableInterrupts(void);
extern void GPIOPortB_Handler(void);
extern void WaitForInterrupt(void);

void PortB_Init(void);
void PortF_Init(void);
void LEDToggler(void);

static volatile uint8_t done=0;
static volatile uint32_t distance=0;
static volatile uint32_t temp = 0x40;

volatile unsigned long duty_high, duty_low;

int main(void){
	PLL_Init();
	PortB_Init();
	PortF_Init();
	SysTick_Init(&LEDToggler,(NVIC_ST_RELOAD_M/4)-1);
  EnableInterrupts();
	
  while(1){
		done = 0;
		distance = 0;
		TRIGGER_PIN &= ~TRIGGER_VALUE; // send low to trigger
		Timer1A_Wait1us(2);
		TRIGGER_PIN |= TRIGGER_VALUE; // send high to trigger
		Timer1A_Wait1us(11);
		TRIGGER_PIN &= ~TRIGGER_VALUE; // send low to trigger
    while (!done){
			WaitForInterrupt();
		};
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
	if (ECHO_PIN==ECHO_VALUE){  
		Timer1A_Init();	
	}
	else {
		Timer1A_Stop();
		distance = (uint32_t)(Timer1A_Get_MC_Elapsed()*MC_LEN*SOUND_SPEED)/2;		
		done = 1;
	}
  GPIO_PORTB_ICR_R = 0x40;     
}

void PortF_Init(void){
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF;
	while((SYSCTL_RCGC2_R&SYSCTL_RCGC2_GPIOF)!=SYSCTL_RCGC2_GPIOF);
	GPIO_PORTF_DIR_R |= 0x0E;        // make PF3-1 output (PF3-1 built-in LEDs)
  GPIO_PORTF_AFSEL_R &= ~0x0E;     // disable alt funct on PF3-1
  GPIO_PORTF_DEN_R |= 0x0E;        // enable digital I/O on PF3-1
	                                 // configure PF3-1 as GPIO
  GPIO_PORTF_PCTL_R = (GPIO_PORTF_PCTL_R&0xFFFF000F)+0x00000000;
  GPIO_PORTF_AMSEL_R = 0;          // disable analog functionality on PF
}

void LEDToggler(void){
	NVIC_ST_CTRL_R 	&= ~NVIC_ST_CTRL_ENABLE;  
	if (distance < 10){
		//Set reload to be 2Hz
		NVIC_ST_RELOAD_R = PERIOD_BLNK;
		
		LED &= ~LED_BLUE;
		LED &= ~LED_GREEN;
		LED ^= LED_RED;
	}else{
		
		
		if (distance < 20){
			duty_high = PERIOD_PWM*0.9;
		}else if (distance <30){
			duty_high = PERIOD_PWM*0.8;
		}else if (distance <40){
			duty_high = PERIOD_PWM*0.7;
		}else if (distance <50){
			duty_high = PERIOD_PWM*0.6;
		}else if (distance <60){
			duty_high = PERIOD_PWM*0.5;
		}else if (distance <70){
			duty_high = PERIOD_PWM*0.4;
		}
		
		duty_low = PERIOD_PWM-duty_high;
		
		if(LED&LED_ALL){
			LED = ~LED_ALL;
			NVIC_ST_RELOAD_R = duty_low -1;
		}else{
			LED = LED_ALL;
			NVIC_ST_RELOAD_R = duty_high -1;
		}

	}
	NVIC_ST_CURRENT_R = 0;
	NVIC_ST_CTRL_R |= NVIC_ST_CTRL_ENABLE;
}

	

