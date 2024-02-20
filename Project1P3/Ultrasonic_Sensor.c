// Ultrasonic_Sensor.c
// By Oliver Cabral and Jason Chan
// February 19, 2023

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
#define	PORTF_LED				0x07
#define PORTF_RED				0x02
#define PERIOD_PWM			80000					
#define PERIOD_BLNK			4000000								//0.25ms
#define HALF_DUTY				PERIOD_PWM/2
#define LED							(*((volatile unsigned long *)0x40025038))
#define LED_RED					0x2
#define LED_BLUE				0x4
#define LED_GREEN				0x8
#define LED_ALL					0xE

extern void EnableInterrupts(void);
extern void GPIOPortB_Handler(void);
void PortB_Init(void);
void PortF_Init(void);

static volatile uint8_t done=0;
static volatile uint32_t distance=0;

volatile unsigned long duty_high, duty_low;

/*
	Implements a Ultrasonic Sensor measuring tool that flashes LED according the distance
*/
int main(void){
	//Initializes modules...
	PLL_Init();
	PortB_Init();
	PortF_Init();
  EnableInterrupts();
  while(1){
		done = 0;
		distance = 0;
		TRIGGER_PIN &= ~TRIGGER_VALUE; // send low to trigger
		SysTick_Wait1us(2);
		TRIGGER_PIN |= TRIGGER_VALUE; // send high to trigger
		SysTick_Wait1us(10);
		TRIGGER_PIN &= ~TRIGGER_VALUE; // send low to trigger
		
    while (!done);//Wait until measurement is done
		
		//Based on the distance, flash the LED acordingly
		if (distance < 10){
			//Flash Red with fixed 2hz
			SysTick_Start();
			SysTick_Wait(PERIOD_BLNK);
			LED &= ~LED_BLUE;
			LED &= ~LED_GREEN;
			LED ^= LED_RED;
		}else if (distance < 70){
			
			SysTick_Start();
			//Set duty high based on the distance. 
			if (distance < 20){
				duty_high = PERIOD_PWM*0.9;		//90%
			}else if (distance <30){
				duty_high = PERIOD_PWM*0.8;		//80%
			}else if (distance <40){
				duty_high = PERIOD_PWM*0.7;		//70%
			}else if (distance <50){
				duty_high = PERIOD_PWM*0.6;		//60%
			}else if (distance <60){
				duty_high = PERIOD_PWM*0.5;		//50%
			}else if (distance <70){
				duty_high = PERIOD_PWM*0.4;		//40%
			}
			
			duty_low = PERIOD_PWM-duty_high; //Duty low can be found by subtracting period with duty high
			
			//If LEDs are on... Turn it off. Else turn it on
			if(LED&LED_ALL){
				LED = ~LED_ALL;
				//Duty low phase
				SysTick_Wait(duty_low -1);
			}else{
				LED = LED_ALL;
				//Duty high phase
				SysTick_Wait(duty_high -1);
			}
		
		//If outside the range. Turn off
		}else{
			SysTick_Stop();
			LED = ~LED_ALL;
		}
		
  }
}

/*
	Initialize PORTB with PB6 as input for the Echo Pin, PB7 as output for the Trigger Pin
	Enable Interrupts for PB6 to catch Echo pin signals
*/
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

/*
	Interrupt service routine for PortB. Measures of the ultrasonic sensor based on the Timer1A
*/
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

/*
	Initialize PortF for controlling the LEDs
*/
void PortF_Init(void)
{
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF; // activate port F
	while ((SYSCTL_RCGC2_R&SYSCTL_RCGC2_GPIOF)!=SYSCTL_RCGC2_GPIOF){};
	
	GPIO_PORTF_DIR_R |= LED_ALL; // make PF2 out (built-in blue LED)
  GPIO_PORTF_AFSEL_R &= ~LED_ALL;// disable alt funct on PF2
  GPIO_PORTF_DEN_R |= LED_ALL; // enable digital I/O on PF2
                            // configure PF2 as GPIO
  GPIO_PORTF_PCTL_R = (GPIO_PORTF_PCTL_R&0xFFFFF0FF)+0x00000000;
  GPIO_PORTF_AMSEL_R |= LED_ALL;   // disable analog functionality on PF  
}

