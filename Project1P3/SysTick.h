// SysTick.h
// Runs on TM4C123
// By Dr. Min He
// December 10th, 2018
 
#include <stdint.h>

// Time delay using busy wait.
void SysTick_Wait1us(uint8_t delay);
void SysTick_Start(void);
void SysTick_Stop(void);
uint32_t SysTick_Get_MC_Elapsed(void);
void SysTick_Wait(unsigned long delay);