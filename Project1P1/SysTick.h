// SysTick.c
// Runs on TM4C123
// Implements an SysTick timer with Interrupt that triggers every 2Hz
// February 15, 2024
// Oliver Cabral

//Initialize the SysTick timer with the given task
//
//Input: *task = Function to be executed when the SysTick timer triggers.
//Output: None
void SysTick_Init(void(*task)(void));