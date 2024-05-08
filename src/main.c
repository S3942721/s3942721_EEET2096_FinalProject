
/********************************************
*			STM32F439 Main								  			*
*			Developed for the STM32								*
*			Startup Author: Dr. Glenn Matthews		*
*			Project Authors: Samuel Griffiths 		*
* 										 Cameron Lindsay			*
*			Source File														*
********************************************/

#include <stdint.h>
#include "boardSupport.h"
#include "main.h"

/*
	Methids/Functions/Subroutines we need:
	Config methods
		- RCC
		- GPIO
		- Timer
			- 1 Hz UART timer
			- Possibly more?
		- U(S)ART
		- ADC
	Transmit UART
		- 7 body characters
	Listen for UART
		- 2 body character
	Check GPIO input (switches)
	Update GPIO output (LEDs)
		- ensure only one of Cooling or Heating are active, never both
*/


//******************************************************************************//
// Function: main()
// Input : None
// Return : None
// Description : Entry point into the application.
// *****************************************************************************//
int main(void)
{
	// Bring up the GPIO for the power regulators.
	boardSupport_init();
	
  while (1)
  {
	
  }
} 

