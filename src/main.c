
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

