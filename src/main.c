
/********************************************
*			STM32F439 Main								  			*
*			Developed for the STM32								*
*			Startup Author: Dr. Glenn Matthews		*
*			Project Authors: Samuel Griffiths 		*
* 										 Cameron Lindsay			*
*			Source File														*
********************************************/

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "boardSupport.h"
#include "main.h"

// Timer Constants
#define CPU_CLOCK 84000000 // 84MHz
#define TIM6_PRESCALER 1200 // TODO: Calculate this value
#define TIM7_PRESCALER 1200 // TODO: Calculate this value
#define DEFAULT_TIMER_COUNT 100
#define TIM6_RATE CPU_CLOCK/(TIM6_PRESCALER + 1) // 84MHz bus clock divided by prescaler gets the number of timer ticks per second, +1 Accounts for hardware adding 1 for prescaler
#define TIM7_RATE CPU_CLOCK/(TIM7_PRESCALER + 1) // 84MHz bus clock divided by prescaler gets the number of timer ticks per second, +1 Accounts for hardware adding 1 for prescaler

#define USART_TIMEOUT 1000 // timeout for USART
#define ADC_CONVERSION_TIMEOUT 1000 // timeout for ADC conversion
#define DEFAULT_TIMEOUT 2 // 2s timeout for general use

#define INCOMMING_BUFFER_SIZE 3 // Size of the buffer for incomming data from UART
#define OUTGOING_BUFFER_SIZE 9 // Size of the buffer for outgoing data via UART

// Define masks for ALL LEDs on the board
#define GPIOA_LED_MASK (GPIO_ODR_OD3 | GPIO_ODR_OD8 | GPIO_ODR_OD9 | GPIO_ODR_OD10)
#define GPIOB_LED_MASK (GPIO_ODR_OD0 | GPIO_ODR_OD1 | GPIO_ODR_OD8)
#define GPIOF_LED_MASK (GPIO_ODR_OD8)

#define ASCII_CR 0x0D
#define ASCII_LF 0x0A
#define ASCII_AT 0x40

#define TEMP_DIFF_MIN 0.1 // Minimum temperature difference to stop auto heating or cooling when usart control is active

/*
  Methids/Functions/Subroutines we need:
	Config methods
		- RCC
		- GPIO
      - GPIOA
        - GPIOA3 (Light Output) [LED0]
        - GPIOA8 (Fan Output) [LED1]
        - GPIOA9 (Light Switch) [SW3]
        - GPIOA10 (Light Intensity Sensor) [SW4]
      - GPIOB
        - GPIOB0 (Fan Switch) [SW6]
        - GPIOB1 (Fan Control Output) [LED2]
        - GPIOB8 (Heater Output) [LED6]
        - GPIOB10 (UART3 Transmit)
        - GPIOB11 (UART3 Receive)
      - GPIOF
        - GPIOF8 (Cooling Output)
		- Timer (2 timers, timer 6 for 1Hz, timer 7 for 50ms)
			- Timer for 1 Hz UART
			- Timer for 20s timeout (Coult be done with a counter in the 1Hz timer)
      - Timer for 50ms button debounce/registration timer
		- U(S)ART
		- ADC
      - ADC3 (Temperature Sensor) [Potentiometer on PF10]

	Transmit UART
		- 7 body characters
      - 1 Header character "@" - 0x40 (ASCII_AT)
      - 5 Temperature characters (E.g. +19.2 or -03.9)
        - 1 Sign character "+" or "-"
        - 2 Integer characters (0-99)
        - 1 Decimal point character "."
        - 1 Decimal character (0-9)
      - 1 Control State Character
        - Bits 0-3:
          - Header bits: 0b0011
        - Bit 4 (a):
          - Cooling output bit:
            - 0: Cooling off
            - 1: Cooling on
        - Bit 5 (b):
          - Heating output bit:
            - 0: Heating off
            - 1: Heating on
        - Bit 6 (c):
          - Fan status bit:
            - 0: Fan off
            - 1: Fan on
        - Bit 7 (d):
          - Light status bit:
            - 0: Light off
            - 1: Light on

	Recieve for UART
		- 2 body character
      - 1 Header character "@" - 0x40 (ASCII_AT)
      - 1 Control State Character
        - Bits 0-3:
          - Header bits: 0b0011
        - Bit 4 (a):
          - Cooling output bit:
            - 0: Cooling off
            - 1: Cooling on
        - Bit 5 (b):
          - Heating output bit:
            - 0: Heating off
            - 1: Heating on
        - Bit 6 (c):
          - Fan status bit:
            - 0: Fan off
            - 1: Fan on
        - Bit 7 (d):
          - Light status bit:
            - 0: Light off
            - 1: Light on

	Check GPIO input (switches)
    - Check for button press
    - Debounce button press (50ms)
    - Register button press

	Update GPIO output (LEDs)

  Climate Control methods:
    - Turn on/off cooling
      - Turn on cooling
      - Turn off cooling
    
    - Turn on/off heating
      - Turn on heating
      - Turn off heating
    
    - Turn on/off fan
      - Turn on fan
      - Turn off fan
    
    - Turn on/off light
      - Turn on light
      - Turn off light

		- ensure only one of Cooling or Heating are active, never both
*/

// Method prototypes
// Configuration methods
void configure_RCC(void);
void configure_GPIOs(void);
void configure_GPIOA(void);
void configure_GPIOB(void);
void configure_GPIOF(void);
void configure_Timers(void);
void configure_TIM6(void);
void configure_TIM7(void);
void configure_USART3(void);
void configure_ADC(void);

// UART Communication methods
void transmit_UART(uint8_t data);
void transmit_Status_Packet(char* data);
int8_t receive_UART(void);
void receive_Status_Packet(void);
char* get_outgoing_string(void);

// Device Control methods
void set_cooling(bool on);
void set_heating(bool on);
void set_fan(bool on);
void set_light(bool on);

// Sensor Readings methods
float get_temperature(void);
int get_ADC_temperature(void);
bool get_light_intensity(void);
bool get_fan_switch(void);
bool get_light_switch(void);
bool get_light_intensity_gpio(void);
bool get_fan_switch_gpio(void);
bool get_light_switch_gpio(void);

// Timer Operations methods
void start_TIM6(uint16_t count);
void start_TIM7(uint16_t count);
bool TIM6_expired(void);
bool TIM7_expired(void);
void wait_For_TIM6(void);
void wait_For_TIM7(void);
void stop_TIM6(void);
void stop_TIM7(void);

// Handle inputs and outputs methods
void handle_cooling_and_heating(void);
bool handle_fan(void);
bool handle_light(void);
void handle_outputs(void);

// Helper methods
unsigned short count_from_rate_TIM6(float rate);
unsigned short count_from_delay_ms_TIM7(int delay_ms);

// Global Variables
bool cooling_output;
bool heating_output;
bool fan_output;
bool light_output;

bool cooling_input;
bool heating_input;
bool fan_input;
bool light_input;

bool fan_timer_active;
int fan_timer_count = 0; // Time count for 20s fan off timer
bool heating_cooling_manual_override_timer_active;
int manual_override_count;

bool debounce_timer_active;

bool light_intensity_sensor;
bool fan_switch;
bool light_switch;
float temperature_input;
float temperature_output;
int adc_temperature;
int incomming_string_index;

bool status_packet_recieved;

int global_timer = 0; // Global timer for 1Hz timer to allow for 20s timeout and UART vs switch preference

// Character array for UART communication, one for incomming string and one for outgoing string
char incomming_string[INCOMMING_BUFFER_SIZE] = {0};
char outgoing_string[OUTGOING_BUFFER_SIZE] = {0};

/**
 * @brief Main function of the program.
 * 
 * This function initializes the GPIO for the power regulators, configures RCC and GPIOs,
 * and then enters an infinite loop where it continuously reads inputs, handles them, and updates GPIO outputs.
 * It also periodically transmits status packets and receives and processes status packets over UART.
 * 
 * @return int The exit status of the program.
 */
int main(void)
{
	// Bring up the GPIO for the power regulators.
	boardSupport_init();

  // Configure the peripherals
  configure_RCC();
  configure_GPIOs();
  configure_Timers();
  configure_USART3();
  configure_ADC();
	
  // Initialise Program State Variables
  // Output is used to control the GPIO outputs
  cooling_output = false;
  heating_output = false;
  fan_output = false;
  light_output = false;

  // Input is used to control the program inputs
  cooling_input = false;
  heating_input = false;
  fan_input = false;
  light_input = false;

  light_intensity_sensor = false;
  fan_switch = false;
  light_switch = false;
  temperature_input = 0.0;
  adc_temperature = 0;

  incomming_string_index = 0;

  status_packet_recieved = false;
  debounce_timer_active = false;
  fan_timer_active = false;
  heating_cooling_manual_override_timer_active = false;

  while (1)
  { 
    // If 1Hz timer has expired then transmit data
    if (TIM6_expired())
    {
      transmit_Status_Packet(outgoing_string);

      uint16_t timer_Count = count_from_rate_TIM6(1.0);
      start_TIM6(timer_Count); // Restart timer

      // Increment global timer
      global_timer++;

      // If fan timer is active then increment count
      if (fan_timer_active)
      {
        fan_timer_count++;
      }
			
			if (heating_cooling_manual_override_timer_active)
			{
				manual_override_count++;
			}
    }

    // Recieve/Check for USART input
    receive_Status_Packet();

    // Check if status frame is recieved and it is valid (First 4 bits in the status frame are 0b0011 and @ is the header character)
    if (status_packet_recieved && (incomming_string[0] == ASCII_AT) && ((incomming_string[1] && 0xF0) == 0x30))
    {
      // Set inputs based on status frame (These are bool so no shifting required, any non 0 value will be true)
      cooling_input = incomming_string[1] & 0x10; // Clear all but 4th bit
      heating_input = incomming_string[1] & 0x20; // Clear all but 5th bit
      fan_input = incomming_string[1] & 0x40; // Clear all but 6th bit
      light_input = incomming_string[1] & 0x80; // Clear all but 7th bit

      // Reset status packet flag and clear current status string
      status_packet_recieved = false;
      incomming_string[0] = '\0';
      incomming_string_index = 0;
    }

    // Read Switches
    light_switch = get_light_switch();
    fan_switch = get_fan_switch();

    // Handle inputs
    handle_cooling_and_heating();
    fan_output = handle_fan();
    light_output = handle_light();
    temperature_output = get_temperature();

    // Receive array of characters to be transmitted
    char* outgoing_string_ptr = get_outgoing_string();

    // Assign outgoing string to the outgoing buffer
    for (int i = 0; i < OUTGOING_BUFFER_SIZE; i++)
    {
      outgoing_string[i] = outgoing_string_ptr[i];
    }
		
		// Handle temperature for auto heating/cooling (If an input has not been given

    // Update GPIO Outputs
    handle_outputs();
  }
}

/**
 * @brief Configures USART3 for communication.
 */
void configure_USART3(void)
{
  // TODO: Verify this configuration with requirements

	// UART is configured at 19200bps, 8 data bits, No Parity, 1 stop bit
	// UART APB1 Bus Clock is 42MHz
	// Baud Rate required is 136.75
	
	// Configure GPIOB MODER for alternate function mode
	GPIOB->MODER &= ~(GPIO_MODER_MODE11_Msk | GPIO_MODER_MODE10_Msk);		// Clear MODE already set for 10 and 11
	GPIOB->MODER |=  (0x02 << GPIO_MODER_MODE11_Pos) | (0x02 << GPIO_MODER_MODE10_Pos); // Set mode for 10 and 11 to be alternate function (0b10)

	// Set the alternate function to be AF7 (USART)
	GPIOB->AFR[1] &= ~(GPIO_AFRH_AFSEL11_Msk | GPIO_AFRH_AFSEL10_Msk);
	GPIOB->AFR[1] |= (0x07 << GPIO_AFRH_AFSEL11_Pos) | (0x07 << GPIO_AFRH_AFSEL10_Pos); // Set mode for 10 and 11 to be alternate function 7 (0b111)

	// Turn on OVER16 16-times over sampling (By clearing the OVER8 bit)
	USART3->CR1 &= ~(USART_CR1_OVER8);
	
	// Set USART Baud Rate
	USART3->BRR &= 0xFFFF0000; // Clear Baud Rate register
	USART3->BRR |= (0x88 << USART_BRR_DIV_Mantissa_Pos | 0x0C << USART_BRR_DIV_Fraction_Pos); // Set Baud rate ~19200 (Actually 19195) (136.75 -> 136 (0x88) and 0.75 * 16 (0xc))
	
	// Set numberof bits per transfer to 8-bits (By clearing M)
	USART3->CR1 &= ~(USART_CR1_M);
	
	// Set number of stop bits to 1
	USART3->CR2 &= ~(USART_CR2_STOP_Msk); // Clear stop bits
	USART3->CR2 |= (0x00 << USART_CR2_STOP_Pos);
	
	// Disable system parity
	USART3->CR1 &= ~(USART_CR1_PCE);
	
	// Set mode to async (clear all clock control bits)
	USART3->CR2 &= ~(USART_CR2_CLKEN | USART_CR2_CPOL | USART_CR2_CPHA);
	
	// Disable hardware flow control
	USART3->CR3 &= ~(USART_CR3_CTSE | USART_CR3_RTSE);
	
	// Enable USART, transmitter and receive sections
	USART3->CR1 |= (USART_CR1_TE | USART_CR1_UE | USART_CR1_RE);
}

/**
 * @brief Configures the ADC for temperature sensing.
 */
void configure_ADC(void)
{
  // Assuming RCC is already configured and enabled for ADC3
  // ADC3 is on PF10 Channel 8 (Page 56/241 of reference manual)

  // Configure to run in single channel mode and in single conversion and sample mode
  // Prescaler is set to /8
  // Resolution is set to 12-bit
  // Conversion time is set to 56 cycles (This is the minimum value for 12-bit resolution)
  // Data alignment is set to right
  
  // Disable battery sensing channel
  ADC123_COMMON->CCR &= ~(ADC_CCR_VBATE);

  // Disable scan mode and set resolution to 12 bits
  ADC3->CR1 &= ~(ADC_CR1_SCAN | (0x00 << ADC_CR1_RES_Pos));

  // Set the data alignment to right and set single mode for conversion and sample
  ADC3->CR2 &= ~(ADC_CR2_ALIGN | ADC_CR2_CONT | ADC_CR2_SWSTART);

  // Set to select channel 8 (PF10) for "temperature" (Potentiometer) sensing
  ADC3->SQR3 &= ~(ADC_SQR3_SQ1_Msk);
  ADC3->SQR3 |= (0x08 << ADC_SQR3_SQ1_Pos);
  ADC3->SQR1 &= ~(ADC_SQR1_L_Msk);

  // Set sample time to 56 cycles
  ADC3->SMPR2 &= ~(ADC_SMPR2_SMP8_Msk); 
  ADC3->SMPR2 |= (0x03 << ADC_SMPR2_SMP8_Pos);

  // Enable ADC3
  ADC3->CR2 |= ADC_CR2_ADON;
}

/**
 * @brief Transmits a byte of data via UART.
 * 
 * @param data The data to be transmitted.
 */
void transmit_UART(uint8_t data)
{
  // Wait for the transmit buffer to be empty or timeout (TIMEOUT NOT YET IMPLEMENTED)
  while ((USART3->SR & USART_SR_TXE) == 0x00)
  {
    // Wait for the transmit buffer to be empty
  }

  // Transmit the data
  USART3->DR = data;
}

/**
 * @brief Transmits a status packet via UART from character array.
 */
void transmit_Status_Packet(char* data)
{
  // Check if data is valid @ is the header character
  if (data[0] == ASCII_AT)
  {
    // Transmit the data
    for (int i = 0; i < OUTGOING_BUFFER_SIZE; i++)
    {
      transmit_UART(data[i]);
    }
  }
}

/**
 * @brief Receives a byte of data via UART.
 * 
 * @return The received data.
 */
int8_t receive_UART(void)
{
  // Assume there is a character to be recieved
  // Recieve and return the character
  // This function will be called when a character is recieved on the USART and will only read one character to avoid being hung up in this method
  int8_t incomming_character = -1;

  // Check if the recieve buffer is not empty
  if ((USART3->SR & USART_SR_RXNE) == 0x01)
  {
    // Read the incomming character
    incomming_character = (volatile int8_t)USART3->DR;
  }
  return incomming_character;
}

/**
 * @brief Receives a status packet via UART.
 */
void receive_Status_Packet(void)
{
  // Assume there is a frame to be recieved and processed
  // Read the single character, append this to the incomming string buffer
  // If the buffer is full then process the frame by setting 
  
  // Read the incomming character and append to the incomming string buffer based on the current position in the string
  // This function will be called when a character is recieved on the USART and will only read one character to avoid being hung up in this method
  // This will incrementally add the latest character to the buffer and check if the buffer is full before signalling that a frame has been recieved

  // Recieve incomming character
  int8_t incomming_character = receive_UART();

  // Check if the incomming character is valid and append to the incomming string buffer
  if ((incomming_string_index < INCOMMING_BUFFER_SIZE) && (incomming_character != -1))
  {
    // Check if character is valid and append to the incomming string buffer
    if ((incomming_string_index == 0 && incomming_character == ASCII_AT) || (incomming_string_index == 1 && (incomming_character & 0xF0) == 0x30) || (incomming_string_index == 2 && incomming_character == ASCII_CR) || (incomming_string_index == 3 && incomming_character == ASCII_LF))
    {
      incomming_string[incomming_string_index] = (char)incomming_character;
      incomming_string_index++;
    }
  }
  else
  {
    // Set flag to process the frame
    status_packet_recieved = true;
  }
}

/**
 * @brief Gets the string to be transmitted via UART.
 */
char *get_outgoing_string(void)
{
  // Format the string to be transmitted based upon the current state of the system
  // 1 Header character "@" - 0x40 (ASCII_AT)
  // 5 Temperature characters (E.g. +19.2 or -03.9)
  // 1 Control State Character (0b0011abcd) (a: Cooling, b: Heating, c: Fan, d: Light)
  // 1 CR character
  // 1 LF character

  // Set header character at the start of the string (0)
  // Temperature is a float, convert to string and split characters into char* array
  // Control State is a byte, convert to string

  // Set header character
  outgoing_string[0] = ASCII_AT;

  // Set temperature characters
  // Convert temperature to string
  char* temperature_string = {0};
  sprintf(temperature_string, "%+05.1f", (double)temperature_output);

  // Set temperature characters
  for (int i = 0; i < 5; i++)
  {
    outgoing_string[i + 1] = temperature_string[i];
  }

  // Set control state character
  // Control State is a byte, convert to string
  outgoing_string[6] = (char)(0x30 | (cooling_output << 3) | (heating_output << 2) | (fan_output << 1) | light_output);

  // Set CR character
  outgoing_string[7] = ASCII_CR;

  // Set LF character
  outgoing_string[8] = ASCII_LF;

  return outgoing_string;
}

/**
 * @brief Configures TIM6 for timing operations.
 */
void configure_TIM6(void)
{
	// Ensure timer is off and all configurations are reset
	TIM6->CR1 &= ~(TIM_CR1_ARPE | TIM_CR1_OPM | TIM_CR1_UDIS | TIM_CR1_CEN);				
	
	// Disable TIM6 interrupts
	TIM6->DIER &= ~(TIM_DIER_UIE);
	
	// Clear prescaler, count and autoreload registers
	TIM6->PSC &= ~(TIM_PSC_PSC_Msk);	// Clear Prescaler register
	TIM6->CNT &= ~(TIM_CNT_CNT_Msk);	// Clear count register
	TIM6->ARR &= ~(TIM_ARR_ARR_Msk);	// Clear autoreload register
	
	// Set OPM to on and ARPE
	TIM6->CR1 |= (TIM_CR1_ARPE | TIM_CR1_OPM);
	
	// Set Prescaler
	TIM6->PSC |= (TIM6_PRESCALER << TIM_PSC_PSC_Pos);				// Divide APB1 Clock by TIM6PRESCALER + 1
																													// Thus, at one cycle it will be TIM6RATE Hz (Assuming 82MHz of APB1)

	TIM6->ARR |= (DEFAULT_TIMER_COUNT << TIM_ARR_ARR_Pos); 	// Initialise auto-reload register with default value (what count will auto reload)

	// Enable timer for one cycle to clear and reload psc buffer
	start_TIM6(DEFAULT_TIMER_COUNT);
	wait_For_TIM6();
	stop_TIM6();
}

/**
 * @brief Configures TIM7 for timing operations.
 */
void configure_TIM7(void)
{
  // Ensure timer is off and all configurations are reset
	TIM7->CR1 &= ~(TIM_CR1_ARPE | TIM_CR1_OPM | TIM_CR1_UDIS | TIM_CR1_CEN);				
	
	// Disable TIM7 interrupts
	TIM7->DIER &= ~(TIM_DIER_UIE);
	
	// Clear prescaler, count and autoreload registers
	TIM7->PSC &= ~(TIM_PSC_PSC_Msk);	// Clear Prescaler register
	TIM7->CNT &= ~(TIM_CNT_CNT_Msk);	// Clear count register
	TIM7->ARR &= ~(TIM_ARR_ARR_Msk);	// Clear autoreload register
	
	// Set OPM to on and ARPE
	TIM7->CR1 |= (TIM_CR1_ARPE | TIM_CR1_OPM);
	
	// Set Prescaler
	TIM7->PSC |= (TIM7_PRESCALER << TIM_PSC_PSC_Pos);				// Divide APB1 Clock by TIM6PRESCALER + 1
																													// Thus, at one cycle it will be TIM6RATE Hz (Assuming 82MHz of APB1)

	TIM7->ARR |= (DEFAULT_TIMER_COUNT << TIM_ARR_ARR_Pos); 	// Initialise auto-reload register with default value (what count will auto reload)

	// Enable timer for one cycle to clear and reload psc buffer
	start_TIM6(DEFAULT_TIMER_COUNT);
	wait_For_TIM6();
	stop_TIM6();
}

/**
 * @brief Configures the RCC (Reset and Clock Control) for system initialization.
 * 
 * This function enables the RCC for Timer 6, Timer 7, and USART3, and enables the GPIOs for GPIOA, GPIOB, and GPIOF.
 */
void configure_RCC(void) //Cam + Sam (Minor fixes and documentation)
{
  // Enable RCC for Timer 6, Timer 7, and USART3
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN | RCC_APB1ENR_TIM7EN | RCC_APB1ENR_USART3EN;						// Enable RCC for Timer 6, Timer 7, and USART3
	RCC->APB1RSTR |= RCC_APB1RSTR_TIM6RST | RCC_APB1RSTR_TIM7RST | RCC_APB1RSTR_USART3RST;		// Reset the peripheral interface
	// Enable RCC for GPIOA, GPIOB, and GPIOF
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOFEN;					// Config to enable GPIOA, B, and F
	RCC->AHB1RSTR |= RCC_AHB1RSTR_GPIOARST | RCC_AHB1RSTR_GPIOBRST | RCC_AHB1RSTR_GPIOFRST;		// Reset control interface
	// Enable RCC for ADC3
  RCC->APB2ENR |= RCC_APB2ENR_ADC3EN;																												// Enable RCC for ADC 3
	RCC->APB2RSTR |= RCC_APB2RSTR_ADCRST;																											// Reset control interface
	__asm("nop");	__asm("nop");
	// Clear reset bits for Timer 6, Timer 7, USART3, GPIOA, GPIOB, GPIOF, and ADC3
  RCC->APB1RSTR &= ~(RCC_APB1RSTR_TIM6RST | RCC_APB1RSTR_TIM7RST | RCC_APB1RSTR_USART3RST);		//Clear reset bit
	RCC->AHB1RSTR &= ~(RCC_AHB1RSTR_GPIOARST | RCC_AHB1RSTR_GPIOBRST | RCC_AHB1RSTR_GPIOFRST);//Clear reset	bit
	RCC->APB2RSTR &= ~(RCC_APB2RSTR_ADCRST);
	__asm("nop");	__asm("nop");
}

/**
 * @brief Configures the GPIOs (General Purpose Input/Output) for system initialization.
 */
void configure_GPIOs(void) //Cam
{
	configure_GPIOA();
	configure_GPIOB();
	configure_GPIOF();
}

/**
 * @brief Configures GPIOA for system initialization.
 * 
 * GPIOA3 - Light Output (LED0)
 * GPIOA8 - Fan Output (LED1)
 * GPIOA9 - Light Switch (SW3)
 * GPIOA10 - Light Intensity Sensor (SW4)
 * 
 */
void configure_GPIOA(void) //Cam (Sam - Bug fixes and documentation/formatting)
{
	// Configure:
	// LED0: PA3 	(Output, Clear Floating Values, Active Low)
	// LED1: PA8 	(Output, Clear Floating Values, Active Low)
	// LED2: PA9 	(Input, Clear Floating Values, Active Low)
	// LED3: PA10 (Input, Clear Floating Values, Active Low)

	// Configure I/O Ports
	// Clear GPIO Modes (Set 0b00 for all bit pairs)
	GPIOA->MODER &= ~(GPIO_MODER_MODER3_Msk | GPIO_MODER_MODER8_Msk | 
										GPIO_MODER_MODER9_Msk | GPIO_MODER_MODER10_Msk);

	// Set LED Modes as Output (Set 0b01 for bit pairs)
	GPIOA->MODER |= ((0x01 << GPIO_MODER_MODER3_Pos) | 
                  (0x01 << GPIO_MODER_MODER8_Pos));

	// Enable Push-Pull Output Mode (Clear Relevant Bits)
	GPIOA->OTYPER &= ~(GPIO_OTYPER_OT3 | GPIO_OTYPER_OT8);

	// Clear Speed Modes (Set 0b00 for bit pairs, by anding ~0b11)
	GPIOA->OSPEEDR &= (unsigned int) (~(0x03 << GPIO_OSPEEDR_OSPEED3_Pos) | 
                                    ~(0x03 << GPIO_OSPEEDR_OSPEED8_Pos) | 
                                    ~(0x03 << GPIO_OSPEEDR_OSPEED9_Pos) | 
                                    ~(0x03 << GPIO_OSPEEDR_OSPEED10_Pos));

	// Set Speed Mode to Medium (Set 0b01 for bit pairs)
	GPIOA->OSPEEDR |= ((unsigned int) (0x01 << GPIO_OSPEEDR_OSPEED3_Pos) | 
										 (unsigned int) (0x01 << GPIO_OSPEEDR_OSPEED8_Pos) | 
										 (unsigned int) (0x01 << GPIO_OSPEEDR_OSPEED9_Pos) | 
										 (unsigned int) (0x01 << GPIO_OSPEEDR_OSPEED10_Pos));

	// Clear Pull-Up-Pull Down Register (Pull-up)
	GPIOA->PUPDR &= (unsigned int) (~(0x01 << GPIO_PUPDR_PUPD3_Pos) | 
                                  ~(0x01 << GPIO_PUPDR_PUPD8_Pos) | 
                                  ~(0x01 << GPIO_PUPDR_PUPD9_Pos) | 
                                  ~(0x01 << GPIO_PUPDR_PUPD10_Pos));

	// Set Output Data Register to TURN OFF LEDS by default
	GPIOA->ODR |= (GPIO_ODR_OD3 | GPIO_ODR_OD8);
}

/**
 * @brief Configures GPIOB for system initialization.
 * 
 * GPIOB0 - Fan Switch (SW6)
 * GPIOB1 - Fan Control Output (LED2)
 * GPIOB8 - Heater Output (LED6)
 * GPIOB10 - UART3 Transmit
 * GPIOB11 - UART3 Receive
 * 
 */
void configure_GPIOB(void) //Cam (Sam - Bug fixes, added functionality and documentation/formatting)
{
  // Configure:
  // LED2: PB1 	(Output, Clear Floating Values, Active Low)
  // LED6: PB8 	(Output, Clear Floating Values, Active Low)
  // SW6: PB0 	(Input, Clear Floating Values, Active Low)
  // UART3 Transmit: PB10 (Alternate Function, Clear Floating Values, Active Low)
  // UART3 Receive: PB11 (Alternate Function, Clear Floating Values, Active Low)

  // Configure I/O Ports
  // Clear GPIO Modes (Set 0b00 for bit pairs)
  GPIOB->MODER &= ~(GPIO_MODER_MODER0_Msk | GPIO_MODER_MODER1_Msk |
										GPIO_MODER_MODER8_Msk | GPIO_MODER_MODER10_Msk 
										| GPIO_MODER_MODER11_Msk);

  // Set LED Modes as Output (Set 0b01 for bit pairs)
  GPIOB->MODER |= ((0x01 << GPIO_MODER_MODER1_Pos) | 
                   (0x01 << GPIO_MODER_MODER8_Pos));

  // Enable Push-Pull Output Mode (Clear Relevant Bits)
  GPIOB->OTYPER &= ~(GPIO_OTYPER_OT1 | GPIO_OTYPER_OT8);

  // Clear Speed Modes (Set 0b00 for bit pairs, by anding ~0b11)
  GPIOB->OSPEEDR &= (unsigned int) (~(0x03 << GPIO_OSPEEDR_OSPEED1_Pos) | 
                                    ~(0x03 << GPIO_OSPEEDR_OSPEED8_Pos) | 
                                    ~(0x03 << GPIO_OSPEEDR_OSPEED10_Pos) | 
                                    ~(0x03 << GPIO_OSPEEDR_OSPEED11_Pos));

  // Set Speed Mode to Medium (Set 0b01 for bit pairs)
  GPIOB->OSPEEDR |= ((unsigned int) (0x01 << GPIO_OSPEEDR_OSPEED1_Pos) | 
                     (unsigned int) (0x01 << GPIO_OSPEEDR_OSPEED8_Pos) | 
                     (unsigned int) (0x01 << GPIO_OSPEEDR_OSPEED10_Pos) | 
                     (unsigned int) (0x01 << GPIO_OSPEEDR_OSPEED11_Pos));

  // Clear Pull-Up-Pull Down Register (Pull-up)
  GPIOB->PUPDR &= (unsigned int) (~(0x01 << GPIO_PUPDR_PUPD1_Pos) | 
                                  ~(0x01 << GPIO_PUPDR_PUPD8_Pos) |
                                  ~(0x01 << GPIO_PUPDR_PUPD10_Pos) |
                                  ~(0x01 << GPIO_PUPDR_PUPD11_Pos));
  
  // Set Output Data Register to TURN OFF LEDS by default
  GPIOB->ODR |= (GPIO_ODR_OD1 | GPIO_ODR_OD8);
}

/**
 * @brief Configures GPIOF for system initialization.
 * 
 * GPIOF8 - Cooling Output (LED7)
 * 
 */
void configure_GPIOF(void)
{
  // Configure:
	// LED7: PF8	(Output, Clear Floating Values, Active Low)
	
	// Configure I/O Ports
	// Clear LED Modes (Set 0b00 for bit pairs)
	GPIOF->MODER &= ~(GPIO_MODER_MODER8_Msk); 																// Bit 8 LED7

	// Set LED Modes as Output (Set 0b01 for bit pairs)
	GPIOF->MODER |= (0x01 << GPIO_MODER_MODER8_Pos); 													// Bit 8 LED7
	
	// Enable Push-Pull Output Mode (Clear Relevant Bits)
	GPIOF->OTYPER &= ~(GPIO_OTYPER_OT8); 																			// Bit 8 LED7
	
	// Clear Speed Modes (Set 0b00 for bit pairs, by anding ~0b11)
	GPIOF->OSPEEDR &= (unsigned int) (~(0x03 << GPIO_OSPEEDR_OSPEED8_Pos)); 	// Bit 8 LED7
	
	// Set Speed Mode to Medium (Set 0b01 for bit pairs)
	GPIOF->OSPEEDR |= (unsigned int) (0x01 << GPIO_OSPEEDR_OSPEED8_Pos); 			// Bit 8 LED7
	
	// Clear Pull-Up-Pull Down Register (Pull-up)
	GPIOF->PUPDR &= (unsigned int) (~(0x01 << GPIO_PUPDR_PUPD8_Pos)); 				// Bit 8 LED7
	
	// Set Output Data Register to TURN OFF LEDS by default
	GPIOF->ODR |= GPIO_ODR_OD8;
}

/**
 * @brief Configures the timers for system initialization.
 */
void configure_Timers(void)
{
	configure_TIM6();
	configure_TIM7();
}

/**
 * @brief Handles the output control based on the output states.
 */
void handle_outputs(void)
{
  if (cooling_output)
  {
    set_cooling(true);
  } 
  else 
  {
    set_cooling(false);
  }
  
  if (heating_output)
  {
    set_heating(true);
  } 
  else 
  {
    set_heating(false);
  }
  
  if (fan_output)
  {
    set_fan(true);
  } 
  else 
  {
    set_fan(false);
  }
  
  if (light_output)
  {
    set_light(true);
  } 
  else 
  {
    set_light(false);
  }
}

/**
 * Calculates the count value for TIM6 based on the desired rate.
 *
 * @param rate The desired rate in Hz.
 * @return The count value for TIM6.
 */
unsigned short count_from_rate_TIM6(float rate)
{
  return (uint16_t)(TIM6_RATE / rate);
}

/**
 * Calculates the count value for TIM7 based on the desired delay in milliseconds.
 *
 * @param delay_ms The desired delay in milliseconds.
 * @return The count value for TIM7.
 */
uint16_t count_from_delay_ms_TIM7(int delay_ms)
{
  return (uint16_t)(TIM7_RATE * delay_ms / 1000);
}

/**
 * @brief Sets the cooling output state. (LED7 - PF8)
 * 
 * @param on Whether the cooling output should be turned on or off.
 */
void set_cooling(bool on)
{
  // Cooling is on PF8
  if (on)
  {
    GPIOF->ODR |= GPIO_ODR_OD8;
  } 
  // Cooling is off
  else 
  {
    GPIOF->ODR &= ~GPIO_ODR_OD8;
  }
}

/**
 * @brief Sets the heating output state. (LED6 - PB8)
 * 
 * @param on Whether the heating output should be turned on or off.
 */
void set_heating(bool on)
{
  // Heater is on PB8
  if (on)
  {
    GPIOB->ODR |= GPIO_ODR_OD8;
  } 
  // Heater is off
  else 
  {
    GPIOB->ODR &= ~GPIO_ODR_OD8;
  }
}

/**
 * @brief Sets the fan output state. (LED1 - PA8)
 * 
 * @param on Whether the fan output should be turned on or off.
 */
void set_fan(bool on)
{
  // Fan is on PB1
  if (on)
  {
    GPIOA->ODR |= GPIO_ODR_OD8;
  } 
  // Fan is off
  else 
  {
    GPIOA->ODR &= ~GPIO_ODR_OD8;
  }
}

/**
 * @brief Sets the light output state. (LED0 - PA3)
 * 
 * @param on Whether the light output should be turned on or off.
 */
void set_light(bool on)
{
  // Light is on PA3
  if (on)
  {
    GPIOA->ODR |= GPIO_ODR_OD3;
  } 
  // Light is off
  else 
  {
    GPIOA->ODR &= ~GPIO_ODR_OD3;
  }
}

/**
 * @brief Gets the temperature reading.
 * 
 * @return The temperature reading in degrees Celsius.
 */
float get_temperature(void)
{
  // 0-4095 to -5 to 45 by linear relationship between ADC and temperature 0 ADC is -5 and 4095 ADC is 45
  return (float)((get_ADC_temperature() / 4095.0) * 50.0 - 5.0);
}

/**
 * @brief Gets the ADC temperature reading. (0-4095) from (ADC3 Channel 8  - PF10)
 * 
 * @return The ADC temperature reading.
 */
int get_ADC_temperature(void)
{
  // ADC3 is on PF10
  // Read ADC3 and return the value (Mask to only keep the 12 bits of the ADC value)
  return (int)(ADC3->DR & 0x0FFF);
}

/**
 * @brief Gets the light intensity based on 50ms debounce.
 * 
 * @return The light intensity.
 */
bool get_light_intensity(void)
{
  // Light Intensity Sensor is on PA10
  bool state = get_light_intensity_gpio();
  // Sensor is active low, return inverted value of the sensor state
  // If 50ms timer is started, expired AND switch isn't pressed
  if (debounce_timer_active && TIM7_expired() && !state)
  {
    // Reset debounce timer, return true (Switch is confirmed pressed)
    debounce_timer_active = false;
    return true;
  }
  // Else, if sensor is active, start debounce timer
  else if (state)
  {
    // Start debounce timer
    debounce_timer_active = true;
    start_TIM7(count_from_delay_ms_TIM7(50));
  }
  // Else, return false
  return false;
}

/**
 * @brief Gets the light intensity.
 * 
 * @return The light intensity GPIO value.
 */
bool get_light_intensity_gpio(void)
{
  // Light Intensity Sensor is on PA10
  // Sensor is active low, return inverted value of the sensor state
  return (bool)!(GPIOA->IDR & GPIO_IDR_ID10_Msk);
}

/**
 * @brief Gets the fan switch state based on 50ms debounce.
 * 
 * @return The fan switch state.
 */
bool get_fan_switch(void)
{
  // Fan Switch is on PB0
  bool state = get_fan_switch_gpio();
  // Switch is active low, return inverted value of the switch state
  // If 50ms timer is started, expired AND switch isn't pressed
  if (debounce_timer_active && TIM7_expired() && !state)
  {
    // Reset debounce timer, return true (Switch is confirmed pressed)
    debounce_timer_active = false;
    return true;
  }
  // Else, if sensor is active, start debounce timer
  else if (state)
  {
    // Start debounce timer
    debounce_timer_active = true;
    start_TIM7(count_from_delay_ms_TIM7(50));
  }
  // Else, return false
  return false;
}

/**
 * @brief Gets the fan switch state.
 * 
 * @return The fan switch state.
 */
bool get_fan_switch_gpio(void)
{
  // Fan Switch is on PB0
  // Switch is active low, return inverted value of the switch state
  return (bool)!(GPIOB->IDR & GPIO_IDR_ID0_Msk);
}

/**
 * @brief Gets the light switch state based on 50ms debounce.
 * 
 * @return The light switch state.
 */
bool get_light_switch(void)
{
  // Light Switch is on PA9
  bool state = get_fan_switch_gpio();
  // Switch is active low, return inverted value of the switch state
  // If 50ms timer is started, expired AND switch isn't pressed
  if (debounce_timer_active && TIM7_expired() && !state)
  {
    // Reset debounce timer, return true (Switch is confirmed pressed)
    debounce_timer_active = false;
    return true;
  }
  // Else, if sensor is active, start debounce timer
  else if (state)
  {
    // Start debounce timer
    debounce_timer_active = true;
    start_TIM7(count_from_delay_ms_TIM7(50));
  }
  // Else, return false
  return false;
}

/**
 * @brief Gets the light switch state.
 * 
 * @return The light switch state.
 */
bool get_light_switch_gpio(void)
{
  // Light Switch is on PA9
  // Switch is active low, return inverted value of the switch state
  return (bool)!(GPIOA->IDR & GPIO_IDR_ID9_Msk);  
}

/**
 * @brief Checks if Timer 6 has expired and clears the expiration flag.
 * @return true if Timer 6 has expired, false otherwise.
 */
bool TIM6_expired(void)
{
  // Return if timer expired and clear expiration flag
  bool expired = (TIM6->SR & TIM_SR_UIF) == 0x01;
  // Clear UIF overflow flag
  TIM6->SR &= ~(TIM_SR_UIF_Msk);
  return expired;
}

/**
 * @brief Checks if Timer 7 has expired and clears the expiration flag.
 * @return true if Timer 7 has expired, false otherwise.
 */
bool TIM7_expired(void)
{
  // Return if timer expired and clear expiration flag
  bool expired = (TIM7->SR & TIM_SR_UIF) == 0x01;
  // Clear UIF overflow flag
  TIM7->SR &= ~(TIM_SR_UIF_Msk);
  return expired;
}

/**
 * @brief Waits for TIM6 to expire.
 * 
 * This function waits until the TIM6 timer has expired before returning.
 * The TIM6_expired() function is used to check if the timer has expired.
 */
void wait_For_TIM6(void)
{
  // Wait for TIM6 to expire
  while (!TIM6_expired())
  {
    // Wait for TIM6 to expire
  }
}

/**
 * @brief Waits for TIM7 to expire.
 * 
 * This function waits until the TIM7 timer has expired before returning.
 * The TIM7_expired() function is used to check if the timer has expired.
 */
void wait_For_TIM7(void)
{
  // Wait for TIM7 to expire
  while (!TIM7_expired())
  {
    // Wait for TIM7 to expire
  }
}

/**
 * @brief Starts TIM6 with the specified count value.
 * 
 * @param count The count value.
 */
void start_TIM6(uint16_t count)
{
	// Ensure timer is off
	stop_TIM6();

	// Clear UIF overflow flag
	TIM6->SR &= ~(TIM_SR_UIF_Msk);
	
  // Set autoreload register and enable counter
	TIM6->ARR &= ~(TIM_ARR_ARR_Msk);							          // Clear autoreload register
	TIM6->ARR |= ((unsigned int)count << TIM_ARR_ARR_Pos);  // Set autoreload register
	TIM6->CR1 |= TIM_CR1_CEN;											          // Enable counter
}

/**
 * @brief Starts TIM7 with the specified count value.
 * 
 * @param count The count value.
 */
void start_TIM7(uint16_t count)
{
  // Ensure timer is off
  stop_TIM7();

  // Clear UIF overflow flag
  TIM7->SR &= ~(TIM_SR_UIF_Msk);

  // Set autoreload register and enable counter
  TIM7->ARR &= ~(TIM_ARR_ARR_Msk);							          // Clear autoreload register
  TIM7->ARR |= ((unsigned int)count << TIM_ARR_ARR_Pos);  // Set autoreload register
  TIM7->CR1 |= TIM_CR1_CEN;											          // Enable counter
}

/**
 * @brief Stops TIM6.
 */
void stop_TIM6(void)
{
	// Ensure timer is off
	TIM6->CR1 &= ~TIM_CR1_CEN;
	// Clear UIF overflow flag
	TIM6->SR &= ~(TIM_SR_UIF_Msk);
}

/**
 * @brief Stops TIM7.
 */
void stop_TIM7(void)
{
	// Ensure timer is off
	TIM7->CR1 &= ~TIM_CR1_CEN;
	// Clear UIF overflow flag
	TIM7->SR &= ~(TIM_SR_UIF_Msk);
}

/**
 * Checks if the temperature has changed a bit.
 * 
 * @return true if the temperature has changed, false otherwise.
 */
bool temperature_changed() 
{
  return (get_temperature() - temperature_input) > TEMP_DIFF_MIN || (temperature_input - get_temperature()) > TEMP_DIFF_MIN;
}

/**
 * @brief Handles the cooling/heating logic.
 * 
 * This function checks the cooling input and updates the cooling and heating outputs accordingly.
 * If the cooling input is true, the cooling output is toggled and the heating output is set to the opposite value.
 * 
 * @return The current value of the cooling output.
 */
void handle_cooling_and_heating(void)
{
  // Via the UART, the heating / cooling can be turned on or off, however, it will
  // consider the current temperature. If the temperature is between 16℃ and 25℃, then the
  // command to turn on / off the heating / cooling / fan will be accepted for 15 seconds and
  // then the automatic control (as indicated in Section 3c) will resume. If the temperature
  // range is above 25℃ or below 16℃, the UART command to control the heater / cooling
  // and fan will be ignored. Should the temperature value change during the 15 second
  // UART control (heating_cooling_manual_override_timer_active), the automatic control should not resume.

  // Handle cooling logic based on the cooling input and temperature
  // If override timer is active and expired and temperature has changed, reset timer and resume automatic control
  if ((heating_cooling_manual_override_timer_active && manual_override_count >= 15) || (heating_cooling_manual_override_timer_active && temperature_changed()))
  {
    // If manual override timer has expired, reset timer and resume automatic control
    heating_cooling_manual_override_timer_active = false;
    manual_override_count = 0;
  }

  if (temperature_input >= 25.0 && !heating_cooling_manual_override_timer_active)
  {
    // If temperature is above 25 degrees, turn off cooling and turn on heating
    cooling_output = false;
    heating_output = true;
  }
  else if (temperature_input <= 16.0)
  {
    // If temperature is below 16 degrees, turn on cooling and turn off heating
    cooling_output = true;
    heating_output = false;
  }
  // If temperature is between 16 and 25 degrees and manual override timer is active and not expired
  else if (heating_cooling_manual_override_timer_active && manual_override_count < 15)
  {
    // If temperature is between 16 and 25 degrees, handle cooling and heating inputs and update outputs
    if (cooling_input)
    {
      cooling_output = !cooling_output;
      heating_output = !cooling_output;
    }
    if (heating_input)
    {
      heating_output = !heating_output;
      cooling_output = !heating_output;
    }
  }
  else
  {
    // If temperature is between 16 and 25 degrees, handle cooling and heating inputs and update outputs
    if (cooling_input)
    {
      cooling_output = !cooling_output;
      heating_output = !cooling_output;
      // If cooling input is hit, start 15s timer
      heating_cooling_manual_override_timer_active = true;
      manual_override_count = 0;
    }
    if (heating_input)
    {
      heating_output = !heating_output;
      cooling_output = !heating_output;
      // If heating input is hit, start 15s timer
      heating_cooling_manual_override_timer_active = true;
      manual_override_count = 0;
    }
  }
}

/**
 * Handles the fan operation based on the current state and inputs.
 * If the fan "off" timer is active, it checks if 20 seconds have passed and turns the fan back on.
 * If the fan switch is hit, it either turns off the fan and starts a 20-second timer or turns on the fan.
 * If the fan switch is not hit and the timeout is not running, it returns the previous value of the fan output.
 *
 * @return true if the fan should be turned on, false if the fan should be kept off.
 */
bool handle_fan(void)
{
  // Check if fan "off" timer is currently active
  if (fan_timer_active)
  {
    // If 20s has passed then turn fan back on
    if (fan_timer_count >= 20)
    {
      // Turn fan back on
      fan_timer_active = false;
      fan_timer_count = 0;
      return true;
    }
    // If 20s has not passed then keep fan off
    else
    {
      return false;
    }
  }
  // If fan switch was hit
  else if (fan_input)
  {
    // If fan already on then turn off and start 20s timer
    if (fan_output)
    {
      fan_timer_active = true;
      return false;
    }
    // If fan off then turn on
    else
    {
      return true;
    }
  }
  // If fan switch not hit and timeout not running then return previous value
  else
  {
    return fan_output;
  }
}

/**
 * @brief Handles the light switch functionality.
 * 
 * This function checks if the light switch has been hit with a 50ms debounce. 
 * If the light switch is hit and the light is not already detected, it toggles the light state.
 * If the light switch is not hit, it returns the previous light state.
 * 
 * @return The updated light state.
 */
bool handle_light(void) //Cam (Sam - fix bug in function and add functionality)
{
  // If Light switch was confirmed to be hit (With 50ms debounce)
  if (light_input)
  {
    // If light already detected keep light off
    if (get_light_intensity())
    {
      return false;
    }
    // Return opposite of current light state (as switch was hit and it is a toggle)
    else
    {
      return !light_output;
    }
  // Switch not hit, return prev value
  } else {
    return light_output;
  }
}

/*
COMMENTS FOR REPORT CONEXT

EEET2096 – Embedded System Design and Implementation – Laboratory Project
SCHOOL OF ENGINEERING
EEET2096 – EMBEDDED SYSTEM DESIGN AND IMPLEMENTATION
LABORATORY PROJECT
HOME MANAGEMENT SYSTEM
1 AIMS
(i) To design, simulate, implement and test a series of external peripheral interfaces
using Keil uVision and an STM32F439 microcontroller.
(ii) To develop an understanding of the design workflow when writing code
(firmware) for an ARM-based microcontroller using an Integrated
Development Environment (IDE) such as Keil uVision.
(iii) To use the C programming language to develop comprehensive firmware that
controls integrated peripherals such as ADC, Timers, GPIO and the UART.
(iv) To use the STM32F439 datasheets to determine and interpret essential
characteristics of the microcontroller.
(v) To develop a large-scale C project comprised of numerous sub-modules that are
required to work together to achieve a common complex task.
2 INTRODUCTION
In this project, you will use Keil uVision to develop a complex system that builds on the
knowledge gained from previous laboratory assessments. In developing the project, you will
work with one additional student (maximum group size of two students) enrolled in the same
laboratory session and demonstrate your achievements to the Laboratory Demonstrator. The
application that your group will develop is to be written in C and deployed to a physical
STM32F439.
The assessment for the project consists of two components – a report and a demonstration
which are equally weighted at 15% each of the overall course grade (30% total). The project
runs over three weeks (weeks 8, 9 and 10) and the final demonstration will occur during your
timetabled laboratory session time in Week 11. The report (in PDF format) and corresponding
code is to be submitted to Canvas by Friday, Week 12 at 11:59pm. Note that a late penalty
of 10 marks per 24-hour period (inclusive of weekends) will apply for all components that are
to be submitted to Canvas.
All work must be original, and plagiarism will be taken very seriously. You must develop all
the code between the group and hence reference code from external sources is not permitted
(apart from the STM32F439_Template.zip file available on Canvas).
The project should follow a traditional design cycle where the system is developed ‘on paper’
before proceeding to the actual firmware implementation. Important aspects of the code should
EEET2096 Laboratory Project Page 4.2
EEET2096 – Embedded System Design and Implementation – Laboratory Project
be simulated and the laboratory time used to deploy the actual developed code to the hardware.
The assessment will have a strong focus on verification of the simulation and how the firmware
was developed.
In this project, you are required to develop a ‘Home Management System (HMS)’ using the
RMIT STM32F439 Development platform. The aim of the Home Management System is to
control a set of virtual sensors and corresponding outputs (simulated fan / heater and light
control). The STM32F439 is to be used as a central controller with a range of the internal
peripherals used to emulate the sensors. The HMS block diagram is as shown as in Figure 1.
Figure 1 - Home Management System (HMS) - Block Diagram
The system comprises of a temperature sensor (analogue input), a fan and light switch, a heater
and cooling output as well as a light intensity sensor (threshold detection). The required I/O
mapping (based on the RMIT Development Hardware) can be found in Table 1:
HMS Function Port / Pin I/O Type Development Board Function
Temperature Sensor PF10 Analogue Input Potentiometer
Fan Switch PB1 Digital Input ‘B’ Switch (SW6)
Light Intensity Sensor PA10 Digital Input Right Switch (SW4)
Light Switch PA9 Digital Input Left Switch (SW3)
Light Control Output PA3 Digital Output LED0
Fan Control Output PA8 Digital Output LED1
Cooling Output PF8 Digital Output LED7
Heater Output PB8 Digital Output LED6
chUART3 Receive PB11
I/O Alternate
Function N/A
UART3 Transmit PB10
I/O Alternate
Function N/A
Table 1 - Home Management System - I/O Mapping
To control the system, either the switches / potentiometer indicated in Table 1 or the UART
can be used. The UART should be configured to operate at 38,400bps, 8 data-bits, No Parity
and 1 Stop Bit (38,400, 8, N, 1) and for simplicity should use UART3 which already provides
a USB to UART conversion IC (FT232RL). To simplify the design process, the communication
protocol (in terms of the packet structure) for transmission of data from the HMS to a PC can
EEET2096 Laboratory Project Page 4.3
EEET2096 – Embedded System Design and Implementation – Laboratory Project
be found in Table 2. A terminal emulator (such as TeraTerm) can be used to transmit and
receive data to / from the development hardware.
The temperature sensor is emulated by the analogue input, with an ADC value of 0 equating to
a temperature of -5℃ and 4095 (full-scale) equalling 45℃. Assuming a linear relationship
between the ADC value and simulated temperature, an expression will need to be derived to
determine the value to be displayed on the terminal window.
A header byte (0x40 – ASCII ‘@’) is used to provide a marker (or synchronisation) for the PC
to know that the packet coming in is from the HMS. The signed temperature value (with one
decimal point) should be transmitted next. A total of 5 ASCII characters should be used to
indicate the simulated temperature (i.e., +22.8 should be printed to the console to represent
22.8℃, equally -04.8 should be printed to the console to represent -4.8℃). The fields ‘a’,‘b’,’c’
and ‘d’ are used to indicate the status of the cooling (Bit ‘a’), heater (Bit ‘b’), fan (Bit ‘c’) and
light (Bit ‘d’) outputs. A logic 1 indicates that the particular device is on, whereas a logic 0
indicates that it is off. The data should be transmitted back to the PC at 1.0Hz. Hint: Use a
timer with a 1.0Hz timeout to trigger the update to the PC.
0 1 0 0 0 0 0 0 x x x x x x x x 0 0 1 1 a b c d
First byte is a header - 0x40 Temperature Value (5-
ASCII values including sign)
a – Cooling Output (On / Off) – Logic 0 or 1.
b – Heater Output (On / Off) – Logic 0 or 1.
c – Fan Status (On / Off) - Logic 0 or 1.
d – Light Status (On / Off) - Logic 0 or 1.
Table 2 - HMS to PC Communication Protocol
In terms of sending data from the PC back to the HMS, a simplified protocol is to be used as
illustrated in Table 3. The header byte is 0x40 (ASCII ‘@’) followed by a second byte which
is used to turn on / off the cooling, heating, fan and light control lines as described in Section
3. Note that the heating and cooling outputs are mutually exclusive.
0 1 0 0 0 0 0 0 0 1 0 0 a b c d
First byte is a header - 0x40
a – Cooling Output (On / Off) – Logic 0 or 1.
b – Heater Output (On / Off) – Logic 0 or 1.
c – Fan Output (On / Off) - Logic 0 or 1.
d – Light Output (On / Off) - Logic 0 or 1.
Table 3 - PC to HMS Communication Protocol
The Light and Fan switches operate as toggle buttons. For example, assume that the current
status of a switch is ‘on’. If a press of the switch is detected, then the device will turn ‘off’. If
another press is detected, the device will turn back ‘on’. Figure 2 illustrates a simplified
electrical interpretation of a single switch being pressed. It is assumed that the switch is high
(logic 1) when not pressed and low (logic 0) when pressed.
Figure 2 – Switch Edge Detection
Not Pressed Not PressedSwitch Pressed
Rising edge
x x + T
EEET2096 Laboratory Project Page 4.4
EEET2096 – Embedded System Design and Implementation – Laboratory Project
In Section 3, a switch should only be registered after it has been released (rising edge
detection). One way that this can be achieved is by detecting the first edge transition at ‘x’
(falling-edge) and then sampling the I/O pin for at least 50ms. If the button is released prior to
50ms (T < 50ms), then the system should detect that the minimum timeout has not been
achieved. Provided that the button is held down for at least 50ms (x + T), and the rising edge
detected, then a button press can be registered and passed through for processing.
To detect the rising edge, the program has to keep the previous input pin reading denoted
switchValue(x). If the previous reading switchValue(x) is low and the current reading
switchValue (x + T) is 1, then a button press is detected at which point a flag indicating a
toggle should be set, provided a one second timeout has occurred.
3 PROBLEM DESCRIPTION
This project requires students to build a Home Management System (HMS). The specifications
of the system are as follows:
a) At 1.0Hz the system parameters are to be sent out via the UART (to the PC) for
monitoring. The default set of parameters are:
• Signed Current Temperature Value (5-ASCII values, truncated to a single
decimal point).
• Cooling Control Output Status (On / Off).
• Heater Control Output Status (On / Off).
• Fan Control Output Status (On / Off).
• Light Control Output Status (On / Off).
b) The switches to control the light (Light Switch) and fan (Fan Switch) are to operate in a
‘toggle’ mode. For example, pressing the switch once will turn on / off the function
(latched output). The switch must be pressed for a minimum of 50ms to be successfully
registered.
c) The target temperature to be maintained is 23℃. If the temperature is below 21℃, then
the heater and fan outputs should be turned on. If the temperature is above 25℃, the
cooling and fan outputs should be turned on. The heater and cooling outputs are mutually
exclusive (only one can be on at a given point in time). The user can press the fan switch
to automatically switch the fan off for a period of 20 seconds (the system will still be in
heater / cooling mode). If the temperature is between 22.5℃ and 23.5℃ then both the
heater and cooling modes turn off (1℃ of hysteresis). The fan will still be active when
in this hysteresis band.
d) The light intensity sensor is used to detect whether a preset illumination already exists.
If SW4 is pressed (and held), then it indicates that the room is fully lit and pressing the
Light Switch will not turn on the light output. If SW4 is not pressed (and held), then the
lights can activate by pressing the Light Switch as described in 3b).
e) Via the UART, the light can be turned on / off irrespective of the light intensity value. If
the UART light turn off command and light switch press occur within one second of each
other, then the UART command will take priority.
f) Equally, via the UART, the heating / cooling can be turned on or off, however, it will
consider the current temperature. If the temperature is between 16℃ and 25℃, then the
command to turn on / off the heating / cooling / fan will be accepted for 15 seconds and
then the automatic control (as indicated in Section 3c) will resume. If the temperature
EEET2096 Laboratory Project Page 4.5
EEET2096 – Embedded System Design and Implementation – Laboratory Project
range is above 25℃ or below 16℃, the UART command to control the heater / cooling
and fan will be ignored. Should the temperature value change during the 15 second
UART control, the automatic control should not resume.
4 REPORTING REQUIREMENTS
The assessment schedule and a description of the required documents appear below. Please
ensure that you thoroughly read through the assessment rubrics to understand the required
outcomes of the project.
Week 11 – Project Technical Demonstration
You will be required to demonstrate your complete project to the Laboratory Demonstrator
during your allocated laboratory slot in Week 11. Each group will have approximately seven
(7) minutes to describe and demonstrate your technical achievements. A further three (3)
minutes will be made available for questions. Additional material, such as diagrams and images
can be used to support the discussion. The demonstration component accounts for 15% of the
total available course grade. Note that the demonstration is informal and will generally involve
viewing the project outcomes around one of the laboratory computers. You are requested to
ensure that your project is functional prior to the assessment time. No time compensation will
be given if the project is not ready to view at the scheduled time.
Week 12 – Project Technical Report
You are required to individually submit a group final project technical report (maximum of ten
(10) pages in body of report) describing the work undertaken during the project. The report
should be written in such a way that it can be read and understood by another Engineer with a
background in STM32F439 design and development. Note that the same report is to be
submitted by both group members.
The report (and background material relating to the development of the project including
schematics, PCB layouts and source code) must be submitted to the subject Canvas website
by Friday, Week 12 at 11:59pm. The report is to be in PDF format only and the entire Keil
uVision project submitted in .zip format.
A late penalty of 10 marks per day 24-hour period (including weekends) will apply if the
required content is not received by the due date / time. The report accounts for a further 15%
of the available course grade.
As a general guideline, the project report should include, but not be limited to, the following
sections:
• Title page: include the project title, the date, student ID, name(s), the lab session day
and time, and the revision number.
• Executive summary: state the main achievements of the project. This is a summary of
key findings, achievements, and measurements. It is not an introduction. The words
limit is 150.
• Table of contents: section titles and page numbers for your report.
• Introduction: provide an overview and define the scope of the project.
• Essential background information: a brief indication of what references and external
information were sought and used. The background information should be limited to
technical information that is relevant to explain the concepts and problems addressed
in the project.
• Technical work and Results (You may choose your own section titles here): this part
may contain a description of the process used to develop the deliverables and a
EEET2096 Laboratory Project Page 4.6
EEET2096 – Embedded System Design and Implementation – Laboratory Project
complete description of what has been created. You can elaborate on your contribution
to the project and compare obtained results with those in literature or other known
solutions. A comparison should be undertaken against the original deliverables of the
project and what has been delivered. If discrepancies exist then the reasons should be
elaborated (even incorrect or unexpected results and still worth discussing). This
section should form the bulk of the report. Technical content may include diagrams,
relevant truth-tables and block diagrams explaining the ‘C’ code used in realising the
solution. Simulation results can also be included in the report to explain / demonstrate
project outcomes.
• Discussion and Conclusion: You should provide a discussion of the results, clearly
stating their achievements, lessons learnt and possible future works.
• Appendices: These must also be properly titled and should contain details which are of
secondary importance in understanding the report. Examples include schematics,
detailed specifications of important components, and derivation of not so well known
mathematical functions or theorems used in the report. Note that the full code solution
does not need to appear in the appendices as it is to be submitted electronically.
5 RECOMMENDED REFERENCES
[1] ST Microelectronics, “RM0090 – Reference Manual”, ST Microelectronics, June 2018,
Available Online.
[2] ST Microelectronics, “STM32F437xx, STM32F439xx Datasheet”, ST Microelectronics,
June 2018, Available Online
EEET2096 Laboratory Project Assessment Schedule Page 4.7
EEET2096 – Embedded System Design and Implementation – Laboratory Project
EEET2096 – Laboratory Group Project Demonstration (Week 11)
The Project Technical Demonstration accounts for a total of 15% of the course grade for EEET2096. The demonstration is to be marked out of 100 and then will be converted
to an appropriate percentage. When marking the demonstration factors such as technical complexity, clarity of presentation, technical content, demonstrated technical skills
and analysis of results should be considered.
0 – 49 (NN) 50 – 59 (PA) 60 – 69 (CR) 70 – 79 (DI) 80 – 100 (HD) Score
(%)
Presentation -
Style (10%)
Relies entirely on reading
from notes and / or did
not look at assessor.
Unable to convey to
assessor the nature of the
project.
Unable to answer
questions posed by the
assessor.
Vague contribution to the
discussion / relies on
other group member(s).
Relies heavily on notes and
/ or makes little eye contact
with assessor.
Struggles to clearly explain
the nature of the project to
assessor.
Struggles to answer
questions posed by the
assessor.
Limited contribution to the
discussion / relies on other
group member(s).
Refers to notes at times
but makes reasonable
eye-contact with assessor.
Nature of the project is
conveyed reasonably well
and is generally
understandable.
Responds to most
questions but answers
some incorrectly / lack of
confidence.
Discussion is vague and
relies on prompts from
group member(s).
Has little to no reliance on
notes and makes good eye
contact with audience.
Shows an understanding
of the nature of the
project and conveys this
well to assessor.
Can answer all questions
with reasonable
confidence.
Time is evenly split
between group
member(s) however
limited indication on the
individual contributions.
No reliance on notes and
presents in an effective
and innovative style.
Clearly explains the
nature of the project and
generates enthusiasm for
the topic with the
assessor.
Answers questions
confidently and correctly.
Time equally divided
between both group
member(s) with clear
evidence of individual
contributions.
Presentation /
Demonstration
- Technical
Content (90%)
Minimal technical
content presented.
Inappropriate or
insufficient details to
support results, e.g.
opinions stated instead of
facts. No analysis of
results or comparison
against original aims.
System has not been
simulated to determine
whether it is functionally
operational.
Investigation method not
discussed or described
poorly.
Understands some of the
topic but struggles to make
connections with the
results.
Investigation method
discussed but is flawed. No
alternative methods
described.
The simulations developed
do not demonstrate
sufficient technical
complexity or are lacking in
analysis.
Poor comparisons drawn
between original
specifications and the
presented outcomes.
Superficial evaluation of
results presented.
Investigation method is
generally sound, mention
has been made to
alternative methods but
justifications may not
have been given for why
one method was used
over another.
Functional simulations
have been performed to
verify the sub-module
design, however may not
include all possible states
/ or contain minor errors
or invalid assumptions.
Sound analysis of results
presented.
Investigation method
used has been described
and justified. Some
alternative methods have
been considered and
described.
Functional simulations
are verified against the
system output. Advanced
diagnostic tools such as
oscilloscopes have been
utilised to determine (and
rectify) errors where
appropriate.
In-depth analysis of
results presented.
Investigation method(s)
well demonstrated and
justified. Alternative
methods have been
thoroughly researched.
Functional simulations
are complete and verify
complete system
functionality.
Demonstrated ability to
resolve errors and
implement appropriate
solutions.
Where appropriate output
has been verified with
EEET2096 Laboratory Project Assessment Schedule Page 4.8
EEET2096 – Embedded System Design and Implementation – Laboratory Project
Has not been able to solve
technical problems at an
appropriate level.
Technical skill and
complexity of solution
are inadequate.
Presented outcomes are
not satisfactory.
System is not in working
condition and / or system
is not available for
assessor to see.
Limited ability to solve
technical problems or
incorrect / inappropriate
methods applied.
Technical skill and
complexity of the solution
are marginal.
Students have performed
little technical work over
the project.
Minor parts of the system
may be working but largely
it does not work to
specifications.
Comparison between
expected outcomes and
actual results lacks
technical detail.
Technical problems have
been addressed and
solution is adequate
although cumbersome.
Fair progress has been
demonstrated by the
student.
System achieves most of
its specifications but may
stop working
intermittently or have an
inconsistent output.
System may need to be
constantly reset to negate
errors in the design.
Comparison of actual
results versus original
specifications is sound.
Technical detail correct
and informative.
Has demonstrated an
ability to solve technical
problems using standard
accepted techniques.
Good progress has been
demonstrated by the
student.
A system has been
developed that works to
required specifications.
System was well
demonstrated to show its
effectiveness and
reliability.
tools such as
oscilloscopes or logic
analysis.
Excellent comparison
between actual and
expected results.
Clearly demonstrated
ability to solve technical
problems using advanced
methods.
Exceptional progress has
been demonstrated by the
student.
An innovative system /
solution has been
developed that works
exactly as specifications
require.
Extra functionality may
have also been included.
EEET2096 Laboratory Project Assessment Schedule Page 4.9
EEET2096 – Embedded System Design and Implementation – Laboratory Project
EEET2096 – Laboratory Group Project Technical Report (Week 12)
The Project Technical Report accounts for a total of 15% of the course grade for EEET2096. The report is to be marked out of 100 and then will be converted to an appropriate
percentage. When marking the report factors such as technical complexity, clarity of discussion, appropriate choice of detail, demonstrated technical skills and analysis of
results will be considered. Important concepts must be clearly explained and sources quoted appropriately.
0 – 49 (NN) 50 – 59 (PA) 60 – 69 (CR) 70 – 79 (DI) 80 – 100 (HD) Score
(%)
Reference
Reading and
Theoretical
Backing
(10%)
Report contains limited
relevant background
information and restates
simple facts.
Significant technical
errors are present in the
report illustrating gaps in
knowledge.
Irrelevant technical
references are included or
no references at all.
Basic background theory
presented covering the topic
on a superficial level and
missing key technical
details.
Technical errors exist in the
document which raises
concerns on the presented
outcome.
Background theory
(device operation) is
sound and covers many of
the relevant areas of the
project.
Report is technically
sound, however may be
lacking in appropriate
technical detail.
Background theory
(device operation) shows
good research abilities and
covers most of the relevant
areas of the project.
A solid technical
discussion has been held
demonstrating an in-depth
understanding of the topic.
Background theory
(device operation)
demonstrates exceptional
research skills covering all
relevant topics for the
project.
A comprehensive
summary of considered
techniques have been
presented and fully
discussed which have led
the successful completion
of the stage of the project.
Logical and
Convincing
Presentation
/ Layout
Diagrams
and
Photographs
(10%)
Report contains a large
number of spelling and
grammatical errors.
Figures are incorrect /
difficult to interpret and
no discussion has been
held on the material
presented.
Some spelling and
grammatical errors present.
Supporting figures are
present however may not be
100% clear or limited
discussion has been held on
their meaning.
Spelling and grammar was
of an acceptable level.
Graphs and figures were
clear but may have had
unclear titles/captions.
Figures have been linked
back to the main text.
Spelling and grammar
mostly correct.
Graphs and figures are
mostly clear and labelled.
A discussion has been held
on the figure and how it
relates to the project.
Exceptional use of
language. No spelling /
grammatical errors.
All figures are clear and
well labelled. A thorough
discussion has been held
on their meaning and
purpose.
Technical
Merit (80%)
Functional block diagram
of system is not present /
does not describe the
relevant I/O.
Design simulation is
inadequate or not
discussed.
Deployed code was non-
functional / results
incorrect.
Functional block diagram
of system is present
however does not describe
the relevant I/O or is
lacking in detail.
Design simulation has
been performed, however
no discussion has been
held.
Functional block diagram
of system is sufficient and
describes the basic I/O
requirements.
Design simulation is
suitable and important
features noted.
Simulation output was
operational however
results were not explained.
Functional block diagram
and of system is well
constructed.
Simulation output
demonstrates a fully
operational design and
important features
discussed.
Code is functional and
described in detail.
A comprehensive
functional block diagram
has been developed listing
essential details.
Simulation is exceptional
and comparisons have
been clearly drawn
between theoretical and
experimental results.
EEET2096 Laboratory Project Assessment Schedule Page 4.10
EEET2096 – Embedded System Design and Implementation – Laboratory Project
Design / schematic /
relevant portions of code
not presented.
Investigation method not
discussed or described
poorly.
No analysis performed on
results obtained.
Unable to make links to
theoretical concepts and /
or irrelevant facts were
used to try to explain
results.
The techniques employed
do not demonstrate a
sound technical
understanding of the
topic.
Results have not been
presented in a coherent
fashion with limited
discussion.
Conclusions are not
present or are technically
flawed.
Significant issues exist
with the outcome of the
project.
Partially functional code
with poor verification and
justification.
Design / schematic
relevant code presented.
Simulation was partially
functional with
questionable results
presented.
Superficial analysis of
results presented. Results
have not been linked back
to existing theory
presented in the technical
literature review section.
Can make basic links to
theoretical concepts but
lacks in-depth
understanding.
Significant gaps exist in
the analysis of the results.
Inappropriate conclusions
have been drawn from the
presented results.
Outcomes are marginal
and rely on existing work
rather than demonstrating
the students’ ability.
The techniques
demonstrated highlight
sufficient gaps in the
students’ knowledge.
Conclusions are lacking
detail.
Code is functional
however limited
discussion on design
verification / justification.
Design / schematic /
relevant code presented.
A reasonable analysis has
been made of results, but
may be lacking some
depth. Can make
reasonable links to
theoretical concepts to
explain results.
Investigation method
undertaken is adequate but
student may not have
considered more effective
alternatives.
Slight gaps exist in the
students’ knowledge.
Conclusions are
sufficiently detailed
however no technical
justification has been
presented.
Outcomes are acceptable
however a more thorough
analysis should have been
performed to explain
unexpected results.
Design / schematic
relevant code presented
and appropriately
justified.
A good analysis of results
presented.
Investigation method
undertaken was sound and
appropriate for the project
and student demonstrates
they have a firm grasp of
the theoretical aspects of
the project.
Outcomes are well
documented and have
been thoroughly
explained. Unexpected
results have been
considered and an
appropriate hypothesis
formed.
Conclusions and
recommendations are
justified and a solid
technical discussion is
present.
Simulated design was
fully operational and a
complete comparison
drawn between the
theoretical and
experimental results.
Developed code clearly
meets all prescribed
targets and verified with
supporting
documentation.
Design / schematic /
relevant code presented
and fully explained.
In-depth analysis of
results presented.
Advanced investigation
method proposed that
shows solid understanding
of project requirements.
Students demonstrate an
advanced comprehension
of the topic and have
successfully completed
the project.
Conclusions made should
be justified and well
explained.
Outcomes are exceptional.
*/