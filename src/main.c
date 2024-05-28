
/********************************************
 *			STM32F439 Main								  			*
 *			Developed for the STM32								*
 *			Startup Author: Dr. Glenn Matthews		*
 *			Project Author: Samuel Griffiths 			*
 *			Source File														*
 ********************************************/

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "boardSupport.h"
#include "main.h"

// Timer Constants
#define CPU_CLOCK 84000000  // 84MHz
#define TIM6_PRESCALER 2560 // Calculations in report
#define TIM7_PRESCALER 128  // Calculations in report
#define DEFAULT_TIMER_COUNT 100 // Default timer count for TIM6 and TIM7 (Only used for initialisation)
#define TIM6_RATE CPU_CLOCK / (TIM6_PRESCALER + 1) // 84MHz bus clock divided by prescaler gets the number of timer ticks per second, +1 Accounts for hardware adding 1 for prescaler
#define TIM7_RATE CPU_CLOCK / (TIM7_PRESCALER + 1) // 84MHz bus clock divided by prescaler gets the number of timer ticks per second, +1 Accounts for hardware adding 1 for prescaler

#define USART_CONTROL_TIMEOUT 15 // 15s timeout for USART control (Software timer combined with 1Hz hardware timer)
#define FAN_TIMER_TIMEOUT 20     // 20s timeout for fan off timer (Software timer combined with 1Hz hardware timer)

#define INCOMMING_BUFFER_SIZE 4 // Size of the buffer for incomming data from UART
#define OUTGOING_BUFFER_SIZE 9  // Size of the buffer for outgoing data via UART

#define ASCII_CR 0x0D
#define ASCII_LF 0x0A
#define ASCII_AT 0x40

#define HEAT_ON_TEMP 22.5f // Temperature to turn on heating
#define COOL_ON_TEMP 23.5f // Temperature to turn on cooling

#define AUTO_CONTROL_MIN 16.0f // Temperature min to accept USART control
#define AUTO_CONTROL_MAX 25.0f // Temperature min to accept USART control

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
void transmit_Status_Packet(char *data);
int8_t receive_UART(void);
void receive_Status_Packet(void);
char *get_outgoing_string(void);

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
bool USART_control_timer_expired(void);
bool fan_timer_expired(void);

// Handle inputs and outputs methods
void handle_auto_thermostat(void);
bool handle_fan(void);
bool handle_light(void);
void handle_outputs(void);

// Helper methods
unsigned short count_from_rate_TIM6(float rate);
unsigned short count_from_delay_ms_TIM7(int delay_ms);

// Global Variables
static volatile bool cooling_output;
static volatile bool heating_output;
static volatile bool fan_output;
static volatile bool light_output;

static volatile bool cooling_input;
static volatile bool heating_input;
static volatile bool fan_input;
static volatile bool light_input;

static volatile bool fan_timer_active;
static volatile int fan_timer_count = 0; // Time count for 20s fan off timer
static volatile bool usart_control;
static volatile int usart_control_timer_count = 0; // Time count for 15s USART control timer

static volatile bool light_intensity_sensor;
static volatile bool fan_switch;
static volatile bool light_switch;
static volatile float temperature_input;
static volatile float temperature_output;
static volatile int adc_temperature;
static volatile int incomming_string_index;

// Character array for UART communication, one for incomming string and one for outgoing string
static volatile char incomming_string[INCOMMING_BUFFER_SIZE] = {0};
static volatile char outgoing_string[OUTGOING_BUFFER_SIZE] = {0};
static volatile bool status_packet_recieved; // Flag to indicate if a status packet has been recieved

// Flags to hold the state of the switches
static volatile bool fan_held = false;
static volatile bool light_held = false;

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
  uint16_t timer_Count;
  char temperature_string[6] = {0, 0, 0, 0, 0, 0};

  // Bring up the GPIO for the power regulators.
  boardSupport_init();

  // Configure the peripherals
  configure_RCC();
  configure_GPIOs();
  configure_Timers();
  configure_USART3();
  configure_ADC();

  // Initialise Program State Variables
  cooling_output = false;
  heating_output = false;
  fan_output = false;
  light_output = false;
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
  fan_timer_active = false;
  usart_control = false;

  while (1)
  {
    // If 1Hz timer has expired then transmit data and tick any other software timers
    if (TIM6_expired())
    {
      transmit_Status_Packet(outgoing_string);

      // Get the count for the next timer cycle (1Hz, this is variable as it could be changed to 2Hz or 0.5Hz etc.)
      timer_Count = count_from_rate_TIM6(1.0);
      // Restart timer
      start_TIM6(timer_Count);

      // If fan timer is active then increment count
      if (fan_timer_active)
      {
        fan_timer_count++;
      }

      // If USART control is active then increment count
      if (usart_control)
      {
        usart_control_timer_count++;
      }
    }

    // Recieve/Check for USART input
    receive_Status_Packet();

    // Check if status frame is recieved and it is valid (First 4 bits in the status frame are 0b0011 and @ is the header character)
    if (status_packet_recieved)
    {
      // Check if the first character is the header character
      if (incomming_string[0] == ASCII_AT)
      {
        // Check if the control state is valid (First 4 bits are 0b0100)
        if ((incomming_string[1] & 0xF0) == 0x40)
        {
          // Set inputs based on status frame (These are bool so no shifting required, any non 0 value will be true)
          cooling_input = incomming_string[1] & 0x8; // Clear all but 4th bit
          heating_input = incomming_string[1] & 0x4; // Clear all but 5th bit
          fan_input = incomming_string[1] & 0x2;     // Clear all but 6th bit
          light_input = incomming_string[1] & 0x1;   // Clear all but 7th bit
					incomming_string[0] = '\0';
					incomming_string_index = 0;
					
					usart_control = true;
				}
			}
    }

    // If USART control is active and not expired then handle inputs
    if (usart_control && !USART_control_timer_expired()) {
      // Handle USART control
	
			// If we just recieved the command, set outputs accordingly
			if (status_packet_recieved)
			{
				// If the temperature is valid then accept USART command
				if (get_temperature() >= AUTO_CONTROL_MIN && get_temperature() <= AUTO_CONTROL_MAX)
				{
					// If cooling and heating are mutually exclusive then set output to input
					if (!(cooling_input && heating_input)) {
						// If both cooling and heating are given then ignore input
						// If only one is set then set output to input
						cooling_output = cooling_input;
						heating_output = heating_input;
					}
					else {
						// cooling and heating set, cancel and ignore input
						handle_auto_thermostat();
						usart_control = false;
					}
				}
				// If temperature is out of range (and we just recieved the input)
				else
				{
					// Cancel and ignore input 
					handle_auto_thermostat();
					usart_control = false;
				}
				// Set fan output to input
				fan_output = fan_input;
      }
			// If we have an existing command, keep going
			else {
				// Nothing changes
			}
			// Set light output to input (regardless of temp range)
			light_output = light_input;

			// Reset status packet flag and clear current status string
			status_packet_recieved = false;

    }
    // Follow complete local control (including user switches) if USART control is not active
    else
    {
      usart_control = false;
      // Read Switches
      light_switch = get_light_switch();
      fan_switch = get_fan_switch();
      fan_output = handle_fan();
      light_output = handle_light();

      // Handle thermostat based on temperature
      handle_auto_thermostat();
    }

    temperature_output = get_temperature();

    // Set header character
    outgoing_string[0] = ASCII_AT;

    // Set temperature characters
    // Convert temperature to string
    snprintf((char *)temperature_string, 6, "%+05.1f", (double)temperature_output);

    // Set temperature characters
    for (int i = 0; i < 5; i++)
    {
      outgoing_string[i + 1] = temperature_string[i];
    }

    // Set control state character
    // Control State is a byte, convert to string
    outgoing_string[6] = (char)((0x03 << 4) | (cooling_output << 3) | (heating_output << 2) | (fan_output << 1) | light_output);

    // Set CR character
    outgoing_string[7] = ASCII_CR;

    // Set LF character
    outgoing_string[8] = ASCII_LF;

    // Update GPIO Outputs
    handle_outputs();
  }
}

/**
 * @brief Configures USART3 for communication.
 */
void configure_USART3(void)
{
  // UART is configured at 38,400bps, 8 data bits, No Parity, 1 stop bit
  // UART APB1 Bus Clock is 42MHz
  // Baud Rate required is 68.375

  // Configure GPIOB MODER for alternate function mode
  GPIOB->MODER &= ~(GPIO_MODER_MODE11_Msk | GPIO_MODER_MODE10_Msk);                  // Clear MODE already set for 10 and 11
  GPIOB->MODER |= (0x02 << GPIO_MODER_MODE11_Pos) | (0x02 << GPIO_MODER_MODE10_Pos); // Set mode for 10 and 11 to be alternate function (0b10)

  // Set the alternate function to be AF7 (USART)
  GPIOB->AFR[1] &= ~(GPIO_AFRH_AFSEL11_Msk | GPIO_AFRH_AFSEL10_Msk);
  GPIOB->AFR[1] |= (0x07 << GPIO_AFRH_AFSEL11_Pos) | (0x07 << GPIO_AFRH_AFSEL10_Pos); // Set mode for 10 and 11 to be alternate function 7 (0b111)

  // Turn on OVER16 16-times over sampling (By clearing the OVER8 bit)
  USART3->CR1 &= ~(USART_CR1_OVER8);

  // Set USART Baud Rate
  USART3->BRR &= 0xFFFF0000;                                                                    // Clear Baud Rate register
  USART3->BRR |= ((0x44 << USART_BRR_DIV_Mantissa_Pos) | (0x06 << USART_BRR_DIV_Fraction_Pos)); // Set Baud rate

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

  // Set prescaler to /8
  ADC123_COMMON->CCR &= (0x03 << ADC_CCR_ADCPRE_Pos);

  // Disable scan mode and set resolution to 12 bits
  ADC3->CR1 &= ~(ADC_CR1_SCAN | ADC_CR1_RES_Msk);

  // Set the data alignment to right and set single mode for conversion and sample
  ADC3->CR2 &= ~(ADC_CR2_ALIGN | ADC_CR2_CONT | ADC_CR2_SWSTART);

  // Set to select channel 8 (PF10) for "temperature" (Potentiometer) sensing
  ADC3->SQR3 &= ~(ADC_SQR3_SQ1_Msk);
  ADC3->SQR3 |= (0x08 << ADC_SQR3_SQ1_Pos);
  ADC3->SQR1 &= ~(ADC_SQR1_L_Msk);

  // Set sample time to 56 cycles
  ADC3->SMPR2 &= ~(ADC_SMPR2_SMP0_Msk);
  ADC3->SMPR2 |= (0x03 << ADC_SMPR2_SMP0_Pos);

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
  __ASM("NOP");
}

/**
 * @brief Transmits a status packet via UART from character array.
 */
void transmit_Status_Packet(char *data)
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
  if (USART3->SR & USART_SR_RXNE)
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

  status_packet_recieved = false;

  // If complete string not already recieved
  if (incomming_string_index < INCOMMING_BUFFER_SIZE && !(incomming_character == ASCII_CR || incomming_character == ASCII_LF))
  {
    // If incomming character is recieved
    if (incomming_character != -1)
    {
      // Check if character is valid and append to the incomming string buffer
      if ((incomming_character >= (int8_t)'@' && incomming_character <= (int8_t)'O') || incomming_character == ASCII_CR || incomming_character == ASCII_LF)
      {
        // Append character to incomming string
        incomming_string[incomming_string_index] = (char)incomming_character;
        incomming_string_index++;
      }
    }
  }
  else
  {
    // Set flag to process the frame
    status_packet_recieved = true;
    incomming_string_index = 0;
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
  char* temperature_string = {0,0,0,0,0};
  snprintf(temperature_string, "%+05.1f", 6,(double)temperature_output);

  // Set temperature characters
  for (int i = 0; i < 5; i++)
  {
    outgoing_string[i + 1] = temperature_string[i];
  }

  // Set control state character
  // Control State is a byte, convert to string
  outgoing_string[6] = (char)((0x03 << 4) | (cooling_output << 3) | (heating_output << 2) | (fan_output << 1) | light_output);

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
  TIM6->PSC &= ~(TIM_PSC_PSC_Msk); // Clear Prescaler register
  TIM6->CNT &= ~(TIM_CNT_CNT_Msk); // Clear count register
  TIM6->ARR &= ~(TIM_ARR_ARR_Msk); // Clear autoreload register

  // Set Prescaler
  TIM6->PSC |= (TIM6_PRESCALER << TIM_PSC_PSC_Pos);      // Divide APB1 Clock by TIM6PRESCALER + 1
                                                         // Thus, at one cycle it will be TIM6RATE Hz (Assuming 82MHz of APB1)
  TIM6->ARR |= (DEFAULT_TIMER_COUNT << TIM_ARR_ARR_Pos); // Initialise auto-reload register with default value (what count will auto reload)

  // Set OPM to on
  TIM6->CR1 |= (TIM_CR1_OPM);

  // Enable timer for one cycle to clear and reload psc buffer
  start_TIM6(DEFAULT_TIMER_COUNT);

  wait_For_TIM6();

  TIM6->CR1 |= (TIM_CR1_CEN);
  TIM6->SR |= (TIM_SR_UIF);
}

/**
 * @brief Configures TIM7 for timing operations.
 */
void configure_TIM7(void)
{
  // Ensure timer is off and all configurations are reset
  TIM7->CR1 &= ~(TIM_CR1_ARPE | TIM_CR1_OPM | TIM_CR1_UDIS | TIM_CR1_CEN);

  // Disable TIM6 interrupts
  TIM7->DIER &= ~(TIM_DIER_UIE);

  // Clear prescaler, count and autoreload registers
  TIM7->PSC &= ~(TIM_PSC_PSC_Msk); // Clear Prescaler register
  TIM7->CNT &= ~(TIM_CNT_CNT_Msk); // Clear count register
  TIM7->ARR &= ~(TIM_ARR_ARR_Msk); // Clear autoreload register

  // Set Prescaler
  TIM7->PSC |= (TIM7_PRESCALER << TIM_PSC_PSC_Pos);      // Divide APB1 Clock by TIM6PRESCALER + 1
                                                         // Thus, at one cycle it will be TIM6RATE Hz (Assuming 82MHz of APB1)
  TIM7->ARR |= (DEFAULT_TIMER_COUNT << TIM_ARR_ARR_Pos); // Initialise auto-reload register with default value (what count will auto reload)

  // Set OPM to on
  TIM7->CR1 |= (TIM_CR1_OPM);

  // Enable timer for one cycle to clear and reload psc buffer
  start_TIM7(DEFAULT_TIMER_COUNT);

  wait_For_TIM7();

  TIM7->CR1 |= (TIM_CR1_CEN);
  TIM7->SR |= (TIM_SR_UIF);
}

/**
 * @brief Configures the RCC (Reset and Clock Control) for system initialization.
 *
 * This function enables the RCC for Timer 6, Timer 7, and USART3, and enables the GPIOs for GPIOA, GPIOB, and GPIOF.
 */
void configure_RCC(void)
{
  // Enable RCC for Timer 6, Timer 7, and USART3
  RCC->APB1ENR |= RCC_APB1ENR_TIM6EN | RCC_APB1ENR_TIM7EN | RCC_APB1ENR_USART3EN;         // Enable RCC for Timer 6, Timer 7, and USART3
  RCC->APB1RSTR |= RCC_APB1RSTR_TIM6RST | RCC_APB1RSTR_TIM7RST | RCC_APB1RSTR_USART3RST;  // Reset the peripheral interface
                                                                                          // Enable RCC for GPIOA, GPIOB, and GPIOF
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOFEN;        // Config to enable GPIOA, B, and F
  RCC->AHB1RSTR |= RCC_AHB1RSTR_GPIOARST | RCC_AHB1RSTR_GPIOBRST | RCC_AHB1RSTR_GPIOFRST; // Reset control interface
                                                                                          // Enable RCC for ADC3
  RCC->APB2ENR |= RCC_APB2ENR_ADC3EN;                                                     // Enable RCC for ADC 3
  RCC->APB2RSTR |= RCC_APB2RSTR_ADCRST;                                                   // Reset control interface
  __asm("nop");
  __asm("nop");
  // Clear reset bits for Timer 6, Timer 7, USART3, GPIOA, GPIOB, GPIOF, and ADC3
  RCC->APB1RSTR &= ~(RCC_APB1RSTR_TIM6RST | RCC_APB1RSTR_TIM7RST | RCC_APB1RSTR_USART3RST);  // Clear reset bit
  RCC->AHB1RSTR &= ~(RCC_AHB1RSTR_GPIOARST | RCC_AHB1RSTR_GPIOBRST | RCC_AHB1RSTR_GPIOFRST); // Clear reset	bit
  RCC->APB2RSTR &= ~(RCC_APB2RSTR_ADCRST);
  __asm("nop");
  __asm("nop");
}

/**
 * @brief Configures the GPIOs (General Purpose Input/Output) for system initialization.
 */
void configure_GPIOs(void)
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
void configure_GPIOA(void)
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
  GPIOA->OSPEEDR &= (unsigned int)(~(0x03 << GPIO_OSPEEDR_OSPEED3_Pos) |
                                   ~(0x03 << GPIO_OSPEEDR_OSPEED8_Pos) |
                                   ~(0x03 << GPIO_OSPEEDR_OSPEED9_Pos) |
                                   ~(0x03 << GPIO_OSPEEDR_OSPEED10_Pos));

  // Set Speed Mode to Medium (Set 0b01 for bit pairs)
  GPIOA->OSPEEDR |= ((unsigned int)(0x01 << GPIO_OSPEEDR_OSPEED3_Pos) |
                     (unsigned int)(0x01 << GPIO_OSPEEDR_OSPEED8_Pos) |
                     (unsigned int)(0x01 << GPIO_OSPEEDR_OSPEED9_Pos) |
                     (unsigned int)(0x01 << GPIO_OSPEEDR_OSPEED10_Pos));

  // Clear Pull-Up-Pull Down Register (Pull-up)
  GPIOA->PUPDR &= (unsigned int)(~(0x01 << GPIO_PUPDR_PUPD3_Pos) |
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
void configure_GPIOB(void)
{
  // Configure:
  // SW6: PB1 	(Input, Clear Floating Values, Active Low)
  // LED6: PB8 	(Output, Clear Floating Values, Active Low)
  // UART3 Transmit: PB10 (Alternate Function, Clear Floating Values, Active Low)
  // UART3 Receive: PB11 (Alternate Function, Clear Floating Values, Active Low)

  // Configure I/O Ports
  // Clear GPIO Modes (Set 0b00 for bit pairs)
  GPIOB->MODER &= ~(GPIO_MODER_MODER0_Msk | GPIO_MODER_MODER1_Msk |
                    GPIO_MODER_MODER8_Msk | GPIO_MODER_MODER10_Msk | GPIO_MODER_MODER11_Msk);

  // Set LED Modes as Output (Set 0b01 for bit pairs)
  GPIOB->MODER |= (0x01 << GPIO_MODER_MODER8_Pos);

  // Enable Push-Pull Output Mode (Clear Relevant Bits)
  GPIOB->OTYPER &= ~(GPIO_OTYPER_OT8);

  // Clear Speed Modes (Set 0b00 for bit pairs, by anding ~0b11)
  GPIOB->OSPEEDR &= (unsigned int)(~(0x03 << GPIO_OSPEEDR_OSPEED1_Pos) |
                                   ~(0x03 << GPIO_OSPEEDR_OSPEED8_Pos) |
                                   ~(0x03 << GPIO_OSPEEDR_OSPEED10_Pos) |
                                   ~(0x03 << GPIO_OSPEEDR_OSPEED11_Pos));

  // Set Speed Mode to Medium (Set 0b01 for bit pairs)
  GPIOB->OSPEEDR |= ((unsigned int)(0x01 << GPIO_OSPEEDR_OSPEED1_Pos) |
                     (unsigned int)(0x01 << GPIO_OSPEEDR_OSPEED8_Pos) |
                     (unsigned int)(0x01 << GPIO_OSPEEDR_OSPEED10_Pos) |
                     (unsigned int)(0x01 << GPIO_OSPEEDR_OSPEED11_Pos));

  // Clear Pull-Up-Pull Down Register (Pull-up)
  GPIOB->PUPDR &= (unsigned int)(~(0x01 << GPIO_PUPDR_PUPD1_Pos) |
                                 ~(0x01 << GPIO_PUPDR_PUPD8_Pos) |
                                 ~(0x01 << GPIO_PUPDR_PUPD10_Pos) |
                                 ~(0x01 << GPIO_PUPDR_PUPD11_Pos));

  // Set Output Data Register to TURN OFF LEDS by default (set to logic high)
  GPIOB->ODR |= GPIO_ODR_OD8;
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
  // ADC3: PF10 (Analogue)

  // Configure I/O Ports
  // Clear Modes (Set 0b00 for bit pairs)
  GPIOF->MODER &= ~(GPIO_MODER_MODER8_Msk | GPIO_MODER_MODER10_Msk); // Bit 8 LED7

  // Set LED Modes as Output (Set 0b01 for bit pairs) and PF10 as Analogue (0b11 for bit pair)
  GPIOF->MODER |= ((0x01 << GPIO_MODER_MODER8_Pos) | 0x03 << GPIO_MODER_MODER10_Pos); // Bit 8 LED7

  // Enable Push-Pull Output Mode (Clear Relevant Bits)
  GPIOF->OTYPER &= ~(GPIO_OTYPER_OT8 | GPIO_OTYPER_OT10); // Bit 8 LED7

  // Clear Speed Modes (Set 0b00 for bit pairs, by anding ~0b11)
  GPIOF->OSPEEDR &= (unsigned int)~(GPIO_OSPEEDR_OSPEED8_Msk | GPIO_OSPEEDR_OSPEED8_Msk); // Bit 8 LED7

  // Set Speed Mode to Medium (Set 0b01 for bit pairs)
  GPIOF->OSPEEDR |= (unsigned int)((0x01 << GPIO_OSPEEDR_OSPEED8_Pos) | (0x01 << GPIO_OSPEEDR_OSPEED10_Pos)); // Bit 8 LED7

  // Clear Pull-Up-Pull Down Register (Pull-up)
  GPIOF->PUPDR &= (unsigned int)(~(0x01 << GPIO_PUPDR_PUPD8_Pos)); // Bit 8 LED7

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
  return (uint16_t)((TIM7_RATE * delay_ms) / 1000);
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
    GPIOF->ODR &= ~GPIO_ODR_OD8;
  }
  // Cooling is off
  else
  {
    GPIOF->ODR |= GPIO_ODR_OD8;
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
    GPIOB->ODR &= ~GPIO_ODR_OD8;
  }
  // Heater is off
  else
  {
    GPIOB->ODR |= GPIO_ODR_OD8;
  }
}

/**
 * @brief Sets the fan output state. (LED1 - PA8)
 *
 * @param on Whether the fan output should be turned on or off.
 */
void set_fan(bool on)
{
  // Fan is on PA8
  if (on)
  {
    GPIOA->ODR &= ~GPIO_ODR_OD8;
  }
  // Fan is off
  else
  {
    GPIOA->ODR |= GPIO_ODR_OD8;
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
    GPIOA->ODR &= ~GPIO_ODR_OD3;
  }
  // Light is off
  else
  {
    GPIOA->ODR |= GPIO_ODR_OD3;
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
  uint16_t adc_sample = 0;

  // Trigger ADC conversion
  ADC3->CR2 |= ADC_CR2_SWSTART;

  // Wait for conversion to complete
  while ((ADC3->SR & ADC_SR_EOC) == 0x00)
    ;

  // Get value from ADC
  adc_sample = ADC3->DR & 0x0000FFFF;

  return (int)(adc_sample);
}

/**
 * @brief Gets the light intensity based on 50ms debounce.
 *
 * @return The light intensity.
 */
bool get_light_intensity(void)
{
  // Light Intensity Sensor is on PA10
  bool pressed = get_light_intensity_gpio();

  // No debounce required as this is not a "user pressed input"
  return pressed;
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
  bool pressed = get_fan_switch_gpio();

  // If a full down up debounced press is detected this will be set to true
  bool confirmed_press = false;

  // Switch is active low, return inverted value of the switch state
  // If 50ms timer is started, expired AND switch isn't pressed
  // If it IS being pressed
  if (pressed)
  {
    // and it was not being held previously
    if (!fan_held)
    {
      // (it has gone from not pressed to pressed...)
      // Record fan as held
      fan_held = true;
      start_TIM7(count_from_delay_ms_TIM7(50));
      // No full press confirmed
    }
  }
  // If fan switch is not pressed
  else
  {
    // And it was being held
    if (fan_held)
    {
      // It has gone from pressed to not pressed
      fan_held = false;

      // If 50ms timer has expired
      if (TIM7_expired())
      {
        // Full press confirmed
        confirmed_press = true;
      }
    }
  }
  return confirmed_press;
}

/**
 * @brief Gets the fan switch state.
 *
 * @return The fan switch state.
 */
bool get_fan_switch_gpio(void)
{
  // Fan Switch is on PB1
  // Switch is active low, return inverted value of the switch state
  return (bool)!(GPIOB->IDR & GPIO_IDR_ID1_Msk);
}

/**
 * @brief Gets the light switch state based on 50ms debounce.
 *
 * @return The light switch state.
 */
bool get_light_switch(void)
{
  // Fan Switch is on PB0
  bool pressed = get_light_switch_gpio();

  // If a full down up debounced press is detected this will be set to true
  bool confirmed_press = false;

  // Switch is active low, return inverted value of the switch state
  // If 50ms timer is started, expired AND switch isn't pressed
  // If it IS being pressed
  if (pressed)
  {
    // and it was not being held previously
    if (!light_held)
    {
      // (it has gone from not pressed to pressed...)
      // Record fan as held
      light_held = true;
      start_TIM7(count_from_delay_ms_TIM7(50));
      // No full press confirmed
    }
  }
  // If fan switch is not pressed
  else
  {
    // And it was being held
    if (light_held)
    {
      // It has gone from pressed to not pressed
      light_held = false;

      // If 50ms timer has expired
      if (TIM7_expired())
      {
        // Full press confirmed
        confirmed_press = true;
      }
    }
  }
  return confirmed_press;
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
  volatile bool expired = (TIM6->SR & TIM_SR_UIF);
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
  volatile bool expired = (TIM7->SR & TIM_SR_UIF);
  // Clear UIF overflow flag
  TIM7->SR &= ~(TIM_SR_UIF_Msk);
  return expired;
}

bool USART_control_timer_expired(void)
{
  // Return if the timer is > 15s and clear USART
  if (usart_control_timer_count >= USART_CONTROL_TIMEOUT)
  {
    usart_control = false;
    usart_control_timer_count = 0;
    fan_output = false;
    return true;
  }
  return false;
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
  TIM6->CR1 &= ~TIM_CR1_CEN;

  // Set autoreload register and enable counter
  TIM6->ARR &= ~(TIM_ARR_ARR_Msk);                       // Clear autoreload register
  TIM6->ARR |= ((unsigned int)count << TIM_ARR_ARR_Pos); // Set autoreload register

  // Clear UIF overflow flag
  TIM6->SR &= ~(TIM_SR_UIF_Msk);
  TIM6->CR1 |= TIM_CR1_CEN; // Enable counter
}

/**
 * @brief Starts TIM7 with the specified count value.
 *
 * @param count The count value.
 */
void start_TIM7(uint16_t count)
{
  // Ensure timer is off
  TIM7->CR1 &= ~TIM_CR1_CEN;

  // Clear UIF overflow flag
  TIM7->SR &= ~(TIM_SR_UIF_Msk);

  // Set autoreload register and enable counter
  TIM7->ARR &= ~(TIM_ARR_ARR_Msk);                       // Clear autoreload register
  TIM7->ARR |= ((unsigned int)count << TIM_ARR_ARR_Pos); // Set autoreload register
  TIM7->CR1 |= TIM_CR1_CEN;                              // Enable counter
}

/**
 * @brief Handles the cooling/heating logic.
 *
 * This function checks the cooling input and updates the cooling and heating outputs accordingly.
 * If the cooling input is true, the cooling output is toggled and the heating output is set to the opposite value.
 *
 * @return The current value of the cooling output.
 */
void handle_auto_thermostat(void)
{
  // Set the heating and cooling based on the temperature based on HEAT_ON_TEMP and COOL_ON_TEMP
  // If the temperature is above COOL_ON_TEMP then turn on cooling and turn off heating
  // If the temperature is below HEAT_ON_TEMP then turn on heating and turn off cooling
  // If the temperature is between HEAT_ON_TEMP and COOL_ON_TEMP then turn off both heating and cooling
  if (get_temperature() > COOL_ON_TEMP)
  {
    cooling_output = true;
    heating_output = false;
  }
  else if (get_temperature() < HEAT_ON_TEMP)
  {
    cooling_output = false;
    heating_output = true;
  }
  else
  {
    cooling_output = false;
    heating_output = false;
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
    if (fan_timer_count >= FAN_TIMER_TIMEOUT)
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
  else if (fan_switch)
  {
    fan_timer_active = true;
    fan_timer_count = 0;
    return false;
  }
  // If fan switch not hit and timeout not running then return previous value
  else
  {
    return true;
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
bool handle_light(void)
{
  // If Light switch was confirmed to be hit (With 50ms debounce)
  if (light_switch)
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
  }
  else
  {
    return light_output;
  }
}
