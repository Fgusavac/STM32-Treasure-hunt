#ifndef SERIAL_PORT_HEADER
#define SERIAL_PORT_HEADER

#include <stdint.h>
#include "stm32f303xc.h"

#define NEWLINE '\n'
#define RETURN '\r' // Newline and carriage return defined here for convenience

// The user might want to select the baud rate
enum {
  BAUD_9600,
  BAUD_19200,
  BAUD_38400,
  BAUD_57600,
  BAUD_115200
};

// Defining the serial port struct
typedef struct _SerialPort SerialPortMaster;
struct _SerialPort {
	USART_TypeDef *UART;
	GPIO_TypeDef *GPIO;
	volatile uint32_t MaskAPB2ENR;	// mask to enable RCC APB2 bus registers
	volatile uint32_t MaskAPB1ENR;	// mask to enable RCC APB1 bus registers
	volatile uint32_t MaskAHBENR;	// mask to enable RCC AHB bus registers
	volatile uint32_t SerialPinModeValue;
	volatile uint32_t SerialPinSpeedValue;
	volatile uint32_t SerialPinAlternatePinValueLow;
	volatile uint32_t SerialPinAlternatePinValueHigh;
	void (*completion_function)(uint32_t);

	volatile uint32_t SerialPinModeMask;         // MODER mask for pins
	volatile uint32_t SerialPinSpeedMask;        // OSPEEDR mask
	volatile uint32_t SerialPinAlternatePinMaskLow;   // AFR[0] mask
	volatile uint32_t SerialPinAlternatePinMaskHigh;  // AFR[1] mask
};

// Declare external SerialPort instances
extern SerialPortMaster USART2_PORT;

// SerialInitialise - initialise the serial port
// Input: baud rate as defined in the enum
void SerialInitialiseMaster(uint32_t baudRate, SerialPortMaster *serial_port, void (*completion_function)(uint32_t));

void RecieveChar(SerialPortMaster* serial_port, uint8_t *data);
void TransmitChar(SerialPortMaster *serial_port, uint8_t data);

#endif
