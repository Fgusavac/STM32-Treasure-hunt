//#include "master.h"
//#include "stm32f303xc.h"
//
//
//// PE0 -> AF7 -> USART1_TX
//// PE1 -> AF7 -> USART1_RX
//
//// USART2: Discover Map
//// PB3 -> AF7 -> USART2_TX
//// PB4 -> AF7 -> USART2_RX
//
//// USART3: Map Control
//// PB10 -> AF7 -> USART3_TX
//// PB11 -> AF7 -> USART3_RX
//
//// UART4: Minigame 1
//// PC10 -> AF5 -> UART4_TX
//// PC11 -> AF5 -> UART4_RX
//
//// UART5: Minigame 2
//// PB12 -> AF5 -> UART5_TX
//// PD2 -> AF5 -> UART5_RX
//
//
//// USART2: PB3 (TX), PB4 (RX), AF7
//SerialPortMaster USART2_PORT = {
//	USART2,
//    GPIOB,
//    0x00, // APB2 enable not needed
//    RCC_APB1ENR_USART2EN,
//    RCC_AHBENR_GPIOBEN,
//    (0b10 << (3*2)) | (0b10 << (4*2)), // MODER: Alternate Function for PB3/PB4
//    (0b11 << (3*2)) | (0b11 << (4*2)), // OSPEEDR: High Speed for PB3/PB4
//    (0x7 << (3*4)) | (0x7 << (4*4)),   // AFR[0]: AF7 for PB3/PB4
//    0x00,
//    0x00,
//	((0b11 << (3*2)) | (0b11 << (4*2))),
//	((0b11 << (3*2)) | (0b11 << (4*2))),
//	((0xF << (3*4)) | (0xF << (4*4))),
//	0x00
//};
//
//// InitialiseSerial - Initialise the serial port
//// Input: baudRate is from an enumerated set
//void SerialInitialiseMaster(uint32_t baudRate, SerialPortMaster *serial_port, void (*completion_function)(uint32_t)) {
//
//	__disable_irq();
//
//	serial_port->completion_function = completion_function;
//
//	RCC->APB1ENR |= RCC_APB1ENR_PWREN;				// enable clock power, system configuration clock and GPIOC
//	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
//	RCC->AHBENR |= serial_port->MaskAHBENR;			// enable the GPIO which is on the AHB bus
//
//	// ---- SAFELY MODIFY GPIO SETTINGS ----
//	// MODER: Set alternate function mode for selected pins
//	serial_port->GPIO->MODER &= ~serial_port->SerialPinModeMask;  // Clear mode bits for the target pins
//	serial_port->GPIO->MODER |= serial_port->SerialPinModeValue;  // Set alternate function mode
//
//	// OSPEEDR: Set high-speed mode for the selected pins
//	serial_port->GPIO->OSPEEDR &= ~serial_port->SerialPinSpeedMask;  // Clear speed bits
//	serial_port->GPIO->OSPEEDR |= serial_port->SerialPinSpeedValue;  // Set high speed
//
//	// AFR[0] and AFR[1]: Set alternate function for selected pins
//	serial_port->GPIO->AFR[0] &= ~serial_port->SerialPinAlternatePinMaskLow;   // Clear relevant AFR[0] bits
//	serial_port->GPIO->AFR[0] |= serial_port->SerialPinAlternatePinValueLow;   // Set AFR[0] values
//
//	serial_port->GPIO->AFR[1] &= ~serial_port->SerialPinAlternatePinMaskHigh;  // Clear relevant AFR[1] bits
//	serial_port->GPIO->AFR[1] |= serial_port->SerialPinAlternatePinValueHigh;  // Set AFR[1] values
//
//	RCC->APB1ENR |= serial_port->MaskAPB1ENR;		// enable the device based on the bits defined in the serial port definition
//	RCC->APB2ENR |= serial_port->MaskAPB2ENR;
//
//	uint16_t *baud_rate_config = (uint16_t*)&serial_port->UART->BRR; 	// Get a pointer to the 16 bits of the BRR register that we want to change
//	switch(baudRate) {
//		case BAUD_9600:   *baud_rate_config = 0x341; break;	// 9600 at 8MHz   -> 833 -> 0x341
//		case BAUD_19200:  *baud_rate_config = 0x1A1; break; // 19200 at 8MHz  -> 417 -> 0x1A1
//		case BAUD_38400:  *baud_rate_config = 0xD0;  break; // 38400 at 8MHz  -> 208 -> 0xD0
//		case BAUD_57600:  *baud_rate_config = 0x8B;  break; // 57600 at 8MHz  -> 139 -> 0x8B
//		case BAUD_115200: *baud_rate_config = 0x46;  break; // 115200 at 8MHz -> 70  -> 0x46
//	}
//
//	// Enable serial port with TX, RX and RX interrupt
//	serial_port->UART->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
//	serial_port->UART->CR1 |= USART_CR1_RXNEIE;  // Enable RX interrupt
//
//    // Enable the corresponding NVIC interrupt
//    if (serial_port->UART == USART2) {
//    	NVIC_SetPriority(USART2_IRQn, 2);
//    	NVIC_EnableIRQ(USART2_IRQn);
//    }
//    else if (serial_port->UART == USART3) {
//    	NVIC_SetPriority(USART3_IRQn, 1);
//    	NVIC_EnableIRQ(USART3_IRQn);
//    }
//    else if (serial_port->UART == UART4) {
//    	NVIC_SetPriority(UART4_IRQn, 3);
//    	NVIC_EnableIRQ(UART4_IRQn);
//    }
//    else if (serial_port->UART == UART5) {
//    	NVIC_SetPriority(UART5_IRQn, 4);
//    	NVIC_EnableIRQ(UART5_IRQn);
//    }
//
//    __enable_irq();
//}
//
//// RECIEVING
//void RecieveChar(SerialPort* serial_port, uint8_t *data) {
//	// Check for overrun and frame errors and clear
//	while((serial_port->UART->ISR & (USART_ISR_ORE | USART_ISR_FE)) != 0){
//		serial_port->UART->ICR = USART_ICR_ORECF | USART_ICR_FECF; // Clear error flags
//	}
//
//	while ((serial_port->UART->ISR & USART_ISR_RXNE) == 0) {;;}
//	*data = serial_port->UART->RDR;
//}
//
//
//// TRANSMITTING
//void TransmitChar(SerialPort *serial_port, uint8_t data) {
//	while((serial_port->UART->ISR & USART_ISR_TXE) == 0){;;}
//	serial_port->UART->TDR = data;
//}
