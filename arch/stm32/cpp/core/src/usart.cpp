/*
	usart.h - Implementation of the USART functionality.
	
*/


#include <usart.h>

#include <vector>


// --- USART DEVICES ---
std::vector<USART_device>* USART_list() {
	USART_device device;
	static std::vector<USART_device>* devicesStatic = new std::vector<USART_device>(12, device);
	
#if defined RCC_APB2ENR_USART1EN
	((*devicesStatic))[USART_1].regs = USART1;
	((*devicesStatic))[USART_1].irqType = USART1_IRQn;
#endif

#if defined RCC_APB1ENR_USART2EN
	((*devicesStatic))[USART_2].regs = USART2;
	((*devicesStatic))[USART_2].irqType = USART2_IRQn;
#endif

#if defined RCC_APB1ENR_USART3EN
	((*devicesStatic))[USART_3].regs = USART3;
	((*devicesStatic))[USART_3].irqType = USART3_IRQn;
#endif

#if defined RCC_APB1ENR_UART4EN
	((*devicesStatic))[USART_4].regs = USART4;
	((*devicesStatic))[USART_4].irqType = USART4_IRQn;
#endif

#if defined RCC_APB1ENR_UART5EN
	((*devicesStatic))[USART_5].regs = USART5;
	((*devicesStatic))[USART_5].irqType = USART5_IRQn;
#endif

#if defined RCC_APB2ENR_USART6EN
	((*devicesStatic))[USART_6].regs = USART6;
	((*devicesStatic))[USART_6].irqType = USART6_IRQn;
#endif
	
	return devicesStatic;
}

static std::vector<USART_device>* devicesStatic = USART_list();


GPIO USART::gpio;


// Callback handlers.
// Overrides the default handlers and allows the use of custom callback functions.
extern "C" {
	void USART1_IRQHandler(void);
	void USART2_IRQHandler(void);
	void USART3_IRQHandler(void);
	void USART4_IRQHandler(void);
	void USART5_IRQHandler(void);
	void USART6_IRQHandler(void);
}

volatile char rxb = 'a';

#if defined __stm32f0 || defined __stm32f7

void USART1_IRQHandler(void) {
	USART_device &instance = (*devicesStatic)[0];
	if (instance.regs->ISR & USART_ISR_RXNE) {
		instance.regs->TDR = 'h';
		rxb = instance.regs->RDR;
		instance.callback(rxb);
	}
}

void USART2_IRQHandler(void) {
	USART_device &instance = (*devicesStatic)[1];
	if (instance.regs->ISR & USART_ISR_RXNE) {
		rxb = instance.regs->RDR;
		instance.callback(rxb);
	}
}

void USART3_IRQHandler(void) {
	USART_device &instance = (*devicesStatic)[2];
	if (instance.regs->ISR & USART_ISR_RXNE) {
		rxb = instance.regs->RDR;
		instance.callback(rxb);
	}
}

void USART4_IRQHandler(void) {
	USART_device &instance = (*devicesStatic)[3];
	if (instance.regs->ISR & USART_ISR_RXNE) {
		rxb = instance.regs->RDR;
		instance.callback(rxb);
	}
}

void USART5_IRQHandler(void) {
	USART_device &instance = (*devicesStatic)[4];
	if (instance.regs->ISR & USART_ISR_RXNE) {
		rxb = instance.regs->RDR;
		instance.callback(rxb);
	}
}

void USART6_IRQHandler(void) {
	USART_device &instance = (*devicesStatic)[5];
	if (instance.regs->ISR & USART_ISR_RXNE) {
		rxb = instance.regs->RDR;
		instance.callback(rxb);
	}
}

#elif defined __stm32f4

void USART1_IRQHandler(void) {
	USART_device &instance = (*devicesStatic)[0];
	if (instance.regs->SR & USART_SR_RXNE) {
		rxb = instance.regs->DR;
		instance.callback(rxb);
	}
}

void USART2_IRQHandler(void) {
	USART_device &instance = (*devicesStatic)[1];
	if (instance.regs->SR & USART_SR_RXNE) {
		rxb = instance.regs->DR;
		instance.callback(rxb);
	}
}

void USART3_IRQHandler(void) {
	USART_device &instance = (*devicesStatic)[2];
	if (instance.regs->SR & USART_SR_RXNE) {
		rxb = instance.regs->DR;
		instance.callback(rxb);
	}
}

void USART4_IRQHandler(void) {
	USART_device &instance = (*devicesStatic)[3];
	if (instance.regs->SR & USART_SR_RXNE) {
		rxb = instance.regs->DR;
		instance.callback(rxb);
	}
}

void USART5_IRQHandler(void) {
	USART_device &instance = (*devicesStatic)[4];
	if (instance.regs->SR & USART_SR_RXNE) {
		rxb = instance.regs->DR;
		instance.callback(rxb);
	}
}

void USART6_IRQHandler(void) {
	USART_device &instance = (*devicesStatic)[5];
	if (instance.regs->SR & USART_SR_RXNE) {
		rxb = instance.regs->DR;
		instance.callback(rxb);
	}
}

#endif


// --- START UART ---
bool USART::startUart(USART_devices device, GPIO_ports tx_port, uint8_t tx_pin, uint8_t tx_af,
											GPIO_ports rx_port, uint8_t rx_pin, uint8_t rx_af,
											uint32_t baudrate, std::function<void(char)> callback) {
	// Check input parameters.
	// If device is already active, return false.
	// Otherwise try to activate it.
	if (tx_pin > 15 || rx_pin > 15) { return false;}
	if (tx_af > 7 || rx_af > 7) { return false; }
	USART_device &instance = (*devicesStatic)[device];
	RccPeripheral per;
	
	if (instance.active) { return true; }
	
	// Set AF mode on the specified pins.
	if (!gpio.set_af(tx_port, tx_pin, tx_af)) {
		Rcc::disablePort((RccPort) tx_port);
		return false;
	}
	
	// Set TX as high speed, push-pull.
	if (!gpio.set_output_parameters(tx_port, tx_pin, GPIO_PULL_UP, GPIO_PUSH_PULL, GPIO_HIGH)) {
		Rcc::disablePort((RccPort) tx_port);
		return false;
	}
		
	if (!gpio.set_af(rx_port, rx_pin, rx_af)) {
		Rcc::disablePort((RccPort) tx_port);
		Rcc::disablePort((RccPort) rx_port);
		return false;
	}
		
	if (!gpio.set_output_parameters(rx_port, rx_pin, GPIO_PULL_UP, GPIO_PUSH_PULL, GPIO_HIGH)) {
		Rcc::disablePort((RccPort) tx_port);
		Rcc::disablePort((RccPort) rx_port);
		return false;
	}
	
	// Set up and enable the USART peripheral.
	if (device == USART_1) { per = RCC_USART1; }
	else if (device == USART_2) { per = RCC_USART2; }
	else if (device == USART_3) { per = RCC_USART3; }
	else if (device == USART_4) { per = RCC_USART4; }
	else if (device == USART_5) { per = RCC_USART5; }
	else if (device == USART_6) { per = RCC_USART6; }
	
	if (!Rcc::enable(per)) { 
		Rcc::disablePort((RccPort) tx_port);
		Rcc::disablePort((RccPort) rx_port);
		return false;
	}
	
	instance.active = true;
	
	// Set the baud rate (BR register).
	// TODO: adjust for STM32F1.
	//instance.regs->BRR = SystemCoreClock / baudrate;
	uint16_t uartdiv = SystemCoreClock / baudrate;
#if defined __stm32f0 || defined __stm32f7
	instance.regs->BRR = (((uartdiv / 16) << USART_BRR_DIV_MANTISSA_Pos) |
							((uartdiv % 16) << USART_BRR_DIV_FRACTION_Pos));
#elif defined __stm32f4
	instance.regs->BRR = (((uartdiv / 16) << USART_BRR_DIV_Mantissa_Pos) |
							((uartdiv % 16) << USART_BRR_DIV_Fraction_Pos));
#endif
	
	// Enable the USART via its CR1 register.
	instance.regs->CR1 |= (USART_CR1_RE | USART_CR1_TE | USART_CR1_UE | USART_CR1_RXNEIE);
	
	// Save parameters.
	instance.tx_pin 	= tx_pin;
	instance.tx_port 	= tx_port;
	instance.tx_af 		= tx_af;
	instance.rx_pin 	= rx_pin;
	instance.rx_port 	= rx_port;
	instance.rx_af 		= rx_af;
	instance.baudrate 	= baudrate;
	instance.per		= per;
	instance.callback	= callback;
	
	// Configure interrupt.
	//instance.regs->CR1 |= USART_CR1_RXNEIE;
	NVIC_SetPriority(instance.irqType, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 1));
	NVIC_EnableIRQ(instance.irqType);
	
	return true;
}


// --- SEND UART ---
bool USART::sendUart(USART_devices device, char &ch) {
	USART_device &instance = (*devicesStatic)[device];
	if (!instance.active) { return false; }
	
	// Copy bit to the device's transmission register.
#if defined __stm32f0 || defined __stm32f7
	instance.regs->TDR = (uint8_t) ch;
#elif defined __stm32f4
	instance.regs->DR = (uint8_t) ch;
#endif
	
	return true;
}


// --- STOP UART ---
bool USART::stopUart(USART_devices device) {
	USART_device &instance = (*devicesStatic)[device];
	if (!instance.active) { return false; }
	
	// Disable interrupt.
	instance.regs->CR1 &= ~USART_CR1_RXNEIE;
	NVIC_DisableIRQ(instance.irqType);
	
	// Disable USART & ativated ports.
	instance.regs->CR1 &= ~(USART_CR1_RE | USART_CR1_TE | USART_CR1_UE | USART_CR1_RXNEIE);
	if (!Rcc::disable(instance.per)) { return false; }
	if (!Rcc::disablePort((RccPort) instance.tx_port)) { return false; }
	if (!Rcc::disablePort((RccPort) instance.rx_port)) { return false; }
	
	return true;
}
