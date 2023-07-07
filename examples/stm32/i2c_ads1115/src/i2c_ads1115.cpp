// ADS1115 example for Nodate's STM32 framework.

#include <usart.h>
#include <io.h>
#include <i2c.h>
#include <timer.h>
#include <ADS1115/ADS1115.h>

#include "printf.h"


void uartCallback(char ch) {
	// Copy character into send buffer.
	//USART::sendUart(USART_2, ch);
	USART::sendUart(USART_1, ch);
}


void i2cCallback(uint8_t byte) {
	// Unused.
}


int main () {
	// Initialise UART.
	// Nucleo-F042K6 (STM32F042): USART2 (TX: PA2 (AF1), RX: PA15 (AF1)).
	// USART2 is normally connected to USB (ST-Link) on the Nucleo board.
	//USART::startUart(USART_2, GPIO_PORT_A, 2, 1, GPIO_PORT_A, 15, 1, 9600, uartCallback);
	// USART 1, (TX) PA9:1 [D1], (RX) PA10:1 [D0].
	//USART::startUart(USART_1, GPIO_PORT_A, 9, 1, GPIO_PORT_A, 10, 1, 9600, uartCallback);
	
	// STM32F4-Discovery (STM32F407): USART2, (TX) PA2:7, (RX) PA3:7.
	//USART::startUart(USART_2, GPIO_PORT_A, 2, 7, GPIO_PORT_A, 3, 7, 9600, uartCallback);

	// Blue Pill (STM32F103): USART1 (TX: PA9 (AF0), RX: PA10 (AF0)).
	// USART 1, (TX) PA9:0, (RX) PA10:0.
	USART::startUart(USART_1, GPIO_PORT_A, 9, 0, GPIO_PORT_A, 10, 0, 9600, uartCallback);
	// USART 1 – REMAPPED, PB6:1 (TX), PB7:1 (RX). 
	//USART::startUart(USART_1, GPIO_PORT_B, 6, 1, GPIO_PORT_B, 7, 1, 9600, uartCallback);
	
	//const uint8_t led_pin = 3; // Nucleo-f042k6: Port B, pin 3.
	//const GPIO_ports led_port = GPIO_PORT_B;
	//const uint8_t led_pin = 13; // STM32F4-Discovery: Port D, pin 13 (orange)
	//const GPIO_ports led_port = GPIO_PORT_D;
	//const uint8_t led_pin = 7; // Nucleo-F746ZG: Port B, pin 7 (blue)
	//const GPIO_ports led_port = GPIO_PORT_B;
	const uint8_t led_pin = 13;	// Blue Pill: Port C, pin 13.
	const GPIO_ports led_port = GPIO_PORT_C;
	
	// Set the pin mode on the LED pin.
	GPIO::set_output(led_port, led_pin, GPIO_PULL_UP);
	GPIO::write(led_port, led_pin, GPIO_LEVEL_LOW);

	// Set up stdout.
	IO::setStdOutTarget(USART_1);
	
	printf("Starting I2C ADS1115 example...\n");
	char ch = 'S';
	USART::sendUart(USART_1, ch);
	
//	GPIO::set_output(GPIO_PORT_B, 6, GPIO_FLOATING); // DEBUG
//		printf("RCC->APB2ENR: %d.\n", RCC->APB2ENR); // DEBUG: USART, IOPA (USART I/O pins), IOPB (I2C1 pins) and IOPC (PC13 LED pin) bits set? – AFIO not needed if no USART port is remapped.

	
	// Start I2C.
	// Nucleo-F042K6: I2C_1, SCL -> PA11:5, SDA -> PA12:5.
	// Blue Pill: I2C_1, SCL -> PB6:0, SDA -> PB7:0.
	if (!I2C::startI2C(I2C_1, GPIO_PORT_B, 6, 0, GPIO_PORT_B, 7, 0)) {
		ch = 'p';
		USART::sendUart(USART_1, ch);
		while (1) { }
	}

	//	printf("RCC->APB2ENR: %d.\n", RCC->APB2ENR); // DEBUG: USART, IOPA (USART I/O pins), IOPB (I2C1 pins) and IOPC (PC13 LED pin) bits set? – AFIO not needed if no USART port is remapped.
		//RCC->APB2ENR |= RCC_APB2ENR_IOPBEN; // DEBUG: IOPB bit set?
	//	printf("RCC->APB2ENR: %d.\n", RCC->APB2ENR); // DEBUG: IOPB bit set?
		//RCC->APB1ENR |= RCC_APB1ENR_I2C1EN; // DEBUG: I2C1EN bit set?
	//	printf("RCC->APB1ENR: %d.\n", RCC->APB1ENR); // DEBUG: I2C1EN bit set?
	//	printf("GPIOA->CRL: %d.\n", GPIOA->CRL); // DEBUG: no bits set? – Config bits set to 'Floating input (reset state)'.
	//	printf("GPIOA->CRH: %d.\n", GPIOA->CRH); // DEBUG: bits 4-11 set (USART pins: A9 and A10)?
	//	printf("GPIOB->CRL: %d.\n", GPIOB->CRL); // DEBUG: bits 24-31 set (I2C1 pins: B6 and B7)?
	//	printf("GPIOB->CRH: %d.\n", GPIOB->CRH); // DEBUG: no bits set? – Config bits set to 'Floating input (reset state)'.
	
	// Start I2C in master mode.
	// Use Fast Mode. The ADS1115 ADC supports this and slower speeds.
	if (!I2C::startMaster(I2C_1, I2C_MODE_FM, i2cCallback)) {
		ch = 'm';
		USART::sendUart(USART_1, ch);
		while (1) { }
	}
	
	ch = 'R';
	USART::sendUart(USART_1, ch);
	//printf("I2C1->CR1: %d.\n", I2C1->CR1); // DEBUG
	//printf("I2C1->CR2: %d.\n", I2C1->CR2); // DEBUG
	//printf("I2C1->CCR: %d.\n", I2C1->CCR); // DEBUG
	//printf("I2C1->TRISE: %d.\n", I2C1->TRISE); // DEBUG

	
	// Create ADS1115 ADC instance, on the first I2C bus, slave address 0x48 (address pin low – GND)
    // Note that, depending on the connection of the ADS1115 addr. pin, the slave address changes – datasheet.
	ADS1115 adc(I2C_1, ADS1115_DEFAULT_ADDRESS);
	
	// Initialize the ADC IC.
	// This fetches the calibration factors from the ADC device.
	if (!adc.initialize()) {
		printf("ADC init failed!\n");
		while (1) { }
	}

	if (!adc.isReady()) {
		ch = 'n';
		USART::sendUart(USART_1, ch);
		while (1) { }
	}
	
	ch = 'R';
	USART::sendUart(USART_1, ch);
	
	Timer timer;
	timer.delay(1000);

	if (!adc.testConnection()) {
		printf("ADC connection failed!\n");
		while (1) { }
	}

	adc.setMode(ADS1115_MODE_CONTINUOUS);

	//--- DEBUG ---
	// Read and print out ADC Config.
	//printf("\rMUX settings: %d.\n\r", adc.getMultiplexer());
	//printf("PGA settings: %d.\n\r", adc.getGain());
	//printf("Mode settings: %d.\n\r", adc.getMode());
	//printf("Rate settings: %d.\n\r", adc.getRate());
	//printf("Comparator settings: %d.\n\r", adc.getComparatorMode());
	//printf("Comparator polarity settings: %d.\n\r", adc.getComparatorPolarity());
	//printf("Comparator latch settings: %d.\n\r", adc.getComparatorLatchEnabled());
	//printf("Comparator queue settings: %d.\n\r", adc.getComparatorQueueMode());
	//uint16_t config = 0;
	//config = adc.showConfigRegister();
	//GPIO::write(led_port, led_pin, GPIO_LEVEL_HIGH);
	//printf("ADC Config.: %d\n\r", config);
	//if (config == ADS1115_DEFAULT_CONFIG || config == ADS1115_DEFAULT_CONFIG2) {
	//	printf("Matches ADS1115 default config.\n\r");
	//}
	
	timer.delay(2000);
	
	ch = 'C';
	USART::sendUart(USART_1, ch);
	
	while (1){
	GPIO::write(led_port, led_pin, GPIO_LEVEL_HIGH);
	// Get raw voltage.
	timer.delay(500);
	GPIO::write(led_port, led_pin, GPIO_LEVEL_LOW);
	int16_t raw;
	if (!adc.getConversion(raw)) {
		printf("Reading raw conversion failed.\n");
		while (1) { }
	}
	
	printf("Raw conv.: %d.  ", raw);
	//printf("Raw conv. vanilla: %d.  ", adc.getConversion());
	
	
	// Read voltage.
	int16_t mV;
	if (!adc.voltage(mV)) {
		printf("Reading voltage failed.\n");
		while (1) { }
	}
	
	printf("Voltage: %d mV.\n\r", mV);
	timer.delay(500);
	}
	
	return 0;
}
