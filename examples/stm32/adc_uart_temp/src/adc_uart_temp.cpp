/*
	adc_uart_temp.cpp - Implementation of ADC to UART for MCU temperature example.
	
	2022/04/20, Maya Posch
	2023/06/07, Andriandreo – Updated for STM32F103C8T6 (Blue Pill).
*/


#include <nodate.h>
#include <printf.h>

#ifdef __stm32f0
// Reference: RM0091, A.7.16.
// Note: calibration address may differ for other MCUs.
#define TEMP30_CAL_ADDR ((uint16_t*) ((uint32_t) 0x1FFFF7B8))
#define TEMP110_CAL_ADDR ((uint16_t*) ((uint32_t) 0x1FFFF7C2))
#define VDD_CALIB ((uint16_t) (330))
#define VDD_APPLI ((uint16_t) (330))
#elif defined __stm32f3
#define TS_CAL_30 ((uint16_t*) ((uint32_t) 0x1FFFF7B8))
#define TS_CAL_110 ((uint16_t*) ((uint32_t) 0x1FFFF7C2))
#elif defined __stm32f4
#define TS_CAL_30 ((uint16_t*) ((uint32_t) 0x1FFF7A2C))
#define TS_CAL_110 ((uint16_t*) ((uint32_t) 0x1FFF7A2E))
#elif defined __stm32f1 // NOTE: Keep in mind the 32-bit limitation and no FPU hardware.
#define ADC_V25 ((uint32_t) (1430)) // 1430 mV = 1.43 V at 25 ºC. Using fixed-point arithmetic as FPU hardware is not available in Blue Pill.
#define ADC_AVG_SLOPE ((uint16_t) (4300)) // 4300 µV/ºC = 4.3 mV/ºC. Using fixed-point arithmetic as FPU hardware is not available in Blue Pill.
#endif


void uartCallback(char ch) {
	//USART::sendUart(usartTarget, ch);
}


int main() {
	// 1. Set up UART
	
	// Blue Pill (STM32F103): USART1 (TX: PA9 (AF0), RX: PA10 (AF0)).
	// USART 1, (TX) PA9:0, (RX) PA10:0.
	USART::startUart(USART_1, GPIO_PORT_A, 9, 0, GPIO_PORT_A, 10, 0, 9600, uartCallback);
	// USART 1 – REMAPPED, PB6:1 (TX), PB7:1 (RX). 
	//USART::startUart(USART_1, GPIO_PORT_B, 6, 1, GPIO_PORT_B, 7, 1, 9600, uartCallback);
								
	//const uint8_t led_pin = 13;	// Blue Pill: Port C, pin 13.
	//const GPIO_ports led_port = GPIO_PORT_C;


	// Set up stdout.
	IO::setStdOutTarget(USART_1);
	
	// Start SysTick.
	McuCore::initSysTick();
	
	printf("Starting ADC & USART example...\n");
	
	// IMPORTANT DEBUG NOTE: By default, STM32 peripherals are powered down (bits are set to 0).
	// Thus ADC1->CR2 is 0, and `ADC1->CR2 |= 1;` will not work. ADC must be powered on first to r/w.
	// This is done by setting the RCC_APB2ENR bit for ADC1 to 1, which is done in this code by `ADC::configure`.
	// So no matter what you do, you cannot enable the ADON bit before calling `ADC::configure`.

		//	printf("RCC->APB2ENR: %d.\n", RCC->APB2ENR); // DEBUG: USART and IOPA (USART I/O pins) bits set – AFIO not needed if no USART port is remapped.

	// 2. Set up ADC – `calibrate()` calibration function is called during `ADC::configure()`.

	if (!ADC::configure(ADC_1, ADC_MODE_SINGLE)) {
		printf("ADC configure failed.\n");
		while (1) { }
	}

		//	printf("ADC1->CR2: %d.\n", ADC1->CR2); // DEBUG: 1 (ADON bit in Blue Pill – CAL bit is set during calibration, but RESET to 0 BY HARDWARE when done within `calibrate()`).
		//	printf("RCC->APB2ENR: %d.\n", RCC->APB2ENR); // DEBUG: USART, ADC1, IOPA bits set.
 	
	if (!ADC::channel(ADC_1, ADC_VSENSE, 7)) {	// Sample Vsense temperature sensor. Long sampling time (7 seconds).
		printf("ADC channel configure failed.\n");
		while (1) { }
	}

		//	printf("ADC1->CR2: %d.\n", ADC1->CR2); // DEBUG: TSVREF bit + 1 (ADON bit).
		//	printf("ADC1->SQR3: %d.\n", ADC1->SQR3); // DEBUG: Regular Sequence Register 3 (< 7 conversions). Value 16 (ADC_VSENSE channel).
		//	printf("ADC1->SQR1: %d.\n", ADC1->SQR1); // DEBUG: Regular Sequence Register 1. Value 0 – not set yet for L[3:0] bits
	
	ADC::finishChannelConfig(ADC_1);

		//	printf("GPIOA->CRH: %d.\n", GPIOA->CRH); // DEBUG: All good by `gpio.cpp`.
		//	printf("GPIOA->CRL: %d.\n", GPIOA->CRL); // DEBUG: All good by `gpio.cpp`.
	
		//	printf("ADC1->SQR1: %d.\n", ADC1->SQR1); // DEBUG: Regular Sequence Register 1. Value 1 (SHOWN: 0 (?)) for L[3:0] bits (just 1 conversion).

	// 3. Start the ADC.

	//ADC1->CR2 |= 1;	// Start ADC conversion (ADON bit RE-set – Ref. Manual) for Debug Cal.

	//timer.delay(1); // Wait for (1 ms – more than) `tSTAB` (ADC startup time) – Ref. Manual, 11.12.3.

	//	OR consider `SWSTART` bit (ADC_CR2_SWSTART) to start conversion (Ref. Manual, 11.12.3).
	//ADC1->CR2 |= ADC_CR2_EXTSEL;
	//ADC1->CR2 |= ADC_CR2_EXTTRIG;
	//ADC1->CR2 |= ADC_CR2_SWSTART;

	if (!ADC::start(ADC_1)) {
		printf("ADC start fail.\n");
		while (1) { }
	}
	
	Timer timer;
	while (1) {
		timer.delay(5000);
	
		//	printf("ADC1->SR: %d.\n", ADC1->SR); // DEBUG: Check for EOC bit (end of conversion).
		//	printf("ADC1->DR: %d.\n", ADC1->DR); // DEBUG: Check for Data Register value (because probably `getValue()` is not working).
		
		// 4. Start sampling.
		if (!ADC::startSampling(ADC_1)) {
			printf("ADC start sampling failed.\n");
			while (1) { }
		}// vs. already done above (uncomment) by {RE-SETTING ADON bit} vs. {SWSTART}.

		timer.delay(1); // Wait for conversion time (tweak)
		
		// 4. Get the sampled value.
		//uint16_t raw = ADC1->DR;
		uint16_t raw;
		if (!ADC::getValue(ADC_1, raw)) {
			printf("ADC get value failed.\n");
			while (1) { }
		}
		
		// 5. Format and send to UART.
		printf("Raw: %d.\n", raw);
		
		// DEBUG
#ifdef __stm32f0
		printf("C30: %d.\n", *TEMP30_CAL_ADDR);
		printf("C110: %d.\n", *TEMP110_CAL_ADDR);
#elif __stm32f1
		printf("ADC_V25: %d mV.\n", ADC_V25);
		printf("ADC_AVG_SLOPE: %d µV/ºC.\n", ADC_AVG_SLOPE);
#else
		printf("C30: %d.\n", *TS_CAL_30);
		printf("C110: %d.\n", *TS_CAL_110);
#endif
		
		// 6. Calculate Celsius value.
		// Ref.: RM0091, A.7.16. Adapt for other MCUs.
		int32_t temperature;
#ifdef __stm32f0
		//temperature = (((int32_t) raw * VDD_APPLI / VDD_CALIB) - (int32_t) *TEMP30_CAL_ADDR);
		temperature = (((int16_t) raw) - *TEMP30_CAL_ADDR);
		temperature = temperature * (int32_t)(110 - 30);
		temperature = temperature / (int32_t)(*TEMP110_CAL_ADDR - *TEMP30_CAL_ADDR);
		temperature = temperature + 30;
		//temperature = (((110 - 30) * ((int32_t) raw - (int32_t) *TEMP30_CAL_ADDR)) / ((int32_t) *TEMP110_CAL_ADDR - (int32_t) *TEMP30_CAL_ADDR)) + 30;
#elif __stm32f1
		temperature = (((int32_t) raw) * (int32_t)(3300)); // 3300 mV = 3.3 V. NOTE: Unable to do this in µV (32-bit limit).
		temperature = temperature / (int32_t)(4095); // 4095 = 12-bit resolution.
		temperature = (ADC_V25 - temperature);
		temperature = temperature * (int32_t)(1000); // mV to µV.
		temperature = temperature / ADC_AVG_SLOPE;
		temperature = temperature + 25;
#else
		temperature = (((int16_t) raw) - *TS_CAL_30);
		temperature = temperature * (int32_t)(110 - 30);
		temperature = temperature / (int32_t)(*TS_CAL_110 - *TS_CAL_30);
		temperature = temperature + 30;
#endif
		
		// 7. Print out value.
		printf("Temp: %d °C.\n", temperature);
	}
}