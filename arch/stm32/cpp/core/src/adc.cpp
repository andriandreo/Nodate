/*
	adc.cpp - Implementation of the ADC module.
	
	2021/04/26, Maya Posch
*/


#ifdef NODATE_ADC_ENABLED

#include <adc.h>

const int adc_count = 3;

// --- ADC DEVICES ---
ADC_device* ADC_list() {
	ADC_device item;
	static ADC_device adc_devices[adc_count];
	for (int i = 0; i < adc_count; ++i) {
		adc_devices[i] = item;
	}
	
#ifdef RCC_APB2ENR_ADC1EN
	adc_devices[ADC_1].regs = ADC1;
#if defined __stm32f4 || defined __stm32f1 || defined __stm32f7
	adc_devices[ADC_1].irqType = ADC_IRQn;
#else
	adc_devices[ADC_1].irqType = ADC1_IRQn;
#endif
#endif

#ifdef RCC_APB2ENR_ADC2EN
	adc_devices[ADC_2].regs = ADC2;
#if defined __stm32f4 || defined __stm32f1 || defined __stm32f7
	adc_devices[ADC_2].irqType = ADC_IRQn;
#else
	adc_devices[ADC_2].irqType = ADC2_IRQn;
#endif
#endif

#ifdef RCC_APB2ENR_ADC3EN
	adc_devices[ADC_3].regs = ADC3;
#if defined __stm32f4 || defined __stm32f1 || defined __stm32f7
	adc_devices[ADC_3].irqType = ADC_IRQn;
#else
	adc_devices[ADC_3].irqType = ADC3_IRQn;
#endif
#endif
	
	return adc_devices;
}

ADC_device* adcList = ADC_list();


#ifdef __stm32f0
extern "C" {
	void ADC1_IRQHandler(void) {
		// Check which interrupt got triggered in ADC_ISR.
		// Trigger appropriate callback.
		ADC_device &instance = adcList[0];
		if (instance.regs->ISR & ADC_ISR_AWD1 == ADC_ISR_AWD1) {
			instance.cbs.watchdog();
			instance.regs->ISR |= ADC_ISR_AWD1;
		}
		else if (instance.regs->ISR & ADC_ISR_OVR == ADC_ISR_OVR) {
			instance.cbs.overrun();
			instance.regs->ISR |= ADC_ISR_OVR;
		}
		else if (instance.regs->ISR & ADC_ISR_EOS == ADC_ISR_EOS) {
			instance.cbs.eoseq();
			instance.regs->ISR |= ADC_ISR_EOS;
		}
		else if (instance.regs->ISR & ADC_ISR_EOC == ADC_ISR_EOC) {
			instance.cbs.eoc();
			if (instance.regs->ISR & ADC_ISR_EOC == ADC_ISR_EOC) {	// Was ADC_DR read?
				instance.regs->ISR |= ADC_ISR_EOC;	// Clear ISR flag.
			}
		}
		else if (instance.regs->ISR & ADC_ISR_EOSMP == ADC_ISR_EOSMP) {
			instance.cbs.eosmp();
			instance.regs->ISR |= ADC_ISR_EOSMP;
		}
		else if (instance.regs->ISR & ADC_ISR_ADRDY == ADC_ISR_ADRDY) {
			instance.cbs.ready();
			instance.regs->ISR |= ADC_ISR_ADRDY;
		}
	}
}
#endif


bool ADC::calibrate(ADC_devices device) {
	ADC_device &instance = adcList[device];
#ifdef __stm32f0
	// Ensure that the ADC is disabled.
	if ((instance.regs->CR & ADC_CR_ADEN) != 0) {
		instance.regs->CR |= ADC_CR_ADDIS; // Disable ADC.
	}
	
	uint32_t timeout = 400; // TODO: make configurable.
	uint32_t ts = McuCore::getSysTick();
	while ((instance.regs->CR & ADC_CR_ADEN) != 0) {
		if (((McuCore::getSysTick() - ts) > timeout) || timeout == 0) {
			// TODO: set status.
			return false;
		}
	}
	
	// Clear DMAEN.
	instance.regs->CFGR1 &= ~ADC_CFGR1_DMAEN;
	
	// Enable ADCAL to start the calibration.
	// The hardware will toggle the bit to 0 once calibration is complete.
	instance.regs->CR |= ADC_CR_ADCAL;
	ts = McuCore::getSysTick();
	while ((instance.regs->CR & ADC_CR_ADCAL) != 0) {
		if (((McuCore::getSysTick() - ts) > timeout) || timeout == 0) {
			// TODO: set status.
			return false;
		}
	}
						
	instance.calibrated = true;
	
	return true;
#else
	return false;
#endif
}


// --- CONFIGURE ---
// Perform basic setup on an ADC device.
bool ADC::configure(ADC_devices device, ADC_modes mode) {
	ADC_device &instance = adcList[device];
	if (instance.active) { return true; } // Already active.
	if (!instance.calibrated) { 
		if (!calibrate(device)) { return false; }
	}
	
#ifdef __stm32f0
	// Check status. Set parameters.
	if (device == ADC_1) 		{ instance.per = RCC_ADC1; }
	else if (device == ADC_2)	{ instance.per = RCC_ADC2; }
	else if (device == ADC_3)	{ instance.per = RCC_ADC3; }
	
	// Start ADC device if needed.
	if (!instance.active) {
		if (!Rcc::enable(instance.per)) {
			// TODO: set status.
			return false;
		}
	}
	
	// Select PCLK/2 by writing 01 in CKMODE.
	//instance.regs->CFGR2 |= ADC_CFGR2_CKMODE_0;
	
	// Select asynchronous clock source (CLKMODE 00).
	// This requires that the 14 MHz clock is configured in the RCC.
	// TODO: Make clock source configurable.
	instance.regs->CFGR2 &= ~ADC_CFGR2_CKMODE;
	RCC->CR2 |= RCC_CR2_HSI14ON;
	uint32_t timeout = 400; // TODO: make configurable.
	uint32_t ts = McuCore::getSysTick();
	while ((RCC->CR2 & RCC_CR2_HSI14RDY) == 0) {
		if (((McuCore::getSysTick() - ts) > timeout) || timeout == 0) {
			// TODO: set status.
			return false;
		}
	}
	
	// Select the continuous or single mode.
	if (mode == ADC_MODE_SINGLE) {
		instance.regs->CFGR1 &= ~ADC_CFGR1_CONT;
	}
	else {
		instance.regs->CFGR1 |= ADC_CFGR1_CONT;
	}
	
	instance.active = true;
	
	return true;
#else
	return false;
#endif
}


// --- CHANNEL ---
// Configure a specific channel.
bool ADC::channel(ADC_devices device, uint8_t channel, GPIO_ports port, uint8_t pin, uint8_t time) {
	ADC_device &instance = adcList[device];
	if (instance.sampling) { return false; } // Can't change channels while sampling.
	
#ifdef __stm32f0
	if (pin > 18) { return false; } // Only 19 channels available.
	
	// Set the target pin to analogue mode.
	GPIO::set_analog(port, pin);
	
	// Select the channel as active.
	instance.regs->CHSELR |= (1 << pin);
	
	// Set the sampling time. On F0 this is the same for all channels.
	if (time > 7) { return false; } // three bits value.
	instance.regs->SMPR = time;
	
	return true;
#else
	return false;
#endif
}


// --- CHANNEL ---
// Configure a specific channel, targeting the internal channels (Vsense, Vrefint).
bool ADC::channel(ADC_devices device, ADC_internal channel, uint8_t time) {
	ADC_device &instance = adcList[device];
	if (instance.sampling) { return false; } // Can't change channels while sampling.
	
#ifdef __stm32f0
	// Ensure the relevant device is enabled.
	if (channel == ADC_VSENSE) {
		// Enable TSEN in ADC_CCR.
		ADC1_COMMON->CCR |= ADC_CCR_TSEN;
		
		// Minimum sample rate for STM32F042 is 4 microseconds.
		// Set sample rate to 239.5 ADC cycles (14 MHz clock src) for >17 microseconds.
		instance.regs->SMPR |= 7; // b111, all bits set.
		
		// Use ADC channel 16.
		instance.regs->CHSELR |= (1 << 16);
	}
	else if (channel == ADC_VREFINT) {
		// Enable VREFEN in ADC_CCR.
		ADC1_COMMON->CCR |= ADC_CCR_VREFEN;
		
		// Use ADC channel 17.
		instance.regs->CHSELR |= (1 << 17);
	}
	else if (channel == ADC_VBAT) {
		// Enable VBATEN.
		ADC1_COMMON->CCR |= ADC_CCR_VBATEN;
		
		// Use channel 18.
		instance.regs->CHSELR |= (1 << 18);
	}
	else {
		// The universe broke. Again.
		return false;
	}
	
	// Set the sampling time. On F0 this is the same for all channels.
	if (time > 7) { return false; } // three bits value.
	instance.regs->SMPR = time;
	
	return true;
#else
	return false;
#endif
}


// --- ENABLE INTERRUPT ---
//
bool ADC::enableInterrupt(ADC_devices device, ADC_interrupts isr) {
	ADC_device &instance = adcList[device];
	if (instance.sampling) { return false; } // Can't change channels while sampling.
	
#ifdef __stm32f0
	// Enable interrupts for which a callback exists.
	if (isr.watchdog) {
		instance.regs->IER |= ADC_IER_AWDIE;
	}
	else if (isr.overrun) {
		instance.regs->IER |= ADC_IER_OVRIE;
	}
	else if (isr.eoseq) {
		instance.regs->IER |= ADC_IER_EOSEQIE;
	}
	else if (isr.eoc) {
		instance.regs->IER |= ADC_IER_EOCIE;
	}
	else if (isr.eosmp) {
		instance.regs->IER |= ADC_IER_EOSMPIE;
	}
	else if (isr.ready) {
		instance.regs->IER |= ADC_IER_ADRDYIE;
	}
	
	instance.cbs = isr;

	// Configure NVIC for ADC:
	// - Enable Interrupt.
	// - Set priority.
	NVIC_EnableIRQ(instance.irqType);
	NVIC_SetPriority(instance.irqType, 0);
	
	return true;
#else
	return false;
#endif
}


// --- DISABLE INTERRUPTS ---
//
bool ADC::disableInterrupts(ADC_devices device) {
	ADC_device &instance = adcList[device];
	if (instance.sampling) { return false; } // Can't alter interrupts while sampling.
	
#ifdef __stm32f0
	// Disable interrupts and reset ISR register.
	NVIC_DisableIRQ(instance.irqType);
	instance.regs->IER = 0x00;
	
	return true;
#else
	return false;
#endif
}
	

// --- START ---
// Start the ADC device.
bool ADC::start(ADC_devices device) {
	ADC_device &instance = adcList[device];
	if (!instance.active) { return false; }
	if (!instance.calibrated) { return false; }
	
#ifdef __stm32f0
	// Ensure that ADRDY is 0.
	if ((instance.regs->ISR & ADC_ISR_ADRDY) != 0) {
		instance.regs->ISR |= ADC_ISR_ADRDY;
	}
	
	// Enable the AD device.
	instance.regs->CR |= ADC_CR_ADEN;
	uint32_t timeout = 400; // TODO: make configurable.
	uint32_t ts = McuCore::getSysTick();
	while ((instance.regs->ISR & ADC_ISR_ADRDY) == 0) {
		if (((McuCore::getSysTick() - ts) > timeout) || timeout == 0) {
			// TODO: set status.
			return false;
		}
	}
	
	return true;
#else
	return false;
#endif
}
	
	
// --- START SAMPLING ---
bool ADC::startSampling(ADC_devices device) {
	ADC_device &instance = adcList[device];
	if (!instance.active) { return false; }
	if (!instance.calibrated) { return false; }
	
#ifdef __stm32f0
	// Start sampling.
	instance.regs->CR |= ADC_CR_ADSTART;
	
	instance.sampling = true;
	
	return true;
#else
	return false;
#endif
}


// --- GET VALUE ---
// Obtain the last sampled value.
bool ADC::getValue(ADC_devices device, uint16_t &val) {
	ADC_device &instance = adcList[device];
	if (!instance.active) { return false; }
	if (!instance.sampling) { return false; }
	
#ifdef __stm32f0
	uint32_t timeout = 400; // TODO: make configurable.
	uint32_t ts = McuCore::getSysTick();
	while ((instance.regs->ISR & ADC_ISR_EOC) == 0) {
		if (((McuCore::getSysTick() - ts) > timeout) || timeout == 0) {
			// TODO: set status.
			return false;
		}
	}
	
	val = instance.regs->DR;
	
	instance.sampling = false;
	
	return true;
#else
	return false;
#endif
}


// --- STOP ---
// Stop the ADC device.
bool ADC::stop(ADC_devices device) {
	ADC_device &instance = adcList[device];
	if (!instance.active) { return false; }
	if (!instance.calibrated) { return false; }
	
#ifdef __stm32f0
	instance.regs->CR |= ADC_CR_ADSTP;
	uint32_t timeout = 400; // TODO: make configurable.
	uint32_t ts = McuCore::getSysTick();
	while ((instance.regs->CR & ADC_CR_ADSTP) != 0) {
		if (((McuCore::getSysTick() - ts) > timeout) || timeout == 0) {
			// TODO: set status.
			return false;
		}
	}
	
	instance.regs->CR |= ADC_CR_ADDIS;
	ts = McuCore::getSysTick();
	while ((instance.regs->CR & ADC_CR_ADEN) != 0) {
		if (((McuCore::getSysTick() - ts) > timeout) || timeout == 0) {
			// TODO: set status.
			return false;
		}
	}
	
	return true;
#else
	return false;
#endif
}


#ifdef NODATE_DMA_ENABLED

// --- CONFIGURE DMA ---
bool ADC::configureDMA(ADC_devices device, uint32_t* buffer, uint16_t count, DMA_callbacks cb) {
	ADC_device &instance = adcList[device];
	if (!instance.active) { return false; }
	if (!instance.calibrated) { return false; }
	
#ifdef __stm32f0
	// Enable DMA transfer on ADC and circular mode.
	instance.regs->CFGR1 |= ADC_CFGR1_DMAEN | ADC_CFGR1_DMACFG;
	
	DMA_config cfg;
	cfg.channel = 1;
	cfg.source = (uint32_t*) &(instance.regs->DR);
	cfg.target = buffer;
	cfg.prio = DMA_PRIO_MEDIUM;
	cfg.count = count;
	cfg.src_size = 2;
	cfg.des_size = 2;
	cfg.circular = true;
	cfg.src_incr = false;
	cfg.des_incr = false;
	DMA::configureChannel(DMA_1, cfg, cb);
	
	DMA::start(DMA_1);
	
	return true;
#else
	return false;
#endif
}


// --- STOP DMA ---
// Terminate DMA transfer.
bool ADC::stopDMA(ADC_devices device) {
	ADC_device &instance = adcList[device];
	if (!instance.active) { return false; }
	
#ifdef __stm32f0
	instance.regs->CFGR1 &= ~ADC_CFGR1_DMAEN;
	DMA::abort(DMA_1, 1);
	
	return true;
#else
	return false;
#endif
}


#endif


#endif
