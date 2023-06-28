// I2C library collection - ADS1115 I2C device class
// Based on Texas Instruments ADS1113/4/5 datasheet, May 2009 (SBAS444B, revised October 2009)
// Note that the ADS1115 uses 16-bit registers, not 8-bit registers.
// 8/2/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/I2Clib
//
// Changelog:
//     2023-06-28 - Fix functionality bugs (`Nodate` framework), andriandreo
//     2023-06-27 - Implementation for `Nodate` framework, andriandreo
//
//     2013-05-05 - Add debug information.  Rename methods to match datasheet.
//     2011-11-06 - added getVoltage, F. Farzanegan
//     2011-10-29 - added getDifferentialx() methods, F. Farzanegan
//     2011-08-02 - initial release
/* ============================================
I2C device library code is placed under the MIT license
Copyright (c) 2011 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#include "ADS1115.h"

/** Default constructor, uses default I2C address.
 * @see ADS1115_DEFAULT_ADDRESS
 */
ADS1115::ADS1115() {
    devAddr = ADS1115_DEFAULT_ADDRESS;
}

/** Specific address constructor.
 * @param address I2C address
 * @see ADS1115_DEFAULT_ADDRESS
 * @see ADS1115_ADDRESS_ADDR_GND
 * @see ADS1115_ADDRESS_ADDR_VDD
 * @see ADS1115_ADDRESS_ADDR_SDA
 * @see ADS1115_ADDRESS_ADDR_SDL
 */
ADS1115::ADS1115(I2C_devices device, uint8_t address) {
    ready = true;
    this->i2c_device = device;
    this->address = address;
    devAddr = address;
}

// --- IS READY ---
// Returns true if the sensor instance has been fully configured and is ready for read/write.
bool ADS1115::isReady() {
	return ready;
}

/** Power on and prepare for general usage.
 * This device is ready to use automatically upon power-up. It defaults to
 * single-shot read mode, P0/N1 mux (diff. mode), 2.048v gain, 128 samples/sec,
 * default comparator with hysterysis, active-low polarity, non-latching comparator,
 * and comparator-disabled operation. 
 */
bool ADS1115::initialize() {
    setMultiplexer(ADS1115_MUX_P0_N1);
    setGain(ADS1115_PGA_2P048);
    setMode(ADS1115_MODE_SINGLESHOT);
    setRate(ADS1115_RATE_128);
    setComparatorMode(ADS1115_COMP_MODE_HYSTERESIS);
    setComparatorPolarity(ADS1115_COMP_POL_ACTIVE_LOW);
    setComparatorLatchEnabled(ADS1115_COMP_LAT_NON_LATCHING);
    setComparatorQueueMode(ADS1115_COMP_QUE_DISABLE);

    // QUICKSTART EXAMPLE – DATASHEET
    //uint8_t data[3];
    //data[0] = 0x01;
    //data[1] = 0x84; //Differential: 0P - 1N. Continuous conversion mode.
    // //data[1] = 0xC4; //Single-ended: 0P - GND. Continuous conversion mode.
    //data[2] = 0x83;
    //if (!send(data)) { return false; }

  return true;
}

/** Verify the I2C connection. 
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, false otherwise
 */
bool ADS1115::testConnection() {
    reg = ADS1115_RA_CONVERSION;
    return receive(buffer) == 1;
}

/** Poll the operational status bit until the conversion is finished
 * Retry at most 'max_retries' times
 * conversion is finished, then return true;
 * @see ADS1115_CFG_OS_BIT
 * @return True if data is available, false otherwise
 */
bool ADS1115::pollConversion(uint16_t max_retries) {  
  for(uint16_t i = 0; i < max_retries; i++) {
    if (isConversionReady()) return true;
  }
  return false;
}

/** Read differential value based on current MUX configuration.
 * The default MUX setting sets the device to get the differential between the
 * AIN0 and AIN1 pins. There are 8 possible MUX settings, but if you are using
 * all four input pins as single-end voltage sensors, then the default option is
 * not what you want; instead you will need to set the MUX to compare the
 * desired AIN* pin with GND. There are shortcut methods (getConversion*) to do
 * this conveniently, but you can also do it manually with setMultiplexer()
 * followed by this method.
 *
 * In single-shot mode, this register may not have fresh data. You need to write
 * a 1 bit to the MSB of the CONFIG register to trigger a single read/conversion
 * before this will be populated with fresh data. This technique is not as
 * effortless, but it has enormous potential to save power by only running the
 * comparison circuitry when needed.
 *
 * @param triggerAndPoll If true (and only in singleshot mode) the conversion trigger 
 *        will be executed and the conversion results will be polled.
 * @return 16-bit signed differential value
 * @see getConversionP0N1();
 * @see getConversionPON3();
 * @see getConversionP1N3();
 * @see getConversionP2N3();
 * @see getConversionP0GND();
 * @see getConversionP1GND();
 * @see getConversionP2GND();
 * @see getConversionP3GND);
 * @see setMultiplexer();
 * @see ADS1115_RA_CONVERSION
 * @see ADS1115_MUX_P0_N1
 * @see ADS1115_MUX_P0_N3
 * @see ADS1115_MUX_P1_N3
 * @see ADS1115_MUX_P2_N3
 * @see ADS1115_MUX_P0_NG
 * @see ADS1115_MUX_P1_NG
 * @see ADS1115_MUX_P2_NG
 * @see ADS1115_MUX_P3_NG
 */
int16_t ADS1115::getConversion(bool triggerAndPoll) {
    if (triggerAndPoll && devMode == ADS1115_MODE_SINGLESHOT) {
      triggerConversion();
      pollConversion(I2CDEV_DEFAULT_READ_TIMEOUT);
    }
    reg = ADS1115_RA_CONVERSION;
    receive(buffer);
    return static_cast<int16_t>((buffer[0] << 8) | buffer[1]);
}
/** Get AIN0/N1 differential.
 * This changes the MUX setting to AIN0/N1 if necessary, triggers a new
 * measurement (also only if necessary), then gets the differential value
 * currently in the CONVERSION register.
 * @return 16-bit signed differential value
 * @see getConversion()
 */
int16_t ADS1115::getConversionP0N1() {
    if (muxMode != ADS1115_MUX_P0_N1) setMultiplexer(ADS1115_MUX_P0_N1);
    return getConversion();
}

/** Get AIN0/N3 differential.
 * This changes the MUX setting to AIN0/N3 if necessary, triggers a new
 * measurement (also only if necessary), then gets the differential value
 * currently in the CONVERSION register.
 * @return 16-bit signed differential value
 * @see getConversion()
 */
int16_t ADS1115::getConversionP0N3() {
    if (muxMode != ADS1115_MUX_P0_N3) setMultiplexer(ADS1115_MUX_P0_N3);
    return getConversion();
}

/** Get AIN1/N3 differential.
 * This changes the MUX setting to AIN1/N3 if necessary, triggers a new
 * measurement (also only if necessary), then gets the differential value
 * currently in the CONVERSION register.
 * @return 16-bit signed differential value
 * @see getConversion()
 */
int16_t ADS1115::getConversionP1N3() {
    if (muxMode != ADS1115_MUX_P1_N3) setMultiplexer(ADS1115_MUX_P1_N3);
    return getConversion();
}

/** Get AIN2/N3 differential.
 * This changes the MUX setting to AIN2/N3 if necessary, triggers a new
 * measurement (also only if necessary), then gets the differential value
 * currently in the CONVERSION register.
 * @return 16-bit signed differential value
 * @see getConversion()
 */
int16_t ADS1115::getConversionP2N3() {
    if (muxMode != ADS1115_MUX_P2_N3) setMultiplexer(ADS1115_MUX_P2_N3);
    return getConversion();
}

/** Get AIN0/GND differential.
 * This changes the MUX setting to AIN0/GND if necessary, triggers a new
 * measurement (also only if necessary), then gets the differential value
 * currently in the CONVERSION register.
 * @return 16-bit signed differential value
 * @see getConversion()
 */
int16_t ADS1115::getConversionP0GND() {
    if (muxMode != ADS1115_MUX_P0_NG) setMultiplexer(ADS1115_MUX_P0_NG);
    return getConversion();
}
/** Get AIN1/GND differential.
 * This changes the MUX setting to AIN1/GND if necessary, triggers a new
 * measurement (also only if necessary), then gets the differential value
 * currently in the CONVERSION register.
 * @return 16-bit signed differential value
 * @see getConversion()
 */
int16_t ADS1115::getConversionP1GND() {
    if (muxMode != ADS1115_MUX_P1_NG) setMultiplexer(ADS1115_MUX_P1_NG);
    return getConversion();
}
/** Get AIN2/GND differential.
 * This changes the MUX setting to AIN2/GND if necessary, triggers a new
 * measurement (also only if necessary), then gets the differential value
 * currently in the CONVERSION register.
 * @return 16-bit signed differential value
 * @see getConversion()
 */
int16_t ADS1115::getConversionP2GND() {
    if (muxMode != ADS1115_MUX_P2_NG) setMultiplexer(ADS1115_MUX_P2_NG);
    return getConversion();
}
/** Get AIN3/GND differential.
 * This changes the MUX setting to AIN3/GND if necessary, triggers a new
 * measurement (also only if necessary), then gets the differential value
 * currently in the CONVERSION register.
 * @return 16-bit signed differential value
 * @see getConversion()
 */
int16_t ADS1115::getConversionP3GND() {
    if (muxMode != ADS1115_MUX_P3_NG) setMultiplexer(ADS1115_MUX_P3_NG);
    return getConversion();
}

/** Get the current voltage reading
 * Read the current differential and return it multiplied
 * by the constant for the current gain.  mV is returned to
 * increase the precision of the voltage
 * @param triggerAndPoll If true (and only in singleshot mode) the conversion trigger 
 *        will be executed and the conversion results will be polled.
 */
float ADS1115::getMilliVolts(bool triggerAndPoll) { // NEED `float` library [!!!]
  switch (pgaMode) { 
    case ADS1115_PGA_6P144:
      return (getConversion(triggerAndPoll) * ADS1115_MV_6P144);
      break;    
    case ADS1115_PGA_4P096:
      return (getConversion(triggerAndPoll) * ADS1115_MV_4P096);
      break;             
    case ADS1115_PGA_2P048:    
      return (getConversion(triggerAndPoll) * ADS1115_MV_2P048);
      break;       
    case ADS1115_PGA_1P024:     
      return (getConversion(triggerAndPoll) * ADS1115_MV_1P024);
      break;       
    case ADS1115_PGA_0P512:      
      return (getConversion(triggerAndPoll) * ADS1115_MV_0P512);
      break;       
    case ADS1115_PGA_0P256:           
    case ADS1115_PGA_0P256B:          
    case ADS1115_PGA_0P256C:      
      return (getConversion(triggerAndPoll) * ADS1115_MV_0P256);
      break;       
  }
}

/**
 * Return the current multiplier for the PGA setting.
 * 
 * This may be directly retreived by using getMilliVolts(),
 * but this causes an independent read.  This function could
 * be used to average a number of reads from the getConversion()
 * getConversionx() functions and cut down on the number of 
 * floating-point calculations needed.
 *
 */
 
float ADS1115::getMvPerCount() { // NEED `float` library [!!!]
  switch (pgaMode) {
    case ADS1115_PGA_6P144:
      return ADS1115_MV_6P144;
      break;    
    case ADS1115_PGA_4P096:
      return  ADS1115_MV_4P096;
      break;             
    case ADS1115_PGA_2P048:    
      return ADS1115_MV_2P048;
      break;       
    case ADS1115_PGA_1P024:     
      return ADS1115_MV_1P024;
      break;       
    case ADS1115_PGA_0P512:      
      return ADS1115_MV_0P512;
      break;       
    case ADS1115_PGA_0P256:           
    case ADS1115_PGA_0P256B:          
    case ADS1115_PGA_0P256C:      
      return ADS1115_MV_0P256;
      break;       
  }
}

// CONFIG register

/** Get operational status.
 * @return Current operational status (false for active conversion, true for inactive)
 * @see ADS1115_RA_CONFIG
 * @see ADS1115_CFG_OS_BIT
 */
bool ADS1115::isConversionReady() {
    reg = ADS1115_RA_CONFIG;
    if (!receive(buffer)){ return false; }
    buffer[0] &= (1 << (ADS1115_CFG_OS_BIT - 8));
    buffer[0] = buffer[0] >> (ADS1115_CFG_OS_BIT - 8);
    return buffer[0];
}
/** Trigger a new conversion.
 * Writing to this bit will only have effect while in power-down mode (no conversions active).
 * @see ADS1115_RA_CONFIG
 * @see ADS1115_CFG_OS_BIT
 */
void ADS1115::triggerConversion() {
    reg = ADS1115_RA_CONFIG;
    receive(buffer);
    buffer [0] |= (1 << (ADS1115_CFG_OS_BIT - 8));
    send(buffer);
}
/** Get multiplexer connection.
 * @return Current multiplexer connection setting
 * @see ADS1115_RA_CONFIG
 * @see ADS1115_CFG_MUX_BIT
 * @see ADS1115_CFG_MUX_LENGTH
 */
uint8_t ADS1115::getMultiplexer() {
    reg = ADS1115_RA_CONFIG;
    receive(buffer);
    buffer[0] &= (0x07 << (ADS1115_CFG_MUX_BIT - ADS1115_CFG_MUX_LENGTH + 1 - 8)); // TODO: REWORK using functions below [!!!]
    buffer[0] = buffer[0] >> (ADS1115_CFG_MUX_BIT - ADS1115_CFG_MUX_LENGTH + 1 - 8); // TODO: REWORK using functions below [!!!]
    muxMode = (uint8_t)buffer[0];
    return muxMode;
}
/** Set multiplexer connection.  Continous mode may fill the conversion register
 * with data before the MUX setting has taken effect.  A stop/start of the conversion
 * is done to reset the values.
 * @param mux New multiplexer connection setting
 * @see ADS1115_MUX_P0_N1
 * @see ADS1115_MUX_P0_N3
 * @see ADS1115_MUX_P1_N3
 * @see ADS1115_MUX_P2_N3
 * @see ADS1115_MUX_P0_NG
 * @see ADS1115_MUX_P1_NG
 * @see ADS1115_MUX_P2_NG
 * @see ADS1115_MUX_P3_NG
 * @see ADS1115_RA_CONFIG
 * @see ADS1115_CFG_MUX_BIT
 * @see ADS1115_CFG_MUX_LENGTH
 */
void ADS1115::setMultiplexer(uint8_t mux) {
    reg = ADS1115_RA_CONFIG;
    // Clear bits
    receive(buffer);
    buffer[0] &= ~(0x07 << (ADS1115_CFG_MUX_BIT - ADS1115_CFG_MUX_LENGTH + 1 - 8)); // TODO: REWORK using functions below [!!!]
    // Set bits
    buffer[0] |= (mux << (ADS1115_CFG_MUX_BIT - ADS1115_CFG_MUX_LENGTH + 1 - 8));
    if (send(buffer)) {
        muxMode = mux;
        if (devMode == ADS1115_MODE_CONTINUOUS) {
          // Force a stop/start
          setMode(ADS1115_MODE_SINGLESHOT);
          getConversion();
          setMode(ADS1115_MODE_CONTINUOUS);
        }
    }
}
/** Get programmable gain amplifier level.
 * @return Current programmable gain amplifier level
 * @see ADS1115_RA_CONFIG
 * @see ADS1115_CFG_PGA_BIT
 * @see ADS1115_CFG_PGA_LENGTH
 */
uint8_t ADS1115::getGain() {
    reg = ADS1115_RA_CONFIG;
    receive(buffer);
    buffer[0] &= (0x07 << (ADS1115_CFG_PGA_BIT - ADS1115_CFG_PGA_LENGTH + 1 - 8)); // TODO: REWORK using functions below [!!!]
    buffer[0] = buffer[0] >> (ADS1115_CFG_PGA_BIT - ADS1115_CFG_PGA_LENGTH + 1 - 8); // TODO: REWORK using functions below [!!!]
    pgaMode=(uint8_t)buffer[0];
    return pgaMode;
}
/** Set programmable gain amplifier level.  
 * Continous mode may fill the conversion register
 * with data before the gain setting has taken effect.  A stop/start of the conversion
 * is done to reset the values.
 * @param gain New programmable gain amplifier level
 * @see ADS1115_PGA_6P144
 * @see ADS1115_PGA_4P096
 * @see ADS1115_PGA_2P048
 * @see ADS1115_PGA_1P024
 * @see ADS1115_PGA_0P512
 * @see ADS1115_PGA_0P256
 * @see ADS1115_RA_CONFIG
 * @see ADS1115_CFG_PGA_BIT
 * @see ADS1115_CFG_PGA_LENGTH
 */
void ADS1115::setGain(uint8_t gain) {
    reg = ADS1115_RA_CONFIG;
    // Clear bits
    receive(buffer);
    buffer[0] &= ~(0x07 << (ADS1115_CFG_PGA_BIT - ADS1115_CFG_PGA_LENGTH + 1 - 8)); // TODO: REWORK using functions below [!!!]
    // Set bits
    buffer[0] |= (gain << (ADS1115_CFG_PGA_BIT - ADS1115_CFG_PGA_LENGTH + 1 - 8));
    if (send(buffer)) {
        pgaMode = gain;
         if (devMode == ADS1115_MODE_CONTINUOUS) {
            // Force a stop/start
            setMode(ADS1115_MODE_SINGLESHOT);
            getConversion();
            setMode(ADS1115_MODE_CONTINUOUS);
         }
    }
}
/** Get device mode.
 * @return Current device mode
 * @see ADS1115_MODE_CONTINUOUS
 * @see ADS1115_MODE_SINGLESHOT
 * @see ADS1115_RA_CONFIG
 * @see ADS1115_CFG_MODE_BIT
 */
bool ADS1115::getMode() {
    reg = ADS1115_RA_CONFIG;
    receive(buffer);
    buffer[0] &= (1 << (ADS1115_CFG_MODE_BIT - 8)); // TODO: REWORK using functions below [!!!]
    buffer[0] = buffer[0] >> (ADS1115_CFG_MODE_BIT - 8); // TODO: REWORK using functions below [!!!]
    devMode = buffer[0];
    return devMode;
}
/** Set device mode.
 * @param mode New device mode
 * @see ADS1115_MODE_CONTINUOUS
 * @see ADS1115_MODE_SINGLESHOT
 * @see ADS1115_RA_CONFIG
 * @see ADS1115_CFG_MODE_BIT
 */
void ADS1115::setMode(bool mode) {
    reg = ADS1115_RA_CONFIG;
    // Clear bit
    receive(buffer);
    buffer[0] &= ~(1 << (ADS1115_CFG_MODE_BIT - 8)); // TODO: REWORK using functions below [!!!]
    // Set bit
    buffer[0] |= (mode << (ADS1115_CFG_MODE_BIT - 8));
    if(send(buffer)){
        devMode = mode;
    }
}
/** Get data rate.
 * @return Current data rate
 * @see ADS1115_RA_CONFIG
 * @see ADS1115_CFG_DR_BIT
 * @see ADS1115_CFG_DR_LENGTH
 */
uint8_t ADS1115::getRate() {
    reg = ADS1115_RA_CONFIG;
    receive(buffer);
    buffer[1] &= (0x07 << (ADS1115_CFG_DR_BIT - ADS1115_CFG_DR_LENGTH + 1)); // TODO: REWORK using functions below [!!!]
    buffer[1] = buffer[1] >> (ADS1115_CFG_DR_BIT - ADS1115_CFG_DR_LENGTH + 1); // TODO: REWORK using functions below [!!!]
    return (uint8_t)buffer[1];
}
/** Set data rate.
 * @param rate New data rate
 * @see ADS1115_RATE_8
 * @see ADS1115_RATE_16
 * @see ADS1115_RATE_32
 * @see ADS1115_RATE_64
 * @see ADS1115_RATE_128
 * @see ADS1115_RATE_250
 * @see ADS1115_RATE_475
 * @see ADS1115_RATE_860
 * @see ADS1115_RA_CONFIG
 * @see ADS1115_CFG_DR_BIT
 * @see ADS1115_CFG_DR_LENGTH
 */
void ADS1115::setRate(uint8_t rate) {
    reg = ADS1115_RA_CONFIG;
    // Clear bits
    receive(buffer);
    buffer[1] &= ~(0x07 << (ADS1115_CFG_DR_BIT - ADS1115_CFG_DR_LENGTH + 1)); // TODO: REWORK using functions below [!!!]
    // Set bits
    buffer[1] |= (rate << (ADS1115_CFG_DR_BIT - ADS1115_CFG_DR_LENGTH + 1));
    send(buffer);
}
/** Get comparator mode.
 * @return Current comparator mode
 * @see ADS1115_COMP_MODE_HYSTERESIS
 * @see ADS1115_COMP_MODE_WINDOW
 * @see ADS1115_RA_CONFIG
 * @see ADS1115_CFG_COMP_MODE_BIT
 */
bool ADS1115::getComparatorMode() {
    reg = ADS1115_RA_CONFIG;
    receive(buffer);
    buffer[1] &= (1 << ADS1115_CFG_COMP_MODE_BIT); // TODO: REWORK using functions below [!!!]
    buffer[1] = buffer[1] >> ADS1115_CFG_COMP_MODE_BIT; // TODO: REWORK using functions below [!!!]
    return (uint8_t)buffer[1];
}
/** Set comparator mode.
 * @param mode New comparator mode
 * @see ADS1115_COMP_MODE_HYSTERESIS
 * @see ADS1115_COMP_MODE_WINDOW
 * @see ADS1115_RA_CONFIG
 * @see ADS1115_CFG_COMP_MODE_BIT
 */
void ADS1115::setComparatorMode(bool mode) {
    reg = ADS1115_RA_CONFIG;
    // Clear bit
    receive(buffer);
    buffer[1] &= ~(1 << ADS1115_CFG_COMP_MODE_BIT); // TODO: REWORK using functions below [!!!]
    // Set bit
    buffer[1] |= (mode << ADS1115_CFG_COMP_MODE_BIT);
    send(buffer);
}
/** Get comparator polarity setting.
 * @return Current comparator polarity setting
 * @see ADS1115_COMP_POL_ACTIVE_LOW
 * @see ADS1115_COMP_POL_ACTIVE_HIGH
 * @see ADS1115_RA_CONFIG
 * @see ADS1115_CFG_COMP_POL_BIT
 */
bool ADS1115::getComparatorPolarity() {
    reg = ADS1115_RA_CONFIG;   
    receive(buffer);
    buffer[1] &= (1 << ADS1115_CFG_COMP_POL_BIT); // TODO: REWORK using functions below [!!!]
    buffer[1] = buffer[1] >> ADS1115_CFG_COMP_POL_BIT; // TODO: REWORK using functions below [!!!]
    return (uint8_t)buffer[1];
}
/** Set comparator polarity setting.
 * @param polarity New comparator polarity setting
 * @see ADS1115_COMP_POL_ACTIVE_LOW
 * @see ADS1115_COMP_POL_ACTIVE_HIGH
 * @see ADS1115_RA_CONFIG
 * @see ADS1115_CFG_COMP_POL_BIT
 */
void ADS1115::setComparatorPolarity(bool polarity) {
    reg = ADS1115_RA_CONFIG;
    // Clear bit
    receive(buffer);
    buffer[1] &= ~(1 << ADS1115_CFG_COMP_POL_BIT); // TODO: REWORK using functions below [!!!]
    // Set bit
    buffer[1] |= (polarity << ADS1115_CFG_COMP_POL_BIT);
    send(buffer);
}
/** Get comparator latch enabled value.
 * @return Current comparator latch enabled value
 * @see ADS1115_COMP_LAT_NON_LATCHING
 * @see ADS1115_COMP_LAT_LATCHING
 * @see ADS1115_RA_CONFIG
 * @see ADS1115_CFG_COMP_LAT_BIT
 */
bool ADS1115::getComparatorLatchEnabled() {
    reg = ADS1115_RA_CONFIG;
    receive(buffer);
    buffer[1] &= (1 << ADS1115_CFG_COMP_LAT_BIT); // TODO: REWORK using functions below [!!!]
    buffer[1] = buffer[1] >> ADS1115_CFG_COMP_LAT_BIT; // TODO: REWORK using functions below [!!!]
    return (uint8_t)buffer[1];
}
/** Set comparator latch enabled value.
 * @param enabled New comparator latch enabled value
 * @see ADS1115_COMP_LAT_NON_LATCHING
 * @see ADS1115_COMP_LAT_LATCHING
 * @see ADS1115_RA_CONFIG
 * @see ADS1115_CFG_COMP_LAT_BIT
 */
void ADS1115::setComparatorLatchEnabled(bool enabled) {
    reg = ADS1115_RA_CONFIG;
    // Clear bit
    receive(buffer);
    buffer[1] &= ~(1 << ADS1115_CFG_COMP_LAT_BIT); // TODO: REWORK using functions below [!!!]
    // Set bit
    buffer[1] |= (enabled << ADS1115_CFG_COMP_LAT_BIT);
    send(buffer);
}
/** Get comparator queue mode.
 * @return Current comparator queue mode
 * @see ADS1115_COMP_QUE_ASSERT1
 * @see ADS1115_COMP_QUE_ASSERT2
 * @see ADS1115_COMP_QUE_ASSERT4
 * @see ADS1115_COMP_QUE_DISABLE
 * @see ADS1115_RA_CONFIG
 * @see ADS1115_CFG_COMP_QUE_BIT
 * @see ADS1115_CFG_COMP_QUE_LENGTH
 */
uint8_t ADS1115::getComparatorQueueMode() {
    reg = ADS1115_RA_CONFIG;
    receive(buffer);
    buffer[1] &= 0x03;
    return (uint8_t)buffer[1];
}
/** Set comparator queue mode.
 * @param mode New comparator queue mode
 * @see ADS1115_COMP_QUE_ASSERT1
 * @see ADS1115_COMP_QUE_ASSERT2
 * @see ADS1115_COMP_QUE_ASSERT4
 * @see ADS1115_COMP_QUE_DISABLE
 * @see ADS1115_RA_CONFIG
 * @see ADS1115_CFG_COMP_QUE_BIT
 * @see ADS1115_CFG_COMP_QUE_LENGTH
 */
void ADS1115::setComparatorQueueMode(uint8_t mode) {
    reg = ADS1115_RA_CONFIG;
    // Clear bit
    receive(buffer);
    buffer[1] &= ~(0x03);
    // Set bit
    buffer[1] |= mode;
    send(buffer);
}

// *_THRESH registers

/** Get low threshold value.
 * @return Current low threshold value
 * @see ADS1115_RA_LO_THRESH
 */
int16_t ADS1115::getLowThreshold() {
    reg = ADS1115_RA_LO_THRESH;
    receive(buffer);
    return ((buffer[0] << 8) | buffer[1]);
}
/** Set low threshold value.
 * @param threshold New low threshold value
 * @see ADS1115_RA_LO_THRESH
 */
void ADS1115::setLowThreshold(int16_t threshold) {
    reg = ADS1115_RA_LO_THRESH;
    buffer[0] = (threshold >> 8);
    buffer[1] = (threshold & 0xFF);
    send(buffer);
}
/** Get high threshold value.
 * @return Current high threshold value
 * @see ADS1115_RA_HI_THRESH
 */
int16_t ADS1115::getHighThreshold() {
    reg = ADS1115_RA_HI_THRESH;
    receive(buffer);
    return ((buffer[0] << 8) | buffer[1]);
}
/** Set high threshold value.
 * @param threshold New high threshold value
 * @see ADS1115_RA_HI_THRESH
 */
void ADS1115::setHighThreshold(int16_t threshold) {
    reg = ADS1115_RA_HI_THRESH;
    buffer[0] = (threshold >> 8);
    buffer[1] = (threshold & 0xFF);
    send(buffer);
}

/** Configures ALERT/RDY pin as a conversion ready pin.
 *  It does this by setting the MSB of the high threshold register to '1' and the MSB 
 *  of the low threshold register to '0'. COMP_POL and COMP_QUE bits will be set to '0'.
 *  Note: ALERT/RDY pin requires a pull up resistor.
 */
void ADS1115::setConversionReadyPinMode() {
    reg = ADS1115_RA_HI_THRESH;
    receive(buffer);
    buffer[0] = (1 << 8);
    send(buffer);
    reg = ADS1115_RA_LO_THRESH;
    receive(buffer);
    buffer[0] = 0x00;
    send(buffer);
    setComparatorPolarity(0);
    setComparatorQueueMode(0);
}

// Create a mask between two bits
unsigned createMask(unsigned a, unsigned b) {
   unsigned mask = 0;
   for (unsigned i=a; i<=b; i++)
       mask |= 1 << i;
   return mask;
}

uint16_t shiftDown(uint16_t extractFrom, int places) {
  return (extractFrom >> places);
}


uint16_t getValueFromBits(uint16_t extractFrom, int high, int length) {
   int low= high-length +1;
   uint16_t mask = createMask(low ,high);
   return shiftDown(extractFrom & mask, low); 
}

/** Show all the config register settings (DEBUG)
 */
uint16_t ADS1115::showConfigRegister() {
    reg = ADS1115_RA_CONFIG;
    receive(buffer);
    uint16_t configRegister = static_cast<uint16_t>((buffer[0] << 8) | buffer[1]);
    return configRegister;
}


// --- GET VOLTAGE ---
bool ADS1115::getConversion(int16_t &rawV){
    triggerConversion();

    reg = ADS1115_RA_CONVERSION;
    // Initiate the read sequence.
    if (!receive(buffer)) { return false; }
    rawV = static_cast<int16_t>((buffer[0] << 8) | buffer[1]);

    return true;
}

bool ADS1115::voltage(int16_t &mV){
    int16_t rawV;
    if (!getConversion(rawV)) { return false; }
    mV = rawV * 2048 / 32768;
    return true;
}


// --- SET REGISTER ---
// Set the register to R/W in the ADS1115
bool ADS1115::setRegister(uint8_t reg){
    // Send register to read to the device.
    buffer[0] = reg;
#ifdef NODATE_I2C_ENABLED
		// Use I2C send.
		I2C::setSlaveTarget(i2c_device, address);
		if(!I2C::sendToSlave(i2c_device, buffer, 1)) { return false; }
#endif

    return true;
}


// --- SEND ---
// Perform a send (write) request on a register.
bool ADS1115::send(uint8_t* data) {
    data[2] = buffer[1];
    data[1] = buffer[0];
    data[0] = reg;
#ifdef NODATE_I2C_ENABLED
		// Use I2C send.
		I2C::setSlaveTarget(i2c_device, address);
		I2C::sendToSlave(i2c_device, data, 3);
#endif
	
	return true;
}


// --- RECEIVE ---
bool ADS1115::receive(uint8_t* data) {
    setRegister(reg);
#ifdef NODATE_I2C_ENABLED
		// Use I2C receive.
		I2C::receiveFromSlave(i2c_device, 2, data);
#endif
	
	return true;
}


// --- TRANSCEIVE ---
// Combined send/receive.
bool ADS1115::transceive(uint8_t* txdata, uint16_t txlen, uint8_t* rxdata, uint16_t rxlen) { // TODO: Implement regarding `setRegister`
    setRegister(reg);
#ifdef NODATE_I2C_ENABLED
		// Use I2C send and receive.
		I2C::setSlaveTarget(i2c_device, address);
		I2C::sendToSlave(i2c_device, txdata, txlen);
		I2C::receiveFromSlave(i2c_device, rxlen, rxdata);
#endif

	return true;
}