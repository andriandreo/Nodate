// I2C library collection - ADS1115 I2C device class
// Based on Texas Instruments ADS1113/4/5 datasheet, May 2009 (SBAS444B, revised October 2009)
// Note that the ADS1115 uses 16-bit registers, not 8-bit registers.
// 8/2/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/I2Clib
//
// Changelog:
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
  //setMultiplexer(ADS1115_MUX_P0_N1);
  //setGain(ADS1115_PGA_2P048);
  //setMode(ADS1115_MODE_SINGLESHOT);
  //setRate(ADS1115_RATE_128);
  //setComparatorMode(ADS1115_COMP_MODE_HYSTERESIS);
  //setComparatorPolarity(ADS1115_COMP_POL_ACTIVE_LOW);
  //setComparatorLatchEnabled(ADS1115_COMP_LAT_NON_LATCHING);
  //setComparatorQueueMode(ADS1115_COMP_QUE_DISABLE);

  uint8_t data[3];
  data[0] = 0x01;
  //data[1] = 0x84; //Differential: 0P - 1N.
  data[1] = 0xC4; //Single-ended: 0P - GND. 
  data[2] = 0x83;
  if (!write(data, 3)) { return false; }

  return true;
}

/** Verify the I2C connection. 
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, false otherwise
 */
bool ADS1115::testConnection() {
    //return I2C::readWord(devAddr, ADS1115_RA_CONVERSION, buffer) == 1;
}

// --- GET VOLTAGE ---
bool ADS1115::getConversion(int16_t &rawV){
  // Send register to read to the device.
  uint8_t data[1];
  data[0] = 0x00;
  send(data, 1);

  // Initiate the read sequence
  uint8_t buffer[2];
  if (!receive(buffer, 2)) { return false; }
  rawV = static_cast<int16_t>((buffer[0] << 8) | buffer[1]);

  return true;
}

bool ADS1115::voltage(int16_t &mV){
  int16_t rawV;
  if (!getConversion(rawV)) { return false; }
  mV = rawV * 2048 / 32768;
  return true;
}

// --- SEND ---
// Perform a read request on a register.
bool ADS1115::send(uint8_t* data, uint16_t len) {
#ifdef NODATE_I2C_ENABLED
		// Use I2C send.
		I2C::setSlaveTarget(i2c_device, address);
		I2C::sendToSlave(i2c_device, data, len);
#endif
	
	return true;
}


// --- WRITE ---
// Perform a write action to a register.
bool ADS1115::write(uint8_t* data, uint16_t len) {
#ifdef NODATE_I2C_ENABLED
		// Use I2C send.
		I2C::setSlaveTarget(i2c_device, address);
		I2C::sendToSlave(i2c_device, data, len);
#endif
	
	return true;
}


// --- RECEIVE ---
bool ADS1115::receive(uint8_t* data, uint16_t len) {
#ifdef NODATE_I2C_ENABLED
		// Use I2C receive.
		I2C::receiveFromSlave(i2c_device, len, data);
#endif
	
	return true;
}


// --- TRANSCEIVE ---
// Combined send/receive.
bool ADS1115::transceive(uint8_t* txdata, uint16_t txlen, uint8_t* rxdata, uint16_t rxlen) {
#ifdef NODATE_I2C_ENABLED
		// Use I2C send and receive.
		I2C::setSlaveTarget(i2c_device, address);
		I2C::sendToSlave(i2c_device, txdata, txlen);
		I2C::receiveFromSlave(i2c_device, rxlen, rxdata);
#endif

	return true;
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

