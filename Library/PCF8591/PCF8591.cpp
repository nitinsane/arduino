//
// PCF8591
//

#include <Wire.h>
#ifdef __AVR__
#include <avr/pgmspace.h>
#endif
#include "PCF8591.h"

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

// ctor
PCF8591::PCF8591 ()
{
}

// begin
void PCF8591::begin (uint8_t addr)
{
	// i2cAddr cannot be higher than 7
	if (addr > 7)
		addr = 7;
	
	
	_i2cAddr = PCF8591_ADDRESS | addr;
	
	// initialize i2c library
	Wire.begin ();
}

uint8_t PCF8591::analogRead (const uint8_t pin)
{
	// there are only 4 pins. so value cannot be more than 3
	if (pin > 3)
		return 0;
	
	Wire.beginTransmission(_i2cAddr);
	
	Wire.write (pin);
	
	Wire.endTransmission ();
	
	Wire.requestFrom (_i2cAddr, 2);
	
	// read and discard first byte
	Wire.read ();
	
	// return 2nd byte
	return Wire.read ();
}
