//
// PCF8591
//
#ifndef _PCF8591_H_
#define _PCF8591_H_

#include "Arduino.h"

class PCF8591 {
	private:
		uint8_t _i2cAddr;
		
	public:
		// ctor
		PCF8591 ();
		
		// begin
		void begin (uint8_t i2cAddr = 0);
		
		// analogRead
		uint8_t analogRead (const uint8_t pin);
};

#define PCF8591_ADDRESS 0x48

#endif // _PCF8591_H_
