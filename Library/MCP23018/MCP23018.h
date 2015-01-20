// Only modify this file to include
// - function definitions (prototypes)
// - include files
// - extern variable definitions
// In the appropriate section

#ifndef _MCP23018_H_
#define _MCP23018_H_
#include "Arduino.h"

//add your includes for the project MCP23018 here
#include "Wire.h"

//end of add your includes here
#ifdef __cplusplus
extern "C" {
#endif
void loop();
void setup();
#ifdef __cplusplus
} // extern "C"
#endif

const struct {
	char *regName;
	uint8_t bank1Addr;
	uint8_t bank0Addr;
} addrMap [] = {
		{"IODIRA", 0x00, 0x00},
		{"IODIRB", 0x10, 0x01},
		{"IOPOLA", 0x01, 0x02},
		{"IOPOLB", 0x11, 0x03},
		{"GPINTENA", 0x02, 0x04},
		{"GPINTENB", 0x12, 0x05},
		{"DEFVALA", 0x03, 0x06},
		{"DEFVALB", 0x13, 0x07},
		{"INTCONA", 0x04, 0x08},
		{"INTCONB", 0x14, 0x09},
		{"IOCONA", 0x05, 0x0A},
		{"IOCONB", 0x15, 0x0B},
		{"GPPUA", 0x06, 0x0C},
		{"GPPUB", 0x16, 0x0D},
		{"INTFA", 0x07, 0x0E},
		{"INTFB", 0x17, 0x0F},
		{"INTCAPA", 0x08, 0x10},
		{"INTCAPB", 0x18,0x11},
		{"GPIOA", 0x09, 0x12},
		{"GPIOB", 0x19, 0x13},
		{"OLATA", 0x0A, 0x14},
		{"OLATB", 0x1A, 0x15}
};



//add your function definitions for the project MCP23018 here
class MCP23018
{
private:

	const uint8_t _i2cAddr;
	bool _debug;
	uint8_t _bank;


public:

typedef enum {
	IODIRA,
	IODIRB,
	IOPOLA,
	IOPOLB,
	GPINTENA,
	GPINTENB,
	DEFVALA,
	DEFVALB,
	INTCONA,
	INTCONB,
	IOCONA,
	IOCONB,
	GPPUA,
	GPPUB,
	INTFA,
	INTFB,
	INTCAPA,
	INTCAPB,
	GPIOA,
	GPIOB,
	OLATA,
	OLATB
} RegAddr_t;

typedef enum port {PortA, PortB} Port;
typedef enum direction {DIR_INPUT, DIR_OUTPUT} Direction;

private:

	uint8_t getRegisterAddr (const RegAddr_t reg)
	{
		return (_bank == 0) ? addrMap [reg].bank0Addr : addrMap[reg].bank1Addr;
	}

	char* getRegisterName (const RegAddr_t reg)
	{
		return addrMap [reg].regName;
	}

    void dumpValueInBinAndHex (byte val)
    {
      Serial.print (val, HEX);
      Serial.print ("  (");
      Serial.print (val, BIN);
      Serial.print (")");
    }

    void writeRegister (byte regAddr, byte data)
    {
      if  (_debug) {
        Serial.print ("     writeRegister: I2cAddr = ");
        Serial.print (_i2cAddr, HEX);
        Serial.print (" regAddr = ");
        Serial.print (regAddr, HEX);
        Serial.print (" data = ");
        dumpValueInBinAndHex (data);
        Serial.println ("");
      }

      Wire.beginTransmission (_i2cAddr);
      Wire.write (regAddr);
      Wire.write (data);
      Wire.endTransmission ();
    }

    byte readRegister (byte regAddr)
    {
      if (_debug) {
        Serial.print ("     readRegister: I2cAddr = ");
        Serial.print (_i2cAddr, HEX);
        Serial.print (" regAddr = ");
        Serial.print (regAddr, HEX);
      }

      Wire.beginTransmission (_i2cAddr);
      Wire.write (regAddr);
      Wire.endTransmission ();

      Wire.requestFrom (_i2cAddr, (uint8_t) 1);

      byte data;

      if (Wire.available ())
      {
        data = Wire.read ();
      } else {
        Serial.println ("ERROR");
      }

      if (_debug) {
        Serial.print (" data = ");
        dumpValueInBinAndHex (data);
        Serial.println ("");
      }

      return data;
    }

    boolean readBit (byte regAddr, int index)
    {
      byte b = readRegister (regAddr);

      b = (b >> index) & 0x01;

      return b;
    }

    void setBit (byte regAddr, int index)
    {
      // read current value of the register
      byte data = readRegister (regAddr);


      // ensure that the bit is set
      byte mask = 0x01 << index;

      byte data2 = data | mask;

      if (_debug) {
        Serial.print ("     setBit : orig = ");
        dumpValueInBinAndHex (data);
        Serial.print ("  new = ");
        dumpValueInBinAndHex (data2);
        Serial.println ("");
      }

      writeRegister (regAddr, data2);
    }

    void resetBit (byte regAddr, int index)
    {
      // read current value of the register
      byte data = readRegister (regAddr);

      // ensure that the bit is set
      byte mask = 0x01 << index;

      byte data2 = data & ~mask;

     if (_debug) {
        Serial.print ("     resetBit : orig = ");
        dumpValueInBinAndHex (data);
        Serial.print ("  new = ");
        dumpValueInBinAndHex (data2);
        Serial.println ("");
     }

      writeRegister (regAddr, data2);
    }

    void setPort (const byte regAddr)
    {
    	writeRegister (regAddr, 0xFF);
    }

    void resetPort (const byte regAddr)
    {
    	writeRegister (regAddr, 0);
    }

    byte getPort (const byte regAddr)
    {
    	return readRegister (regAddr);
    }

public:
	/**
	 * constructor
	 */
	MCP23018 (uint8_t i2cAddr, bool debug) : _i2cAddr (i2cAddr), _debug (debug), _bank (0)
	{
	  Serial.begin (9600);

	  if (_debug) {
	    Serial.print ("initializing MCP. Addr = ");
	    Serial.println (i2cAddr, HEX);
	  }

	  Wire.begin ();

	  // set IOCON
	  writeRegister (getRegisterAddr (IOCONA), 0x21);
	}

    void debug (int v) { _debug = v; }

    void dumpRegisters ()
    {
      // read all registers
      for (int index = 0; index < 22; index++)
      {
        byte data = getPort (getRegisterAddr ((RegAddr_t) index));

        Serial.print (index,HEX);
        Serial.print ("  ");
        Serial.print (getRegisterName ((RegAddr_t) index));
        Serial.print ("        ");

        dumpValueInBinAndHex (data);

        Serial.println ("");
      }
    }

	/**
	 * setPortDirection
	 */
	void setPortDirection (const Port port, const Direction dir)
	{
		if (_debug) {
			Serial.println (__FUNCTION__);
		}

		byte portAddr = getRegisterAddr ((port == PortA) ? IODIRA : IODIRB);

		if (dir == DIR_OUTPUT)
			resetPort (portAddr);
		else
			setPort (portAddr);
	}

	byte getPortDirection (const Port port)
	{
		if (_debug) {
			Serial.println (__FUNCTION__);
		}

		return readRegister (getRegisterAddr ((port == PortA) ? IODIRA : IODIRB));
	}

	/**
	 * setPinDirection
	 */
	void setPinDirection (const Port port, const Direction dir, const byte index)
	{
		if (_debug) {
			Serial.println (__FUNCTION__);
		}

		byte portAddr = getRegisterAddr ((port == PortA) ? IODIRA : IODIRB);

	      if (dir == DIR_OUTPUT)
	    	  resetBit (portAddr, index);
	      else
	    	  setBit (portAddr, index);

	}

	/**
	 * getPinDirection
	 */
	bool getPinDirection (const Port port, const byte index)
	{
		if (_debug) {
			Serial.println (__FUNCTION__);
		}

		return readBit (getRegisterAddr ((port == PortA) ? IODIRA : IODIRB), index);
	}

	/*
	 * setPortPullup
	 */
	void setPortPullup (const Port port, const byte mask)
	{
		if (_debug) {
			Serial.println (__FUNCTION__);
		}

		writeRegister (getRegisterAddr ((port == PortA) ? GPPUA : GPPUB), mask);
	}

	byte getPortPullup (const Port port)
	{
		if (_debug) {
			Serial.println (__FUNCTION__);
		}

		return readRegister (getRegisterAddr ((port == PortA) ? GPPUA : GPPUB));
	}

	/**
	 * setPinPullup
	 */
	void setPinPullup (const Port port, const byte index)
	{
		if (_debug) {
			Serial.println (__FUNCTION__);
		}

		setBit (getRegisterAddr ((port == PortA) ? GPPUA : GPPUB), index);
	}

	void resetPinPullup (const Port port, const byte index)
	{
		if (_debug) {
			Serial.println (__FUNCTION__);
		}

		resetBit (getRegisterAddr ((port == PortA) ? GPPUA : GPPUB), index);
	}

	bool getPinPullup (const Port port, const byte index)
	{
		if (_debug) {
			Serial.println (__FUNCTION__);
		}

		return readBit (getRegisterAddr ((port == PortA) ? GPPUA : GPPUB), index);
	}

	// polarity
	void setPortPolarity (const Port port, const byte mask)
	{
		if (_debug) {
			Serial.println (__FUNCTION__);
		}

		writeRegister (getRegisterAddr ((port == PortA) ? IOPOLA : IOPOLB), mask);
	}

	byte getPortPolarity (const Port port)
	{
		if (_debug) {
			Serial.println (__FUNCTION__);
		}

		return readRegister (getRegisterAddr ((port == PortA) ? IOPOLA : IOPOLB));
	}

	/**
	 *
	 */
	void setPinPolarity (const Port port, const byte index)
	{
		if (_debug) {
			Serial.println (__FUNCTION__);
		}

		setBit (getRegisterAddr ((port == PortA) ? IOPOLA : IOPOLB), index);
	}

	void resetPinPolarity (const Port port, const byte index)
	{
		if (_debug) {
			Serial.println (__FUNCTION__);
		}

		resetBit (getRegisterAddr ((port == PortA) ? IOPOLA : IOPOLB), index);
	}

	bool getPinPolarity (const Port port, const byte index)
	{
		if (_debug) {
			Serial.println (__FUNCTION__);
		}

		return readBit (getRegisterAddr ((port == PortA) ? IOPOLA : IOPOLB), index);
	}




	/**
	 * setPortValue
	 */
	void setPortValue (const Port port, const byte value)
	{
		if (_debug) {
			Serial.println (__FUNCTION__);
		}

		writeRegister (getRegisterAddr ((port == PortA) ? GPIOA : GPIOB), value);
	}

	/**
	 * getPortValue
	 */
	byte getPortValue (const Port port)
	{
		if (_debug) {
			Serial.println (__FUNCTION__);
		}

		return readRegister (getRegisterAddr ((port == PortA) ? GPIOA : GPIOB));
	}

	/**
	 * setPin
	 */
	void setPin (const Port port, const byte index)
	{
		if (_debug) {
			Serial.println (__FUNCTION__);
		}

		setBit (getRegisterAddr ((port == PortA) ? GPIOA : GPIOB), index);
	}

	/**
	 * resetPin
	 */
	void resetPin (const Port port, const byte index)
	{
		if (_debug) {
			Serial.println (__FUNCTION__);
		}

		resetBit (getRegisterAddr ((port == PortA) ? GPIOA : GPIOB), index);
	}

	/**
	 * getPin
	 */
	bool getPin (const Port port, const byte index)
	{
		if (_debug) {
			Serial.println (__FUNCTION__);
		}

		return readBit (getRegisterAddr ((port == PortA) ? GPIOA : GPIOB), index);
	}

};



//Do not add code below this line
#endif /* _MCP23018_H_ */
