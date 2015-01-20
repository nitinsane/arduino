// Do not remove the include below
#include "MCP23018.h"

/**
 * MCP23018::MCP23018
 */






MCP23018* mcp;

const byte myPin = 7;
const byte myInputPin = 6;

//The setup function is called once at startup of the sketch
void setup()
{
// Add your initialization code here
	mcp = new MCP23018 (0x20, 1);

	mcp->setPinDirection (MCP23018::PortA, MCP23018::DIR_OUTPUT, myPin);

	mcp->setPinDirection (MCP23018::PortA, MCP23018::DIR_INPUT, myInputPin);
	mcp->setPinPullup (MCP23018::PortA, myInputPin);


	// turn LED off
	mcp->setPin (MCP23018::PortA, myPin);

	mcp->debug (0);


}

// The loop function is called in an endless loop
void loop()
{
//Add your repeated code here

	// turn LED on
	mcp->resetPin (MCP23018::PortA, myPin);
	delay (25);
	// turn LED off
	mcp->setPin (MCP23018::PortA, myPin);
	delay (1500);

#if 0
	 if (mcp->getPin (MCP23018::PortA, myInputPin) == 0)
	 {
		 delay (150);
		 if (mcp->getPin (MCP23018::PortA, myInputPin) == 0)
			 Serial.println ("button pressed");
	 }
#endif

}
