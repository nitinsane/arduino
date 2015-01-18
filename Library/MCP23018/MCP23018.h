// Only modify this file to include
// - function definitions (prototypes)
// - include files
// - extern variable definitions
// In the appropriate section

#ifndef _MCP23018_H_
#define _MCP23018_H_
#include "Arduino.h"
//add your includes for the project MCP23018 here


//end of add your includes here
#ifdef __cplusplus
extern "C" {
#endif
void loop();
void setup();
#ifdef __cplusplus
} // extern "C"
#endif

//add your function definitions for the project MCP23018 here
class MCP23018
{
private:
	char *registerNames [] = {"IODIRA", "IODIRB", "IPOLA", "IPOLB", "GPINTENA", "GPINTENB", "DEFVALA", "DEFVALB", "INTCONA", "INTCONB", "IOCON",
	                          "IOCON", "GPPUA", "GPPUB", "INTFA", "INTFB", "INTCAPA", "INTCAPB", "GPIOA", "GPIOB", "OLATA", "OLATB"};

public:

};



//Do not add code below this line
#endif /* _MCP23018_H_ */
