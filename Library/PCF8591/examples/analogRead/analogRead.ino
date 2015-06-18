//
//
//
#include <Wire.h>
#include "PCF8591.h"

// create an instance
PCF8591 pcf;

void setup ()
{
	Serial.begin (9600);
	pcf.begin (0);
}

void loop ()
{
  for (int i = 0; i < 4; i++) {
    Serial.print (i);
    Serial.print (" ==> ");
    Serial.print (pcf.analogRead (i), HEX);
    Serial.print ("       ");
  }

  Serial.println ("");
  
  delay (1000);
}
