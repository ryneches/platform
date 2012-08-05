//
// This sketch allows the Arduino board to act as a serial relay between
// an Atlas Scientific pH Stamp and the USB host serial.
//

#include <SoftwareSerial.h>

#define pHtxPin 3
#define pHrxPin 11

SoftwareSerial GSMSerial( pHrxPin, pHtxPin ); 

void setup()
{
  GSMSerial.begin(38400);             // the pH stamp baud rate   
  Serial.begin(9600);                 // the host baud rate   
}
 
void loop()
{
  if (GSMSerial.available())
    Serial.write(GSMSerial.read());
  if (Serial.available())
    GSMSerial.write(Serial.read());  
 
}
