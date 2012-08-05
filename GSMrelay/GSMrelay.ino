//
// This sketch allows an Arduino board to act as a relay between the
// Seeed Studio SIM900-based GSM shield. The host is connected to the
// Arduino's hardware UART, and the SIM900 module communicates through
// the SoftwareSerial library on pins 7 and 8. 
// 
// Note that this shield is *NOT* compatible with the Arduino Leonardo;
// the Leonardo cannot attach interrupts to pin 7, and so it cannot be
// used as an RX line for SoftwareSerial.
// 
// Connect using a terminal emulator (preferably *NOT* the serial monitor
// built into the Arduino IDE) at 19200 baud, 8N1.
// 
// http://www.seeedstudio.com/wiki/index.php?title=GPRS_Shield_v0.9b
//
// Useful commands :
//
// AT                       see if the module is alive
// AT+CSCA="+1NNNNNNNNNN"   set SMS message gateway
// AT+CMGF=1                set the message mode to text
// AT+CMGS="+1NNNNNNNNNN"   send an SMS message (ctrl-z to end)
// 


#include <SoftwareSerial.h>

#define GSMtxPin 8
#define GSMrxPin 7

SoftwareSerial GSMSerial( GSMrxPin, GSMtxPin ); 

void setup()
{
  GSMSerial.begin(9600);               // the GPRS baud rate   
  Serial.begin(9600);                 // the GPRS baud rate   
}
 
void loop()
{
  if (GSMSerial.available())
    Serial.write(GSMSerial.read());
  if (Serial.available())
    GSMSerial.write(Serial.read());  
 
}
