#include "Arduino.h"
#include <SoftwareSerial.h>

#define USBbaud 9600
#define GSMbaud 9600
#define pHbaud  38400

#define pHrxPin 11
#define pHtxPin 3

#define GSMtxPin 8
#define GSMrxPin 7

#define GSMpowerPin 9
#define GSMgpioPin 13

// Analog input pin to which VBAT is attached
#define analogInPin 0

#define pHBufferLength 16

SoftwareSerial pHSerial(  pHrxPin,  pHtxPin  );
SoftwareSerial GSMSerial( GSMrxPin, GSMtxPin );

// battery charger circuit stuff
int       BatteryValue  = 0;    // value read from the VBAT pin
float     outputValue   = 0;    // variable for voltage calculation
char      voltstring[4];        // charbuf for text voltage output

String SMSmessage;              // SMS message object

char pHBuffer[pHBufferLength];  // string buffer for pH stamp
char pHInChar = -1;             // char buffer for pH stamp
byte pHIndex  = 0;              // buffer indes for pH stamp
float pH      = 0;              // pH value

int i = 0;

void readpH() {  
  pHSerial.listen();
  delay(10);
  pHSerial.print( "23.24\r" );
  delay(1000);
  
  // null out the buffer (doesn't seem to help)
  //for( i=0; i < pHBufferLength; i++ ) {
  //  pHBuffer[i] = 0;
  //}
  
  pHIndex = 0;
  while( pHSerial.available() > 0 ) {
    if( pHSerial.overflow() ) {
      Serial.println( "Whoops. The pH serial line has overflowed." );
    }
    if( pHIndex < pHBufferLength - 1 ) {
      pHInChar = (char)pHSerial.read();  // Read a character
      pHBuffer[pHIndex] = pHInChar;      // Store it
      pHIndex++;                         // Increment where to write next
      pHBuffer[pHIndex] = '\0';          // Null terminate the string
    }
  }
  pH = atof(pHBuffer);
}

// relay GSM conversation to the USB console
void outGSM() {
  GSMSerial.listen();
  while( GSMSerial.available() > 0 ) {
    Serial.print( (char)GSMSerial.read() );
  }
}

// check the power state of the GSM module
boolean GSMpower() {
  if (digitalRead( GSMgpioPin ) == HIGH ) {
    Serial.println( "GSM GPIO power is active." );
    return true;
  } else {
    Serial.println( "GSM GPIO power is inactive." );
    return false;
  }
}

// toggle the soft switch
void toggleGSM() {
  pinMode( GSMpowerPin, OUTPUT ); 
  digitalWrite( GSMpowerPin, LOW );
  delay( 1000 );
  digitalWrite( GSMpowerPin, HIGH );
  delay( 2500 );
  digitalWrite( GSMpowerPin, LOW );
  delay( 5500 );
}

// turn the GSM module on
void GSMon() {
  GSMSerial.listen(); 
  outGSM();
  if( GSMpower() ) {
    Serial.println( "GSM module is already active!" );
  } else {
    Serial.println( "Turning GSM module on..." );
    toggleGSM();
  }
  outGSM();
}

// turn the GSM module off
void GSMoff() {
  GSMSerial.listen();
  outGSM();
  if( ! GSMpower() ) {
    Serial.println( "GSM module is already inactive!" );
  } else {
    Serial.println( "Turning GSM module off..." );
    toggleGSM();
  }
  outGSM();
}

void sms( String message ) {
  
  Serial.println( "sending message :" );
  Serial.println( message );
  
  GSMSerial.listen();
  GSMSerial.println( "AT+CREG?" );                 // query signal strength
  delay(200);
  outGSM();
  GSMSerial.println( "AT+CSCA=\"+13123149810\"" );
  delay(200);
  outGSM();
  /*
  GSMSerial.println( "AT+CMGD=1" );                // clear location 1
  delay(200);
  outGSM();
  GSMSerial.println( "AT+CMSS=1" );                // save in location 1
  delay(200);
  outGSM();
  */
  GSMSerial.println( "AT+CMGF=1" );                // SMS text mode
  delay(200);
  outGSM();
  GSMSerial.println( "AT+CMGS=\"40404\"" );        // the recipient (Twitter)
  delay(1200);
  outGSM();
  GSMSerial.print( message );                      // the message body
  delay(200);
  outGSM();
  GSMSerial.write(26);                             // send ctrl-Z
  delay(3000);
  outGSM();

}

void setup() {

  // the 2.8v pin on the SIM900 GPIO is used to detect
  // when the GSM module is powered on
  pinMode( GSMgpioPin, INPUT);
  
  // bring serial lines up
  Serial.begin( USBbaud );
  pHSerial.begin(  pHbaud  );
  GSMSerial.begin( GSMbaud );
  
  Serial.println( "Russell\'s pH meter\n\n" );
  
  // pH meter startup
  pHSerial.listen();
  pHSerial.print( "i\r"  );      // query the firmware version
  delay( 10 );
  Serial.println( "pH stamp firmware version : " );
  while( pHSerial.available() > 0 ) {
    Serial.print( (char)pHSerial.read() );
  }
  pHSerial.print( "l0\r" );    // turn the LED off
  delay( 1000 );
  pHSerial.print( "l1\r" );    // turn the LED on
  delay( 1000 );
  pHSerial.print( "l0\r" );    // turn the LED off
  delay( 1000 );
  pHSerial.print( "l1\r" );    // turn the LED on
  
  // boot up the GSM modem to print diagnostics
  GSMon();
  GSMoff();
}

void loop() {
  
  Serial.println( "taking reading..." );
  delay(100);

  // take a pH reading
  readpH();
  
  // read the analog in value:
  BatteryValue = analogRead(analogInPin);            
  // Calculate the battery voltage value
  outputValue = (BatteryValue*5)/1023;
  // print the results to the serial monitor:
  Serial.print("Analog value = " );                       
  Serial.print(BatteryValue);      
  Serial.print(", voltage = ");
  Serial.println(outputValue);
  
  dtostrf( outputValue, 4, 2, voltstring );
  
  SMSmessage = String( "The latest pH measurement is " + String(pHBuffer) + ", battery voltage " + voltstring );
  
  Serial.println( SMSmessage );
  
  GSMon();
  sms( SMSmessage );
  GSMoff();

  //delay(10000);
  //toggleGSM();
  delay(1800000); // every 20 minutes
  //toggleGSM();
  //delay(10000);

}
extern "C" void __cxa_pure_virtual() { while (1) ; }
#include <Arduino.h>

int main(void)
{
	init();

#if defined(USBCON)
	USB.attach();
#endif
	
	setup();
    
	for (;;) {
		loop();
		if (serialEventRun) serialEventRun();
	}
        
	return 0;
}

