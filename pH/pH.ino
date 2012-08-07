#include <SoftwareSerial.h>
#include <avr/pgmspace.h>

// place all message strings into PROGMEM
prog_char boot_message[]              PROGMEM = "Russell\'s pH meter\n\n";
prog_char boot_ph_firmware_message[]  PROGMEM = "pH stamp firmware version : ";
prog_char run_reading_message[]       PROGMEM = "taking reading...";

prog_char overflowerror[]             PROGMEM = "Whoops. The pH serial line has overflowed.";
prog_char GPIOactive_message[]        PROGMEM = "GSM GPIO power is active.";
prog_char GPIOinactive_message[]      PROGMEM = "GSM GPIO power is inactive.";
prog_char GSM_already_active_err[]    PROGMEM = "GSM module is already active!";
prog_char GSM_already_inactive_err[]  PROGMEM = "GSM module is already inactive!";
prog_char GSM_on_message[]            PROGMEM = "Turning GSM module on...";
prog_char GSM_off_message[]           PROGMEM = "Turning GSM module off...";
prog_char SMS_sending_message[]       PROGMEM = "sending message :";

prog_char AT_sig_check[]              PROGMEM = "AT+CREG?";
prog_char AT_set_sms_gateway[]        PROGMEM = "AT+CSCA=\"+13123149810\"";
prog_char AT_set_text_mode[]          PROGMEM = "AT+CMGF=1";
prog_char AT_sms_to[]                 PROGMEM = "AT+CMGS=\"40404\"";

prog_char batt_volt_str_1[]           PROGMEM = "Analog value = ";
prog_char batt_volt_str_2[]           PROGMEM = ", voltage = ";

prog_char sms_str_1[]                 PROGMEM = "The latest pH measurement is ";
prog_char sms_str_2[]                 PROGMEM = ", battery voltage ";

prog_char temp_str_1[]                PROGMEM = ", temp ";
prog_char temp_str_2[]                PROGMEM = " C";

// table of pointers to back to strings in PROGMEM
const char *string_table[] PROGMEM = {
  boot_message,              // string 0
  boot_ph_firmware_message,  // string 1
  run_reading_message,       // string 2

  overflowerror,             // string 3
  GPIOactive_message,        // string 4
  GPIOinactive_message,      // string 5
  GSM_already_active_err,    // string 6
  GSM_already_inactive_err,  // string 7
  GSM_on_message,            // string 8
  GSM_off_message,           // string 9
  SMS_sending_message,       // string 10

  AT_sig_check,              // string 11
  AT_set_sms_gateway,        // string 12
  AT_set_text_mode,          // string 13
  AT_sms_to,                 // string 14
  
  batt_volt_str_1,           // string 15
  batt_volt_str_2,           // string 16

  sms_str_1,                 // string 17
  sms_str_2,                 // string 18
  
  temp_str_1,                // string 19
  temp_str_2,                // string 20
};

// macros for accessing pointers from the PROGMEM string lookup table
#define _boot_message              0
#define _boot_ph_firmware_message  1
#define _run_reading_message       2

#define _overflowerror             3
#define _GPIOactive_message        4
#define _GPIOinactive_message      5
#define _GSM_already_active_err    6
#define _GSM_already_inactive_err  7
#define _GSM_on_message            8
#define _GSM_off_message           9
#define _SMS_sending_message       10

#define _AT_sig_check              11
#define _AT_set_sms_gateway        12
#define _AT_set_text_mode          13
#define _AT_sms_to                 14
  
#define _batt_volt_str_1           15
#define _batt_volt_str_2           16

#define _sms_str_1                 17
#define _sms_str_2                 18

#define _temp_str_1                19
#define _temp_str_2                20

// buffer for holding strings copied out of PROGMEM
char buffer[64]; // buffer for string table

// baud rates of attached devices
#define USBbaud 9600
#define GSMbaud 9600
#define pHbaud  38400

// pH stamp RX/TX are on digital pins 11 and 3
#define pHrxPin 11
#define pHtxPin 3

// GSM module RX/TX are on digital pins 7 nad 8 
#define GSMrxPin 7
#define GSMtxPin 8

// GSM module soft power switch is on pin 9
#define GSMpowerPin 9

// GSM module GPIO power (for power state sensing) is 
// on digital pin 13
#define GSMgpioPin 13

// Analog input pin to which VBAT is attached
#define analogInPin 0

// pH string buffer 
#define pHBufferLength 16
char pHBuffer[pHBufferLength];  // string buffer for pH stamp
char pHInChar = -1;             // char buffer for pH stamp
byte pHIndex  = 0;              // buffer indes for pH stamp
float pH      = 0;              // pH value

// thermistor constants
// which analog pin to connect
#define THERMISTORPIN A1         
// resistance at 25 degrees C
#define THERMISTORNOMINAL 10000      
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25   
// how many samples to take and average, more takes longer
// but is more 'smooth'
#define NUMSAMPLES 5
// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT 3950
// the value of the 'other' resistor
#define SERIESRESISTOR 10000

int       t_samples[NUMSAMPLES]; // vector for holding subsamples
char      t_str[5];              // string output for temperature
float     t_output = 0;          // output value for temperature

// initialize the SoftwareSerial lines
SoftwareSerial pHSerial(  pHrxPin,  pHtxPin  );
SoftwareSerial GSMSerial( GSMrxPin, GSMtxPin );

// battery charger circuit variables
int       BatteryValue  = 0;    // value read from the VBAT pin
float     outputValue   = 0;    // variable for voltage calculation
char      voltstring[5];        // charbuf for text voltage output

// string objects
String SMSmessage;              // SMS message object
String VoltString;              // Voltage message object
String TempString;              // Temperature message object

// a counter variable for counting stuff
byte i = 0;

// pull strings out of the PROGMEM string table and return them
// bs stands for "buffer string," I swear.
char* bs( int strnum ) {
  strcpy_P( buffer, (char*)pgm_read_word( &( string_table[strnum] ) ) );
  return buffer;
}

// take a reading from the pH stamp
// result stored in pH and pHbuffer
void readpH() {
  pHSerial.listen();
  delay(10);
  pHSerial.print( "23.24\r" );
  delay(1000);
  
  pHIndex = 0;
  while( pHSerial.available() > 0 ) {
    if( pHSerial.overflow() ) {
      Serial.println( bs( _overflowerror ) );
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

// turn the pH stamp LED on
void pHledON() {
  pHSerial.listen();
  pHSerial.print( "l1\r" );    // turn the LED on
}

// turn the pH stamp LED off
void pHledOFF() {
  pHSerial.listen();
  pHSerial.print( "l0\r" );    // turn the LED off
}

// measure temperature
// borrowed from : http://learn.adafruit.com/thermistor/using-a-thermistor
float getTemp() {
  uint8_t i;
  float average;
 
  // take N samples in a row, with a slight delay
  for (i=0; i< NUMSAMPLES; i++) {
   t_samples[i] = analogRead(THERMISTORPIN);
   delay(10);
  }
 
  // average all the samples out
  average = 0;
  for (i=0; i< NUMSAMPLES; i++) {
     average += t_samples[i];
  }
  average /= NUMSAMPLES;
 
  Serial.print("Average analog reading "); 
  Serial.println(average);
 
  // convert the value to resistance
  average = 1023 / average - 1;
  average = SERIESRESISTOR / average;
  Serial.print("Thermistor resistance "); 
  Serial.println(average);
  
  float steinhart;
  steinhart = average / THERMISTORNOMINAL;     // (R/Ro)
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                 // Invert
  steinhart -= 273.15;                         // convert to C

  return (steinhart);
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
    Serial.println( bs( _GPIOactive_message ) );
    return true;
  } else {
    Serial.println( bs( _GPIOinactive_message ) );
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
    Serial.println( bs( _GSM_already_active_err ) );
  } else {
    Serial.println( bs( _GSM_on_message ) );
    toggleGSM();
  }
  outGSM();
}

// turn the GSM module off
void GSMoff() {
  GSMSerial.listen();
  outGSM();
  if( ! GSMpower() ) {
    Serial.println( bs( _GSM_already_inactive_err ) );
  } else {
    Serial.println( bs( _GSM_off_message ) );
    toggleGSM();
  }
  outGSM();
}

// send an SMS message
void sms( String message ) {
  
  Serial.println( bs( _SMS_sending_message ) );
  Serial.println( message );
  
  GSMSerial.listen();
  GSMSerial.println( bs( _AT_sig_check ) );        // query signal strength
  delay(200);
  outGSM();
  GSMSerial.println( bs( _AT_set_sms_gateway ) );
  delay(200);
  outGSM();
  GSMSerial.println( bs( _AT_set_text_mode ) );    // SMS text mode
  delay(200);
  outGSM();
  GSMSerial.println( bs( _AT_sms_to ) );           // the recipient (Twitter)
  delay(1200);
  outGSM();
  GSMSerial.print( message );                      // the message body
  delay(3000);
  outGSM();
  GSMSerial.write(26);                             // send ctrl-Z
  delay(3000);
  outGSM();

}

void setup() {

  // set external reference voltage for more stable amalog mesusrements
  analogReference(EXTERNAL);
  
  // the 2.8v pin on the SIM900 GPIO is used to detect
  // when the GSM module is powered on
  pinMode( GSMgpioPin, INPUT);
  
  // bring serial lines up
  Serial.begin( USBbaud );
  pHSerial.begin(  pHbaud  );
  GSMSerial.begin( GSMbaud );
  
  Serial.println( bs( _boot_message ) );
  
  // pH meter startup
  pHSerial.listen();
  pHSerial.print( "i\r"  );      // query the firmware version
  delay( 10 );
  Serial.println( bs( _boot_ph_firmware_message ) );
  while( pHSerial.available() > 0 ) {
    Serial.print( (char)pHSerial.read() );
  }
  Serial.println();
  pHledOFF();                    // turn off pH diagnostic LED

  // boot up the GSM modem to print diagnostics
  //GSMon();
  //GSMoff();
}

void loop() {
  
  Serial.println( bs( _run_reading_message ) );
  delay(100);

  // take a pH reading
  pHledON();
  readpH();
  pHledOFF();  

  // read the analog in value:
  BatteryValue = analogRead( analogInPin );       
  // Calculate the battery voltage value
  outputValue = ( BatteryValue * 5 ) / 1023;
  // print the results to the serial monitor:
  Serial.print( bs( _batt_volt_str_1 ) );
  Serial.print( BatteryValue );
  Serial.print( bs( _batt_volt_str_2 ) );
  Serial.println( outputValue );
  
  dtostrf( outputValue, 4, 2, voltstring );
  VoltString = String( voltstring );
  VoltString.trim();
  
  t_output = getTemp();
  dtostrf( t_output, 4, 2, t_str );
  
  TempString = String( String(bs(_temp_str_1)) + String( t_str ) + String(bs(_temp_str_2)) );
  
  SMSmessage = String(pHBuffer);
  SMSmessage.trim();
  SMSmessage = String( String(bs(_sms_str_1)) + SMSmessage + String(bs(_sms_str_2)) + VoltString + TempString );
  SMSmessage.trim();
  
  GSMon();
  delay(30000);
  sms( SMSmessage );
  delay(3000);
  GSMoff();

  delay(1800000); // every 20 minutes

}
