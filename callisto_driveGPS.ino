#include <LiquidCrystal_I2C.h>

// e_callisto_drive uses an arduino to control a DiSEqC 1.2 satellite motor to
// an antenna can track the Sun in right ascension.
// The arduino calculates the position of the Sun and sends tone controls to 
// the drive via pin 8.  Extrernal electronics are required to inject the tone 
// signals onto the drive power lines.
//
// Single precision limits the angular precision. A comparison with the JPL Horizons
// system show discrete errors of up to 1 minute in hour angle without the lookup tables.
// With the tables the errors are less (not checcked how much less!)
//
// 22 kHz signal should be set to 650 mV p-p
// see http://www.eutelsat.com/satellites/pdf/Diseqc/Reference%20docs/bus_spec.pdf
// and http://www.eutelsat.com/satellites/pdf/Diseqc/associated%20docs/position_app_note_v1.pdf
// but note these are incomplete.
//
// Set the time at the serial console (9600 baud) with the format "yyyy mm dd hh mm ss".
//
// Graham Woan 13/10/2012
// still to do:
// display sidereal time
#include <TimeLib.h>
#include <math.h>
#include  <util/parity.h>
#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

#define I2C_ADDR    0x27  // Define I2C Address where the PCF8574A is
#define BACKLIGHT_PIN     3
#define En_pin  2
#define Rw_pin  1
#define Rs_pin  0
#define D4_pin  4
#define D5_pin  5
#define D6_pin  6
#define D7_pin  7

long time_offset = 619315200; // set to zero if there is no bug but 1024 weeks (in seconds) if there is

LiquidCrystal_I2C  lcd(I2C_ADDR,En_pin,Rw_pin,Rs_pin,D4_pin,D5_pin,D6_pin,D7_pin);

SoftwareSerial mySerial(6, 4); // from the GPS module, green to 6, white to 4
Adafruit_GPS GPS(&mySerial);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences. 
#define GPSECHO  false

// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

float longitude = -4.30722; // default telescope longitude, degrees, E positive
float latitude = 55.90222; // default telescope latitude, degrees north
float alat;
int datapin = 8;// the pin on which the arduino generates the tone 
int sw = 3; // switch pin
int switchstate = 0;  // variable for reading the switch status

long t;
float d0,d,L,g,X,Y,RA,delta,lambda,epsilon,GMST,LST,hh,altitude, temp;
float ohh = 1000.0;
float dtor = 0.01745329251;
String readString=""; 
int yr,mth,dy,h,m,s;
unsigned int epoch=1;

//values of offsets every 600 days for 25*600 days from J2000.0 (ie about 41 years)
// this reduces the effects of single precision arithmetic to an error of just a few seconds
const long tlookup[] = {
 946728000,  998568000, 1050408000, 1102248000, 1154088000,  
1205928000, 1257768000, 1309608000, 1361448000, 1413288000,  
1465128000, 1516968000, 1568808000, 1620648000, 1672488000,  
1724328000, 1776168000, 1828008000, 1879848000, 1931688000,  
1983528000, 2035368000, 2087208000, 2139048000, 2190888000
};
const float Llookup[] = {
280.461, 151.849, 23.238, 254.626, 126.015,  
357.403, 228.792, 100.180, 331.569, 202.957,  
74.345, 305.734, 177.122, 48.511, 279.899,  
151.288, 22.676, 254.064, 125.453, 356.841,  
228.230, 99.618, 331.007, 202.395, 73.784
};
const float glookup[] = {
357.528, 228.888, 100.248, 331.609, 202.969,  
74.329, 305.689, 177.049, 48.409, 279.770,  
151.130, 22.490, 253.850, 125.210, 356.571,  
227.931, 99.291, 330.651, 202.011, 73.371,  
304.732, 176.092, 47.452, 278.812, 150.172
};
const float stlookup[] = {
280.461000, 151.849422, 23.237844, 254.626266, 126.014688,  
357.403110, 228.791532, 100.179954, 331.568376, 202.956798,  
74.345220, 305.733642, 177.122064, 48.510486, 279.898908,  
151.287330, 22.675752, 254.064174, 125.452596, 356.841018,  
228.229440, 99.617862, 331.006284, 202.394706, 73.783128
};
//////////////////////////////////////////////////////////////////////////////
void setup() {
  pinMode(datapin,OUTPUT); // tone output
  pinMode(sw, INPUT_PULLUP); //pin for switch

//start gps setup
 lcd.begin (16,2);
  
// Switch on the backlight
  lcd.setBacklightPin(BACKLIGHT_PIN,POSITIVE);
  lcd.setBacklight(HIGH);
  lcd.clear();
  lcd.home ();                   // go home
  lcd.setCursor (0,0);
  lcd.print("  DiSEqC solar  "); 
  lcd.setCursor (0,1);
  lcd.print("  tracker v1.1  "); 

  Serial.begin(115200);
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);  
  useInterrupt(true);
  delay(2000);
} // end setup

/////////////////////////////////////////////////////////////////////////////
// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

uint32_t timer = millis();
/////////////////////////////////////////////////////////////////////////////
float reduce(float x) { //reduce and angle to the range (0,360)
 while (x>360.0) {
   x = x-360.0;
 }
 while (x<0.0) {
   x = x+360.0;
 }
 return(x);
}
/////////////////////////////////////////////////////////////////////////////
///////////
void write0(){                      // write a '0' bit toneburst
 for (int i=1; i<=22; i++){         // 1 ms of 22 kHz (22 cycles)
  digitalWrite(datapin,HIGH);
  delayMicroseconds(16);
  digitalWrite(datapin,LOW);
  delayMicroseconds(17);
 }
delayMicroseconds(500);             // 0.5 ms of silence
}
///////////
void write1(){                      // write a '1' bit toneburst
 for (int i=1; i<=11; i++){         // 0.5 ms of 22 kHz (11 cycles)
  digitalWrite(datapin,HIGH);
  delayMicroseconds(16);
  digitalWrite(datapin,LOW);
  delayMicroseconds(17);
 }
delayMicroseconds(1000);            // 1 ms of silence
}
//////////
// write the parity of a byte (as a toneburst)
// note parity_even_bit(B1101101) returns 1 (odd parity)
//      parity_even_bit(B1101001) returns 0 (even parity)
void write_parity(byte x){
 if (parity_even_bit(x)) write0(); else write1();
}
//////////////////
// write out a byte (as a toneburst)
// high bit first (ie as if reading from the left)
void write_byte(byte x){
  for (int j=7; j>=0; j--){
  if (x & (1<<j)) write1(); else write0();
   }
}
////////////////
// write out a byte with parity attached (as a toneburst)
void write_byte_with_parity(byte x){
  write_byte(x);
  write_parity(x);
}
//////////////////////////////////////////////////////////////
// goto position angle a in degrees, south = 0.
// (a must be in the range +/- 75 degrees)
void goto_angle(float a){
/*
Note the diseqc "goto x.x" command is not well documented.  
The decription in http://www.eutelsat.com/satellites/4_5_5.html 
is for 'terrestrial positioners' rather than satellite positioners
Which drives use the terrestial command set and which use the satellite
set is not clear, but various web resources, including 
http://www.techwatch.co.uk/forums/14112-raw-diseqc-1-2-commands-a-rough-guide.html
indicate the following is right for the satellite set:
*/
 float fa16;
 byte n1,n2,n3,n4,d1,d2;
 int a16;
// get the angle in range +/- 75 degrees.  Sit at these limits and switch
// over at ~ midnight unless otherwise instructed.

if (a <-75.0) {a=-75;}
if (a > 75.0) {a= 75;}

// set the sign nibble in n1 to E (east) or D (west). 
if (a<0) { n1=0xE0;} else {n1=0xD0;}
// shift everything up so the fraction (1/16) nibble is in the 
//integer, and round to the nearest integer:
a16 =  (int) (16.0*abs(a)+0.5); 
// n2 is the top nibble of the three-nibble number a16:
n2 = (a16 & 0xF00)>>8;
// the second data byte is the bottom two nibbles:
d2 = a16 & 0xFF;
//the first data byte is
d1 = n1 | n2;
// send the command to the positioner
  noInterrupts();
  write_byte_with_parity(0xE0);
  write_byte_with_parity(0x31);
  write_byte_with_parity(0x6E);
  write_byte_with_parity(d1);
  write_byte_with_parity(d2);
  interrupts();
}
//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////
//
// main loop ////////////////////////////////////////////////////
//
void loop() {
// determine the position of the Sun, approximately
// see thr Explanatory Supplement to the Astronomical Almanac p484
// and http://www.stargazing.net/kepler/sun.html
// a better way to do this would be via the equation of time.
// Note, 12h J2000.0 in unix time is 946728000
//

// check the switch state
 switchstate = digitalRead(sw);

 // start gps and display stuff
  // in case you are not using the interrupt above, you'll
  // need to 'hand query' the GPS, not suggested :(
  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
      if (c) Serial.print(c);
  }
  
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every 1 sec, print out the current stats
  if (millis() - timer > 900) { 
    timer = millis(); // reset the timer

     setTime(GPS.hour,GPS.minute,GPS.seconds,GPS.day,GPS.month,2000+GPS.year); 
     while (year()<2022) adjustTime(time_offset); // loop is there because the adjustment does not always succeed for some reason!
     t = now();

 //   lcd.home();
     char ln[16];
     lcd.home();
 //    sprintf(ln, "%02d/%02d/%02d   %02d:%02d", GPS.day, GPS.month, GPS.year, GPS.hour, GPS.minute);
     sprintf(ln, "%02d/%02d/%02d   %02d:%02d", day(), month(), year()-2000, hour(), minute());
     lcd.print(ln);

 // end gps and display stuff
 

 d0 = (t-946728000)/86400.0; //Find the days since J2000.0
 epoch = (int) d0/600.0; // lookup table index
 if (epoch<0) {epoch=0;}
 if (epoch>24) {epoch=24;}
 d = (t-tlookup[epoch])/86400.0;
 L = reduce(Llookup[epoch] + 0.9856474*d);     //Find the Mean Longitude (L) of the Sun in degrees
 g = reduce(glookup[epoch]+ 0.9856003*d);   //Find the Mean anomaly (g) of the Sun in degrees
 
 lambda = L + 1.915*sin(dtor*g) + 0.020*sin(dtor*2*g); //Find the ecliptic longitude (lambda) of the sun
 epsilon = 23.439 - 0.0000004*d0;  //Find the obliquity of the ecliptic plane (epsilon)
//
// determine RA and dec
 Y = cos(dtor*epsilon) * sin(dtor*lambda);
 X = cos(dtor*lambda);
 RA = atan2(Y,X)*3.81971863421; //RA in hours
 delta = asin(sin(dtor*epsilon)*sin(dtor*lambda))/dtor; //dec in degrees

// determine local sidereal time and hour angle of the Sun
  LST=reduce(stlookup[epoch]+360.98564737*d+longitude); // LST in degrees
  hh = LST/15.0-RA; // hour angle
 
 altitude = asin(sin(delta*dtor)*sin(latitude*dtor) + cos(delta*dtor)*cos(latitude*dtor)*cos(hh/12.0*3.1415926))/dtor;
 if (altitude<-5.0) {hh=0.0;} // if the Sun has set then go to the central position

// get the hour angle in range -12 to 12
 while (hh>12.0) { hh = hh-24.0; }
 while (hh<-12.0) { hh = hh+24.0;}

     if (switchstate == HIGH) { //home the drive
       goto_angle(0.0);
       ohh=0.0;
       altitude = asin(sin(delta*dtor)*sin(latitude*dtor) + cos(delta*dtor)*cos(latitude*dtor)*cos(0.0/12.0*3.1415926))/dtor;
       lcd.clear();
     }
    while (switchstate == HIGH) { //display home info
      switchstate = digitalRead(sw);

      lcd.setCursor (0,0); 
      lcd.print("solar alt "); lcd.print( altitude,2);lcd.print((char)223);
      if (GPS.fix) {
      // use the GPS coordinates for the calculations
       alat = abs(GPS.latitude);
       latitude = int(alat/100) + (alat-int(alat/100)*100.0)/60.0;
       if (GPS.lat == 'S') {latitude = -latitude;}
       alat = abs(GPS.longitude);
       longitude = int(alat/100) + (alat-int(alat/100)*100.0)/60.0;
       if (GPS.lon == 'W') {longitude = -longitude;}
       lcd.setCursor (0,1); 

      lcd.print(GPS.latitude/100, 0); lcd.print((char)223); 
      lcd.print(GPS.latitude-int(GPS.latitude/100)*100, 0); lcd.print("'"); 
      lcd.print(GPS.lat);
      lcd.print("   "); 
      lcd.print(GPS.longitude/100, 0); lcd.print((char)223);
      lcd.print(GPS.longitude-int(GPS.longitude/100)*100, 0);lcd.print("'"); 
      lcd.print(GPS.lon);        
      
      
      
      } 
     else
     { 
      lcd.setCursor (0,1); 
      lcd.print("** no GPS fix **");
    }
 }
 if (ohh!=hh) {
// a new position is available, so print it out...
/*
Serial.print(hour(),DEC); Serial.print(":"); Serial.print(minute(),DEC);
 Serial.print(":"); Serial.print(second(),DEC); Serial.print("  ");
 Serial.print(day(),DEC); Serial.print("-"); Serial.print(month(),DEC);
 Serial.print("-"); Serial.print(year(),DEC);
 Serial.print(", dec (d):"); Serial.print(delta,2);  
 Serial.print(" , hh (h):"); Serial.print(hh,4); 
 Serial.print(", altitude (d): "); Serial.print( altitude,2); 
 Serial.print(", angle (d): "); Serial.println( hh*15.0,2); 
*/
 lcd.setCursor (0,1);
 char chhh[5];
 dtostrf(hh*15.0, 6, 1, chhh);
 lcd.print("drive to ");
 lcd.print(chhh); lcd.print((char)223);
// send command to the diseqc motor, converting hours to degrees
 goto_angle(hh*15.0);
 if (ohh<500.0) {delay(8000.0*abs(hh-ohh));} else {delay(8000.0*abs(hh));}
 ohh = hh;
 }
  }

}
