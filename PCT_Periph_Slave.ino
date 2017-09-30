// Wire Slave Receiver
// by Nicholas Zambetti <http://www.zambetti.com>

// Demonstrates use of the Wire library
// Receives data as an I2C/TWI slave device
// Refer to the "Wire Master Writer" example for use with this

// Created 29 March 2006

// This example code is in the public domain.


#include <Adafruit_NeoPixel.h>
#include <Servo.h>
#include "Maxbotix.h"
#include <Math.h> 
#include <Wire.h>

#define runway_strip_pin      4 
#define runway_strip_led_num  8
#define frontstrip_pin        12
#define frontstrip_led_num    8

Maxbotix rangeSensorAD(A0, Maxbotix::AN, Maxbotix::LV, Maxbotix::BEST, 9);
//AltSoftSerial altSerial;

Adafruit_NeoPixel runway_strip = Adafruit_NeoPixel(runway_strip_led_num, runway_strip_pin, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel front_strip = Adafruit_NeoPixel(frontstrip_led_num, frontstrip_pin, NEO_GRB + NEO_KHZ800);

//-----------Define the IO Pins---------//
int PIR         = 2;  //PIR Motion Sensor
int MAGR_MOSFET = 7;  //MOSFET For the Magnetic Right Servo
int MAGL_MOSFET = 8; //MOSFET for the Magnetic Left Servo
int SONAR_INPUT = A0; //Ultrasonic Sonar Range Finder
int LD_RED      = 10; //Red LED
int LD_BLUE     = 11; //Blue LED
Servo GPRO_PAN;     //GoPro Pan
Servo GPRO_TILT;    //GoPro Tilt
Servo TURRET_PAN;   //Turret pan
Servo TURRET_TILT;    //Turret Tilt
Servo MAGR;       //Magnetic Locking System Right Servo
Servo MAGL;       //Magnetic Locking System Left Servo
int MAGR_MIN  = 0;
int MAGR_MAX  = 180;
int MAGL_MIN  = 0;
int MAGL_MAX  = 180;
int NEO_STATE = LOW;
//-----------Program Control  Variables---------//
  int XBEE_READ             = 0;
  String GPRO_PAN_ANGLE     = 0;
  String GPRO_TILT_ANGLE    = 0;
  String TURRET_PAN_ANGLE   = 0;
  String TURRET_TILT_ANGLE  = 0;
  String MAGL_ANGLE         = 0;
  String MAGR_ANGLE         = 0;
  String RUNWAY_MODE_SELECT  = 0;
  String FRONT_MODE_SELECT   = 0;
  String MP3_SELECT         = 0;
  int TURRET_FIRE           = 0;
  int PERIPH_I2C_ID         = 4;  //I2C Bus Address 
  int j=0;
  int range = 0;
  int range_old = 0;
  int PIR_STATE = 0;
  boolean LOCKING_FETS;

  int pos = 0, dir = 1; // Position, direction of "eye"
  long previousmillis = 0;        // will store last time LED was updated
  int red_blinky = 10;
  long middlepreviousmillis=0;
  char c = 0;

  long previousMillis = 0;
  long range_sample_interval = 600000;
  int DATA_ARRAY[] = {10, 10, 10, 10, 1, 1, 1, 1, 100, 100, 100};
String case1;
int i2c_char = 0;
int i=0;

/************************************************************************/
/*            Initial Setup Routine                                     */
/************************************************************************/
void setup()
{
  Wire.begin(PERIPH_I2C_ID);                // join i2c bus with address #4
  Wire.onReceive(receiveEvent); // register event

  Serial.begin(57600);           // start serial for output
  //Serial.println("Slave");

//--------- Declare Digital Ouputs ---------------//
  pinMode(PIR,          OUTPUT);
  pinMode(MAGR_MOSFET,  OUTPUT);
  pinMode(MAGL_MOSFET,  OUTPUT);
  pinMode(TURRET_FIRE,  OUTPUT);
  pinMode(LD_RED,       OUTPUT);
  pinMode(LD_BLUE,      OUTPUT);

  digitalWrite(MAGR_MOSFET, LOW);  
  digitalWrite(MAGL_MOSFET, LOW);
  digitalWrite(TURRET_FIRE, LOW);
  digitalWrite(LD_RED,      HIGH);
  digitalWrite(LD_BLUE,     LOW);


//--------- Attach Servos ------------------------//
  MAGL.attach(6);
  MAGR.attach(9);
  GPRO_PAN.attach(3);
  GPRO_TILT.attach(5);
  //TURRET_PAN.attach(9);
  //TURRET_TILT.attach(11);
  GPRO_PAN.write(0);
  GPRO_TILT.write(100);
  MAGL.write(0);
  MAGR.write(0);
  
//----------Start up the NEOPIXEL strips ----------//
  front_strip.begin();
  runway_strip.begin();

  // Set the delay between AD readings to 10ms
  rangeSensorAD.setADSampleDelay(10);

}

/************************************************************************/
/*            Infinite Loop/Run Mode                                    */
/************************************************************************/
void loop()
{

unsigned long currentMillis = millis();

//Used to shut off the Locking Servos after they are used (to save energy)
if(LOCKING_FETS == HIGH)
{
     if(currentMillis - previousMillis > range_sample_interval) 
     {

        previousMillis = currentMillis; 
        digitalWrite(MAGR_MOSFET, LOW);
        digitalWrite(MAGL_MOSFET, LOW);
      }
}
/*
//Range Finder Read ever xxxx ms
   unsigned long currentMillis = millis();
     if(currentMillis - previousMillis > range_sample_interval) 
     {

      previousMillis = currentMillis;   

        //PIR Motion Sensor Read
        if(digitalRead(PIR) == HIGH && PIR_STATE == LOW)
        {
          PIR_STATE = HIGH;
          digitalWrite(LD_BLUE, HIGH);
          Serial.write(7);
          Serial.write(1);
        }

        else if (PIR_STATE == HIGH && digitalRead(PIR) == LOW)
        {
          PIR_STATE = LOW;
          digitalWrite(LD_BLUE, LOW);
          Serial.write(7);
          Serial.write(0);
        }
/*
//Range Finder Reads
      range = (rangeSensorAD.getRange());
      if(range_old != range)
      {
        //Serial.write(6);
        //Serial.write(range);
        //Serial.println(range);
      }

      range_old = range;
      //Serial.write(range);
    //Serial.println(MAGL_ANGLE.toInt());
      //TARGET_CALIBRATE();
    }//end cal
*/

}//end loop


/************************************************************************/
/*                I2C Interrupt Handler on ID #4                        */
/************************************************************************/
void receiveEvent(int howMany)
{
  while(Wire.available()) // loop through all but the last
  {
    char periph_tag = Wire.read(); // receive byte as a character
    
    switch(periph_tag)
    {
    //GoPro Pan servo angle
      case '0':
        GPRO_PAN_ANGLE = Wire.readStringUntil('\r');
        GPRO_PAN.write(map(GPRO_PAN_ANGLE.toInt(), 0, 255, 0, 180));
      break;

    //GoPro Tilt servo Angle
      //case 0x01:
      case '1':
        GPRO_TILT_ANGLE = Wire.readStringUntil('\r');
        GPRO_TILT.write(map(GPRO_TILT_ANGLE.toInt(), 0, 255, 100, 180));
      break;

    //Turret Pan Servo Angle
      case '2':
        TURRET_PAN_ANGLE = Wire.readStringUntil('\r');
        TURRET_PAN.write((TURRET_PAN_ANGLE.toInt(), 0, 255, 0, 180));
      break;

    //Turret Tilt Servo Angle
      case '3':
        TURRET_TILT_ANGLE = Wire.readStringUntil('\r');
        TURRET_TILT.write(map(TURRET_TILT_ANGLE.toInt(), 0, 255, 0, 180));
      break;

    //Magnetic Locking System Left Servo
      case '4':
        MAGL_ANGLE = Wire.readStringUntil('\r');
        digitalWrite(MAGL_MOSFET, HIGH);
        MAGL.write(map(MAGL_ANGLE.toInt(), 0, 255, MAGL_MIN, MAGL_MAX));
        LOCKING_FETS = HIGH;
      break;

    //Magnetic Locking System Right Servo
      case '5':
        MAGR_ANGLE = Wire.readStringUntil('\r');
        digitalWrite(MAGR_MOSFET, HIGH);
        MAGR.write(map(MAGR_ANGLE.toInt(), 0, 255, MAGR_MIN, MAGR_MAX));
        LOCKING_FETS = HIGH;
      break;

    //Select which mode to run the Front Neopxiels
      case  '6':
        FRONT_MODE_SELECT = Wire.readStringUntil('\r');
        NEO1_MODE(FRONT_MODE_SELECT);
      break;

    //Select which mode to run the Runway NeoPixels
      case '7':
        RUNWAY_MODE_SELECT = Wire.readStringUntil('\r');
        //NEO2_MODE(RUNWAY_MODE_SELECT);
      break;

      case '8':
        //MP3_SELECT = Wire.readStringUntil('\r');
        //MP3_MODE(MP3_SELECT);
          digitalWrite(PIR, HIGH);
      break;

    //Fire the Turret
      case '9':
          digitalWrite(PIR, LOW);
              //PIR Motion Sensor Read
       /* if(NEO_STATE == LOW)
        {

          NEO_STATE = HIGH;
        }

        else if (NEO_STATE == HIGH )
        {
          NEO_STATE = LOW;
          digitalWrite(PIR, LOW);
        }
*/
      break;

    }//end controls FSM
  }//I2C while
}//end I2C event handler

/************************************************************************/
/*            Front NeoPixel Strip Modes                      */
/************************************************************************/
void NEO1_MODE(String MODE)
{
  unsigned long currentMillis = millis();
  int j;
    // Draw 5 pixels centered on pos. setPixelColor() will clip any
    // pixels off the ends of the strip, we don't need to watch for that.
    front_strip.setPixelColor(pos - 2, 0xFF0000); // Dark red
    front_strip.setPixelColor(pos - 1, 0xFF0000); // Medium red
    front_strip.setPixelColor(pos , 0xFF3000); // Center pixel is brightest
    front_strip.setPixelColor(pos + 1, 0xFF0000); // Medium red
    front_strip.setPixelColor(pos + 2, 0xFF0000); // Dark red
     
  front_strip.show();
  delay(30);
  //if(currentMillis - previousMillis > 30) 
  //{
       
      // Rather than being sneaky and erasing just the tail pixel,
      // it's easier to erase it all and draw a new one next time.
      for(j=-2; j<= 2; j++) 
      {
        front_strip.setPixelColor(pos+j, 0);
      }

      // Bounce off ends of strip
      pos += dir;
      if(pos < 0) {
        pos = 1;
        dir = -dir;
    } 

     else if(pos >= front_strip.numPixels()) {
        pos = front_strip.numPixels() - 2;
        dir = -dir;
      }
   //}

}//END NEO1_MODE


/************************************************************************/
/*            Runway NeoPixel Strip Modes                    */
/************************************************************************/
//void NEO2_MODE(String MODE)
//{
// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  int start2=35;
  int start3=62;
  int total=98;
  int numleft=35;
  int nummid=27;
  int numright=36;
  int offset=numright-numleft;

 unsigned long currentmillis=millis();
 unsigned long middlemillis=millis();
  int midinterval= 70;
  int runinterval=1;
    
  runway_strip.setPixelColor(i, 255,0,0);
  runway_strip.setPixelColor(total-i,255,0,0);
  runway_strip.show();

  if(currentmillis-previousmillis>runinterval){
    previousmillis=currentmillis;

   runway_strip.setPixelColor(i,0);
   runway_strip.setPixelColor(total-i,0);
    i++;
  }
  
  if(i==start2){
      i=0;
  }
  
 //for the middle
   if(middlemillis-middlepreviousmillis>midinterval)
   {
     middlepreviousmillis=middlemillis;
         if(red_blinky == 0)
           {
              for(j=0;j<nummid;j++)
              {
               runway_strip.setPixelColor(start2+j, 0, 15, 0);
               runway_strip.show();
             }
             Serial.println("blink");
             red_blinky = 1;
           }//end if
           
         else
         {     
           Serial.println("noblink");
              for(j=0;j<nummid;j++)
              {
                 runway_strip.setPixelColor(start2+j, 0);
                                runway_strip.show();
              }
                  red_blinky = 0;
         }//end else
   }//end milliif
     
}//end colorwipe
//}//END NEO2_MODE

/************************************************************************/
/*            MP3/Sound Select Modes                          */
/************************************************************************/
void MP3_MODE (String MP3_COMMAND)
{

}//END MP3_MODE

/************************************************************************/
/*        Perimeter/Range Calibration                          */
/************************************************************************/
void TARGET_CALIBRATE()
{
/*  int position = 0;
  int i = 0;
  for(pos = TURRET_PAN_MIN; pos < TURRET_PAN_MAX; pos+=10)  
  {

  }
*/
//Serial.print("Range: ");
//Serial.println(rangeSensorAD.getRange());
//Serial.print("Best Sample: ");
//Serial.println(rangeSensorAD.getSampleBest());

}
