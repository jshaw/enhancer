/*
This code reads the Analog Voltage output from the
LV-MaxSonar sensors
If you wish for code with averaging, please see
playground.arduino.cc/Main/MaxSonar
Please note that we do not recommend using averaging with our sensors.
Mode and Median filters are recommended.
*/

#include <Adafruit_TiCoServo.h>
#include <Ultrasonic.h>

// NeoPixel Ring simple sketch (c) 2013 Shae Erisson
// released under the GPLv3 license to match the rest of the AdaFruit NeoPixel library

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

// Which pin on the Arduino is connected to the NeoPixels?
// On a Trinket or Gemma we suggest changing this to 1
#define PIN            13

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS      60

// 
// == Servos
//
Adafruit_TiCoServo servo;  // create servo object to control a servo
Adafruit_TiCoServo servo2;  // create servo object to control a servo

int min_degree = 40;
int max_degree = 120;

int pos = 90;    // variable to store the servo position
unsigned long lastUpdate; // last update of position

//int  updateInterval = 50;      // interval between updates
int  updateInterval = 200;      // interval between updates
int increment = 2;

// This seems to be smooths than above intervals and increments
//int  updateInterval = 18;      // interval between updates
//int increment = 1;

// 
// == Ultrasonic
//
// Analog Trigger Pin
int triggerPin1 = 37;

const int anPin0 = 0;
const int anPin1 = 1;
const int anPin2 = 2;
const int anPin3 = 3;

long distance0;
long distance1;
long distance2;
long distance3;
long tmp_dist;

// When we setup the NeoPixel library, we tell it how many pixels, and which pin to use to send signals.
// Note that for older NeoPixel strips you might need to change the third parameter--see the strandtest
// example for more information on possible values.
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN);

int delayval = 2; // delay for half a second

unsigned long current_millis = 0;
unsigned long lastSensorUpdate;
unsigned long updateSensorInterval = 50;

int map_val0, map_val1, map_val2, map_val3;

// sensor read millis sudo delay

void setup() {
  Serial.begin(9600);  // sets the serial port to 9600

  servo.attach(8);  // attaches the servo on pin 9 to the servo object
  servo2.attach(9);  // attaches the servo on pin 9 to the servo object

  // This is for Trinket 5V 16MHz, you can remove these three lines if you are not using a Trinket
  #if defined (__AVR_ATtiny85__)
    if (F_CPU == 16000000) clock_prescale_set(clock_div_1);
  #endif
    // End of trinket special code
    
    pixels.begin(); // This initializes the NeoPixel library.
}

void read_sensors(){
  /*
  Scale factor is (Vcc/512) per inch. A 5V supply yields ~9.8mV/in
  Arduino analog pin goes from 0 to 1024, so the value has to be divided by 2 to get the actual inches
  */

  if((current_millis - lastSensorUpdate) > updateSensorInterval)  // time to update
  {
    lastSensorUpdate = millis();
    distance0 = analogRead(anPin0)/2;
    distance1 = analogRead(anPin1)/2;
    distance2 = analogRead(anPin2)/2;
    distance3 = analogRead(anPin3)/2;
  }
}

void start_sensor(){
  digitalWrite(triggerPin1,HIGH);
  delay(1);
  digitalWrite(triggerPin1,LOW);
}

void print_all(){
  Serial.print("S0");
  Serial.print(" ");
  Serial.print(distance0);
  Serial.print(" inches");
  Serial.print(" || ");
  Serial.print("S1");
  Serial.print(" ");
  Serial.print(distance1);
  Serial.print(" inches");
  Serial.print(" || ");
  Serial.print("S2");
  Serial.print(" ");
  Serial.print(distance2);
  Serial.print(" inches");
  Serial.print(" || ");
  Serial.print("S3");
  Serial.print(" ");
  Serial.print(distance3);
  Serial.print(" inches");
  Serial.println();
}

void loop() {

  current_millis = millis();
  start_sensor();
  read_sensors();
//  print_all();

  // For a set of NeoPixels the first NeoPixel is 0, second is 1, all the way up to the count of pixels minus one.

  if((current_millis - lastUpdate) > updateInterval)  // time to update
  {
    lastUpdate = millis();
    pos += increment;
    if ((pos >= max_degree) || (pos <= min_degree)) // end of sweep
    {
      // reverse direction
      increment = -increment;
    }
    
    servo.write(pos);
    servo2.write(pos);
  }
  
  if(pos % 10 == 0){
    map_val0 = map(distance0, 5, 250, 0, 250);
    map_val1 = map(distance1, 5, 250, 0, 250);
    map_val2 = map(distance2, 5, 250, 0, 250);
    map_val3 = map(distance3, 5, 250, 0, 250);

    Serial.print(map_val0);
    Serial.print(" inches");

    Serial.print(" || ");

    Serial.print(map_val1);
    Serial.print(" inches");

    Serial.print(" || ");
      
    Serial.print(map_val2);
    Serial.print(" inches");

    Serial.print(" || ");
      
    Serial.print(map_val3);
    Serial.print(" inches");
    Serial.println();
        
    for(int i=0;i<NUMPIXELS;i++){
  
      if(i < 15)
      {
        pixels.setPixelColor(i, pixels.Color(map_val0, map_val0, map_val0));
      } else if(i < 30)
      {
        pixels.setPixelColor(i, pixels.Color(map_val1, map_val1, map_val1));
      } else if(i < 45)
      {
        pixels.setPixelColor(i, pixels.Color(map_val2, map_val2, map_val2));
      }else
      {
        pixels.setPixelColor(i, pixels.Color(map_val3, map_val3, map_val3));
      }  
    }
  
    pixels.show(); // This sends the updated pixel color to the hardware.
  }

  delay(delayval);
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return pixels.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return pixels.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return pixels.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}


