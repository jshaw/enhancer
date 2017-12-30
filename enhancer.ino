/*
This code reads the Analog Voltage output from the
LV-MaxSonar sensors
If you wish for code with averaging, please see
playground.arduino.cc/Main/MaxSonar
Please note that we do not recommend using averaging with our sensors.
Mode and Median filters are recommended.
*/

// ---------------------------------------------------------------------------
// Enhancer
// By: Jordan Shaw
// Libs: Adafruit_TiCoServo, Adafruit_NeoPixel, Ultrasonic, Array
// ---------------------------------------------------------------------------

#define DEBUG false

#include <Adafruit_TiCoServo.h>
#include <Ultrasonic.h>
#include <SimplexNoise.h>
#include <Array.h>

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
Adafruit_TiCoServo servo0;  // create servo object to control a servo
Adafruit_TiCoServo servo1;  // create servo object to control a servo
Adafruit_TiCoServo servo2;  // create servo object to control a servo
Adafruit_TiCoServo servo3;  // create servo object to control a servo
Adafruit_TiCoServo servo4;  // create servo object to control a servo
Adafruit_TiCoServo servo5;  // create servo object to control a servo
Adafruit_TiCoServo servo6;  // create servo object to control a servo
Adafruit_TiCoServo servo7;  // create servo object to control a servo
Adafruit_TiCoServo servo8;  // create servo object to control a servo
Adafruit_TiCoServo servo9;  // create servo object to control a servo
Adafruit_TiCoServo servo10;  // create servo object to control a servo
Adafruit_TiCoServo servo11;  // create servo object to control a servo
Adafruit_TiCoServo servo12;  // create servo object to control a servo
Adafruit_TiCoServo servo13;  // create servo object to control a servo
Adafruit_TiCoServo servo14;  // create servo object to control a servo
Adafruit_TiCoServo servo15;  // create servo object to control a servo

int min_degree = 40;
int max_degree = 120;

int pos = 90;    // variable to store the servo position
unsigned long lastUpdate; // last update of position

// interval between updates
int updateInterval = 50;
int update_interval = updateInterval;
int increment = 2;

// This seems to be smooths than above intervals and increments
// interval between updates
//int  updateInterval = 18;
//int increment = 1;

// 
// == Ultrasonic
//

const int anPin0 = A0;
const int anPin1 = A1;
const int anPin2 = A2;
const int anPin3 = A3;
const int anPin4 = A4;
const int anPin5 = A5;
const int anPin6 = A6;
const int anPin7 = A7;
const int anPin8 = A8;
const int anPin9 = A9;
const int anPin10 = A10;
const int anPin11 = A11;
const int anPin12 = A12;
const int anPin13 = A13;
const int anPin14 = A14;
const int anPin15 = A15;

long distance0;
long distance1;
long distance2;
long distance3;
long distance4;
long distance5;
long distance6;
long distance7;
long distance8;
long distance9;
long distance10;
long distance11;
long distance12;
long distance13;
long distance14;
long distance15;

long tmp_dist;

// When we setup the NeoPixel library, we tell it how many pixels, and which pin to use to send signals.
// Note that for older NeoPixel strips you might need to change the third parameter--see the strandtest
// example for more information on possible values.
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN);

int delayval = 2; // delay for half a second

unsigned long current_millis = 0;
unsigned long lastSensorUpdate;
unsigned long updateSensorInterval = 50;

int map_val1, map_val2;

// sensor read millis sudo delay


class Enhancer
{
  SimplexNoise sn;

  unsigned long current_millis;
  int pin_cache;
  int min_degree = 0;
  int max_degree = 0;
  
  boolean noise_interact_triggered = false;
  int noise_trigger_pos = 0;
  boolean sweep_react_pause_triggered = false;
  int sweep_react_pause_pos = 0;

  double n;
  float increase = 0.01;
  float x = 0.0;
  float y = 0.0;

  int minAngle = 10;
  int maxAngle = 170;

  // create servo object to control a servo
  Adafruit_TiCoServo servo;
  int pos;              // current servo position
  int increment;        // increment to move for each interval
  int  updateInterval;      // interval between updates
  unsigned long lastUpdate; // last update of position
  NewPing *sonar;
  int currentDistance;
  int id;
  int lowPos;
  int highPos;
  int lowDistance;
  int highDistance;

  String sweepString = "";

  unsigned long pausedPreviousMillis;
  unsigned long pausedInterval;
  bool paused;

  unsigned long pauseRepopulateDistanceMeasurementMillis;
  unsigned long pauseRepopulateInterval;
  bool pausedRepopulate;

  // number of pings collected
  unsigned long pingTotalCount = 0;
  // number of pings before send for simplexNoise
  // unsigned long pingRemainderValue = 50;
  unsigned long pingRemainderValue = 10;

  // this section is for interaction smoothing
  //===========================
  static const int numReadings = 5;
  // the readings from the analog input
  int readings[numReadings];
  // the index of the current reading
  int readIndex = 0;
  // the running total
  int total = 0;
  // the average
  int average = 0;
  // END ===========================

  public:
    Enhancer(int ide, int interval, const int pinSonar, int position, String mode, unsigned long pcount)
    {
      
    }

    
    void Attach(int pin)
    {
      // if it is not attached, attach
      // otherwise don't try and re-attach
      if(servo.attached() == 0){
        servo.attach(pin); 
      }

      // cache the pin ID in constructure to use later for 
      // attaching and detaching
      if(!pin_cache){
        PinCache(pin);
      }
    }

    void PinCache (int pin)
    {
      pin_cache = pin;
    }

    void Detach()
    {
      servo.detach();
    }

    void SetPos(int startPosition)
    {
      pos = startPosition;
      servo.write(pos);
    }
  
    int isAttached()
    {
      return servo.attached();
    }

    void switchIncrementDirection()
    {
      increment = -increment;
    }

    void GoTo(int pos)
    {
      servo.write(pos);
    }

    void resetDefaults()
    {
      increment = 2;
      lastUpdate = millis();
      paused = false;
      pos = 90;
      pausedPreviousMillis = millis();
    }

    void Update() {

    }

    void setMode (String md){
      mode = md;
    }
};
      
// Number of sensors.
#define OBJECT_NUM  16

unsigned long pingTimer[OBJECT_NUM]; // Holds the times when the next ping should happen for each sensor.
unsigned int cm[OBJECT_NUM];         // Where the ping distances are stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.

// Sensor object array.
// ID, Update Interval, Sonar Pin ID, Start Possition, mode, ping index offset
Enhancer enhance[OBJECT_NUM] = {
  Enhancer(0, update_interval, anPin0, 90, mode, 0),
  Enhancer(1, update_interval, anPin1, 90, mode, 2),
  Enhancer(2, update_interval, anPin2, 90, mode, 4),
  Enhancer(3, update_interval, anPin3, 90, mode, 6),
  Enhancer(4, update_interval, anPin4, 90, mode, 8),
  Enhancer(5, update_interval, anPin5, 90, mode, 10),
  Enhancer(6, update_interval, anPin6, 90, mode, 12),
  Enhancer(7, update_interval, anPin7, 90, mode, 14),
  Enhancer(8, update_interval, anPin8, 90, mode, 16),
  Enhancer(9, update_interval, anPin9, 90, mode, 18),
  Enhancer(10, update_interval, anPin10, 90, mode, 20),
  Enhancer(11, update_interval, anPin11, 90, mode, 22),
  Enhancer(12, update_interval, anPin12, 90, mode, 24),
  Enhancer(13, update_interval, anPin13, 90, mode, 26),
  Enhancer(14, update_interval, anPin14, 90, mode, 28),
  Enhancer(15, update_interval, anPin15, 90, mode, 30)
};



void setup() {
  Serial.begin(9600);  // sets the serial port to 9600

//  servo.attach(9);  // attaches the servo on pin 9 to the servo object
//  servo2.attach(10);  // attaches the servo on pin 9 to the servo object

  // This is for Trinket 5V 16MHz, you can remove these three lines if you are not using a Trinket
  #if defined (__AVR_ATtiny85__)
    if (F_CPU == 16000000) clock_prescale_set(clock_div_1);
  #endif
    // End of trinket special code
    
  pixels.begin(); // This initializes the NeoPixel library.

  massAttatch();

  establishContact();
}

void establishContact() {
  while (Serial.available() <= 0) {
    // send a capital A
    // Serial.println('A');
    delay(300);
  }
}

void read_sensors(){
  /*
  Scale factor is (Vcc/512) per inch. A 5V supply yields ~9.8mV/in
  Arduino analog pin goes from 0 to 1024, so the value has to be divided by 2 to get the actual inches
  */

  if((current_millis - lastSensorUpdate) > updateSensorInterval)  // time to update
  {
    lastSensorUpdate = millis();
    distance1 = analogRead(anPin1)/2;
    distance2 = analogRead(anPin2)/2;
  }
}

void print_all(){
  Serial.print("S1");
  Serial.print(" ");
  Serial.print(distance1);
  Serial.print(" inches");
  Serial.print(" || ");
  Serial.print("S2");
  Serial.print(" ");
  Serial.print(distance2);
  Serial.print(" inches");
  Serial.println();
}

void loop() {

  current_millis = millis();
  
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
    map_val1 = map(distance1, 0, 350, 0, 250);
    map_val2 = map(distance2, 0, 350, 0, 250);

    Serial.print(map_val1);
    Serial.print(" inches");

    Serial.print(" || ");
      
    Serial.print(map_val2);
    Serial.print(" inches");
    Serial.println();
        
    for(int i=0;i<NUMPIXELS;i++){
  
      if(i < 30){
        pixels.setPixelColor(i, pixels.Color(map_val1, map_val1, map_val1));            
      } else {
        pixels.setPixelColor(i, pixels.Color(map_val2, map_val2, map_val2));
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

// detatch all servos
void massDetatch() {
  // detatch all motors to save energy / motor life span
  // Set the starting time for each sensor.
  for (uint8_t i = 0; i < OBJECT_NUM; i++) {
    sweep[i].Detach();
  }
}

void massAttatch() {
  // Attach all motors
  enhance[0].Attach(A0);
  enhance[1].Attach(A1);
  enhance[2].Attach(A2);
  enhance[3].Attach(A3);
  enhance[4].Attach(A4);
  enhance[5].Attach(A5);
  enhance[6].Attach(A6);
  enhance[7].Attach(A7);
  enhance[8].Attach(A8);
  enhance[9].Attach(A9);
  enhance[10].Attach(40);
  enhance[11].Attach(38);
  enhance[12].Attach(36);
  enhance[13].Attach(34);
  enhance[14].Attach(32);
  enhance[15].Attach(30);
}

