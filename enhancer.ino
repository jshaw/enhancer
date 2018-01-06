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

// Commands

// Go / Start
// g = 103

// Stop
// s = 115

// Next
// n = 110

// Previous
// p = 112

// Set config
// c = 99



#if defined(__AVR_ATtiny85__)
 #error "This code is for ATmega boards, see other example for ATtiny."
#endif

#define DEBUG false

#include <Adafruit_TiCoServo.h>
//#include <Servo.h>
//#include <Ultrasonic.h>
//#include <SimplexNoise.h>
//#include <Array.h>

// NeoPixel Ring simple sketch (c) 2013 Shae Erisson
// released under the GPLv3 license to match the rest of the AdaFruit NeoPixel library

//#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

// Which pin on the Arduino is connected to the NeoPixels?
// On a Trinket or Gemma we suggest changing this to 1
#define PIN            13

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS      60

#define MAX_DISTANCE 400 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 33 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).

//Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

String mode = "stop";
int pos = 0;    // variable to store the servo position
unsigned long ping_current_millis;

// defaults to stop
int incomingByte = 115;

// Mega trigger
int triggerPin = 37;

int update_interval = 18;
int increment = 1;

class Enhancer
{
  // Constructor
//  SimplexNoise sn;
  // create servo object to control a servo
  Adafruit_TiCoServo *servo;
//  Servo servo;

  unsigned long current_millis;

  int pin_cache;
  int min_degree = 40;
  int max_degree = 120;
  
  double n;
  float increase = 0.01;
  float x = 0.0;
  float y = 0.0;

  int pos;              // current servo position
  int increment;        // increment to move for each interval
  int updateInterval;      // interval between updates
  unsigned long lastUpdate; // last update of position
  int sonar;
  int currentDistance;
  int id;

  unsigned long pausedPreviousMillis;
  unsigned long pausedInterval;
  bool paused;

  unsigned long pauseRepopulateDistanceMeasurementMillis;
  unsigned long pauseRepopulateInterval;
  bool pausedRepopulate;

  boolean printJSON = true;
  boolean publish_data = false;

  // number of pings collected
  unsigned long pingTotalCount = 0;
  // number of pings before send for simplexNoise
  // unsigned long pingRemainderValue = 50;
  unsigned long pingRemainderValue = 10;

  int distance;
//  int anPin;

  public:
    Enhancer(Adafruit_TiCoServo servo, int ide, int interval, int sonr, int position, String mode, unsigned long pcount, int incmt)
    {

//      servo = theServo;
      
      mode = mode;
      updateInterval = interval;
      // makes sure the ID never gets out of the number of objects
      id = constrain(ide, 0, 15);
      pos = position;
      increment = incmt;
      paused = false;
      sonar = sonr;
      
      // sets the pingcount offset so it doesn't send all of the scanned data through serial at once.
      pingTotalCount = pcount;

      pausedPreviousMillis = 0;
      pausedInterval = 2000;

      // this is for that pause for repopulating the data acter an avoidance
      pauseRepopulateDistanceMeasurementMillis = 0;
      pauseRepopulateInterval = 2000;
      pausedRepopulate = true;
    
      // sets the noise x pos randomly to prevent objects moving in the same pattern
      x = random(0.0, 20.0);      
    }

    // Not sure if this needs to be individually, but more as an enviroment as a whole.
    // One trigger pin on setup and as a start sensor...
    // Ref multi_sensor_test for example...
    void StartSensor(int pin)
    {
//      digitalWrite(triggerPin,HIGH);
//      delay(1);
//      digitalWrite(triggerPin,LOW);
    }
    
    void Attach(int pin)
    {

      // if it is not attached, attach
      // otherwise don't try and re-attach
      if(servo->attached() == 0){

//        Serial.print("Attache pin: ");
//        Serial.println(pin);
        servo->attach(pin); 
      }

      // cache the pin ID in constructure to use later for 
      // attaching and detaching
      if(!pin_cache){
        PinCache(pin);
      }
      
    }

    void PinCache (int pin){
      pin_cache = pin;
    }

    void Detach()
    {
      servo->detach();
    }

    void SetPos(int startPosition)
    {
      pos = startPosition;
      servo->write(pos);
    }
  
    int isAttached()
    {
      return servo->attached();
    }

    void switchIncrementDirection()
    {
      increment = -increment;
    }

    void GoTo(int pos)
    {
      servo->write(pos);
    }

    void resetDefaults(){
      increment = 2;
      lastUpdate = millis();
      paused = false;
      pos = 90;
      pausedPreviousMillis = millis();
    }

    void setMode (String md){
      mode = md;
    }

    void PrintDistance(int d)
    {
      Serial.print("Print Distance: ");
      Serial.println(d);
      // Serial.println(sonar->ping_result);
      Serial.println("===================");
    }

    void Update() {

//      Serial.print("In Update, ID: ");
//      Serial.println(id);

      if (pos == -1) {
        pos = 0;
        servo->write(pos);
      }

      current_millis = millis();

      // pause after reset to gather new distance measurements

//      if ((current_millis - pauseRepopulateDistanceMeasurementMillis) > pauseRepopulateInterval) {
//        pausedRepopulate = false;
//      }
      
      if (mode == "sweep") {

        if((current_millis - lastUpdate) > updateInterval)  // time to update
        {
          lastUpdate = millis();
          pos += increment;
    
          // 
          // =================
          Attach(pin_cache);
          servo->write(pos);
          delay(2);

          // sweep
          // == BEGINNING OF SWEEP SEND DATA
          if ((pos >= max_degree) || (pos <= min_degree)) // end of sweep
          {
            Detach();
            servo->attach(pin_cache);
            // reverse direction
            increment = -increment;
          }
          // END OF SWEEP SEND DATA

        }

      }

    }

    void readSensor() {
      // #TODO: update values.. for this class here
      distance = analogRead(sonar)/2;
    }

    int getSensorReading() {
      return distance;
    }
};

#define OBJECT_NUM  2

unsigned long pingTimer[OBJECT_NUM]; // Holds the times when the next ping should happen for each sensor.
unsigned int cm[OBJECT_NUM];         // Where the ping distances are stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.

// Sensor object array.
// Each sensor's analog pin, trigger pin, and max distance to ping.
int sonar[OBJECT_NUM][3] = {
  {A0, triggerPin, 800},
  {A1, triggerPin, 800}
};

Adafruit_TiCoServo servo0;
Adafruit_TiCoServo servo1;

// Sensor object array.
// ID, Update Interval, Sonar ID, Start Possition, mode, ping index offset, increment
Enhancer enhance[OBJECT_NUM] = {
  Enhancer(servo0, 0, update_interval, sonar[0], 90, mode, 0, increment),
  Enhancer(servo1, 1, update_interval, sonar[1], 90, mode, 2, increment)
};

void setup() {
  Serial.begin(115200);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // First ping starts at 75ms, gives time for the Arduino to chill before starting.
  pingTimer[0] = millis() + 75;
  // Set the starting time for each sensor.
  for (uint8_t i = 1; i < OBJECT_NUM; i++) {
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
  }

  Serial.println("In Setup");

  massAttatch();
  establishContact();
}

void sensorSetup(){
  pinMode(triggerPin, OUTPUT);
}

void establishContact() {
  while (Serial.available() <= 0) {
    // send a capital A
    // Serial.println('A');
//    delay(300);
  }
}

void loop() {
  
  // read the incoming byte:
  if (Serial.available() > 0) {
    incomingByte = Serial.read();
    Serial.println("Character: " + incomingByte);
  }

    // Different Key Codes
  // =================
  
  // Go
  // g = 103

  // Stop
  // s = 115

  // Next
  // n = 110

  // Previous
  // p = 112

  // Configure
  // c = 99

  // Mode Controles + hex values
  // 1 = Sweep / 49
  // 2 = sweep react / 50
  // 3 = sweep react pause / 51
  // 4 = noise / 52
  // 5 = noise react / 53
  // 6 = pattern_wave_small_v2: smaller wave form / 54
  // 7 = measure / 55
  // 8 = measure + react only / 56

  // END OF CODES
  // ==================

  
  // go
  if (incomingByte == 103) {
    
    massAttatch();
    mode = "sweep";
  
  // stop
  } else if (incomingByte == 115) {
    mode = "stop";
    pos = 90;
    for (uint8_t i = 0; i < OBJECT_NUM; i++) {
      enhance[i].GoTo(pos);
      enhance[i].resetDefaults();
    }
//    delay(500);
    massDetatch();
    return;

  // resets position back to 90
  // 'c'
  } else if (incomingByte == 99) {
    pos = 90;
    for (uint8_t i = 0; i < OBJECT_NUM; i++) {
      enhance[i].GoTo(pos);
      enhance[i].resetDefaults();
    }
    return;

  // Sweep
  } else if (incomingByte == 49) {
    mode = "sweep";
    for (uint8_t i = 0; i < OBJECT_NUM; i++) {
     enhance[i].setMode(mode);
     enhance[i].resetDefaults();
    }
  }
  
  incomingByte = 0;

  for (uint8_t i = 0; i < OBJECT_NUM; i++) {
//    Serial.println("In Object Loop!");
    enhance[i].Update();
  }
}

void massAttatch() {

  Serial.println("In Attach");
  // Attach all motors
  enhance[0].Attach(2);
  enhance[1].Attach(3);
}

// detatch all servos
void massDetatch() {
  Serial.println("In DETATCH");
  // detatch all motors to save energy / motor life span
  // Set the starting time for each sensor.
  for (uint8_t i = 0; i < OBJECT_NUM; i++) {
    enhance[i].Detach();
  }
}

