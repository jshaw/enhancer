// ---------------------------------------------------------------------------
// Enhancer
// By: Jordan Shaw
// Libs: Adafruit_TiCoServo, Adafruit_NeoPixel, Ultrasonic, Array
// ---------------------------------------------------------------------------

/*
This code reads the Analog Voltage output from the
LV-MaxSonar sensors
If you wish for code with averaging, please see
playground.arduino.cc/Main/MaxSonar
Please note that we do not recommend using averaging with our sensors.
Mode and Median filters are recommended.
*/

#if defined(__AVR_ATtiny85__)
 #error "This code is for ATmega boards, see other example for ATtiny."
#endif

#include <SimplexNoise.h>

#include <Adafruit_NeoPixel.h>
#include <Adafruit_TiCoServo.h>

// General Define
// =======
#define OBJECT_NUM  2

// LEDS
// =========

// NeoPixel parameters. These are configurable, but the pin number must
// be different than the servo(s).
#define NUMPIXELS          60
#define LED_PIN            13
Adafruit_NeoPixel  strip = Adafruit_NeoPixel(NUMPIXELS, LED_PIN);

// SERVOS
// =========
#define SERVO_PIN0    2
#define SERVO_PIN1    3
#define SERVO_MIN 1000 // 1 ms pulse
#define SERVO_MAX 2000 // 2 ms pulse

#define MIN_DEGREE    40
#define MAX_DEGREE    120
#define SERVO_PIN1    3

Adafruit_TiCoServo servo0;
Adafruit_TiCoServo servo1;

// ULTRASONIC
// =========
#define SONAR_TIGGER_PIN   37
// Maximum distance (in cm) to ping
#define MAX_DISTANCE 400
// Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).
#define PING_INTERVAL 50

int sensorPins[16] = {A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15};
int distance[16]= {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int mappedDistance[16]= {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// Holds the times when the next ping should happen for each sensor.
unsigned long pingTimer[OBJECT_NUM];
unsigned int cm[OBJECT_NUM];         // Where the ping distances are stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.

// HSB vars
// ===========
SimplexNoise snh;
int saturation = 255;
int brightness = 255;
double nh;
float xh = 0.0;
int posh = 90;
float increase_sb = 0.0005;

// Noise vars
// ===========
SimplexNoise sn0;
SimplexNoise sn1;
//SimplexNoise sn[16]= {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
double n0;
double n1;
float increase = 0.01;
float x0 = 0.0;
float x1 = 0.0;
float y = 0.0;

int pos0 = 90;
int pos1 = 90;


// Start of customizable variables
// ===========================
int pos = 90;    // variable to store the servo position
unsigned long current_millis = 0;
unsigned long lastUpdate; // last update of position
int  updateInterval = 18;      // interval between updates
int increment = 1;

// Specific vars
// ================
unsigned long ping_current_millis;

bool animate_hsb = true;
bool animate_sb = true;
bool animate_noise = true;

void setup(void) {
  Serial.begin(115200);  // sets the serial port to 9600
//  servo.attach(SERVO_PIN, SERVO_MIN, SERVO_MAX);
  servo0.attach(SERVO_PIN0);  // attaches the servo on pin 9 to the servo object
  servo1.attach(SERVO_PIN1);  // attaches the servo on pin 9 to the servo object
  
  strip.begin();

  xh = random(0.0, 20.0);

  x0 = random(0.0, 20.0);
  x1 = random(0.0, 20.0);

  // Sensor Setup
  // ============
  pinMode(SONAR_TIGGER_PIN, OUTPUT);

  // First ping starts at 75ms, gives time for the Arduino to chill before starting.
  pingTimer[0] = millis() + 75;
  // Set the starting time for each sensor.
  for (uint8_t i = 1; i < OBJECT_NUM; i++) {
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
  }
}

void loop(void) {
//  int a, x;
//  a = analogRead(0);                            // 0 to 1023
//  x = map(a, 0, 1023, SERVO_MIN, SERVO_MAX);    // Scale to servo range
//  servo.write(x);                               // Move servo
//  x = map(a, 0, 1023, 0, strip.numPixels());    // Scale to strip length
//  strip.clear();
//  while(x--) strip.setPixelColor(x, 255, 0, 0); // Set pixels
//  strip.show();                                 // Update strip

    current_millis = millis();

    for (uint8_t i = 0; i < OBJECT_NUM; i++) {
      ping_current_millis = millis();
  
      // Is it this sensor's time to ping?
      if (ping_current_millis >= pingTimer[i]) {
        // Set next time this sensor will be pinged.
        pingTimer[i] += PING_INTERVAL * OBJECT_NUM;
        start_sensor();
        distance[i] = analogRead(sensorPins[i])/2;
        mappedDistance[i] = map(distance[i], 0, 350, 0, 250);
      }
    }

    if(pos % 10 == 0){
      
      int pixels_per_section = NUMPIXELS / OBJECT_NUM;
      
      for(int i=0; i<OBJECT_NUM;i++){

        for(int j = 0 + (pixels_per_section*i); j<pixels_per_section + (pixels_per_section*i); j++){
          int pixl = j;

          if(animate_hsb == false){
            strip.setPixelColor(pixl, strip.Color(mappedDistance[i], mappedDistance[i], mappedDistance[i]));
          } else {
            int hue = map(mappedDistance[i],0, 400,0, 359);     // hue is a number between 0 and 360

            if(animate_sb = true){
              // saturation is a number between 0 - 255

              nh = snh.noise(xh, y);
              xh += increase_sb;

              // this works for when it's being used for brightness..
              // posh = (int)map(nh*100, -100, 100, 0, 255);

              // this works best for saturation
              posh = (int)map(nh*100, -100, 100, 100, 255);
        
              saturation = posh;
              brightness = 255;
            } else {
              // saturation is a number between 0 - 255
              saturation = 255;
              brightness = 255;
            }
            

            // This parsing was incluenced by the following post...
            // https://stackoverflow.com/questions/11068450/arduino-c-language-parsing-string-with-delimiter-input-through-serial-interfa
            String returnVal = getRGB(hue, saturation, brightness);
            // Serial.println(returnVal);

            int commaIndex = returnVal.indexOf('/');
            int secondCommaIndex = returnVal.indexOf('/', commaIndex + 1);

            String firstValue = returnVal.substring(0, commaIndex);
            String secondValue = returnVal.substring(commaIndex + 1, secondCommaIndex);
            String thirdValue = returnVal.substring(secondCommaIndex + 1); // To the end of the string

            int r = firstValue.toInt();
            int g = secondValue.toInt();
            int b = thirdValue.toInt();
            
            strip.setPixelColor(pixl, strip.Color(r, g, b));
          }

          // TODO:
          // There could be something super cool by randomly selecting what RGB values are static and have one 
          // rgb value represented by sensors
          // OR OR OR
          // could be super cool to have noise values for two of three RGB and one controlled by light
        }
      }
    
      strip.show(); // This sends the updated pixel color to the hardware.
    }
    
    if((current_millis - lastUpdate) > updateInterval)  // time to update
    {
      lastUpdate = millis();

      if(animate_noise == false){
        pos += increment;
        
        if ((pos >= MAX_DEGREE) || (pos <= MIN_DEGREE)) // end of sweep
        {
          // reverse direction
          increment = -increment;
        }
      } else{
        n0 = sn0.noise(x0, y);
        n1 = sn1.noise(x1, y);
        x0 += increase;
        x1 += increase;
  
        pos0 = (int)map(n0*100, -100, 100, MIN_DEGREE, MAX_DEGREE);
        pos1 = (int)map(n1*100, -100, 100, MIN_DEGREE, MAX_DEGREE);

        servo0.write(pos0);
        servo1.write(pos1);
      }
    }

    if(animate_noise == false){
      servo0.write(pos);
      servo1.write(pos);
    }

    delay(2);
  
}

// Debugging
void print_all(){
  Serial.print("S1");
  Serial.print(" ");
  Serial.print(distance[0]);
  Serial.print(" inches");
  Serial.print(" || ");
  Serial.print("S2");
  Serial.print(" ");
  Serial.print(distance[1]);
  Serial.print(" inches");
  Serial.println();
}

void start_sensor(){
  digitalWrite(SONAR_TIGGER_PIN, HIGH);
  delay(1);
  digitalWrite(SONAR_TIGGER_PIN, LOW);
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}


// HSB to RGB code was influenced by
// https://www.kasperkamperman.com/blog/arduino/arduino-programming-hsb-to-rgb/
const byte dim_curve[] = {
    0,   1,   1,   2,   2,   2,   2,   2,   2,   3,   3,   3,   3,   3,   3,   3,
    3,   3,   3,   3,   3,   3,   3,   4,   4,   4,   4,   4,   4,   4,   4,   4,
    4,   4,   4,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   6,   6,   6,
    6,   6,   6,   6,   6,   7,   7,   7,   7,   7,   7,   7,   8,   8,   8,   8,
    8,   8,   9,   9,   9,   9,   9,   9,   10,  10,  10,  10,  10,  11,  11,  11,
    11,  11,  12,  12,  12,  12,  12,  13,  13,  13,  13,  14,  14,  14,  14,  15,
    15,  15,  16,  16,  16,  16,  17,  17,  17,  18,  18,  18,  19,  19,  19,  20,
    20,  20,  21,  21,  22,  22,  22,  23,  23,  24,  24,  25,  25,  25,  26,  26,
    27,  27,  28,  28,  29,  29,  30,  30,  31,  32,  32,  33,  33,  34,  35,  35,
    36,  36,  37,  38,  38,  39,  40,  40,  41,  42,  43,  43,  44,  45,  46,  47,
    48,  48,  49,  50,  51,  52,  53,  54,  55,  56,  57,  58,  59,  60,  61,  62,
    63,  64,  65,  66,  68,  69,  70,  71,  73,  74,  75,  76,  78,  79,  81,  82,
    83,  85,  86,  88,  90,  91,  93,  94,  96,  98,  99,  101, 103, 105, 107, 109,
    110, 112, 114, 116, 118, 121, 123, 125, 127, 129, 132, 134, 136, 139, 141, 144,
    146, 149, 151, 154, 157, 159, 162, 165, 168, 171, 174, 177, 180, 183, 186, 190,
    193, 196, 200, 203, 207, 211, 214, 218, 222, 226, 230, 234, 238, 242, 248, 255,
};

String getRGB(int hue, int sat, int val) { 
  /* convert hue, saturation and brightness ( HSB/HSV ) to RGB
     The dim_curve is used only on brightness/value and on saturation (inverted).
     This looks the most natural.      
  */

  val = dim_curve[val];
  sat = 255-dim_curve[255-sat];

  int r;
  int g;
  int b;
  int base;

  base = ((255 - sat) * val)>>8;

  switch(hue/60) {
  case 0:
      r = val;
      g = (((val-base)*hue)/60)+base;
      b = base;
  break;

  case 1:
      r = (((val-base)*(60-(hue%60)))/60)+base;
      g = val;
      b = base;
  break;

  case 2:
      r = base;
      g = val;
      b = (((val-base)*(hue%60))/60)+base;
  break;

  case 3:
      r = base;
      g = (((val-base)*(60-(hue%60)))/60)+base;
      b = val;
  break;

  case 4:
      r = (((val-base)*(hue%60))/60)+base;
      g = base;
      b = val;
  break;

  case 5:
      r = val;
      g = base;
      b = (((val-base)*(60-(hue%60)))/60)+base;
  break;
  }

  String valToReturn = String(r) + "/" + String(g) + "/" + String(b);
  return valToReturn;
}
