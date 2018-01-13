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

// Include for which one is mega & which one is uno
#include <EEPROM.h>

// General Define
// =======
#define OBJECT_NUM  2

// EEPRAM PANEL SOLUTION
// 0 = Mega
// 1 = uno
int arduino = 0;
int address = 1;

// this value is representative of the panel ID
// ie 0, 1
byte arduino_byte_value = 1;
byte a1;

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
#define SERVO_PIN2    5
#define SERVO_PIN3    6
#define SERVO_PIN4    7
#define SERVO_PIN5    8
#define SERVO_PIN6    11
#define SERVO_PIN7    12
#define SERVO_PIN8    13
#define SERVO_PIN9    44
#define SERVO_PIN10   45
#define SERVO_PIN11   46
#define SERVO_PIN12   5
#define SERVO_PIN13   6
#define SERVO_PIN14   7
#define SERVO_PIN15   8

#define SERVO_MIN 1000 // 1 ms pulse
#define SERVO_MAX 2000 // 2 ms pulse

#define MIN_DEGREE    55
#define MAX_DEGREE    125

Adafruit_TiCoServo servo0;
Adafruit_TiCoServo servo1;
Adafruit_TiCoServo servo2;
Adafruit_TiCoServo servo3;
Adafruit_TiCoServo servo4;
Adafruit_TiCoServo servo5;
Adafruit_TiCoServo servo6;
Adafruit_TiCoServo servo7;
Adafruit_TiCoServo servo8;
Adafruit_TiCoServo servo9;
Adafruit_TiCoServo servo10;
Adafruit_TiCoServo servo11;
Adafruit_TiCoServo servo12;
Adafruit_TiCoServo servo13;
Adafruit_TiCoServo servo14;
Adafruit_TiCoServo servo15;

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


// Averaging the measurements...
// ===========
// this section is for interaction smoothing
//===========================
static const int numReadings = 5;
// the readings from the analog input
int readings[16][numReadings];
int total[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int average[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
// the index of the current reading
int readIndex[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//int readIndex = 0;
// the running total
//int total = 0;
// the average
//int average = 0;

String sweepString[16] = {"", "", "", "", "", "", "", "", "", "", "", "", "", "", "", ""};
int pingTotalCount[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
unsigned long pingRemainderValue = 10;

// END ===========================

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
SimplexNoise sn2;
SimplexNoise sn3;
SimplexNoise sn4;
SimplexNoise sn5;
SimplexNoise sn6;
SimplexNoise sn7;
SimplexNoise sn8;
SimplexNoise sn9;
SimplexNoise sn10;
SimplexNoise sn11;
SimplexNoise sn12;
SimplexNoise sn13;
SimplexNoise sn14;
SimplexNoise sn15;

//SimplexNoise sn[16]= {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
double n0;
double n1;
double n2;
double n3;
double n4;
double n5;
double n6;
double n7;
double n8;
double n9;
double n10;
double n11;
double n12;
double n13;
double n14;
double n15;

float increase = 0.01;
float x0 = 0.0;
float x1 = 0.0;
float x2 = 0.0;
float x3 = 0.0;
float x4 = 0.0;
float x5 = 0.0;
float x6 = 0.0;
float x7 = 0.0;
float x8 = 0.0;
float x9 = 0.0;
float x10 = 0.0;
float x11 = 0.0;
float x12 = 0.0;
float x13 = 0.0;
float x14 = 0.0;
float x15 = 0.0;
float y = 0.0;

int pos0 = 90;
int pos1 = 90;
int pos2 = 90;
int pos3 = 90;
int pos4 = 90;
int pos5 = 90;
int pos6 = 90;
int pos7 = 90;
int pos8 = 90;
int pos9 = 90;
int pos10 = 90;
int pos11 = 90;
int pos12 = 90;
int pos13 = 90;
int pos14 = 90;
int pos15 = 90;


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

bool animate_all_equal = false;

// these go together
bool animate_hsb = true;
bool animate_sb = true; 

bool animate_noise = true;
bool animate_hsb_fade = false;
bool hsb_saturation = false;

// TODO
// BREAK mode
// When the motors take a break, do the rotating wheel visuals
boolean paused = false;

// control mode
String mode = "stop";
// defaults to stop
int incomingByte = 115;

// Publishing to Pi vars
// =============
// these two vars are pure debug variels to control what gets sent over serial or doesn't
// Help for debugging buffer limit
boolean printJSON = true;
//boolean publish_data = false;
boolean sendJSON = true;
boolean storeDataJSON = true;
boolean printStringTitle = false;
// =============


void setup(void) {
  Serial.begin(115200);  // sets the serial port to 9600
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // EEPROM Panel ID solution.
  // =========================
  // Store the ID in memory so code updates don't have to be custom to each arduino
  // this is uncommented if you are setting the EEPROM val
  //EEPROM.write(address, panel_byte_value);

  // This if for reading the EEPROM value
  // this should be the panel ID from 0, 1, 2
  //  panel = int(EEPROM.read(address));
  a1 = EEPROM.read(address);
  arduino = int(a1);

    // If the panel value isn't anything that is expected. Default to everything is panel id 0
  if (arduino != 0 && arduino != 1){
    Serial.println("Why was no panel data found?");
    arduino = 0;
  } else {
    Serial.println("Logging my arduino: ");  
    Serial.print(arduino);  
  }

  massAttatch();
  
  strip.begin();
  xh = random(0.0, 20.0);
  x0 = random(0.0, 20.0);
  x1 = random(0.0, 20.0);
  x2 = random(0.0, 20.0);
  x3 = random(0.0, 20.0);
  x4 = random(0.0, 20.0);
  x5 = random(0.0, 20.0);
  x6 = random(0.0, 20.0);
  x7 = random(0.0, 20.0);
  x8 = random(0.0, 20.0);
  x9 = random(0.0, 20.0);
  x10 = random(0.0, 20.0);
  x11 = random(0.0, 20.0);
  x12 = random(0.0, 20.0);
  x13 = random(0.0, 20.0);
  x14 = random(0.0, 20.0);
  x15 = random(0.0, 20.0);

  // Sensor Setup
  // ============
  pinMode(SONAR_TIGGER_PIN, OUTPUT);

  // First ping starts at 75ms, gives time for the Arduino to chill before starting.
  pingTimer[0] = millis() + 75;
  // Set the starting time for each sensor.
  for (uint8_t i = 1; i < OBJECT_NUM; i++) {
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
  }

  establishContact();

  for (int sensor_obj = 0; sensor_obj < OBJECT_NUM; sensor_obj++){
    // sets the smoothing array to base number
    for (int thisReading = 0; thisReading < numReadings; thisReading++) {
      readings[sensor_obj][thisReading] = 0;
    }
  }
  
}

void establishContact() {
  while (Serial.available() <= 0) {
    //send a capital A
    Serial.println('A');
    delay(300);
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

  // sweep
  // noise
  // rainbow pause



  // Mode Controles + hex values
  // 1 = Sweep / 49
  // 4 = noise / 52
  // 8 = rainbow/paused 56 

  // this isn't 
  // Previous
  // p = 112
  // Configure
  // c = 99

  // END OF CODES
  // ==================

  if (incomingByte == 103) {
    massAttatch();
    mode = "sweep";
  // STOP movement
  } else if (incomingByte == 115) {
    mode = "stop";
    pos = 90;
    servo0.write(pos);
    servo1.write(pos);
    servo2.write(pos);
    servo3.write(pos);
    servo4.write(pos);
    servo5.write(pos);
    servo6.write(pos);
    servo7.write(pos);
    servo8.write(pos);
    servo9.write(pos);
    servo10.write(pos);
    servo11.write(pos);
    servo12.write(pos);
    servo13.write(pos);
    servo14.write(pos);
    servo15.write(pos);
    delay(500);
    massDetatch();
    return;
  } else if (incomingByte == 22) {
    massAttatch();
    mode = "noise";
  } else if (incomingByte == 49) {
    massAttatch();
    mode = "sweep";
  }

//  int ColorFadeIndex = 0;
//  int Interval = 10;
//  int TotalSteps = 5;

  current_millis = millis();
  int pixels_per_section = NUMPIXELS / OBJECT_NUM;

  // UNO Code
  // this is for the 4 motors that need to be run off of a UNO
  // ================================
  if (arduino == 1){
    
    if((current_millis - lastUpdate) > updateInterval)  // time to update
    {
      lastUpdate = millis();
  
      if(animate_noise == false){
        
        pos12 += increment;
        pos13 += increment;
        pos14 += increment;
        pos15 += increment;
        
        if ((pos12 >= MAX_DEGREE) || (pos12 <= MIN_DEGREE)) // end of sweep
        {
          // reverse direction
          increment = -increment;
        }
      } else{
        n12 = sn12.noise(x12, y);
        n13 = sn13.noise(x13, y);
        n14 = sn14.noise(x14, y);
        n15 = sn15.noise(x15, y);
        x12 += increase;
        x13 += increase;
        x14 += increase;
        x15 += increase;
  
        // TODO: turn this into a function
        pos12 = (int)map(n12*100, -100, 100, MIN_DEGREE, MAX_DEGREE);
        pos13 = (int)map(n13*100, -100, 100, MIN_DEGREE, MAX_DEGREE);
        pos14 = (int)map(n14*100, -100, 100, MIN_DEGREE, MAX_DEGREE);
        pos15 = (int)map(n15*100, -100, 100, MIN_DEGREE, MAX_DEGREE);

      }
    }
  
    servo12.write(pos12);
    servo13.write(pos13);
    servo14.write(pos14);
    servo15.write(pos15);
  
  } else {
  

    // TODO: THIS NEEDS TO TO HAVE A 3D ARRAY added to this code
    // for the reading array
    // which will be used for averaging the sensor readings

    // MEGA Code block
    // this is for the maga, not the UNO
    // ===========
    for (uint8_t i = 0; i < OBJECT_NUM; i++) {
      ping_current_millis = millis();
  
      // Is it this sensor's time to ping?
      if (ping_current_millis >= pingTimer[i]) {

        int i_temp = (int)i;
        
        // Set next time this sensor will be pinged.
        pingTimer[i] += PING_INTERVAL * OBJECT_NUM;
        start_sensor();
        distance[i] = analogRead(sensorPins[i])/2;
        mappedDistance[i] = map(distance[i], 0, 350, 0, 250);

        total[i] = total[i] - readings[i][readIndex[i]];
        readings[i][readIndex[i]] = mappedDistance[i];
        total[i] = total[i] + readings[i][readIndex[i]];
        readIndex[i] = readIndex[i] + 1;
    
        // if we're at the end of the array...
        if (readIndex[i] >= numReadings) {
          // ...wrap around to the beginning:
          readIndex[i] = 0;
        }
    
        // calculate the average:
        average[i] = total[i] / numReadings;

        // save teh motor position and distance

        String tmp_string = (String)i_temp;
        String tmp_motor_pos = "pos";
        
        StoreData(i_temp, distance[i]);

        Serial.println(pos1);
        if(pos1 % 10 == 0){
          
          // this might need to be moved out again
          for(int j = 0 + (pixels_per_section*i); j<pixels_per_section + (pixels_per_section*i); j++){
            int pixl = j;
    
            if(animate_all_equal == true){
              Serial.println("===============");
//              strip.setPixelColor(pixl, strip.Color(mappedDistance[i], mappedDistance[i], mappedDistance[i]));
                strip.setPixelColor(pixl, strip.Color(average[i], average[i], average[i]));
            } else if(animate_hsb_fade == true){  
              
            } else if(hsb_saturation == true){
                 
//              int brightness = map(mappedDistance[i],0, 200, 0, 100);     // hue is a number between 0 and 360
              int brightness = map(average[i],0, 200, 0, 100);     // hue is a number between 0 and 360
              
              
              String returnVal = getRGB(0, 0, brightness);
    
              int commaIndex = returnVal.indexOf('/');
              int secondCommaIndex = returnVal.indexOf('/', commaIndex + 1);
    
              String firstValue = returnVal.substring(0, commaIndex);
              String secondValue = returnVal.substring(commaIndex + 1, secondCommaIndex);
              String thirdValue = returnVal.substring(secondCommaIndex + 1); // To the end of the string
    
              int r = firstValue.toInt();
              int g = secondValue.toInt();
              int b = thirdValue.toInt();
              
              strip.setPixelColor(pixl, strip.Color(r, g, b));
    
            } else if(animate_hsb == true) {
//              int hue = map(mappedDistance[i],0, 250, 0, 359);     // hue is a number between 0 and 360
              int hue = map(average[i],0, 250, 0, 359);     // hue is a number between 0 and 360
    
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
        
          strip.show(); // This sends the updated pixel color to the hardware.
    
        }
  
      /// ======== end
        
      }
      
    }
    
    if((current_millis - lastUpdate) > updateInterval)  // time to update
    {
      lastUpdate = millis();
  
      if(animate_noise == false){
        pos += increment;
        pos0 += increment;
        pos1 += increment;
        pos2 += increment;
        pos3 += increment;
        pos4 += increment;
        pos5 += increment;
        pos6 += increment;
        pos7 += increment;
        pos8 += increment;
        pos9 += increment;
        pos10 += increment;
        pos11 += increment;
        pos12 += increment;
        pos13 += increment;
        pos14 += increment;
        pos15 += increment;
        
        if ((pos >= MAX_DEGREE) || (pos <= MIN_DEGREE)) // end of sweep
        {
          // reverse direction
          increment = -increment;
        }
      } else{
        n0 = sn0.noise(x0, y);
        n1 = sn1.noise(x1, y);
        n2 = sn2.noise(x2, y);
        n3 = sn3.noise(x3, y);
        n4 = sn4.noise(x4, y);
        n5 = sn5.noise(x5, y);
        n6 = sn6.noise(x6, y);
        n7 = sn7.noise(x7, y);
        n8 = sn8.noise(x8, y);
        n9 = sn9.noise(x9, y);
        n10 = sn10.noise(x10, y);
        n11 = sn11.noise(x11, y);
        n12 = sn12.noise(x12, y);
        n13 = sn13.noise(x13, y);
        n14 = sn14.noise(x14, y);
        n15 = sn15.noise(x15, y);
        
        x0 += increase;
        x1 += increase;
        x2 += increase;
        x3 += increase;
        x4 += increase;
        x5 += increase;
        x6 += increase;
        x7 += increase;
        x8 += increase;
        x9 += increase;
        x10 += increase;
        x11 += increase;
        x12 += increase;
        x13 += increase;
        x14 += increase;
        x15 += increase;
  
        // TODO: turn this into a function
        pos0 = (int)map(n0*100, -100, 100, MIN_DEGREE, MAX_DEGREE);
        pos1 = (int)map(n1*100, -100, 100, MIN_DEGREE, MAX_DEGREE);
        pos2 = (int)map(n2*100, -100, 100, MIN_DEGREE, MAX_DEGREE);
        pos3 = (int)map(n3*100, -100, 100, MIN_DEGREE, MAX_DEGREE);
        pos4 = (int)map(n4*100, -100, 100, MIN_DEGREE, MAX_DEGREE);
        pos5 = (int)map(n5*100, -100, 100, MIN_DEGREE, MAX_DEGREE);
        pos6 = (int)map(n6*100, -100, 100, MIN_DEGREE, MAX_DEGREE);
        pos7 = (int)map(n7*100, -100, 100, MIN_DEGREE, MAX_DEGREE);
        pos8 = (int)map(n8*100, -100, 100, MIN_DEGREE, MAX_DEGREE);
        pos9 = (int)map(n9*100, -100, 100, MIN_DEGREE, MAX_DEGREE);
        pos10 = (int)map(n10*100, -100, 100, MIN_DEGREE, MAX_DEGREE);
        pos11 = (int)map(n11*100, -100, 100, MIN_DEGREE, MAX_DEGREE);
        pos12 = (int)map(n12*100, -100, 100, MIN_DEGREE, MAX_DEGREE);
        pos13 = (int)map(n13*100, -100, 100, MIN_DEGREE, MAX_DEGREE);
        pos14 = (int)map(n14*100, -100, 100, MIN_DEGREE, MAX_DEGREE);
        pos15 = (int)map(n15*100, -100, 100, MIN_DEGREE, MAX_DEGREE);  
      }
    }
 
    servo0.write(pos0);
    servo1.write(pos1);
    servo2.write(pos2);
    servo3.write(pos3);
    servo4.write(pos4);
    servo5.write(pos5);
    servo6.write(pos6);
    servo7.write(pos7);
    servo8.write(pos8);
    servo9.write(pos9);
    servo10.write(pos10);
    servo11.write(pos11);

    // Servo Note
    // We don't want to write these servos because they are not attached in the "arduino 0" board
    // This is being controlled by "arduino 1" 
  }
  
  delay(2);
  
}

void StoreData(int id, int currentDistance)
{
  // Just some debugging here
//  if(printStringTitle == true){
//    if(String(currentDistance).length() > 0){
//      Serial.print("currentDistance: ");
//      Serial.print((String)currentDistance);
//      Serial.print(" ||||");
//      Serial.println(" ");
//    }
//  }

  if(String(currentDistance).length() > 0){
    if(paused == false){
      // updated April 4...
      // added panel ID into the string. This will allow to isolate where
      // the data is coming from.... as in what panel, which will help to plot
      // it in the display
//          String tmp = String(panel);
//          tmp.concat("_");
//          tmp.concat(String(id));


      String tmp = String(id);
      tmp.concat(":");
      
      if(id == 0){
        tmp.concat(String(pos0));
      } else if(id == 1){
        tmp.concat(String(pos1));
      } else if(id == 2){
        tmp.concat(String(pos2));
      } else if(id == 3){
        tmp.concat(String(pos3));
      } else if(id == 4){
        tmp.concat(String(pos4));
      } else if(id == 5){
        tmp.concat(String(pos5));
      } else if(id == 6){
        tmp.concat(String(pos6));
      } else if(id == 7){
        tmp.concat(String(pos7));
      } else if(id == 8){
        tmp.concat(String(pos8));
      } else if(id == 9){
        tmp.concat(String(pos9));
      } else if(id == 10){
        tmp.concat(String(pos10));
      } else if(id == 11){
        tmp.concat(String(pos11));
      } else if(id == 12){
        tmp.concat(String(pos12));
      } else if(id == 13){
        tmp.concat(String(pos13));
      } else if(id == 14){
        tmp.concat(String(pos14));
      } else if(id == 15){
        tmp.concat(String(pos15));
      }

      tmp.concat(":");
      tmp.concat(String(currentDistance));
        
      sweepString[id].concat(tmp);
      sweepString[id].concat("/");
      
      pingTotalCount[id]++;

      if(pingTotalCount[id] >= pingRemainderValue){
        SendBatchData(id);
        pingTotalCount[id] = 1;
        
      }
    }
  }
}

void SendBatchData(int id) {
  // helping debug the serial buffer issue
  if(sendJSON == true){
      if(sweepString[id].endsWith("/")){
        int char_index = sweepString[id].lastIndexOf("/");
        sweepString[id].remove(char_index);
      }

      String addPannelToPublish = String(arduino);
      addPannelToPublish.concat("_");
      addPannelToPublish.concat(sweepString[id]);

      // REMEMBER: THIS IS THE SEND BATCH SERIAL PRINT!!
//          Serial.println(sweepString);
      Serial.println(addPannelToPublish);

      ResetPublishDataStatus();
  }
}

void ResetPublishDataStatus()
{

  for (uint8_t i = 0; i < OBJECT_NUM; i++) {
    int tmp = (int)i;
    sweepString[tmp ] = "";
  }
  
}

void massAttatch() {
  
  // servo.attach(SERVO_PIN, SERVO_MIN, SERVO_MAX);
  if(arduino == 0){
    servo0.attach(SERVO_PIN0);  // attaches the servo on pin 9 to the servo object
    servo1.attach(SERVO_PIN1);  // attaches the servo on pin 9 to the servo object
    servo2.attach(SERVO_PIN2);
    servo3.attach(SERVO_PIN3);
    servo4.attach(SERVO_PIN4);
    servo5.attach(SERVO_PIN5);
    servo6.attach(SERVO_PIN6);
    servo7.attach(SERVO_PIN7);
    servo8.attach(SERVO_PIN8);
    servo9.attach(SERVO_PIN9);
    servo10.attach(SERVO_PIN10);
    servo11.attach(SERVO_PIN11);
  } else if(arduino == 1){

    
    
    servo12.attach(SERVO_PIN12);
    servo13.attach(SERVO_PIN13);
    servo14.attach(SERVO_PIN14);
    servo15.attach(SERVO_PIN15);
  } 
}

void massDetatch() {
  if(arduino == 0){
    servo0.detach();
    servo1.detach();
    servo2.detach();
    servo3.detach();
    servo4.detach();
    servo5.detach();
    servo6.detach();
    servo7.detach();
    servo8.detach();
    servo9.detach();
    servo10.detach();
    servo11.detach();
  } else if(arduino == 1){
    servo12.detach();
    servo13.detach();
    servo14.detach();
    servo15.detach();
  } 
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
//
//// Returns the Red component of a 32-bit color
//uint8_t Red(uint32_t color)
//{
//    return (color >> 16) & 0xFF;
//}
//
//// Returns the Green component of a 32-bit color
//uint8_t Green(uint32_t color)
//{
//    return (color >> 8) & 0xFF;
//}
//
//// Returns the Blue component of a 32-bit color
//uint8_t Blue(uint32_t color)
//{
//    return color & 0xFF;
//}
