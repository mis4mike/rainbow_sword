// Program to drive a dotstar LED array based on accelerometer readings

//Debugging memory library
#include <MemoryFree.h>

// Libraries for accelerometer
#include <Wire.h>               // I2C Library
#include <Adafruit_MMA8451.h>   // Accelerometer library
#include <Adafruit_Sensor.h>    // Adafruit sensor library
#define  DEBUG  0    

// Libraries for Dotstar
#include <Adafruit_DotStar.h>
// Because conditional #includes don't work w/Arduino sketches...
//#include <SPI.h>         // COMMENT OUT THIS LINE FOR GEMMA OR TRINKET
#include <avr/power.h> // ENABLE THIS LINE FOR GEMMA OR TRINKET

#define NUMPIXELS 22 // Number of LEDs in strip

// Here's how to control the LEDs from any two pins:
#define DATAPIN    3
#define CLOCKPIN   4

// Here are the colors:
#define DARK 0x000000
#define RED 0x00FF00
#define ORANGE 0x00FF91
#define YELLOW 0x00FFFB
#define GREEN 0x0000FF
#define BLUE 0xFF0000
#define VIOLET 0xFFAA00
#define WHITE 0xFFFFFF
#define DIM_WHITE 0x010101

// This requires about 200 mA for all the 'on' pixels + 1 mA per 'off' pixel.

Adafruit_DotStar strip = Adafruit_DotStar(
  NUMPIXELS, DATAPIN, CLOCKPIN, DOTSTAR_BRG);
// The last parameter is optional -- this is the color data order of the
// DotStar strip, which has changed over time in different production runs.
// Your code just uses R,G,B colors, the library then reassigns as needed.
// Default is DOTSTAR_BRG, so change this if you have an earlier strip.

// Hardware SPI is a little faster, but must be wired to specific pins
// (Arduino Uno = pin 11 for data, 13 for clock, other boards are different).
//Adafruit_DotStar strip = Adafruit_DotStar(NUMPIXELS, DOTSTAR_BRG);

Adafruit_MMA8451 mma = Adafruit_MMA8451();  // Create an accelerometer object

// Accerometer set up
//The maximum (positive) acceleration values read
//from the accelerometer for each axis:
const int MaxXValue = 4096;
const int MaxYValue = 4096;
const int MaxZValue = 4096;
 
//The minimum (negative) acceleration values read
//from the accelerometer for each axis:
const int MinXValue = -4096;
const int MinYValue = -4096;
const int MinZValue = -4096;

uint32_t color = 0x888888;      // 'On' color

void setup() {

#if defined(__AVR_ATtiny85__) && (F_CPU == 16000000L)
  clock_prescale_set(clock_div_1); // Enable 16 MHz on Trinket
#endif

  strip.begin(); // Initialize pins for output
  strip.show();  // Turn all LEDs off ASAP

  setStripColor(ORANGE);

  //rainbowsForever();

  if (! mma.begin()) {  // If the accelerometer cannot be found, flash LED
    while (1) {
      setStripColor(RED);
      delay(350);
      setStripColor(0);
      delay(350);
    }
  } else {
    setStripColor(GREEN);
    delay(350);
    setStripColor(0);
  }

  setStripColor(GREEN);

  mma.setRange(MMA8451_RANGE_2_G);  // 2G Mode is best for hand gestures
  mma.read();                       // get an initial read 
  
}

void loop() {
 
  //listen for accelerometer input
  mma.read();  //   // Read the 'raw' data in 14-bit counts
#if DEBUG
  Serial.print("X:\t"); Serial.print(mma.x); 
  Serial.print("\tY:\t"); Serial.print(mma.y); 
  Serial.print("\tZ:\t"); Serial.println(mma.z); 
#endif

  //setThisManyLEDs(int(NUMPIXELS * getTotalAcceleration(mma.x, mma.y, mma.z)));
  setStripColor(pickRainbowColor(getTotalAcceleration(mma.x, mma.y, mma.z)));


  //setThisManyLEDs(int(NUMPIXELS * (freeMemory() / 2048.0)));

  
  delay(100);
}

float getTotalAcceleration(int16_t x, int16_t y, int16_t z) {
  float acceleration = float(abs(x) + abs(y) + abs(z)) / float(3 * MaxXValue);

  return acceleration - 0.5; //subtract .5 to account for gravity maybe?
}

void setStripColor (uint32_t setColor) {
    for(int i = 0; i < 30; i++) {
      strip.setPixelColor(i, setColor);
    }  
    strip.show();
}

void setThisManyLEDs(int numLEDs) {
  if(numLEDs >= NUMPIXELS) {
    numLEDs = 5;
  }
  
  setStripColor(DARK);
  for(int i = 0; i < numLEDs; i++) {
    strip.setPixelColor(i, color);
  }  
  strip.show();
}

uint32_t pickRainbowColor (float num) {
  if(num < 0.25) {
    return DIM_WHITE; 
  }
  if(num < 0.375) {
    return RED;
  }
  if(num < 0.50) {
    return ORANGE;
  }
  if(num < 0.625) {
    return YELLOW;
  }
  if(num < 0.75) {
    return GREEN;
  }
  if(num < 0.875) {
    return BLUE;
  }
  return VIOLET;  
}
void rainbowCycle () {
  setStripColor(RED);
  delay(200);
  setStripColor(ORANGE);
  delay(200);
  setStripColor(YELLOW);
  delay(200);
  setStripColor(GREEN);
  delay(200);
  setStripColor(BLUE);
  delay(200);
  setStripColor(VIOLET);
  delay(200);
  
}

void rainbowsForever() {
  while(1) {
    rainbowCycle();
  }
}

