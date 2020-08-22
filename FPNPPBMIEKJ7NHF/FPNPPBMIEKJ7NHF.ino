#include <Adafruit_NeoPixel.h>

#define PIN 11                    // our control pin
#define MAX_BRIGHTNESS (255)

Adafruit_NeoPixel strip = Adafruit_NeoPixel(30, PIN, NEO_GRB + NEO_KHZ800);

uint8_t counter;
boolean countUp = true;

int holdcount = 3000;   // in miliseconds
int stepcount = 255;    // in loop() interations, must be less than 255

// color 1... white 
int r1 = 255;
int g1 = 255;
int b1 = 255;

// color 2...  yellow green
int r2 = 100;
int g2 = 210;
int b2 = 0;

// increment variables
float dr = 1.0;
float dg = 1.0;
float db = 1.0;

// current color variables
int cr = 0;
int cg = 0;
int cb = 0;

uint32_t color = strip.Color(cr, cg, cb);

void setup() {
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'

  // figure out the size of the steps
  dr = (float) (r2 - r1) / stepcount;
  dg = (float) (g2 - g1) / stepcount;
  db = (float) (b2 - b1) / stepcount;

  // set the current color to color 1 to get started
  cr = r1;
  cg = g1;
  cb = b1;
}

void complexFade() {
  
  // send current color to shield
  //lg3xCC.jumpToRGB(cr, cg, cb);
  color = strip.Color(cr, cg, cb);
  for (int ii = 0; ii < strip.numPixels(); ii++){
    strip.setPixelColor(ii, color);
    strip.show();
  }  
  // calculate the current rgb
  
  if (countUp) {
   counter++;
  } else {
   counter--;
  }

  cr = r1 + (counter * dr);   
  cg = g1 + (counter * dg);   
  cb = b1 + (counter * db);   

  // check our limits
  if (counter >= stepcount) {
    countUp = false; 
    delay(holdcount);
  } else if (counter == 0) {
    countUp = true;
    delay(holdcount);
  }
  
}


void loop() {
  //simpleFade(); 
  complexFade();
}
