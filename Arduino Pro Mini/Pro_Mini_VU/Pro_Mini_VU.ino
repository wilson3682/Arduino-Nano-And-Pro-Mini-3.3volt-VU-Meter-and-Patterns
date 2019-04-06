#include <Adafruit_NeoPixel.h>
#include <FastLED.h>
#define PIN 6
#define N_PIXELS  50
#define BG 0
#define COLOR_ORDER GRB  // Try mixing up the letters (RGB, GBR, BRG, etc) for a whole new world of color combinations
#define LED_TYPE WS2812B
#define MIC_PIN   A0  // Microphone is attached to this analog pin
#define DC_OFFSET  0  // DC offset in mic signal - if unusure, leave 0
#define NOISE     10  // Noise/hum/interference in mic signal
#define SAMPLES   60  // Length of buffer for dynamic level adjustment
#define TOP       (N_PIXELS) // Allow dot to go slightly off scale
#define PEAK_FALL 40  // Rate of peak falling dot
#define N_PIXELS_HALF (N_PIXELS/2)
#define GRAVITY           -9.81              // Downward (negative) acceleration of gravity in m/s^2
#define h0                1                  // Starting height, in meters, of the ball (strip length)
#define NUM_BALLS         3                  // Number of bouncing balls you want (recommend < 7, but 20 is fun in its own way)
#define SPEED .20       // Amount to increment RGB color by each cycle
#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))
#define TOP2      (N_PIXELS + 1) // Allow dot to go slightly off scale
#define LAST_PIXEL_OFFSET N_PIXELS-1
#define PEAK_FALL_MILLIS 10  // Rate of peak falling dot
#define brightControl A2  //A2 Arduino Pro Mini
int val;

//fire virables
#define COOLING  80
#define SPARKING 120
bool gReverseDirection = false;

//rainbow rotate variables
int ihue = 0;                //-HUE (0-255)
int thisdelay = 20;

//vu2 variables
#define SAMPLE_WINDOW   10  // Sample window for average level
#define PEAK_HANG 24 //Time of pause before peak dot falls
#define PEAK_FALL2 8 //Rate of falling peak dot
#define INPUT_FLOOR 30 //Lower range of analogRead input
#define INPUT_CEILING 300 //Max range of analogRead input, the lower the value the more sensitive (1023 = max)300 (150)
unsigned int sample;
byte peak2 = 16;      // Peak level of column; used for falling dots
byte dotHangCount = 0; //Frame counter for holding peak dot

//config for balls
float h[NUM_BALLS] ;                         // An array of heights
float vImpact0 = sqrt( -2 * GRAVITY * h0 );  // Impact velocity of the ball when it hits the ground if "dropped" from the top of the strip
float vImpact[NUM_BALLS] ;                   // As time goes on the impact velocity will change, so make an array to store those values
float tCycle[NUM_BALLS] ;                    // The time since the last time the ball struck the ground
int   pos[NUM_BALLS] ;                       // The integer position of the dot on the strip (LED index)
long  tLast[NUM_BALLS] ;                     // The clock time of the last ground strike
float COR[NUM_BALLS] ;                       // Coefficient of Restitution (bounce damping)

float
greenOffset = 30,
blueOffset = 150;

byte
peak      = 0,      // Used for falling dot
dotCount  = 0,      // Frame counter for delaying dot-falling speed
volCount  = 0;      // Frame counter for storing past volume data
int
vol[SAMPLES],       // Collection of prior volume samples
    lvl       = 10,      // Current "dampened" audio level
    minLvlAvg = 0,      // For dynamic adjustment of graph low & high
    maxLvlAvg = 512;
long lastTime = 0;
int          myhue =   0;

Adafruit_NeoPixel strip = Adafruit_NeoPixel(N_PIXELS, PIN, NEO_GRB + NEO_KHZ800);

// FOR SYLON ETC
uint8_t thisbeat =  23;
uint8_t thatbeat =  28;
uint8_t thisfade =   2;                                     // How quickly does it fade? Lower = slower fade rate.
uint8_t thissat = 255;                                     // The saturation, where 255 = brilliant colours.
uint8_t thisbri = 255;

//FOR JUGGLE
uint8_t numdots = 4;                                          // Number of dots in use.
uint8_t faderate = 2;                                         // How long should the trails be. Very low value = longer trails.
uint8_t hueinc = 16;                                          // Incremental change in hue between each dot.
uint8_t thishue = 0;                                          // Starting hue.
uint8_t curhue = 0;
uint8_t thisbright = 255;                                     // How bright should the LED/display be.
uint8_t basebeat = 5;
uint8_t max_bright = 255;

// Twinkle
float redStates[N_PIXELS];
float blueStates[N_PIXELS];
float greenStates[N_PIXELS];
float Fade = 0.96;

// Vu meter 4
const uint32_t Red = strip.Color(255, 0, 0);
const uint32_t Yellow = strip.Color(255, 255, 0);
const uint32_t Green = strip.Color(0, 255, 0);
const uint32_t Blue = strip.Color(0, 0, 255);
const uint32_t White = strip.Color(255, 255, 255);
const uint32_t Dark = strip.Color(0, 0, 0);

// constants used here to set pin numbers:
const int buttonPin = 4;     // the number of the pushbutton pin on Arduino Pro Mini 4
int buttonPushCounter = 0;   // counter for the number of button presses
int buttonState = 0;         // current state of the button
int lastButtonState = 0;

//Ripple variables
int color;
int center = 0;
int step = -1;
int maxSteps = 16;
float fadeRate = 0.80;
int diff;

//background color
uint32_t currentBg = random(256);
uint32_t nextBg = currentBg;

CRGB leds[N_PIXELS];
CRGBPalette16 currentPalette(OceanColors_p);
CRGBPalette16 targetPalette(CloudColors_p);

void setup() {
  delay( 2000 ); // power-up safety delay
  Serial.begin(115200);
  FastLED.addLeds<WS2812B, PIN, COLOR_ORDER>(leds, N_PIXELS).setCorrection( TypicalLEDStrip );
  pinMode(buttonPin, INPUT);
  digitalWrite(buttonPin, HIGH);
  
  //analogReference(EXTERNAL);           //No needed on Arduino 3.3 volts
  memset(vol, 0, sizeof(vol));
  LEDS.addLeds<LED_TYPE, PIN, COLOR_ORDER>(leds, N_PIXELS);
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'

  for (int i = 0 ; i < NUM_BALLS ; i++) {    // Initialize variables
    tLast[i] = millis();
    h[i] = h0;
    pos[i] = 0;                              // Balls start on the ground
    vImpact[i] = vImpact0;                   // And "pop" up at vImpact0
    tCycle[i] = 0;
    COR[i] = 0.90 - float(i) / pow(NUM_BALLS, 2);
  }
}

void loop() {
  val = analogRead(brightControl);            // reads the value of the potentiometer (value between 0 and 1023)
  val = map(val, 0, 1023, 0, 255);
  strip.setBrightness(val);
  LEDS.setBrightness(val);

  uint8_t  i;
  uint16_t minLvl, maxLvl;
  int      n, height;

  // read the pushbutton input pin:
  buttonState = digitalRead(buttonPin);
  // compare the buttonState to its previous state
  if (buttonState != lastButtonState) {
    // if the state has changed, increment the counter
    if (buttonState == HIGH) {
      // if the current state is HIGH then the button
      // went from off to on:
      buttonPushCounter++;
      Serial.println("on");
      Serial.print("number of button pushes:  ");
      Serial.println(buttonPushCounter);
      if (buttonPushCounter == 22) {
        buttonPushCounter = 1;
      }
    }
    else {
      // if the current state is LOW then the button
      // wend from on to off:
      Serial.println("off");
    }
  }
  // save the current state as the last state,
  //for next time through the loop
  lastButtonState = buttonState;

  switch (buttonPushCounter) {
    case 1:
      buttonPushCounter == 1; {
        All2(); // NORMAL
        break;
      }
    case 2:
      buttonPushCounter == 2; {
        All();
        break;
      }
    case 3:
      buttonPushCounter == 3; {
        vu(); // Red
        break;
      }
    case 4:
      buttonPushCounter == 4; {
        vu1(); // Red
        break;
      }
    case 5:
      buttonPushCounter == 5; {
        vu2(); // Red
        break;
      }
    case 6:
      buttonPushCounter == 6; {
        vu3(); //
        break;
      }
    case 7:
      buttonPushCounter == 7; {
        vu4(); //
        break;
      }
    case 8:
      buttonPushCounter == 8; {
        vu5(); //
        break;
      }
    case 9:
      buttonPushCounter == 9; {
        vu6(); //
        break;
      }
    case 10:
      buttonPushCounter == 10; {
        rainbow_rotate();
        break;
      }
    case 11:
      buttonPushCounter == 11; {
        blur();
        break;
      }
    case 12:
      buttonPushCounter == 12; {
        ripple();
        break;
      }
    case 13:
      buttonPushCounter == 13; {
        ripple2();
        break;
      }
    case 14:
      buttonPushCounter == 14; {
        Twinkle();
        break;
      }
    case 15:
      buttonPushCounter == 15; {
        pattern2();
        break;
      }
    case 16:
      buttonPushCounter == 16; {
        pattern3();
        break;
      }
    case 17:
      buttonPushCounter == 17; {
        Balls(); //
        break;
      }
    case 18:
      buttonPushCounter == 18; {
        fire(); //
        break;
      }
    case 19:
      buttonPushCounter == 19; {
        juggle2(); //
        break;
      }
    case 20:
      buttonPushCounter == 20; {
        fireblue(); //
        break;
      }
    case 21:
      buttonPushCounter == 21; {
        colorWipe(strip.Color(0, 0, 0), 10); // A Black
        break;
      }
  }
}

void colorWipe(uint32_t c, uint8_t wait) {
  for (uint16_t i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
    if (digitalRead(buttonPin) != lastButtonState)  // <------------- add this
      return;         // <------------ and this
    delay(wait);
  }
}

void vu() {
  uint8_t  i;
  uint16_t minLvl, maxLvl;
  int      n, height;
  n   = analogRead(MIC_PIN);                        // Raw reading from mic
  n   = abs(n - 512 - DC_OFFSET); // Center on zero
  n   = (n <= NOISE) ? 0 : (n - NOISE);             // Remove noise/hum
  lvl = ((lvl * 7) + n) >> 3;    // "Dampened" reading (else looks twitchy)

  // Calculate bar height based on dynamic min/max levels (fixed point):
  height = TOP * (lvl - minLvlAvg) / (long)(maxLvlAvg - minLvlAvg);

  if (height < 0L)       height = 0;     // Clip output
  else if (height > TOP) height = TOP;
  if (height > peak)     peak   = height; // Keep 'peak' dot at top

  // Color pixels based on rainbow gradient
  for (i = 0; i < N_PIXELS; i++) {
    if (i >= height)               strip.setPixelColor(i,   0,   0, 0);
    else strip.setPixelColor(i, Wheel(map(i, 0, strip.numPixels() - 1, 30, 150)));
  }
  
  // Draw peak dot
  if (peak > 0 && peak <= N_PIXELS - 1) strip.setPixelColor(peak, Wheel(map(peak, 0, strip.numPixels() - 1, 30, 150)));

  strip.show(); // Update strip

  // Every few frames, make the peak pixel drop by 1:
  if (++dotCount >= PEAK_FALL) { //fall rate

    if (peak > 0) peak--;
    dotCount = 0;
  }
  
  vol[volCount] = n;                      // Save sample for dynamic leveling
  if (++volCount >= SAMPLES) volCount = 0; // Advance/rollover sample counter

  // Get volume range of prior frames
  minLvl = maxLvl = vol[0];
  for (i = 1; i < SAMPLES; i++) {
    if (vol[i] < minLvl)      minLvl = vol[i];
    else if (vol[i] > maxLvl) maxLvl = vol[i];
  }
  // minLvl and maxLvl indicate the volume range over prior frames, used
  // for vertically scaling the output graph (so it looks interesting
  // regardless of volume level).  If they're too close together though
  // (e.g. at very low volume levels) the graph becomes super coarse
  // and 'jumpy'...so keep some minimum distance between them (this
  // also lets the graph go to zero when no sound is playing):
  if ((maxLvl - minLvl) < TOP) maxLvl = minLvl + TOP;
  minLvlAvg = (minLvlAvg * 63 + minLvl) >> 6; // Dampen min/max levels
  maxLvlAvg = (maxLvlAvg * 63 + maxLvl) >> 6; // (fake rolling average)
}

// Input a value 0 to 255 to get a color value.
// The colors are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  if (WheelPos < 85) {
    return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if (WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
    WheelPos -= 170;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}

void vu1() {
  uint8_t  i;
  uint16_t minLvl, maxLvl;
  int      n, height;
  n   = analogRead(MIC_PIN);                        // Raw reading from mic
  n   = abs(n - 512 - DC_OFFSET); // Center on zero
  n   = (n <= NOISE) ? 0 : (n - NOISE);             // Remove noise/hum
  lvl = ((lvl * 7) + n) >> 3;    // "Dampened" reading (else looks twitchy)

  // Calculate bar height based on dynamic min/max levels (fixed point):
  height = TOP * (lvl - minLvlAvg) / (long)(maxLvlAvg - minLvlAvg);

  if (height < 0L)       height = 0;     // Clip output
  else if (height > TOP) height = TOP;
  if (height > peak)     peak   = height; // Keep 'peak' dot at top


  // Color pixels based on rainbow gradient
  for (i = 0; i < N_PIXELS_HALF; i++) {
    if (i >= height) {
      strip.setPixelColor(N_PIXELS_HALF - i - 1,   0,   0, 0);
      strip.setPixelColor(N_PIXELS_HALF + i,   0,   0, 0);
    }
    else {
      uint32_t color = Wheel(map(i, 0, N_PIXELS_HALF - 1, 30, 150));
      strip.setPixelColor(N_PIXELS_HALF - i - 1, color);
      strip.setPixelColor(N_PIXELS_HALF + i, color);
    }

  }

  // Draw peak dot
  if (peak > 0 && peak <= N_PIXELS_HALF - 1) {
    uint32_t color = Wheel(map(peak, 0, N_PIXELS_HALF - 1, 30, 150));
    strip.setPixelColor(N_PIXELS_HALF - peak - 1, color);
    strip.setPixelColor(N_PIXELS_HALF + peak, color);
  }

  strip.show(); // Update strip

  // Every few frames, make the peak pixel drop by 1:

  if (++dotCount >= PEAK_FALL) { //fall rate

    if (peak > 0) peak--;
    dotCount = 0;
  }

  vol[volCount] = n;                      // Save sample for dynamic leveling
  if (++volCount >= SAMPLES) volCount = 0; // Advance/rollover sample counter

  // Get volume range of prior frames
  minLvl = maxLvl = vol[0];
  for (i = 1; i < SAMPLES; i++) {
    if (vol[i] < minLvl)      minLvl = vol[i];
    else if (vol[i] > maxLvl) maxLvl = vol[i];
  }
  // minLvl and maxLvl indicate the volume range over prior frames, used
  // for vertically scaling the output graph (so it looks interesting
  // regardless of volume level).  If they're too close together though
  // (e.g. at very low volume levels) the graph becomes super coarse
  // and 'jumpy'...so keep some minimum distance between them (this
  // also lets the graph go to zero when no sound is playing):
  if ((maxLvl - minLvl) < TOP) maxLvl = minLvl + TOP;
  minLvlAvg = (minLvlAvg * 63 + minLvl) >> 6; // Dampen min/max levels
  maxLvlAvg = (maxLvlAvg * 63 + maxLvl) >> 6; // (fake rolling average)
}

float fscale( float originalMin, float originalMax, float newBegin, float newEnd, float inputValue, float curve) {
  float OriginalRange = 0;
  float NewRange = 0;
  float zeroRefCurVal = 0;
  float normalizedCurVal = 0;
  float rangedValue = 0;
  boolean invFlag = 0;
  // condition curve parameter
  // limit range
  if (curve > 10) curve = 10;
  if (curve < -10) curve = -10;
  curve = (curve * -.1) ; // - invert and scale - this seems more intuitive - postive numbers give more weight to high end on output
  curve = pow(10, curve); // convert linear scale into lograthimic exponent for other pow function
  // Check for out of range inputValues
  if (inputValue < originalMin) {
    inputValue = originalMin;
  }
  if (inputValue > originalMax) {
    inputValue = originalMax;
  }
  // Zero Refference the values
  OriginalRange = originalMax - originalMin;
  if (newEnd > newBegin) {
    NewRange = newEnd - newBegin;
  }
  else {
    NewRange = newBegin - newEnd;
    invFlag = 1;
  }
  zeroRefCurVal = inputValue - originalMin;
  normalizedCurVal  =  zeroRefCurVal / OriginalRange;   // normalize to 0 - 1 float
  // Check for originalMin > originalMax  - the math for all other cases i.e. negative numbers seems to work out fine
  if (originalMin > originalMax ) {
    return 0;
  }
  if (invFlag == 0) {
    rangedValue =  (pow(normalizedCurVal, curve) * NewRange) + newBegin;
  }
  else {  // invert the ranges
    rangedValue =  newBegin - (pow(normalizedCurVal, curve) * NewRange);
  }
  return rangedValue;
}

void vu2() {
  unsigned long startMillis = millis(); // Start of sample window
  float peakToPeak = 0;   // peak-to-peak level
  unsigned int signalMax = 0;
  unsigned int signalMin = 1023;
  unsigned int c, y;
  while (millis() - startMillis < SAMPLE_WINDOW) {
    sample = analogRead(MIC_PIN);
    if (sample < 1024) {
      if (sample > signalMax) {
        signalMax = sample;
      }
      else if (sample < signalMin) {
        signalMin = sample;
      }
    }
  }
  peakToPeak = signalMax - signalMin;
  // Serial.println(peakToPeak);

  for (int i = 0; i <= N_PIXELS_HALF - 1; i++) {
    uint32_t color = Wheel(map(i, 0, N_PIXELS_HALF - 1, 30, 150));
    strip.setPixelColor(N_PIXELS - i, color);
    strip.setPixelColor(0 + i, color);
  }

  c = fscale(INPUT_FLOOR, INPUT_CEILING, N_PIXELS_HALF, 0, peakToPeak, 2);

  if (c < peak) {
    peak = c;        // Keep dot on top
    dotHangCount = 0;    // make the dot hang before falling
  }
  if (c <= strip.numPixels()) { // Fill partial column with off pixels
    drawLine(N_PIXELS_HALF, N_PIXELS_HALF - c, strip.Color(0, 0, 0));
    drawLine(N_PIXELS_HALF, N_PIXELS_HALF + c, strip.Color(0, 0, 0));
  }

  y = N_PIXELS_HALF - peak;
  uint32_t color1 = Wheel(map(y, 0, N_PIXELS_HALF - 1, 30, 150));
  strip.setPixelColor(y - 1, color1);
  //strip.setPixelColor(y-1,Wheel(map(y,0,N_PIXELS_HALF-1,30,150)));

  y = N_PIXELS_HALF + peak;
  strip.setPixelColor(y, color1);
  //strip.setPixelColor(y+1,Wheel(map(y,0,N_PIXELS_HALF+1,30,150)));

  //strip.setBrightness(max_bright);
  strip.show();

  // Frame based peak dot animation
  if (dotHangCount > PEAK_HANG) { //Peak pause length
    if (++dotCount >= PEAK_FALL2) { //Fall rate
      peak++;
      dotCount = 0;
    }
  }
  else {
    dotHangCount++;
  }
}

//Used to draw a line between two points of a given color
void drawLine(uint8_t from, uint8_t to, uint32_t c) {
  uint8_t fromTemp;
  if (from > to) {
    fromTemp = from;
    from = to;
    to = fromTemp;
  }
  for (int i = from; i <= to; i++) {
    strip.setPixelColor(i, c);
  }
}

void vu3() {
  uint8_t i;
  uint16_t minLvl, maxLvl;
  int n, height;
  n = analogRead(MIC_PIN);             // Raw reading from mic
  n = abs(n - 512 - DC_OFFSET);        // Center on zero
  n = (n <= NOISE) ? 0 : (n - NOISE);  // Remove noise/hum
  lvl = ((lvl * 7) + n) >> 3;    // "Dampened" reading (else looks twitchy)

  // Calculate bar height based on dynamic min/max levels (fixed point):
  height = TOP * (lvl - minLvlAvg) / (long)(maxLvlAvg - minLvlAvg);

  if (height < 0L)       height = 0;      // Clip output
  else if (height > TOP) height = TOP;
  if (height > peak)     peak   = height; // Keep 'peak' dot at top

  greenOffset += SPEED;
  blueOffset += SPEED;
  if (greenOffset >= 255) greenOffset = 0;
  if (blueOffset >= 255) blueOffset = 0;

  // Color pixels based on rainbow gradient
  for (i = 0; i < N_PIXELS; i++) {
    if (i >= height) {
      strip.setPixelColor(i, 0, 0, 0);
    } else {
      strip.setPixelColor(i, Wheel(map(i, 0, strip.numPixels() - 1, (int)greenOffset, (int)blueOffset)));
    }
  }
  // Draw peak dot
  if (peak > 0 && peak <= N_PIXELS - 1) strip.setPixelColor(peak, Wheel(map(peak, 0, strip.numPixels() - 1, 30, 150)));

  strip.show(); // Update strip

  // Every few frames, make the peak pixel drop by 1:

  if (++dotCount >= PEAK_FALL) { //fall rate

    if (peak > 0) peak--;
    dotCount = 0;
  }
  strip.show();  // Update strip
  
  vol[volCount] = n;
  if (++volCount >= SAMPLES) {
    volCount = 0;
  }

  // Get volume range of prior frames
  minLvl = maxLvl = vol[0];
  for (i = 1; i < SAMPLES; i++) {
    if (vol[i] < minLvl) {
      minLvl = vol[i];
    } else if (vol[i] > maxLvl) {
      maxLvl = vol[i];
    }
  }

  // minLvl and maxLvl indicate the volume range over prior frames, used
  // for vertically scaling the output graph (so it looks interesting
  // regardless of volume level).  If they're too close together though
  // (e.g. at very low volume levels) the graph becomes super coarse
  // and 'jumpy'...so keep some minimum distance between them (this
  // also lets the graph go to zero when no sound is playing):
  if ((maxLvl - minLvl) < TOP) {
    maxLvl = minLvl + TOP;
  }
  minLvlAvg = (minLvlAvg * 63 + minLvl) >> 6; // Dampen min/max levels
  maxLvlAvg = (maxLvlAvg * 63 + maxLvl) >> 6; // (fake rolling average)
}

void vu4() {
  uint8_t  i;
  uint16_t minLvl, maxLvl;
  int      n, height;
  n   = analogRead(MIC_PIN);                        // Raw reading from mic
  n   = abs(n - 512 - DC_OFFSET); // Center on zero
  n   = (n <= NOISE) ? 0 : (n - NOISE);             // Remove noise/hum
  lvl = ((lvl * 7) + n) >> 3;    // "Dampened" reading (else looks twitchy)

  // Calculate bar height based on dynamic min/max levels (fixed point):
  height = TOP * (lvl - minLvlAvg) / (long)(maxLvlAvg - minLvlAvg);

  if (height < 0L)       height = 0;     // Clip output
  else if (height > TOP) height = TOP;
  if (height > peak)     peak   = height; // Keep 'peak' dot at top
  greenOffset += SPEED;
  blueOffset += SPEED;
  if (greenOffset >= 255) greenOffset = 0;
  if (blueOffset >= 255) blueOffset = 0;

  // Color pixels based on rainbow gradient
  for (i = 0; i < N_PIXELS_HALF; i++) {
    if (i >= height) {
      strip.setPixelColor(N_PIXELS_HALF - i - 1,   0,   0, 0);
      strip.setPixelColor(N_PIXELS_HALF + i,   0,   0, 0);
    }
    else {
      uint32_t color = Wheel(map(i, 0, N_PIXELS_HALF - 1, (int)greenOffset, (int)blueOffset));
      strip.setPixelColor(N_PIXELS_HALF - i - 1, color);
      strip.setPixelColor(N_PIXELS_HALF + i, color);
    }
  }

  // Draw peak dot
  if (peak > 0 && peak <= N_PIXELS_HALF - 1) {
    uint32_t color = Wheel(map(peak, 0, N_PIXELS_HALF - 1, 30, 150));
    strip.setPixelColor(N_PIXELS_HALF - peak - 1, color);
    strip.setPixelColor(N_PIXELS_HALF + peak, color);
  }

  strip.show(); // Update strip

  // Every few frames, make the peak pixel drop by 1:

  if (++dotCount >= PEAK_FALL) { //fall rate

    if (peak > 0) peak--;
    dotCount = 0;
  }
  
  vol[volCount] = n;                      // Save sample for dynamic leveling
  if (++volCount >= SAMPLES) volCount = 0; // Advance/rollover sample counter

  // Get volume range of prior frames
  minLvl = maxLvl = vol[0];
  for (i = 1; i < SAMPLES; i++) {
    if (vol[i] < minLvl)      minLvl = vol[i];
    else if (vol[i] > maxLvl) maxLvl = vol[i];
  }
  // minLvl and maxLvl indicate the volume range over prior frames, used
  // for vertically scaling the output graph (so it looks interesting
  // regardless of volume level).  If they're too close together though
  // (e.g. at very low volume levels) the graph becomes super coarse
  // and 'jumpy'...so keep some minimum distance between them (this
  // also lets the graph go to zero when no sound is playing):
  if ((maxLvl - minLvl) < TOP) maxLvl = minLvl + TOP;
  minLvlAvg = (minLvlAvg * 63 + minLvl) >> 6; // Dampen min/max levels
  maxLvlAvg = (maxLvlAvg * 63 + maxLvl) >> 6; // (fake rolling average)
}

void vu5()
{
  uint8_t  i;
  uint16_t minLvl, maxLvl;
  int      n, height;
  n   = analogRead(MIC_PIN);                        // Raw reading from mic
  n   = abs(n - 512 - DC_OFFSET); // Center on zero
  n   = (n <= NOISE) ? 0 : (n - NOISE);             // Remove noise/hum
  lvl = ((lvl * 7) + n) >> 3;    // "Dampened" reading (else looks twitchy)

  // Calculate bar height based on dynamic min/max levels (fixed point):
  height = TOP2 * (lvl - minLvlAvg) / (long)(maxLvlAvg - minLvlAvg);

  if (height < 0L)       height = 0;     // Clip output
  else if (height > TOP2) height = TOP2;
  if (height > peak)     peak   = height; // Keep 'peak' dot at top

#ifdef CENTERED
  // Color pixels based on rainbow gradient
  for (i = 0; i < (N_PIXELS / 2); i++) {
    if (((N_PIXELS / 2) + i) >= height)
    {
      strip.setPixelColor(((N_PIXELS / 2) + i),   0,   0, 0);
      strip.setPixelColor(((N_PIXELS / 2) - i),   0,   0, 0);
    }
    else
    {
      strip.setPixelColor(((N_PIXELS / 2) + i), Wheel(map(((N_PIXELS / 2) + i), 0, strip.numPixels() - 1, 30, 150)));
      strip.setPixelColor(((N_PIXELS / 2) - i), Wheel(map(((N_PIXELS / 2) - i), 0, strip.numPixels() - 1, 30, 150)));
    }
  }
  // Draw peak dot
  if (peak > 0 && peak <= LAST_PIXEL_OFFSET)
  {
    strip.setPixelColor(((N_PIXELS / 2) + peak), 255, 255, 255); // (peak,Wheel(map(peak,0,strip.numPixels()-1,30,150)));
    strip.setPixelColor(((N_PIXELS / 2) - peak), 255, 255, 255); // (peak,Wheel(map(peak,0,strip.numPixels()-1,30,150)));
  }
#else
  // Color pixels based on rainbow gradient
  for (i = 0; i < N_PIXELS; i++)
  {
    if (i >= height)
    {
      strip.setPixelColor(i,   0,   0, 0);
    }
    else
    {
      strip.setPixelColor(i, Wheel(map(i, 0, strip.numPixels() - 1, 30, 150)));
    }
  }
  // Draw peak dot
  if (peak > 0 && peak <= LAST_PIXEL_OFFSET)
  {
    strip.setPixelColor(peak, 255, 255, 255); // (peak,Wheel(map(peak,0,strip.numPixels()-1,30,150)));
  }
#endif

  // Every few frames, make the peak pixel drop by 1:

  if (millis() - lastTime >= PEAK_FALL_MILLIS)
  {
    lastTime = millis();

    //strip.setBrightness(max_bright);
    strip.show(); // Update strip

    //fall rate
    if (peak > 0) peak--;
  }

  vol[volCount] = n;                      // Save sample for dynamic leveling
  if (++volCount >= SAMPLES) volCount = 0; // Advance/rollover sample counter

  // Get volume range of prior frames
  minLvl = maxLvl = vol[0];
  for (i = 1; i < SAMPLES; i++)
  {
    if (vol[i] < minLvl)      minLvl = vol[i];
    else if (vol[i] > maxLvl) maxLvl = vol[i];
  }
  // minLvl and maxLvl indicate the volume range over prior frames, used
  // for vertically scaling the output graph (so it looks interesting
  // regardless of volume level).  If they're too close together though
  // (e.g. at very low volume levels) the graph becomes super coarse
  // and 'jumpy'...so keep some minimum distance between them (this
  // also lets the graph go to zero when no sound is playing):
  if ((maxLvl - minLvl) < TOP2) maxLvl = minLvl + TOP2;
  minLvlAvg = (minLvlAvg * 63 + minLvl) >> 6; // Dampen min/max levels
  maxLvlAvg = (maxLvlAvg * 63 + maxLvl) >> 6; // (fake rolling average)
}

void vu6()
{
  uint8_t  i;
  uint16_t minLvl, maxLvl;
  int      n, height;
  n   = analogRead(MIC_PIN);                        // Raw reading from mic
  n   = abs(n - 512 - DC_OFFSET); // Center on zero
  n   = (n <= NOISE) ? 0 : (n - NOISE);             // Remove noise/hum
  lvl = ((lvl * 7) + n) >> 3;    // "Dampened" reading (else looks twitchy)

  // Calculate bar height based on dynamic min/max levels (fixed point):
  height = TOP2 * (lvl - minLvlAvg) / (long)(maxLvlAvg - minLvlAvg);

  if (height < 0L)       height = 0;     // Clip output
  else if (height > TOP2) height = TOP2;
  if (height > peak)     peak   = height; // Keep 'peak' dot at top

#ifdef CENTERED
  // Draw peak dot
  if (peak > 0 && peak <= LAST_PIXEL_OFFSET)
  {
    strip.setPixelColor(((N_PIXELS / 2) + peak), 255, 255, 255); // (peak,Wheel(map(peak,0,strip.numPixels()-1,30,150)));
    strip.setPixelColor(((N_PIXELS / 2) - peak), 255, 255, 255); // (peak,Wheel(map(peak,0,strip.numPixels()-1,30,150)));
  }
#else
  // Color pixels based on rainbow gradient
  for (i = 0; i < N_PIXELS; i++)
  {
    if (i >= height)
    {
      strip.setPixelColor(i,   0,   0, 0);
    }
    else
    {
    }
  }

  // Draw peak dot
  if (peak > 0 && peak <= LAST_PIXEL_OFFSET)
  {
    strip.setPixelColor(peak, 0, 0, 255); // (peak,Wheel(map(peak,0,strip.numPixels()-1,30,150)));
  }
#endif

  // Every few frames, make the peak pixel drop by 1:
  if (millis() - lastTime >= PEAK_FALL_MILLIS)
  {
    lastTime = millis();

    //strip.setBrightness(max_bright);
    strip.show(); // Update strip

    //fall rate
    if (peak > 0) peak--;
  }

  vol[volCount] = n;                      // Save sample for dynamic leveling
  if (++volCount >= SAMPLES) volCount = 0; // Advance/rollover sample counter

  // Get volume range of prior frames
  minLvl = maxLvl = vol[0];
  for (i = 1; i < SAMPLES; i++)
  {
    if (vol[i] < minLvl)      minLvl = vol[i];
    else if (vol[i] > maxLvl) maxLvl = vol[i];
  }
  // minLvl and maxLvl indicate the volume range over prior frames, used
  // for vertically scaling the output graph (so it looks interesting
  // regardless of volume level).  If they're too close together though
  // (e.g. at very low volume levels) the graph becomes super coarse
  // and 'jumpy'...so keep some minimum distance between them (this
  // also lets the graph go to zero when no sound is playing):
  if ((maxLvl - minLvl) < TOP2) maxLvl = minLvl + TOP2;
  minLvlAvg = (minLvlAvg * 63 + minLvl) >> 6; // Dampen min/max levels
  maxLvlAvg = (maxLvlAvg * 63 + maxLvl) >> 6; // (fake rolling average)
}

void Balls() {
  for (int i = 0 ; i < NUM_BALLS ; i++) {
    tCycle[i] =  millis() - tLast[i] ;     // Calculate the time since the last time the ball was on the ground

    // A little kinematics equation calculates positon as a function of time, acceleration (gravity) and intial velocity
    h[i] = 0.5 * GRAVITY * pow( tCycle[i] / 1000 , 2.0 ) + vImpact[i] * tCycle[i] / 1000;

    if ( h[i] < 0 ) {
      h[i] = 0;                            // If the ball crossed the threshold of the "ground," put it back on the ground
      vImpact[i] = COR[i] * vImpact[i] ;   // and recalculate its new upward velocity as it's old velocity * COR
      tLast[i] = millis();

      if ( vImpact[i] < 0.01 ) vImpact[i] = vImpact0;  // If the ball is barely moving, "pop" it back up at vImpact0
    }
    pos[i] = round( h[i] * (N_PIXELS - 1) / h0);       // Map "h" to a "pos" integer index position on the LED strip
  }

  //Choose color of LEDs, then the "pos" LED on
  for (int i = 0 ; i < NUM_BALLS ; i++) leds[pos[i]] = CHSV( uint8_t (i * 40) , 255, 255);
  FastLED.show();
  //Then off for the next loop around
  for (int i = 0 ; i < NUM_BALLS ; i++) {
    leds[pos[i]] = CRGB::Black;
  }
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  for (j = 0; j < 256 * 5; j++) { // 5 cycles of all colors on wheel
    for (i = 0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
    }
    strip.show();
    if (digitalRead(buttonPin) != lastButtonState)  // <------------- add this
      return;         // <------------ and this
    delay(wait);
  }
}

void ripple() {
  if (currentBg == nextBg) {
    nextBg = random(256);
  }
  else if (nextBg > currentBg) {
    currentBg++;
  } else {
    currentBg--;
  }
  for (uint16_t l = 0; l < N_PIXELS; l++) {
    leds[l] = CHSV(currentBg, 255, 50);         // strip.setPixelColor(l, Wheel(currentBg, 0.1));
  }
  if (step == -1) {
    center = random(N_PIXELS);
    color = random(256);
    step = 0;
  }
  if (step == 0) {
    leds[center] = CHSV(color, 255, 255);         // strip.setPixelColor(center, Wheel(color, 1));
    step ++;
  }
  else {
    if (step < maxSteps) {
      Serial.println(pow(fadeRate, step));

      leds[wrap(center + step)] = CHSV(color, 255, pow(fadeRate, step) * 255);     //   strip.setPixelColor(wrap(center + step), Wheel(color, pow(fadeRate, step)));
      leds[wrap(center - step)] = CHSV(color, 255, pow(fadeRate, step) * 255);     //   strip.setPixelColor(wrap(center - step), Wheel(color, pow(fadeRate, step)));
      if (step > 3) {
        leds[wrap(center + step - 3)] = CHSV(color, 255, pow(fadeRate, step - 2) * 255);   //   strip.setPixelColor(wrap(center + step - 3), Wheel(color, pow(fadeRate, step - 2)));
        leds[wrap(center - step + 3)] = CHSV(color, 255, pow(fadeRate, step - 2) * 255);   //   strip.setPixelColor(wrap(center - step + 3), Wheel(color, pow(fadeRate, step - 2)));
      }
      step ++;
    }
    else {
      step = -1;
    }
  }
  LEDS.show();
  delay(50);
}

int wrap(int step) {
  if (step < 0) return N_PIXELS + step;
  if (step > N_PIXELS - 1) return step - N_PIXELS;
  return step;
}

void one_color_allHSV(int ahue, int abright) {                // SET ALL LEDS TO ONE COLOR (HSV)
  for (int i = 0 ; i < N_PIXELS; i++ ) {
    leds[i] = CHSV(ahue, 255, abright);
  }
}

void ripple2() {
  if (BG) {
    if (currentBg == nextBg) {
      nextBg = random(256);
    }
    else if (nextBg > currentBg) {
      currentBg++;
    } else {
      currentBg--;
    }
    for (uint16_t l = 0; l < N_PIXELS; l++) {
      strip.setPixelColor(l, Wheel(currentBg, 0.1));
    }
  } else {
    for (uint16_t l = 0; l < N_PIXELS; l++) {
      strip.setPixelColor(l, 0, 0, 0);
    }
  }
  if (step == -1) {
    center = random(N_PIXELS);
    color = random(256);
    step = 0;
  }
  if (step == 0) {
    strip.setPixelColor(center, Wheel(color, 1));
    step ++;
  }
  else {
    if (step < maxSteps) {
      strip.setPixelColor(wrap(center + step), Wheel(color, pow(fadeRate, step)));
      strip.setPixelColor(wrap(center - step), Wheel(color, pow(fadeRate, step)));
      if (step > 3) {
        strip.setPixelColor(wrap(center + step - 3), Wheel(color, pow(fadeRate, step - 2)));
        strip.setPixelColor(wrap(center - step + 3), Wheel(color, pow(fadeRate, step - 2)));
      }
      step ++;
    }
    else {
      step = -1;
    }
  }
  strip.show();
  delay(50);
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos, float opacity) {

  if (WheelPos < 85) {
    return strip.Color((WheelPos * 3) * opacity, (255 - WheelPos * 3) * opacity, 0);
  }
  else if (WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color((255 - WheelPos * 3) * opacity, 0, (WheelPos * 3) * opacity);
  }
  else {
    WheelPos -= 170;
    return strip.Color(0, (WheelPos * 3) * opacity, (255 - WheelPos * 3) * opacity);
  }
}

void pattern2() {
  sinelon();                                                  // Call our sequence.
  show_at_max_brightness_for_power();                         // Power managed display of LED's.
}

void sinelon() {
  // a colored dot sweeping back and forth, with fading trails
  fadeToBlackBy( leds, N_PIXELS, thisfade);
  int pos1 = beatsin16(thisbeat, 0, N_PIXELS - 1);
  int pos2 = beatsin16(thatbeat, 0, N_PIXELS - 1);
  leds[(pos1 + pos2) / 2] += CHSV( myhue++ / 64, thissat, thisbri);
}

void pattern3() {
  ChangeMe();
  juggle();
  show_at_max_brightness_for_power();                         // Power managed display of LED's.
}


void juggle() {                                               // Several colored dots, weaving in and out of sync with each other
  curhue = thishue;                                          // Reset the hue values.
  fadeToBlackBy(leds, N_PIXELS, faderate);
  for ( int i = 0; i < numdots; i++) {
    leds[beatsin16(basebeat + i + numdots, 0, N_PIXELS - 1)] += CHSV(curhue, thissat, thisbright); //beat16 is a FastLED 3.1 function
    curhue += hueinc;
  }
}

void ChangeMe() {                                             // A time (rather than loop) based demo sequencer. This gives us full control over the length of each sequence.
  uint8_t secondHand = (millis() / 1000) % 30;                // IMPORTANT!!! Change '30' to a different value to change duration of the loop.
  static uint8_t lastSecond = 99;                             // Static variable, means it's only defined once. This is our 'debounce' variable.
  if (lastSecond != secondHand) {                             // Debounce to make sure we're not repeating an assignment.
    lastSecond = secondHand;
    if (secondHand ==  0)  {
      numdots = 1;  // You can change values here, one at a time , or altogether.
      faderate = 2;
    }
    if (secondHand == 10)  {
      numdots = 4;
      thishue = 128;
      faderate = 8;
    }
    if (secondHand == 20)  {
      hueinc = 48;  // Only gets called once, and not continuously for the next several seconds. Therefore, no rainbows.
      thishue = random8();
    }
  }
}

void Twinkle () {
  if (random(25) == 1) {
    uint16_t i = random(N_PIXELS);
    if (redStates[i] < 1 && greenStates[i] < 1 && blueStates[i] < 1) {
      redStates[i] = random(256);
      greenStates[i] = random(256);
      blueStates[i] = random(256);
    }
  }

  for (uint16_t l = 0; l < N_PIXELS; l++) {
    if (redStates[l] > 1 || greenStates[l] > 1 || blueStates[l] > 1) {
      strip.setPixelColor(l, redStates[l], greenStates[l], blueStates[l]);

      if (redStates[l] > 1) {
        redStates[l] = redStates[l] * Fade;
      } else {
        redStates[l] = 0;
      }

      if (greenStates[l] > 1) {
        greenStates[l] = greenStates[l] * Fade;
      } else {
        greenStates[l] = 0;
      }

      if (blueStates[l] > 1) {
        blueStates[l] = blueStates[l] * Fade;
      } else {
        blueStates[l] = 0;
      }

    } else {
      strip.setPixelColor(l, 0, 0, 0);
    }
  }
  strip.show();
  delay(10);
}

void fire() {
#define FRAMES_PER_SECOND 40
  random16_add_entropy( random());

  // Array of temperature readings at each simulation cell
  static byte heat[N_PIXELS];

  // Step 1.  Cool down every cell a little
  for ( int i = 0; i < N_PIXELS; i++) {
    heat[i] = qsub8( heat[i],  random8(0, ((COOLING * 10) / N_PIXELS) + 2));
  }

  // Step 2.  Heat from each cell drifts 'up' and diffuses a little
  for ( int k = N_PIXELS - 1; k >= 2; k--) {
    heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2] ) / 3;
  }

  // Step 3.  Randomly ignite new 'sparks' of heat near the bottom
  if ( random8() < SPARKING ) {
    int y = random8(7);
    heat[y] = qadd8( heat[y], random8(160, 255) );
  }

  // Step 4.  Map from heat cells to LED colors
  for ( int j = 0; j < N_PIXELS; j++) {
    // Scale the heat value from 0-255 down to 0-240
    // for best results with color palettes.
    byte colorindex = scale8( heat[j], 240);
    CRGB color = ColorFromPalette( CRGBPalette16( CRGB::Black, CRGB::Red, CRGB::Yellow,  CRGB::White), colorindex);
    int pixelnumber;
    if ( gReverseDirection ) {
      pixelnumber = (N_PIXELS - 1) - j;
    } else {
      pixelnumber = j;
    }
    leds[pixelnumber] = color;

  }
  FastLED.show();
}

void fireblue() {
#define FRAMES_PER_SECOND 40
  random16_add_entropy( random());

  // Array of temperature readings at each simulation cell
  static byte heat[N_PIXELS];

  // Step 1.  Cool down every cell a little
  for ( int i = 0; i < N_PIXELS; i++) {
    heat[i] = qsub8( heat[i],  random8(0, ((COOLING * 10) / N_PIXELS) + 2));
  }

  // Step 2.  Heat from each cell drifts 'up' and diffuses a little
  for ( int k = N_PIXELS - 1; k >= 2; k--) {
    heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2] ) / 3;
  }

  // Step 3.  Randomly ignite new 'sparks' of heat near the bottom
  if ( random8() < SPARKING ) {
    int y = random8(7);
    heat[y] = qadd8( heat[y], random8(160, 255) );
  }

  // Step 4.  Map from heat cells to LED colors
  for ( int j = 0; j < N_PIXELS; j++) {
    // Scale the heat value from 0-255 down to 0-240
    // for best results with color palettes.
    byte colorindex = scale8( heat[j], 240);
    CRGB color = ColorFromPalette( CRGBPalette16( CRGB::Black, CRGB::Blue, CRGB::Aqua,  CRGB::White), colorindex);
    int pixelnumber;
    if ( gReverseDirection ) {
      pixelnumber = (N_PIXELS - 1) - j;
    } else {
      pixelnumber = j;
    }
    leds[pixelnumber] = color;

  }
  FastLED.show();
}

void rainbow_rotate() {                      //-m88-RAINBOW FADE FROM FAST_SPI2 in rotation
  ihue -= 1;
  fill_rainbow2( leds, N_PIXELS, ihue, 256.0 / double(N_PIXELS) );
  //FastLED.setBrightness(max_bright);
  FastLED.show();
  delay(thisdelay);
}

void fill_rainbow2( struct CRGB * pFirstLED, int numToFill, uint8_t initialhue, double deltahue ) {
  CHSV hsv;
  double doublehue = double(initialhue);
  hsv.hue = initialhue;
  hsv.val = 255;
  hsv.sat = 255;
  for ( int i = 0; i < numToFill; i++) {
    hsv.hue = int(doublehue);
    hsv2rgb_rainbow( hsv, pFirstLED[i]);
    doublehue += deltahue;
  }
}

void blur() {
  uint8_t blurAmount = dim8_raw( beatsin8(3, 64, 192) );      // A sinewave at 3 Hz with values ranging from 64 to 192.
  blur1d( leds, N_PIXELS, blurAmount);                        // Apply some blurring to whatever's already on the strip, which will eventually go black.

  uint8_t  i = beatsin8(  9, 0, N_PIXELS - 1);
  uint8_t  j = beatsin8( 7, 0, N_PIXELS - 1);
  uint8_t  k = beatsin8(  5, 0, N_PIXELS - 1);

  // The color of each point shifts over time, each at a different speed.
  uint16_t ms = millis();
  leds[(i + j) / 2] = CHSV( ms / 29, 200, 255);
  leds[(j + k) / 2] = CHSV( ms / 41, 200, 255);
  leds[(k + i) / 2] = CHSV( ms / 73, 200, 255);
  leds[(k + i + j) / 3] = CHSV( ms / 53, 200, 255);
  FastLED.show();
}

void juggle2() {  // Several colored dots, weaving in and out of sync with each other
  // eight colored dots, weaving in and out of sync with each other
  fadeToBlackBy( leds, N_PIXELS, 20);
  byte dothue = 0;
  for ( int i = 0; i < 8; i++) {
    leds[beatsin16(i + 7, 0, N_PIXELS - 1)] |= CHSV(dothue, 200, 255);
    dothue += 32;
  }
  FastLED.show();
}

// List of patterns to cycle through.  Each is defined as a separate function below.
typedef void (*SimplePatternList[])();
SimplePatternList gPatterns = {ripple, ripple2, Twinkle, pattern2, pattern3, Balls, fire, rainbow_rotate, blur, juggle2, fireblue};
uint8_t gCurrentPatternNumber = 0; // Index number of which pattern is current

void nextPattern()
{
  // add one to the current pattern number, and wrap around at the end
  gCurrentPatternNumber = (gCurrentPatternNumber + 1) % ARRAY_SIZE( gPatterns);
}
void All()
{
  // Call the current pattern function once, updating the 'leds' array
  gPatterns[gCurrentPatternNumber]();
  EVERY_N_SECONDS( 30 ) {
    nextPattern();  // change patterns periodically
  }
}
// second list

// List of patterns to cycle through.  Each is defined as a separate function below.
typedef void (*SimplePatternList[])();
SimplePatternList qPatterns = {vu, vu1, vu2, vu3, vu4, vu5, vu6};
uint8_t qCurrentPatternNumber = 0; // Index number of which pattern is current

void nextPattern2()
{
  // add one to the current pattern number, and wrap around at the end
  qCurrentPatternNumber = (qCurrentPatternNumber + 1) % ARRAY_SIZE( qPatterns);
}
void All2()
{
  // Call the current pattern function once, updating the 'leds' array
  qPatterns[qCurrentPatternNumber]();
  EVERY_N_SECONDS( 30 ) {
    nextPattern2();  // change patterns periodically
  }
}
