#include <AverageFilter.h>

#define _DEBUG_

#ifdef _DEBUG_
#define _PP(a) Serial.print(a);
#define _PL(a) Serial.println(a);
#else
#define _PP(a)
#define _PL(a)
#endif

// MSGEQ7 setup
const int analogPin = 10; // MSGEQ7 OUT
const int strobePin = 31; // MSGEQ7 STROBE
const int resetPin = 30;  // MSGEQ7 RESET
int spectrumValue[7];
//int filterValue[7] = {200, 150, 250, 300, 400, 200, 150};
//int filterValue[7] = {0, 0, 0, 0, 0, 0, 0};
int filterValue[7] = {400, 400, 300, 400, 600, 400, 400};

// NEOPixel Setup
#include <WS2812B.h>

// LEDs setup
// LED to controller / channel map:
const byte TOTAL_LEDS = 61;
unsigned long WP_INTERVAL = 1000;
uint32_t currentColor;
byte cR, cG, cB;
byte currentWheelPos = 0;
byte wpDirection = 1;
unsigned long ticks = 0;

// Visualization 1 - full 5 circles
const byte TOTAL_LAYERS1 = 5;
const byte LAYERS1[TOTAL_LAYERS1][25] = {
  //00  01  02  03  04  05  06  07  08  09  10  11  12  13  14  15  16  17  18  19  20  21  22  23  24
  { 24,  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23 },
  { 16, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 0,  0,  0,  0,  0,  0,  0,  0 },
  { 12, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 },
  { 8,  52, 53, 54, 55, 56, 57, 58, 59, 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 },
  { 1,  60, 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 }
};
const byte LAYERS1R[TOTAL_LAYERS1][25] = {
  { 24,  0, 23, 22, 21, 20, 19, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9,  8,  7,  6,  5,  4,  3,  2,  1 },
  { 16, 24, 39, 38, 37, 36, 35, 34, 33, 32, 31, 30, 29, 28, 27, 26, 25, 0,  0,  0,  0,  0,  0,  0,  0 },
  { 12, 40, 51, 50, 49, 48, 47, 46, 45, 44, 43, 42, 41,  0,  0,  0,  0, 0,  0,  0,  0,  0,  0,  0,  0 },
  { 8,  52, 59, 58, 57, 56, 55, 54, 53,  0,  0,  0,  0,  0,  0,  0,  0, 0,  0,  0,  0,  0,  0,  0,  0 },
  { 1,  60,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 0,  0,  0,  0,  0,  0,  0,  0 }
};
// Visualization 1

// Visualization 2 - cross and 4 segments from the center
const byte TOTAL_LAYERS2 = 6;
// Structure of each segment data:
// Total number or segments in a layer
// Total number of leds in layer 1
// Led#1, Led#2, ...

const byte LAYERS2[TOTAL_LAYERS2][21] = {
  { 1,  1, 60,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 4,  4, 52, 54, 56, 58,  4, 40, 43, 46, 49,  4, 24, 28, 32, 36,  4, 0, 6, 12, 18 },
  { 4,  1, 55,  2, 44, 45,  3, 29, 30, 31,  5,  7,  8,  9, 10, 11, 0,  0, 0, 0 },
  { 4,  1, 57,  2, 47, 48,  3, 33, 34, 35,  5, 13, 14, 15, 16, 17, 0,  0, 0, 0 },
  { 4,  1, 59,  2, 50, 51,  3, 37, 38, 39,  5, 19, 20, 21, 22, 23, 0,  0, 0, 0 },
  { 4,  1, 53,  2, 41, 42,  3, 25, 26, 27,  1,  2,  3,  4,  5, 0,  0, 0, 0 },
};
// Visualization 2


const byte TOTAL_LAYERS = TOTAL_LAYERS1;
//const byte *LAYERS = &LAYERS1[0][0];

WS2812B pixels = WS2812B(TOTAL_LEDS);

void setup()
{
#ifdef _DEBUG_
  Serial.begin(115200);
  delay(2000);
#endif
  _PL("MSGEQ7 EQ test");

  // Read from MSGEQ7 OUT
  pinMode(analogPin, INPUT);
  // Write to MSGEQ7 STROBE and RESET
  pinMode(strobePin, OUTPUT);
  pinMode(resetPin, OUTPUT);

  // Set analogPin's reference voltage
  //  analogReference(DEFAULT); // 5V

  // Set startup values for pins
  digitalWrite(resetPin, LOW);
  digitalWrite(strobePin, HIGH);

  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  pixels.show();
  //  delay(500);
  randomSeed(analogRead(11));
  currentWheelPos = random(0, 256);
  setCurrentColor();

  for (int i = TOTAL_LAYERS - 1; i >= 0 ; i--) {
    int leds_this_layer = (int) LAYERS1[i][0];
    for (int j = 0; j < leds_this_layer; j++) {
      pixels.setPixelColor( (int) LAYERS1[i][j + 1], currentColor);
    }
    pixels.show();
    delay(100);
    for (int i = 0; i < TOTAL_LEDS; i++) pixels.setPixelColor(i, 0);
  }

  pixels.clear();
  pixels.show();
  ticks = millis();
}

averageFilter<int> v(3);
void loop()
{
  // Set reset pin low to enable strobe
  digitalWrite(resetPin, HIGH);
  digitalWrite(resetPin, LOW);

  // Get all 7 spectrum values from the MSGEQ7
  for (int i = 0; i < 7; i++)
  {
    int tmp;

    digitalWrite(strobePin, LOW);
    delayMicroseconds(30); // Allow output to settle

    tmp = analogRead(analogPin);
    tmp = constrain(tmp , filterValue[i], 4095) ;

    // Constrain any value above 1023 or below filterValue
    //    tmp = tmp > filterValue ? tmp : 0;


    // Remap the value to a number between 0 and 255
    spectrumValue[i] = map(tmp, filterValue[i], 4095, 0, 4095);

#ifdef _DEBUG_
    // Remove serial stuff after debugging
    if (i == 0) {
      
        Serial.print(v.value(spectrumValue[i]));
    Serial.print("\t4095");
    }
#endif
    digitalWrite(strobePin, HIGH);

  }
#ifdef _DEBUG_
    Serial.println();
    delay(5);
#endif

  //  Visualize1();
  //  Visualize2();
//  Visualize3();


  if ( (millis() - ticks) > WP_INTERVAL) {
    ticks = millis();
    currentWheelPos += wpDirection;
    if (currentWheelPos == 0 || currentWheelPos == 255) wpDirection = -wpDirection;
    setCurrentColor();
  }
}

uint32_t Wheel(byte WheelPos)
{
  if (WheelPos < 85)
  {
    return pixels.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  }
  else
  {
    if (WheelPos < 170)
    {
      WheelPos -= 85;
      return pixels.Color(255 - WheelPos * 3, 0, WheelPos * 3);
    }
    else
    {
      WheelPos -= 170;
      return pixels.Color(0, WheelPos * 3, 255 - WheelPos * 3);
    }
  }
}

void setCurrentColor() {
  currentColor = Wheel(currentWheelPos);
  cR = (byte) (currentColor >> 16);
  cG = (byte) (currentColor >> 8);
  cB = (byte) currentColor;
}

void Visualize1() {
  int r, g, b;

  WP_INTERVAL = 1000;
  for (int i = 0; i < TOTAL_LAYERS1; i++) {
    int spectrum = spectrumValue[i];
    int layerLEDs = LAYERS1[i][0];
    int numLeds = map(spectrum, 0, 4095, 1, layerLEDs);
    int baseIntensity = map(spectrum, 0, 4095, 0, 255);
    for (int j = 0; j < layerLEDs; j++) {
      r = g = b = 0;
      if (j < numLeds) {
        int currentIntensity = baseIntensity - map( constrain(j + 1, 1, numLeds), 1, numLeds, 0, baseIntensity * 3 / 4 );
        if ( numLeds == 1 ) currentIntensity = baseIntensity;
        r = map(cR, 0, 255, 0, currentIntensity);
        g = map(cG, 0, 255, 0, currentIntensity);
        b = map(cB, 0, 255, 0, currentIntensity);
      }
      else {
        if (numLeds > layerLEDs / 2) {
          r = map(cR, 0, 255, 0, baseIntensity / 50);
          g = map(cG, 0, 255, 0, baseIntensity / 50);
          b = map(cB, 0, 255, 0, baseIntensity / 50);
        }
      }
      pixels.setPixelColor(LAYERS1[i][j + 1], pixels.Color(r, g, b));
    }
  }
  pixels.show();
  delay(50);
}


void Visualize2() {
  int r, g, b;

  WP_INTERVAL = 1000;
  for (int i = 0; i < TOTAL_LAYERS2; i++) {

    _PP("Layer="); _PP(i);

    int spectrum = spectrumValue[i];
    int numSegments = LAYERS2[i][0];
    int activeSegments = map(spectrum, 0, 3000, 1, numSegments);
    activeSegments = constrain(activeSegments, 1, numSegments);

    _PP("\tnumSegments="); _PP(numSegments);
    _PP("\tactiveSegments="); _PP(activeSegments);

    int baseIntensity = map(spectrum, 0, 4095, 0, 255);
    int dataIindex = 1;
    for (int j = 0; j < numSegments; j++) {
      r = g = b = 0;
      if (j < activeSegments) {
        int currentIntensity = baseIntensity - map( constrain(j + 1, 1, numSegments), 1, numSegments, 0, baseIntensity * 3 / 4 );

        _PP("\tcurrentIntensity="); _PP(currentIntensity);

        if ( numSegments == 1 ) currentIntensity = baseIntensity;
        int bR = cR, bG = cG, bB = cB;
        if (i == 0) {
          bR =  bG =  bB = 255;
        }
        if (i == 1) {
          byte cwp = currentWheelPos + 128;
          uint32_t bC = Wheel( cwp );
          bR = (byte) (bC >> 16);
          bG = (byte) (bC >> 8);
          bB = (byte) bC;
        }
        r = map(bR, 0, 255, 0, currentIntensity);
        g = map(bG, 0, 255, 0, currentIntensity);
        b = map(bB, 0, 255, 0, currentIntensity);
      }
      //      else {
      //        if (activeSegments > numSegments / 2) {
      //          int r = map(cR, 0, 255, 0, baseIntensity / 50);
      //          int g = map(cG, 0, 255, 0, baseIntensity / 50);
      //          int b = map(cB, 0, 255, 0, baseIntensity / 50);
      //        }
      //      }
      int numLeds = LAYERS2[i][dataIindex++];
      _PP("\tnumLeds="); _PP(numLeds);

      for (int k = 0; k < numLeds; k++) {
        pixels.setPixelColor(LAYERS2[i][dataIindex++], pixels.Color(r, g, b));
      }
    }
    _PL();
  }
  pixels.show();
  delay(50);
}

byte indexOffset = 0;
unsigned long tick3;
void Visualize3() {
  int r, g, b;

  WP_INTERVAL = 500;
  if (millis() - tick3 > WP_INTERVAL * 20) {
    tick3 = millis();
    indexOffset++;
  }
  for (int i = 0; i < TOTAL_LAYERS1; i++) {
    int spectrum = spectrumValue[(i + indexOffset) % 7];
    int layerLEDs = LAYERS1[i][0];
    int baseIntensity = map(spectrum, 0, 4095, 0, 255);
    for (int j = 0; j < layerLEDs; j++) {
      int bR = cR, bG = cG, bB = cB;
      byte cwp = currentWheelPos + i<<3;
      uint32_t bC = Wheel( cwp );
      bR = (byte) (bC >> 16);
      bG = (byte) (bC >> 8);
      bB = (byte) bC;
      r = map(bR, 0, 255, 0, baseIntensity);
      g = map(bG, 0, 255, 0, baseIntensity);
      b = map(bB, 0, 255, 0, baseIntensity);
      pixels.setPixelColor(LAYERS1[i][j + 1], pixels.Color(r, g, b));
    }
  }
  pixels.show();
  delay(50);
}

