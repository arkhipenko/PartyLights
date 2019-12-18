/*
   PartyLight v1.0 (c) Copyright Anatoli Arkhipenko, All rights reserved.

   Music visualization using 61 neopixel LEDs and MSGEQ7 chip

   Change log:
    2019-05-01:
    v0.0.1 - Initial Release - everything works

    2019-05-10:
    v0.0.2 - new initial visualization: fireworks

    2019-05-18:
    v1.0.0 - fully assembled release with latest bugs fixed

    2019-05-18:
    v1.0.1 - added visualization rotation every minute in addition to silence

    2019-05-21:
    v1.0.2 - added startup visualization: fireworks

    2019-05-23:
    v1.0.3 - added a bit of an overlap to the fireworks stages.


*/
//#define _DEBUG_
//#define _GRAPH_

#ifdef _DEBUG_
#define _TASK_TIMECRITICAL      // Enable monitoring scheduling overruns
#endif

#define _TASK_SLEEP_ON_IDLE_RUN // Enable 1 ms SLEEP_IDLE powerdowns between tasks if no callback methods were invoked during the pass
// #define _TASK_STATUS_REQUEST    // Compile with support for StatusRequest functionality - triggering tasks on status change events in addition to time only
// #define _TASK_WDT_IDS           // Compile with support for wdt control points and task ids
#define _TASK_LTS_POINTER       // Compile with support for local task storage pointer
#define _TASK_PRIORITY          // Support for layered scheduling priority
// #define _TASK_MICRO_RES         // Support for microsecond resolution
// #define _TASK_STD_FUNCTION      // Support for std::function (ESP8266 and ESP32 ONLY)
// #define _TASK_DEBUG             // Make all methods and variables public for debug purposes
// #define _TASK_INLINE            // Make all methods "inline" - needed to support some multi-tab, multi-file implementations
// #define _TASK_TIMEOUT           // Support for overall task timeout
// #define _TASK_OO_CALLBACKS      // Support for dynamic callback method binding
#include <TaskScheduler.h>
#include <AverageFilter.h>
#include <Servo.h>

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
const int visSwitchPin = 14; // switch visualizations button
const int cycleMicPin = 13;  // change mic dB sensitivity
const int micDBPin = 29;
const int freezeColPin = 12; // freeze color rotation
const int motionPin = 11; // enable/disable motion
const int servoPin = 27; // PWM servo control pin
const int ledPin = 33; // on-board LED pin

volatile bool visPinHandled = true;
volatile bool micPinHandled = true;
volatile bool colPinHandled = true;
volatile bool motPinHandled = true;

const int DEBOUNCE = 500; //ms

#define TOTAL_EQ_BANDS 7
int spectrumValue[TOTAL_EQ_BANDS];
//const int filterValue[TOTAL_EQ_BANDS] = {200, 150, 250, 300, 400, 200, 150};
//const int filterValue[TOTAL_EQ_BANDS] = {0, 0, 0, 0, 0, 0, 0};
const int filterValue[TOTAL_EQ_BANDS] = {400, 400, 300, 400, 500, 400, 400};

#define NUM_SAMPLES 10
averageFilter<int> fV1(NUM_SAMPLES), fV2(NUM_SAMPLES), fV3(NUM_SAMPLES), fV4(NUM_SAMPLES), fV5(NUM_SAMPLES), fV6(NUM_SAMPLES), fV7(NUM_SAMPLES);
averageFilter<int> *fV[] = { &fV1, &fV2, &fV3, &fV4, &fV5, &fV6, &fV7 };
bool useFilter = true;
long filterUpdateCount = 10;

const int PEAK_SAMPLES = 20;
averageFilter<int> peakValue(PEAK_SAMPLES);
averageFilter<int> peakTempo(PEAK_SAMPLES / 2);
int currentPeak = 0, previousPeak = 0, peakMillis = 0;


// NEOPixel Setup
// Driver pin should be connected to SPI1 MOSI pin 4
#include <WS2812B.h>

// LEDs setup
// LED to controller / channel map:
const byte TOTAL_LEDS = 61;
const unsigned long WP_INTERVAL1 = 500;
const unsigned long WP_INTERVAL3 = 200;
const unsigned int  VS_INTERVAL = 50;
const unsigned long EQ_INTERVAL = 5;
uint32_t currentColor;
bool cycleColors = true;
bool cycleVis = true;
bool enableMotion = false;
byte micStatus = 0;
byte cR, cG, cB;
byte currentWheelPos = 0;
byte wpDirection = 1;
const byte LED_MIN = 0;
const byte LED_MAX = 255;
const int  EQ_MIN = 0;
const int  EQ_MAX = 4095;
const int  EQ_SOFT_MAX = 3500;
#define  TWO_THIRD    2 / 3
#define  THREE_FOURTH 3 / 4

byte currentAngle = 90;
byte currentDirection = 10;
const byte MIN_ANGLE = 10;
const byte MAX_ANGLE = 170;
const byte ANGLE_STEP = 10;

// Silence detection
const int  SILENCE_MIN_LEVEL = 100;
const long SILENCE_MIN_TIME = 3000; // ms
bool  isSilent = true;

// Initial visualization: fireworks
const byte TOTAL_LAYERS_F = 5;
const byte FIREWORKS1[] = { 0, 24, 40, 52, 60 };
const byte FIREWORKS2[] = { 1, 60,
                            8, 52, 53, 54, 55, 56, 57, 58, 59,
                            4, 40, 43, 46, 49,
                            8, 24, 26, 28, 30, 32, 34, 36, 38,
                            8, 0, 3, 6, 9, 12, 15, 18, 21,
                          };

const byte FIREWORKS3[] = { 1, 60,
                            4, 53, 55, 57, 59,
                            8, 41, 42, 44, 45, 47, 48, 50, 51,
                            8, 25, 27, 29, 31, 33, 35, 37, 39,
                            16, 1, 2, 4, 5, 7, 8, 10, 11, 13, 14, 16, 17, 19, 20, 22, 23,
                          };
byte fireworks4[30];


// Visualization 1 & 3 - full 5 circles
const byte TOTAL_LAYERS1 = 5;
const byte LAYERS1[TOTAL_LAYERS1][25] = {
  //00  01  02  03  04  05  06  07  08  09  10  11  12  13  14  15  16  17  18  19  20  21  22  23  24
  { 24,  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23 },
  { 16, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39 },
  { 12, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51 },
  { 8,  52, 53, 54, 55, 56, 57, 58, 59 },
  { 1,  60 }
};
const byte LAYERS1R[TOTAL_LAYERS1][25] = {
  { 24,  0, 23, 22, 21, 20, 19, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9,  8,  7,  6,  5,  4,  3,  2,  1 },
  { 16, 24, 39, 38, 37, 36, 35, 34, 33, 32, 31, 30, 29, 28, 27, 26, 25 },
  { 12, 40, 51, 50, 49, 48, 47, 46, 45, 44, 43, 42, 41 },
  { 8,  52, 59, 58, 57, 56, 55, 54, 53 },
  { 1,  60 }
};
// Visualization 1

// Visualization 2 - cross and 4 segments from the center
const byte TOTAL_LAYERS2 = 6;
// Structure of each segment data:
// Total number or segments in a layer
// Total number of leds in layer 1
// Led#1, Led#2, ...

const byte LAYERS2[TOTAL_LAYERS2][21] = {
  { 1,  1, 60 },
  { 4,  4, 52, 54, 56, 58,  4, 40, 43, 46, 49,  4, 24, 28, 32, 36,  4, 0, 6, 12, 18 },
  { 4,  1, 55,  2, 44, 45,  3, 29, 30, 31,  5,  7,  8,  9, 10, 11 },
  { 4,  1, 57,  2, 47, 48,  3, 33, 34, 35,  5, 13, 14, 15, 16, 17 },
  { 4,  1, 59,  2, 50, 51,  3, 37, 38, 39,  5, 19, 20, 21, 22, 23 },
  { 4,  1, 53,  2, 41, 42,  3, 25, 26, 27,  5, 1,  2,  3,  4,  5 },
};
// Visualization 2

// Visualization 4 - fire
const byte TOTAL_LAYERS4 = 7;
// Structure of each segment data:
// Total number or segments in a layer
// Total number of leds in layer 1
// Led#1, Led#2, ...

const byte LAYERS4[TOTAL_LAYERS4][21] = {
  { 2,  11, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,  7, 39, 38, 37, 36, 35, 34, 33 },
  { 2,  7, 25, 26, 27, 28, 29, 30, 31,  11, 23, 22, 21, 20, 19, 18, 17, 16, 15, 14, 13 },
  { 1,  5, 41, 42, 43, 44, 45 },
  { 1,  5, 51, 50, 49, 48, 47 },
  { 1,  3, 53, 54, 55 },
  { 1,  3, 59, 58, 57 },
  { 1,  9, 0, 24, 40, 52, 60, 56, 46, 32, 12 },
};
// Visualization 4

// Visualization 6 - maze
const byte TOTAL_LAYERS6 = 5;

//const byte LAYERS6[TOTAL_LAYERS6][23] = {
//  { 9, 0, 24, 40, 52, 60, 56, 46, 32, 12 },
//  { 22,  1, 23, 2, 22, 3, 21, 4, 20, 5, 19, 6, 18, 7, 17, 8, 16, 9, 15, 10, 14, 11, 13 },
//  { 14,  25, 39, 26, 38, 27, 37, 28, 36, 29, 35, 30, 34, 31, 33 },
//  { 10,  41, 51, 42, 50, 43, 49, 44, 48, 45, 47 },
//  { 6,  53, 59, 54, 58, 55, 57 },
//};

const byte LAYERS6[TOTAL_LAYERS6][32] = {
  { 31, 0, 24, 40, 52, 60, 56, 46, 32, 12, 13, 11, 14, 10, 15, 9, 16, 8, 17, 7, 18, 6, 19, 5, 20, 4, 21, 3, 22, 2, 23, 1 },
  { 21,  24, 40, 52, 60, 56, 46, 32, 33, 31, 34, 30, 35, 29, 36, 28, 37, 27, 38, 26, 39, 25 },
  { 15,  40, 52, 60, 56, 46, 47, 45, 48, 44, 49, 43, 50, 42, 51, 41 },
  { 9,  52, 60, 56, 55, 57, 54, 58, 53, 59 },
  { 1,  60 },
};
// Visualization 6

// Visualization 7 - cross and 4 segments from the center
const byte TOTAL_LAYERS7 = 5;
// Structure of each segment data:
// Total number or segments in a layer
// Total number of leds in layer 1
// Led#1, Led#2, ...

const byte LAYERS7[TOTAL_LAYERS7][24] = {
  { 1,  1, 60 },
  { 4,  3, 52, 53, 54,  4, 40, 41, 42, 43,  5, 24, 25, 26, 27, 28,  7, 0, 1, 2, 3, 4, 5, 6 },
  { 4,  3, 54, 55, 56,  4, 43, 44, 45, 46,  5, 28, 29, 30, 31, 32,  7, 6, 7, 8, 9, 10, 11, 12 },
  { 4,  3, 56, 57, 58,  4, 46, 47, 48, 49,  5, 32, 33, 34, 35, 36,  7, 12, 13, 14, 15, 16, 17, 18 },
  { 4,  3, 58, 59, 52,  4, 49, 50, 51, 40,  5, 36, 37, 38, 39, 24,  7, 18, 19, 20, 21, 22, 23, 0 },
};
// Visualization 7

//const byte TOTAL_LAYERS = TOTAL_LAYERS1;
//const byte *LAYERS = &LAYERS1[0][0];


WS2812B pixels = WS2812B(TOTAL_LEDS);

Servo servo;
// TASKS

Scheduler ts, hts;

void eqCB();
void colorChangeCB();
void Visualize1();
bool Visualize1OE();
void Visualize1OD();
void channelRotateCB();
void silenceCB();
void visButtonHandleCB();
void colButtonHandleCB();
void micButtonHandleCB();
void motButtonHandleCB();
void beatCB();
void BTReceiveCB();
void randomVis();

Task tEQReader        ( EQ_INTERVAL * TASK_MILLISECOND, TASK_FOREVER, &eqCB, &hts, true );
Task tBeat            ( 0, TASK_ONCE, &beatCB, &hts, false);
Task tBTReceive       ( 50 * TASK_MILLISECOND, TASK_ONCE, &BTReceiveCB, &hts, false);
Task tVisualize       ( VS_INTERVAL, TASK_FOREVER, &Visualize1, &ts, false, &Visualize1OE, &Visualize1OD );
Task tVisRotate       ( TASK_MINUTE, TASK_FOREVER, &randomVis, &ts, true );
Task tChRotate        ( WP_INTERVAL1 * 2, TASK_FOREVER, &channelRotateCB, &ts, false);
Task tColorChange     ( WP_INTERVAL1, TASK_FOREVER, &colorChangeCB, &ts, false );
Task tSilenceDetector ( SILENCE_MIN_TIME, TASK_ONCE, &silenceCB, &ts, false );
Task tVisButHandle    ( DEBOUNCE * TASK_MILLISECOND, TASK_ONCE, &visButtonHandleCB, &ts, false);
Task tColButHandle    ( DEBOUNCE * TASK_MILLISECOND, TASK_ONCE, &colButtonHandleCB, &ts, false);
Task tMicButHandle    ( DEBOUNCE * TASK_MILLISECOND, TASK_ONCE, &micButtonHandleCB, &ts, false);
Task tMotButHandle    ( DEBOUNCE * TASK_MILLISECOND, TASK_ONCE, &motButtonHandleCB, &ts, false);

#define NUMOFVIS 7
int currentVisualizer = 0;
void  (*visMethods[NUMOFVIS])() = { &Visualize1, &Visualize2, &Visualize3, &Visualize4, &Visualize5, &Visualize6, &Visualize7 };
bool  (*visOEMethods[NUMOFVIS])() = { &Visualize1OE, &Visualize2OE, &Visualize3OE, &Visualize4OE, &Visualize5OE, &Visualize6OE, &Visualize7OE };
void  (*visODMethods[NUMOFVIS])() = { &Visualize1OD, &Visualize2OD, &Visualize3OD, &Visualize4OD, &Visualize5OD, &Visualize6OD, &Visualize7OD };

#define BTSerial  Serial1
const String BTToken = "LEDD";
const String BTButton = "BUTT";
const String BTColor = "COLR";
const String BTStatus = "STAT";
const int BTBUFLEN = 64;
char BTBuf[BTBUFLEN];
const int BTRXPin = 25;

// CODE ==============================================

void setup_pins() {

  // Read from MSGEQ7 OUT
  pinMode(analogPin, INPUT);
  // Write to MSGEQ7 STROBE and RESET
  pinMode(strobePin, OUTPUT);
  pinMode(resetPin, OUTPUT);

  // Set startup values for pins
  digitalWrite(resetPin, LOW);
  digitalWrite(strobePin, HIGH);

  pinMode(visSwitchPin, INPUT_PULLDOWN);
  attachInterrupt(visSwitchPin, visButtonPressed, RISING);

  pinMode(cycleMicPin, INPUT_PULLDOWN);
  attachInterrupt(cycleMicPin, micButtonPressed, RISING);

  pinMode(freezeColPin, INPUT_PULLDOWN);
  attachInterrupt(freezeColPin, colButtonPressed, RISING);

  pinMode(motionPin, INPUT_PULLDOWN);
  attachInterrupt(motionPin, motButtonPressed, RISING);

  pinMode(micDBPin, OUTPUT);
  digitalWrite(micDBPin, HIGH);

  pinMode(BTRXPin, INPUT);
  attachInterrupt(BTRXPin, BTSerialRx, CHANGE);
  BTSerial.begin(9600);

  pinMode(ledPin, OUTPUT);
}

void setup_servo() {

  servo.attach(servoPin);
#ifdef _DEBUG_
  servo.write(100);
  delay(200);
  servo.write(80);
  delay(200);
#endif
  servo.write(90);
}

void visButtonPressed() {
  if (visPinHandled) {
    enableVisSwitch();
    cycleVis = false;
    tVisRotate.disable();
  }
}

void enableVisSwitch() {
  detachInterrupt(visSwitchPin);
  tVisButHandle.restartDelayed();
  visPinHandled = false;
  peakTempo.initialize();
  peakValue.initialize();
}

void colButtonPressed() {
  if (colPinHandled) {
    detachInterrupt(freezeColPin);
    tColButHandle.restartDelayed();
    colPinHandled = false;
  }
}

void micButtonPressed() {
  if (micPinHandled) {
    detachInterrupt(cycleMicPin);
    tMicButHandle.restartDelayed();
    micPinHandled = false;
  }
}

void motButtonPressed() {
  if (motPinHandled) {
    detachInterrupt(motionPin);
    tMotButHandle.restartDelayed();
    motPinHandled = false;
  }
}

void BTSerialRx() {
  detachInterrupt(BTRXPin);
  tBTReceive.restartDelayed();
}

// BT COMMAND FORMAT:
// LEDDBUTTN, where N=1,2,3,4
// LEDDCOLRCCC, where CCC is a colorwheel number 000-255
// LEDDSTATAB - request status:
// Status: LEDDSTATABC, where each of places can take value of 0 (off) or 1 (on):
// A - color rotation enabled/disabled
// B - motion enabled/disabled
// C - mic high gain enabled/disabled

void BTReceiveCB() {
  _PP(millis()); _PL(": BTReceiveCB");

  for (int i = 0; i < BTBUFLEN - 1 && BTSerial.available(); i++) {
    BTBuf[i] = BTSerial.read();
    BTBuf[i + 1] = 0;
  }
  String s = String(BTBuf);

  if ( s.substring(0, 4).equals(BTToken) ) {
    if (s.substring(4, 8).equals(BTButton) ) {
      switch (s.charAt(8)) {
        case '1': // switch visualization
          visButtonPressed();
          break;
        case '2': // microphone sensitivity cycle
          micButtonPressed();
          break;
        case '3': // color switcher cycle
          colButtonPressed();
          break;
        case '4': // motion button
          motButtonPressed();
          break;
      }
    }
    if (s.substring(4, 8).equals(BTColor) ) {
      currentWheelPos = s.substring(8, 11).toInt();
      setCurrentColor();
    }
    if (s.substring(4, 8).equals(BTStatus) ) {
      char c = cycleColors ? '1' : '0';
      BTSerial.write( c );
      c = enableMotion ? '1' : '0';
      BTSerial.write( c );
      c = micStatus == 0 ? '1' : '0';
      BTSerial.write( c );
    }
  }
  attachInterrupt(BTRXPin, BTSerialRx, CHANGE);
}

void visButtonHandleCB() {
  _PP(millis()); _PL(": visButtonHandleCB");
  _PP("cycleVis="); _PL(cycleVis);
  currentVisualizer = (currentVisualizer + 1) % NUMOFVIS;
  tVisualize.disable();
  tVisualize.set( VS_INTERVAL, TASK_FOREVER, visMethods[currentVisualizer], visOEMethods[currentVisualizer], visODMethods[currentVisualizer]);
  tVisualize.enableDelayed();
  resetFilters();
  attachInterrupt(visSwitchPin, visButtonPressed, RISING);
  visPinHandled = true;
}

void colButtonHandleCB() {
  _PP(millis()); _PL(": colButtonHandleCB");

  colPinHandled = true;
  cycleColors = !cycleColors;
  attachInterrupt(freezeColPin, colButtonPressed, RISING);
}

void micButtonHandleCB() {
  _PP(millis()); _PP(": micButtonHandleCB.\tmicDBPin=");

  micStatus = ( micStatus + 1 ) % 3;
  switch (micStatus) {
    case 0: // Low Gain
      pinMode(micDBPin, OUTPUT);
      digitalWrite(micDBPin, HIGH);
      _PL("HIGH");
      break;
    case 1: // High gain
      pinMode(micDBPin, OUTPUT);
      digitalWrite(micDBPin, LOW);
      _PL("LOW");
      break;
    case 2:
      pinMode(micDBPin, INPUT_FLOATING);
      break;
  }
  attachInterrupt(cycleMicPin, micButtonPressed, RISING);
  micPinHandled = true;
}

void motButtonHandleCB() {
  _PP(millis()); _PL(": motButtonHandleCB");

  enableMotion = !enableMotion;

  if ( !enableMotion ) {
    servo.write(90);
  }

  attachInterrupt(motionPin, motButtonPressed, RISING);
  motPinHandled = true;
}



void resetFilters() {
  for (int i = 0; i < TOTAL_EQ_BANDS; i++) {
    fV[i]->initialize();
  }
}

void setup()
{
  setup_pins();
#if defined(_DEBUG_) || defined(_GRAPH_)
  Serial.begin(115200);
  delay(2000);
#endif
  _PL("MSGEQ7 EQ test");
  setup_servo();
  delay(1000);

  setup_pixel();

  //  initialVis1();
  initialVis2();

  ts.setHighPriorityScheduler(&hts);
  ts.startNow();

  randomVis();
  enableVisSwitch();

  tVisRotate.enableDelayed();
}

void randomVis() {
  currentVisualizer = random(0, NUMOFVIS);
  tVisualize.disable();
  tVisualize.set( VS_INTERVAL, TASK_FOREVER, visMethods[currentVisualizer], visOEMethods[currentVisualizer], visODMethods[currentVisualizer]);
  tVisualize.enableDelayed();
  resetFilters();
}

void setup_pixel() {
  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  pixels.clear();
  pixels.show();
  //  delay(500);
  randomSeed(analogRead(analogPin));
  for (int i = 0; i < random(1, 20); i++) randomSeed(analogRead(analogPin));
  currentWheelPos = random(0, 256);
  setCurrentColor();
}


// Initial Vis 1: simple concentric circle of a random color
void initialVis1() {
  for (int i = TOTAL_LAYERS1 - 1; i >= 0 ; i--) {
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
}

// Initial Vis 2: fireworks
int brt;
void initialVis2() {

  Scheduler s;
  Task *t[3 * TOTAL_LAYERS_F + 1];

  t[0] = new Task(TASK_MILLISECOND * 50, TASK_FOREVER, &showPixelsCB, &s, true);

  // STAGE 1 ====================
  for (int i = 1; i <= TOTAL_LAYERS_F; i++) {
    t[i] = new Task(TASK_MILLISECOND * 10, 50, &fireworksStage1CB, &s, false);
    t[i]->setLtsPointer((void *) &FIREWORKS1[i - 1]);
    t[i]->enableDelayed(i * 200);
  }

  while (t[TOTAL_LAYERS_F]->isEnabled()) {
    s.execute();
  }



  // STAGE 2 ====================
  currentWheelPos = random(0, 256);
  setCurrentColor();
  int index = 0;
  int dly = 10;

  servo.write(85);
  for (int i = TOTAL_LAYERS_F + 1, k = 0; i <= 2 * TOTAL_LAYERS_F; i++, k++) {
    t[i] = new Task(TASK_MILLISECOND * (10 + dly / 2), 50, &fireworksStage2CB, &s, false);
    t[i]->setLtsPointer((void *) &FIREWORKS2[index]);
    index += ( FIREWORKS2[index] + 1 );
    t[i]->restartDelayed(k * (dly + k) * 2);
    dly += 10;
  }
  t[0]->restart();
  delay(40); servo.write(95);
  while (t[2 * TOTAL_LAYERS_F - 2]->isEnabled()) {
    if (!s.execute()) {
      currentWheelPos++;
      setCurrentColor();
    }
  }


  // STAGE 3 ====================
  index = 0;
  dly = 10;
  servo.write(85);
  for (int i = 2 * TOTAL_LAYERS_F + 1, k = 0; i <= 3 * TOTAL_LAYERS_F; i++, k++) {
    t[i] = new Task(TASK_MILLISECOND * (10 + dly / 2), 50, &fireworksStage2CB, &s, false);
    t[i]->setLtsPointer((void *) &FIREWORKS3[index]);
    index += ( FIREWORKS3[index] + 1 );
    t[i]->restartDelayed(k * (dly + k) * 2);
    dly += 20;
  }
  t[0]->restart();
  delay(40); servo.write(95);
  while (t[3 * TOTAL_LAYERS_F - 3]->isEnabled()) {
    if (!s.execute()) {
      currentWheelPos++;
      setCurrentColor();
    }
  }


  // STAGE 4 ====================
  for (brt = 30; brt > 0; brt--) {
    for (int i = 1; i <= TOTAL_LAYERS_F; i++) {
      t[i]->setIterations(2);
      t[i]->setCallback(&fireworksStage3CB);
      t[i]->setLtsPointer((void *) random(0, 61));
      t[i]->setInterval(50);
      t[i]->restartDelayed(random(50, 151));
    }
    t[TOTAL_LAYERS_F]->restartDelayed(random(150, 201));
    t[0]->restart();
    while (t[TOTAL_LAYERS_F]->isEnabled()) {
      s.execute();
    }
  }


  pixels.clear();
  pixels.show();

  for (int i = 0; i <= 3 * TOTAL_LAYERS_F; i++) {
    delete t[i];
  }
  delay(2000);
  servo.write(90);
}

void showPixelsCB() {
  pixels.show();
}

void fireworksStage1CB() {
  _PP(millis()); _PP(": fireworksStage1CB");

  Scheduler &s = Scheduler::currentScheduler();
  byte *c = (byte *) s.currentLts();

  _PP(" c="); _PL(*c);

  Task &t = s.currentTask();
  byte color = 0;
  int iter = t.getRunCounter();

  if ( t.isLastIteration() ) {
    color = 0;
  }
  else if (iter <= 10) {
    color = 25 * iter;
  }
  else {
    color = 250 - (iter - 10) * 6;
  }
  pixels.setPixelColor((int) *c, pixels.Color(color, color, color));
  //  pixels.show();
}

void fireworksStage2CB() {
  Scheduler &s = Scheduler::currentScheduler();
  byte *c = (byte *) s.currentLts();
  Task &t = s.currentTask();
  int r, g, b, f;
  int iter = t.getRunCounter();

  if ( t.isLastIteration() ) {
    f = 0;
  }
  else if (iter <= 10) {
    f = 25 * iter;
  }
  else {
    f = 250 - (iter - 10) * 6;
  }
  r = map(cR, 0, 255, 0, f);
  g = map(cG, 0, 255, 0, f);
  b = map(cB, 0, 255, 0, f);
  byte *cc = c;
  for (int i = 0; i < (int) *c; i++) {
    pixels.setPixelColor(*(++cc), pixels.Color(r, g, b));
  }
}


void fireworksStage3CB() {
  Scheduler &s = Scheduler::currentScheduler();
  int c = (int) s.currentLts();
  Task &t = s.currentTask();

  byte f = map(255, 0, 255, 0, brt);
  if ( t.isLastIteration() ) {
    f = 0;
  }
  pixels.setPixelColor(c, pixels.Color(f, f, f));
}


void loop()
{
  ts.execute();
}

bool peakDetected = false;

void eqCB() {
  //  _PP(millis());
  //  _PL("\teqCB");

#ifdef _TASK_TIMECRITICAL
  if ( ts.isOverrun() ) {
    _PP(millis()); _PL(": eqCB Overrun");
  }
  digitalWrite(ledPin, HIGH);
#endif

  int tmp, tmpValue[TOTAL_EQ_BANDS], tmpSum = 0;

  // Set reset pin low to enable strobe
  digitalWrite(resetPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(resetPin, LOW);

  // Get all 7 spectrum values from the MSGEQ7
  for (int i = 0; i < TOTAL_EQ_BANDS; i++)
  {

    digitalWrite(strobePin, LOW);
    delayMicroseconds(30); // Allow output to settle

    tmp = analogRead(analogPin);
    tmp = constrain(tmp , filterValue[i], 4095) ;

    // Constrain any value above 1023 or below filterValue
    //    tmp = tmp > filterValue ? tmp : 0;


    // Remap the value to a number between 0 and 255
    tmpValue[i] = map(tmp, filterValue[i], EQ_MAX, EQ_MIN, EQ_MAX);

    if (useFilter) {
      if (tmpValue[i] > fV[i]->currentValue() ) {
        fV[i]->initialize();
        spectrumValue[i] = fV[i]->value( tmpValue[i] );
      }
      else if ( tEQReader.getRunCounter() % filterUpdateCount == 0)
        spectrumValue[i] = fV[i]->value( tmpValue[i] );
    }
    else {
      spectrumValue[i] = tmpValue[i];
    }
    tmpSum += spectrumValue[i];

#ifdef _GRAPH_
    Serial.print(spectrumValue[i]);
    Serial.print('\t');
#endif

    digitalWrite(strobePin, HIGH);

    // Beat detection is on band 0 - 60 Hz
    if ( !isSilent && enableMotion && i == 0 ) {
      int v = spectrumValue[i];
      bool fireBeat = false;
      if ( v <= peakValue.currentValue() * TWO_THIRD ) peakDetected = false;
      if ( currentPeak >= previousPeak && currentPeak >= v  && v > EQ_SOFT_MAX) {
        // this is a peak
        if (!peakDetected && currentPeak > peakValue.currentValue() * TWO_THIRD ) {
          int interval = millis() - EQ_INTERVAL - peakMillis;
          if ( peakTempo.samples() < PEAK_SAMPLES / 4  || interval > 450 ) {  // collect enough sample first and don't "beat" more the 4 times a second
            peakValue.value(currentPeak);
            peakTempo.value(interval);
            peakMillis = millis() - EQ_INTERVAL;
            peakDetected = true;
            fireBeat = true;
          }
        }
      }
      previousPeak = currentPeak;
      currentPeak = v;

      if ( peakValue.samples() >= (PEAK_SAMPLES / 2) && fireBeat ) {
        //        _PP("peakValue="); _PP(peakValue.currentValue());
        //        _PP("peakValue samples="); _PP(peakValue.samples());
        //        _PP("\tpeakTempo="); _PL(peakTempo.currentValue());
        tBeat.restart();
      }
    }
  }
#ifdef _GRAPH_
  Serial.println();
#endif

  // Silence detection

  if ( !isSilent ) {
    if ( tmpSum <= SILENCE_MIN_LEVEL ) {
      if (!tSilenceDetector.isEnabled() ) {
        tSilenceDetector.setCallback(&silenceCB);
        tSilenceDetector.restartDelayed();
      }
    }
    else
      tSilenceDetector.disable();
  }
  else {
    if ( tmpSum > SILENCE_MIN_LEVEL / 10 ) {
      if (!tSilenceDetector.isEnabled() ) {
        tSilenceDetector.setCallback(&playingCB);
        tSilenceDetector.restartDelayed();
      }
    }
    else
      tSilenceDetector.disable();
  }

#ifdef _TASK_TIMECRITICAL
  digitalWrite(ledPin, LOW);
#endif
}

void initPeakDetection() {
  peakTempo.initialize();
  peakValue.initialize();
}

void silenceCB() {
  _PP(millis()); _PL(": silenceCB");

  isSilent = true;
  if ( cycleVis ) {
    enableVisSwitch();
    tVisRotate.restartDelayed();
  }
  initPeakDetection();
}

void playingCB() {
  _PP(millis()); _PL(": playingCB");
  isSilent = false;
  initPeakDetection();
}

//void beatCBSetup() {
//  switch ( tBeat.getRunCounter() ) {
//    case 1:
//      servo.write(90 - ANGLE_STEP);
//      break;
//    case 2:
//      servo.write(90 + ANGLE_STEP);
//      break;
//    case 3:
//      servo.write(90);
//      tBeat.set(TASK_IMMEDIATE, 1, &beatCB);
//      enableMotion = true;
//      break;
//
//  }
//}

void beatCB() {
  _PP(millis()); _PL(": beatCB");

  currentDirection = random (1000) < 500 ? ANGLE_STEP : -ANGLE_STEP;
  if ( currentAngle == MIN_ANGLE ) currentDirection = ANGLE_STEP;
  if ( currentAngle == MAX_ANGLE ) currentDirection = -ANGLE_STEP;
  currentAngle += currentDirection;
  servo.write(currentAngle);
}


void colorChangeCB() {
  if (cycleColors) {
    currentWheelPos += wpDirection;
    if (currentWheelPos == 0 || currentWheelPos == 255) wpDirection = -wpDirection;
    setCurrentColor();
  }
}

uint32_t Wheel(byte WheelPos)
{
  if (WheelPos < 85) {
    return pixels.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  }
  else {
    if (WheelPos < 170) {
      WheelPos -= 85;
      return pixels.Color(255 - WheelPos * 3, 0, WheelPos * 3);
    }
    else {
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

// ===================================================
// Visualizers

// Visualizer 1:
// -------------
// EQ bands are concentric circles
// Going clock- or counter-clock-wise (randomly selected at each start)
// A bit of delay on the way down
// Whole ring lights up if half of the ring is engaged.

byte (*LAYERS)[25];
bool Visualize1OE() {
  LAYERS = (byte (*)[25]) (random(1000) < 500 ? LAYERS1 : LAYERS1R);
  tColorChange.setInterval(WP_INTERVAL1);
  tColorChange.setCallback(&colorChangeCB);
  tColorChange.enable();
  useFilter = true;
  filterUpdateCount = 10;
  return true;
}

void Visualize1OD() {
  pixels.clear();
  tColorChange.disable();
}

void Visualize1() {
  int r, g, b;

  for (int i = 0; i < TOTAL_LAYERS1; i++) {
    int spectrum = spectrumValue[i];
    int layerLEDs = LAYERS[i][0];
    int numLeds = map(spectrum, EQ_MIN, EQ_MAX, 1, layerLEDs);
    int baseIntensity = map(spectrum, EQ_MIN, EQ_MAX, LED_MIN, LED_MAX);
    for (int j = 0; j < layerLEDs; j++) {
      r = g = b = 0;
      if (j < numLeds) {
        int currentIntensity = baseIntensity - map( constrain(j + 1, 1, numLeds), 1, numLeds, 0, baseIntensity * THREE_FOURTH );
        if ( numLeds == 1 ) currentIntensity = baseIntensity;
        r = map(cR, LED_MIN, LED_MAX, 0, currentIntensity);
        g = map(cG, LED_MIN, LED_MAX, 0, currentIntensity);
        b = map(cB, LED_MIN, LED_MAX, 0, currentIntensity);
      }
      else {
        if (numLeds > layerLEDs / 2) {
          r = map(cR, LED_MIN, LED_MAX, 0, baseIntensity / 50);
          g = map(cG, LED_MIN, LED_MAX, 0, baseIntensity / 50);
          b = map(cB, LED_MIN, LED_MAX, 0, baseIntensity / 50);
        }
      }
      pixels.setPixelColor(LAYERS[i][j + 1], pixels.Color(r, g, b));
    }
  }
  pixels.show();
}

// Visualizer 2:
// -------------
// EQ bands are:
//  1. Central pixel
//  2. Central Cross
//  3. Each of the 4 segments
// Each segment lights up according to the respective levels
// Delay on the way down

bool Visualize2OE() {
  tColorChange.setInterval(WP_INTERVAL1);
  tColorChange.setCallback(&colorChangeCB);
  tColorChange.enable();
  useFilter = true;
  filterUpdateCount = 10;
  return true;
}

void Visualize2OD() {
  tColorChange.disable();
  pixels.clear();
}

void Visualize2() {
  int r, g, b;

  for (int i = 0; i < TOTAL_LAYERS2; i++) {
    int spectrum = spectrumValue[i];
    int numSegments = LAYERS2[i][0];
    int activeSegments = map(spectrum, 0, EQ_SOFT_MAX, 1, numSegments);
    activeSegments = constrain(activeSegments, 1, numSegments);
    int baseIntensity = map(spectrum, EQ_MIN, EQ_MAX, LED_MIN, LED_MAX);
    int dataIindex = 1;
    for (int j = 0; j < numSegments; j++) {
      r = g = b = LED_MIN;
      if (j < activeSegments) {
        int currentIntensity = baseIntensity - map( constrain(j + 1, 1, numSegments), 1, numSegments, 0, baseIntensity * THREE_FOURTH );
        if ( numSegments == 1 ) currentIntensity = baseIntensity;
        int bR = cR, bG = cG, bB = cB;
        if (i == 0) {
          bR =  bG =  bB = LED_MAX;
        }
        if (i == 1) {
          byte cwp = currentWheelPos + LED_MAX / 2;
          uint32_t bC = Wheel( cwp );
          bR = (byte) (bC >> 16);
          bG = (byte) (bC >> 8);
          bB = (byte) bC;
        }
        r = map(bR, LED_MIN, LED_MAX, 0, currentIntensity);
        g = map(bG, LED_MIN, LED_MAX, 0, currentIntensity);
        b = map(bB, LED_MIN, LED_MAX, 0, currentIntensity);
      }
      int numLeds = LAYERS2[i][dataIindex++];

      for (int k = 0; k < numLeds; k++) {
        pixels.setPixelColor(LAYERS2[i][dataIindex++], pixels.Color(r, g, b));
      }
    }
  }
  pixels.show();
}

// Visualizer 3:
// -------------
// EQ bands are concentric rings, all leds light up according to the level.
// Delay on the way down
// Colors are shifted between bands according to the wheel
// EQ bands are cycled between rings

bool Visualize3OE() {
  tColorChange.setInterval(WP_INTERVAL3);
  tColorChange.setCallback(&colorChangeCB);
  tColorChange.enable();
  tChRotate.restart();
  useFilter = true;
  filterUpdateCount = 5;
  return true;
}

void Visualize3OD() {
  tColorChange.disable();
  tChRotate.disable();
  pixels.clear();
}

byte indexOffset = 0;
void channelRotateCB() {
  indexOffset++;
}


void Visualize3() {
  int r, g, b;

  for (int i = 0; i < TOTAL_LAYERS1; i++) {
    int spectrum = spectrumValue[(i + indexOffset) % 7];
    int layerLEDs = LAYERS1[i][0];
    int baseIntensity = map(spectrum, EQ_MIN, EQ_MAX, LED_MIN, LED_MAX);
    for (int j = 0; j < layerLEDs; j++) {
      int bR = cR, bG = cG, bB = cB;
      byte cwp = currentWheelPos + i << 3;
      uint32_t bC = Wheel( cwp );
      bR = (byte) (bC >> 16);
      bG = (byte) (bC >> 8);
      bB = (byte) bC;
      r = map(bR, LED_MIN, LED_MAX, 0, baseIntensity);
      g = map(bG, LED_MIN, LED_MAX, 0, baseIntensity);
      b = map(bB, LED_MIN, LED_MAX, 0, baseIntensity);
      pixels.setPixelColor(LAYERS1[i][j + 1], pixels.Color(r, g, b));
    }
  }
  pixels.show();
}

// Visualizer 4:
// -------------
// Fireplace
// EQ bands are concentric semi-rings - onion-like.
// 7 bands: Middle vertical pole, 2 sets of 2 semi-circles on the outside and then 4 individual semicircles
// covering all 7 bands
// Short delay on the way down
// Colors are shifted from red at the botton to yellow at the top

bool Visualize4OE() {
  useFilter = true;
  filterUpdateCount = 2;
  return true;
}

void Visualize4OD() {
  pixels.clear();
}

void Visualize4() {
  int r, g, b;

  for (int i = 0; i < TOTAL_LAYERS4; i++) {

    int spectrum = spectrumValue[i];
    int numSegments = LAYERS4[i][0];
    int baseIntensity = map(spectrum, EQ_MIN, EQ_MAX, LED_MIN, LED_MAX);
    int dataIindex = 1;

    for (int j = 0; j < numSegments; j++) {
      int numLeds = LAYERS4[i][dataIindex++];
      int activeLEDs = map(spectrum, 0, 4000, 1, numLeds);
      activeLEDs = constrain(activeLEDs, 1, numLeds);
      int  adj = random(0, 6);
      for (int k = 0; k < numLeds; k++) {
        r = g = b = LED_MIN;
        if (k < activeLEDs) {
          int currentIntensity = baseIntensity - map( constrain(k + 1, 1, numLeds), 1, numLeds, 0, baseIntensity );

          r = currentIntensity;
          g = currentIntensity * k / numLeds * adj / 5;
          if (random(1000) == 777) {
            r = g = b = LED_MAX;
          }
        }
        pixels.setPixelColor(LAYERS4[i][dataIindex++], pixels.Color(r, g, b));
      }
    }
  }
  pixels.show();
}

// Visualizer 5:
// -------------
// EQ bands are concentric circles
// Beat LEDs are the 5 vertical ones
//

uint32_t v5colors[TOTAL_LEDS] = {0};
Task *tHelper;

bool Visualize5OE() {
  LAYERS = (byte (*)[25]) (random(1000) < 500 ? LAYERS1 : LAYERS1R);
  tColorChange.setInterval(WP_INTERVAL1);
  tColorChange.setCallback(&colorChangeCB);
  tColorChange.enable();
  useFilter = true;
  filterUpdateCount = 1;
  tHelper = new Task (100 * TASK_MILLISECOND, TASK_FOREVER, &Visualize5Shift, &ts, true);
  //  tHelper.restart();
  for (int i = 0; i < TOTAL_LEDS; i++) v5colors[i] = LED_MIN;
  return true;
}

void Visualize5OD() {
  pixels.clear();
  tColorChange.disable();
  tHelper->disable();
  delete tHelper;
}

void Visualize5() {
  int r, g, b;

  for (int i = 0; i < TOTAL_LAYERS1; i++) {
    int spectrum = spectrumValue[i];
    int baseIntensity = map(spectrum, EQ_MIN, EQ_MAX, LED_MIN, LED_MAX);
    byte cwp = currentWheelPos + i << 3;
    uint32_t bC = Wheel( cwp );
    r = map((byte) (bC >> 16), LED_MIN, LED_MAX, 0, baseIntensity);
    g = map((byte) (bC >> 8), LED_MIN, LED_MAX, 0, baseIntensity);
    b = map((byte) bC, LED_MIN, LED_MAX, 0, baseIntensity);
    uint32_t c = pixels.Color(r, g, b);
    v5colors[ LAYERS[i][1] ] = c;
    pixels.setPixelColor(LAYERS[i][1], c);
  }
  pixels.show();

  // Periodically change direction of the pixel travel
  if (random(1000) == 777) {
    LAYERS = (byte (*)[25]) ( (LAYERS == ((byte (*)[25]) LAYERS1R)) ? LAYERS1 : LAYERS1R );
  }
}

void Visualize5Shift() {
  uint32_t c;

  for (int i = 0; i < TOTAL_LAYERS1; i++) {
    int layerLEDs = LAYERS[i][0];
    int k = layerLEDs << 1;
    for (int j = layerLEDs; j > 1; j--) {
      c = v5colors[ LAYERS[i][j - 1] ];

      int r = (byte) (c >> 16);
      int g = (byte) (c >> 8);
      int b = (byte) (c);
      r = constrain(r - LED_MAX / k, LED_MIN, LED_MAX);
      g = constrain(g - LED_MAX / k, LED_MIN, LED_MAX);
      b = constrain(b - LED_MAX / k, LED_MIN, LED_MAX);
      c = pixels.Color(r, g, b);
      v5colors[ LAYERS[i][j] ] = c;
      pixels.setPixelColor(LAYERS[i][j], c);
    }
  }
}


// Visualizer 6:
// -------------
// EQ bands "trees" that grow from the bottom LED up and then sideways down

bool Visualize6OE() {
  tColorChange.setInterval(WP_INTERVAL1);
  tColorChange.setCallback(&colorChangeCB);
  tColorChange.enable();
  useFilter = true;
  filterUpdateCount = 12;
  return true;
}

void Visualize6OD() {
  pixels.clear();
  tColorChange.disable();
}

void Visualize6() {
  int r, g, b;

  for (int i = 0; i < TOTAL_LAYERS6; i++) {
    int spectrum = spectrumValue[i];
    int layerLEDs = LAYERS6[i][0];
    int numLeds = map(spectrum, EQ_MIN, EQ_MAX, 1, layerLEDs);
    int baseIntensity = map(spectrum, EQ_MIN, EQ_MAX, LED_MIN, LED_MAX);
    byte cwp = currentWheelPos + i << 4;
    uint32_t bC = Wheel( cwp );
    for (int j = 0; j < layerLEDs; j++) {
      r = g = b = 0;
      if (j < numLeds) {
        int currentIntensity = baseIntensity - map( constrain(j + 1, 1, numLeds), 1, numLeds, 0, baseIntensity); //* THREE_FOURTH );
        r = map((byte) (bC >> 16), LED_MIN, LED_MAX, 0, currentIntensity);
        g = map((byte) (bC >> 8), LED_MIN, LED_MAX, 0, currentIntensity);
        b = map((byte) bC, LED_MIN, LED_MAX, 0, currentIntensity);
      }
      pixels.setPixelColor(LAYERS6[i][j + 1], pixels.Color(r, g, b));
    }
  }
  pixels.show();
}


// Visualizer 7:
// -------------
// EQ bands are ?

byte ledDirection;
bool Visualize7OE() {
  ledDirection = random (1, 3);
  tColorChange.setInterval(WP_INTERVAL1);
  tColorChange.setCallback(&colorChangeCB);
  tColorChange.enable();
  useFilter = true;
  filterUpdateCount = 10;
  return true;
}

void Visualize7OD() {
  tColorChange.disable();
  pixels.clear();
}

void Visualize7() {
  int r, g, b;

  for (int i = 0; i < TOTAL_LAYERS7; i++) {
    if (i == 0 || i == ledDirection || i == (ledDirection + 2) ) {
      int spectrum = spectrumValue[i];
      int numSegments = LAYERS7[i][0];
      int activeSegments = map(spectrum, 0, EQ_SOFT_MAX, 1, numSegments);
      activeSegments = constrain(activeSegments, 1, numSegments);
      int baseIntensity = map(spectrum, EQ_MIN, EQ_MAX, LED_MIN, LED_MAX);
      int dataIindex = 1;
      for (int j = 0; j < numSegments; j++) {
        r = g = b = LED_MIN;
        if (j < activeSegments) {
          int currentIntensity = baseIntensity - map( constrain(j + 1, 1, numSegments), 1, numSegments, 0, baseIntensity * THREE_FOURTH );
          if ( numSegments == 1 ) currentIntensity = baseIntensity;
          int bR = cR, bG = cG, bB = cB;

          byte cwp = currentWheelPos + i << 4;
          uint32_t bC = Wheel( cwp );
          bR = (byte) (bC >> 16);
          bG = (byte) (bC >> 8);
          bB = (byte) bC;

          r = map(bR, LED_MIN, LED_MAX, 0, currentIntensity);
          g = map(bG, LED_MIN, LED_MAX, 0, currentIntensity);
          b = map(bB, LED_MIN, LED_MAX, 0, currentIntensity);
        }
        int numLeds = LAYERS7[i][dataIindex++];

        for (int k = 0; k < numLeds; k++) {
          pixels.setPixelColor(LAYERS7[i][dataIindex++], pixels.Color(r, g, b));
        }
      }
    }
  }
  pixels.show();
  if (random(1000) < 10)  {
    ledDirection = random (1, 3);
    pixels.clear();
  }
}
