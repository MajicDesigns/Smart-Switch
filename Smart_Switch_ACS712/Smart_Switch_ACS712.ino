/*
Current Sensing Switch Controller

Used to switch an secondary AC outlet on/off when a primary outlet has current 
flowing through it.

getVpp source code and article
http://henrysbench.capnfatz.com/henrys-bench/arduino-current-measurements/acs712-arduino-ac-current-tutorial/
*/

#include <MD_KeySwitch.h>

// -------------------------------------
// Hardware definitions
const uint8_t SENSOR_INPUT = A5;
const uint8_t PRIMARY_LED = 4;
const uint8_t SECONDARY_LED = 5;
const uint8_t SECONDARY_OUT = 6;
const uint8_t OVERRIDE_SW = 7;

// Sensor calculation parameters
const uint8_t MV_PER_AMP = 185; // use 185 for 5A module; 100 for 20A; 66 for 30A
const uint8_t MIN_AMPS_DETECTED = 1;  // minimum current for detection (A)
const uint16_t SENSOR_DEADBAND = ((MIN_AMPS_DETECTED * 1000L) / MV_PER_AMP) * (5000/1024);
const uint8_t AC_FREQUENCY = 50;  // in Hz

// -------------------------------------
// Time delay definitions
const uint16_t START_DELAY = 2000;   // in milliseconds
const uint16_t END_DELAY = 5000;     // in milliseconds

// -------------------------------------
// END OF USER CONFIGURATION PARAMETERS
// -------------------------------------
#define DEBUG 1

#if DEBUG
#define PRINTS(s)   { Serial.print(F(s)); }
#define PRINT(s,v)  { Serial.print(F(s)); Serial.print(v); }
#define PRINTX(s,x) { Serial.print(F(s)); Serial.print(v, HEX); }
#else
#define PRINTS(s)
#define PRINT(s,v)
#define PRINTX(s,x)
#endif

// -------------------------------------
// Global definitions
typedef enum stateRun_e { RS_INIT, RS_IDLE, RS_OVERRIDE, RS_START_DELAY, RS_RUN, RS_END_DELAY };

uint16_t zeroCurrent;   // zero current reading
MD_KeySwitch  SW(OVERRIDE_SW);

void setup()
{
#if DEBUG
  Serial.begin(57600);
#endif
  PRINTS("\n[AutoSense Switch Debug]");

  // initialise hardware
  pinMode(SENSOR_INPUT, INPUT);
  pinMode(SECONDARY_OUT, OUTPUT);
  pinMode(PRIMARY_LED, OUTPUT);
  pinMode(SECONDARY_LED, OUTPUT);

  // initialise objects
  SW.begin();
  SW.enableDoublePress(false);
  SW.enableLongPress(false);
  SW.enableRepeat(false);

  // work out the current detection parameters
  zeroCurrent = analogRead(SENSOR_INPUT);
}

bool checkPrimary(void)
// We get a sinusoidal signal so need to detect when the signal stops changing
// over a period that would encompass 1 or 2 waveforms.
{
  const uint8_t timeSampling = 2 * (1000 / AC_FREQUENCY);
  uint32_t timeStart;
  uint16_t sampleMin = 1024;
  uint16_t sampleMax = 0;

  timeStart = millis();
  while (millis() - timeStart < timeSampling)
  {
    uint16_t value = analogRead(SENSOR_INPUT);
    
    if (value > sampleMax) sampleMax = value;
    if (value < sampleMin) sampleMin = value;
  }

  return((sampleMax - sampleMin) > (SENSOR_DEADBAND * 2));
}

void switchPrimary(bool state)
// Do whatever required to switch primary on
{
  digitalWrite(PRIMARY_LED, state ? HIGH : LOW);
}

void switchSecondary(bool state)
// Do whatever required to switch secondary on
{
  digitalWrite(SECONDARY_OUT, state ? HIGH : LOW);
  digitalWrite(SECONDARY_LED, state ? HIGH : LOW);
}

void loop()
{
  static stateRun_e state = RS_INIT;
  static uint32_t timeStart;

#if DEBUG
  static stateRun_e lastState = RS_IDLE; // not equal to state

  if (state != lastState)
  {
    lastState = state;
    switch (state)
    {
      case RS_INIT:        PRINTS("\nRS_INIT");         break;
      case RS_IDLE:        PRINTS("\nRS_IDLE");         break;
      case RS_OVERRIDE:    PRINTS("\nRS_OVERRIDE");     break;
      case RS_START_DELAY: PRINTS("\nRS_START_DEDLAY"); break;
      case RS_RUN:         PRINTS("\nRS_RUN");          break;
      case RS_END_DELAY:   PRINTS("\nRS_END_DELAY");    break;
      default:             PRINTS("\nRS_unknown");      break;
    }
  }
#endif

  switch (state)
  {
  case RS_INIT:   // initialise for the idle state at start of a new run
    switchPrimary(false);
    switchSecondary(false);
    state = RS_IDLE;
    break;

  case RS_IDLE:   // override switch or primary starts
    // check if the override switch has been pressed
    if (SW.read() == MD_KeySwitch::KS_PRESS)
    {
      switchSecondary(true);
      state = RS_OVERRIDE;
      break;
    }
    // has the primary started
    if (checkPrimary())
    {
      switchPrimary(true);
      timeStart = millis();
      state = RS_START_DELAY;
    }
    break;

  case RS_OVERRIDE: // wait for switch to exit this mode
    switchPrimary(checkPrimary());   // keep LED indicator synchronised
    if (SW.read() == MD_KeySwitch::KS_PRESS) 
      state = (checkPrimary() ? RS_RUN : RS_INIT);
    break;

  case RS_START_DELAY:  // delay the secondary startup
    if (millis() - timeStart >= START_DELAY)
    {
      switchSecondary(true);
      state = RS_RUN;
    }
    break;

  case RS_RUN:        // run until primary stops
    if (!checkPrimary())
    {
      switchPrimary(false);
      timeStart = millis();
      state = RS_END_DELAY;
    }
    break;

  case RS_END_DELAY:  // delay the secondary end
    if (millis() - timeStart >= END_DELAY)
      state = RS_INIT;
    break;

  default:
    state = RS_INIT;
  }
}

