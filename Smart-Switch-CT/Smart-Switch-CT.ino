/*
Current Sensing Smart Switch Controller
using a Current Transformer (CT) to detect AC current

Used to switch a secondary AC outlet on/off when it detects the primary outlet 
is in use (ie, has current flowing through it). 
In this application we only want to detect current above MIN_AMPS_DETECTED so 
some error is not material in the scheme of the measurement, and the control  
voltage is assumed to be 5V.

Optional pot (#defined for use) can be used to set the START and END delays 
(both equal) rather than hard coded values.

The Smart Switch operates in 2 different modes:
1. Follower Mode. The switch monitors the primary outlet for a current greater than
   MIN_AMPS_DETECTED. When this threshold is reached the secondary outlet is turned
   on after an optional START_DELAY. When the current in the secondary falls below
   the threshold value, the secondary will be turned off after an optional STOP_DELAY.
2. Override mode. The secondary switch may be turned on at any time by pressing a 
   momentary on switch at OVERRIDE_SW. The behaviour of the primary is ignored in this
   mode until the overrdie is turned off by a second press of OVERRIDE_SW.

Good article for the hardware required to interface the CT to the Arduino at
https://openenergymonitor.org/emon/buildingblocks/ct-sensors-interface

Library dependencies:
MD_KeySwitch found at https://github.com/MajicDesigns/MD_KeySwitch
*/

#include "Smart_Switch_CT.h"

// -------------------------------------
// Global definitions
MD_KeySwitch  SW(OVERRIDE_SW);

bool checkPrimary(bool calibrate = false)
// We get a sinusoidal signal so need to detect when the signal stops changing
// over a period that would encompass 1 or 2 waveforms.
// Return true when detected (min,max) range exceeds the threshold of interest.
{
#define DEBUG_PRIMARY 0
  const uint16_t timeSampling = 2 * (1000L / AC_FREQUENCY); // in milliseconds
  static uint16_t zeroAdjust = 0;
  uint16_t sampleMin = 1024;    // min sensor value detected
  uint16_t sampleMax = 0;       // max sensor value detected
  uint32_t timeStart = millis();

  while (millis() - timeStart < timeSampling)
  {
    uint16_t value = analogRead(CT_SENSOR_INPUT);
    
    // update min/max if we have new candidates for these boundaries
    if (value > sampleMax) sampleMax = value;
    if (value < sampleMin) sampleMin = value;
  }

  if (calibrate)
  {
    zeroAdjust = sampleMax - sampleMin;
    PRINT("\nZero Calibrate ", zeroAdjust);
  }

#if DEBUG_PRIMARY
  PRINT("\nSample (", sampleMin);
  PRINT(",", sampleMax);
#endif

  // work out and adjust the difference with zero calibration (unsigned maths!)
  sampleMax -= sampleMin;
  sampleMax = (sampleMax < zeroAdjust ? 0 : sampleMax - zeroAdjust);
#if DEBUG_PRIMARY
  PRINT(")= ", sampleMax);
#endif
  return(sampleMax > (SENSOR_THRESHOLD * 2));
}

void switchPrimary(bool state)
// Do whatever required to switch primary on
// As this is the control (ie we detect it) there is only the LED
// to manage to show on/off status.
{
  digitalWrite(PRIMARY_LED, state ? HIGH : LOW);
}

void switchSecondary(bool state)
// Do whatever required to switch secondary on
// Need to control both the output to turn on the outlet and also
// mange the LED on/off status
{
  digitalWrite(SECONDARY_OUT, state ? HIGH : LOW);
  digitalWrite(SECONDARY_LED, state ? HIGH : LOW);
}

#if PRINT_DEBUG
char *rs2Label(stateRun_e s)
{
  char *psz;

  switch (s)
  {
  case RS_INIT:        psz = "INIT ";  break;
  case RS_IDLE:        psz = "IDLE ";  break;
  case RS_OVERRIDE:    psz = "ORIDE";  break;
  case RS_START_DELAY: psz = "SRTDL";  break;
  case RS_SETTLE:      psz = "SETTL";  break;
  case RS_RUN:         psz = "RUN  ";  break;
  case RS_STOP_DELAY:  psz = "STPDL";  break;
  default:             psz = "?????";  break;
  }

  return(psz);
}
#endif

void setup()
{
#if PRINT_DEBUG
  Serial.begin(57600);
#endif
  PRINTS("\n[Smart Switch Debug]");

  // initialise hardware
  pinMode(CT_SENSOR_INPUT, INPUT);
#if USE_VARIABLE_DELAY
  pinMode(DELAY_POT_INPUT, INPUT);
#endif
  pinMode(SECONDARY_OUT, OUTPUT);
  pinMode(PRIMARY_LED, OUTPUT);
  pinMode(SECONDARY_LED, OUTPUT);

  // initialise objects
  SW.begin();
  SW.enableDoublePress(false);
  SW.enableLongPress(false);
  SW.enableRepeat(false);

  // calibrate the primary zero current flow
  checkPrimary(true);

  PRINT("\nSENSOR_THRESHOLD ", SENSOR_THRESHOLD);
}

#if USE_VARIABLE_DELAY
uint16_t getWaitDelay()
{
  return(map(analogRead(DELAY_POT_INPUT), 0, 1023, 0, POT_RANGE_MAX));
}
#endif

void loop()
{
  static stateRun_e state = RS_INIT;
  static uint32_t timeStart;

#if PRINT_DEBUG
  static stateRun_e lastState = RS_IDLE; // used to detect change in state for debug message only

  if (state != lastState)
  {
    lastState = state;
    PRINT("\n", rs2Label(state));
  }
#endif  // PRINT_DEBUG

  // Check if the override switch has been pressed as this state can 
  // become active any time. If we are already in OVERRIDE then let
  // the FSM detect and handle the keypress if there is one.
  if (state != RS_OVERRIDE)
  {
    if (SW.read() == MD_KeySwitch::KS_PRESS)
    {
      switchSecondary(true);
      state = RS_OVERRIDE;
    }
  }

  // Run the finite state machine
  switch (state)
  {
  case RS_INIT:   // initialise for the idle state at start of a new run
    {
      switchPrimary(false);
      switchSecondary(false);
      state = RS_IDLE;
    }
    break;

  case RS_IDLE:   // wait for primary to start
    if (checkPrimary())
    {
      switchPrimary(true);
      timeStart = millis();
#if USE_VARIABLE_DELAY
      waitDelay = getWaitDelay();
#else
      waitDelay = START_DELAY;
#endif
      state = RS_START_DELAY;
    }
    break;

  case RS_OVERRIDE: // wait for second press switch to exit this mode
    {
      bool b = checkPrimary();

      switchPrimary(b);   // keep LED indicator synchronised
      if (SW.read() == MD_KeySwitch::KS_PRESS)
        state = (b ? RS_RUN : RS_INIT);
    }
    break;

  case RS_START_DELAY:  // delay secondary start
    if (millis() - timeStart >= waitDelay)
    {
      switchSecondary(true);
      timeStart = millis();
      state = RS_SETTLE;
    }
    break;

  case RS_SETTLE:       // wait for starting transients to settle down
    if (millis() - timeStart >= SETTLE_DELAY)
      state = RS_RUN;
    break;

  case RS_RUN:        // run until primary stops
    if (!checkPrimary())
    {
      switchPrimary(false);
      timeStart = millis();
#if USE_VARIABLE_DELAY
      waitDelay = getWaitDelay();
#else
      waitDelay = STOP_DELAY;
#endif
      state = RS_STOP_DELAY;
    }
    break;

  case RS_STOP_DELAY:  // delay secondary stop
    if (millis() - timeStart >= waitDelay)
      state = RS_INIT;
    break;

  default:
    state = RS_INIT;
  }
}
