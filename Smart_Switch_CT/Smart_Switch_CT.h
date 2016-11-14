#pragma once

#include <Arduino.h>
#include <MD_KeySwitch.h>

// -------------------------------------
// Define hardware setting options
#define USE_VARIABLE_DELAY 1      // define the delay using a pot on DELAY_POT_INPUT

// -------------------------------------
// Hardware definitions
const uint8_t CT_SENSOR_INPUT = A3;  // CT analog input with DC bias circuit to avoid negative voltage
#if USE_VARIABLE_DELAY
const uint8_t DELAY_POT_INPUT = A2;  // pot defining the delay parameters
#endif
const uint8_t PRIMARY_LED = 2;    // LED indicates primary AC outlet is active
const uint8_t SECONDARY_LED = 4;  // LED indicates secondary AC outlet is active
const uint8_t SECONDARY_OUT = 11;  // Activate the secondary outlet
const uint8_t OVERRIDE_SW = 10;    // Switch to override secondary outlet and toggle it on/off

// Sensor calculation parameters
const uint32_t AC_FREQUENCY = 50;         // in Hz
const uint32_t MIN_AMPS_DETECTED = 200;   // minimum current threshold for detection (AC RMS milli Amp)

const uint32_t BURDEN_RESISTANCE = 176; // CT burden Resistance (Ohm)
const uint32_t CT_TURNS_RATIO = 2000;   // CT turns ratio specification
const uint32_t CT_PRI_RMS_CURRENT = 20; // CT maximum primary current (RMS Amps)

const uint16_t SENSOR_THRESHOLD = (1024L * MIN_AMPS_DETECTED) / (CT_PRI_RMS_CURRENT * 1000);   // percentage full scale

// -------------------------------------
// Time delay definitions
uint16_t waitDelay = 0;
#if USE_VARIABLE_DELAY
const uint16_t POT_RANGE_MAX = 4000;  // in milliseconds
#endif
const uint16_t START_DELAY = 2000;   // in milliseconds
const uint16_t SETTLE_DELAY = 1500;  // in milliseconds
const uint16_t STOP_DELAY = 3000;    // in milliseconds

// -------------------------------------
// END OF USER CONFIGURATION PARAMETERS
// -------------------------------------
typedef enum stateRun_e 
{ 
  RS_INIT,        // initialisation state
  RS_IDLE,        // idle state - monitoring the primary for something to happen
  RS_OVERRIDE,    // override state - secondary turned on independently of primary
  RS_START_DELAY, // start delay - waiting for start delay time to expire before starting secondary
  RS_SETTLE,      // settle delay - waiting for AC transients to settle down before continuing monitoring
  RS_RUN,         // run state - watching for primary to turn off
  RS_STOP_DELAY   // stop delay - waiting for stop delay to run out before turning off secondary
};

#define PRINT_DEBUG 0

#if PRINT_DEBUG
#define PRINTS(s)   { Serial.print(F(s)); }
#define PRINT(s,v)  { Serial.print(F(s)); Serial.print(v); }
#define PRINTX(s,x) { Serial.print(F(s)); Serial.print(v, HEX); }
#else
#define PRINTS(s)
#define PRINT(s,v)
#define PRINTX(s,x)
#endif
