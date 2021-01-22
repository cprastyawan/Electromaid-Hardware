#ifndef FilterOnePole_h
#define FilterOnePole_h

#include <math.h>
#include "stm32f3xx_hal.h"

typedef enum {
  HIGHPASS,
  LOWPASS,
  INTEGRATOR,
  DIFFERENTIATOR
}FILTER_TYPE;

// the recursive filter class implements a recursive filter (low / pass / highpass
// note that this must be updated in a loop, using the most recent acquired values and the time acquired
//   Y = a0*X + a1*Xm1
//              + b1*Ylast
typedef struct{
  FILTER_TYPE FT;
  float TauUS;       // decay constant of the filter, in US
  float TauSamps;    // tau, measued in samples (this changes, depending on how long between input()s

  // filter values - these are public, but should not be set externally
  float Y;       // most recent output value (gets computed on update)
  float Ylast;   // prevous output value

  float X;      // most recent input value

  // elapsed times are kept in long, and will wrap every
  // 35 mins, 47 seconds ... however, the wrap does not matter,
  // because the delta will still be correct (always positive and small)
  float ElapsedUS;   // time since last update
  uint64_t LastUS;       // last time measured
} FilterOnePole;

  void FilterOnePole_init(FilterOnePole *filterOnePole, FILTER_TYPE ft, float fc, float initialValue);
  
  // sets or resets the parameters and state of the filter
  void FilterOnePole_setFilter(FilterOnePole *filterOnePole, FILTER_TYPE ft, float fc, float initialValue);

  void FilterOnePole_setFrequency(FilterOnePole *filterOnePole, float newFrequency );
  
  void FilterOnePole_setTau(FilterOnePole *filterOnePole, float newTau );

  float FilterOnePole_input(FilterOnePole *filterOnePole, float inVal);

  float FilterOnePole_output(FilterOnePole *filterOnePole);
  
  void FilterOnePole_setToNewValue(FilterOnePole *filterOnePole, float newVal );  // resets the filter to a new value

// two pole filter, these are very useful
typedef struct{

  FilterOnePole Pole1;
  FilterOnePole Pole2;
  
} FilterOnePoleCascade;

  void FilterOnePoleCascade_init(FilterOnePoleCascade *filterOnePoleCascade, float riseTime, float initialValue);  // rise time to step function, 10% to 90%
  
  // rise time is 10% to 90%, for a step input
  void FilterOnePoleCascade_setRiseTime(FilterOnePoleCascade *filterOnePoleCascade, float riseTime );
  
  void FilterOnePoleCascade_setToNewValue(FilterOnePoleCascade *filterOnePoleCascade, float newVal );
  
  float FilterOnePoleCascade_input(FilterOnePoleCascade *filterOnePoleCascade, float inVal );
  
  float FilterOnePoleCascade_output(FilterOnePoleCascade *filterOnePoleCascade);

  uint64_t micros();

#endif

