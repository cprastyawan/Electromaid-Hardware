#include "FilterOnePole.h"
#include "FloatDefine.h"
#include <inttypes.h>

  void FilterOnePole_init(FilterOnePole *filterOnePole, FILTER_TYPE ft, float fc, float initialValue){
    FilterOnePole_setFilter(filterOnePole, ft, fc, initialValue );
  }
  
  // sets or resets the parameters and state of the filter
  void FilterOnePole_setFilter(FilterOnePole *filterOnePole, FILTER_TYPE ft, float fc, float initialValue ){
    filterOnePole->FT = ft;
    FilterOnePole_setFrequency(filterOnePole, fc );

    filterOnePole->Y = initialValue;
    filterOnePole->Ylast = initialValue;
    filterOnePole->X = initialValue;

    filterOnePole->LastUS = micros();
  }

  void FilterOnePole_setFrequency(FilterOnePole *filterOnePole, float newFrequency ){
    FilterOnePole_setTau(filterOnePole, 1.0 / (TWO_PI*newFrequency ) ); // τ=1/ω
  }
  
  void FilterOnePole_setTau(FilterOnePole *filterOnePole, float newTau ){
      filterOnePole->TauUS = newTau * 1e6;
  }

  float FilterOnePole_input(FilterOnePole *filterOnePole, float inVal ){
    uint64_t time = micros();
    filterOnePole->ElapsedUS = (float)(time - filterOnePole->LastUS);   // cast to float here, for math
    filterOnePole->LastUS = time;                      // update this now

    // shift the data values
    filterOnePole->Ylast = filterOnePole->Y;
    filterOnePole->X = inVal;                          // this is now the most recent input value
  
    // filter value is controlled by a parameter called X
    // tau is set by the user in microseconds, but must be converted to samples here
    filterOnePole->TauSamps = filterOnePole->TauUS / filterOnePole->ElapsedUS;
  
    float ampFactor;
  #ifdef ARM_FLOAT
    ampFactor = expf( -1.0 / TauSamps );     // this is 1 if called quickly
  #else
    ampFactor = exp( -1.0 / filterOnePole->TauSamps );      // this is 1 if called quickly
  #endif
  
    filterOnePole->Y = (1.0 - ampFactor) * filterOnePole->X + ampFactor * filterOnePole->Ylast;     // set the new value

    return FilterOnePole_output(filterOnePole);
  }

float FilterOnePole_output(FilterOnePole* filterOnePole){
        // figure out which button to read
  switch (filterOnePole->FT) {
    case LOWPASS:         
      // return the last value
      return filterOnePole->Y; 
      break;
    case INTEGRATOR:      
      // using a lowpass, but normaize
      return filterOnePole->Y * (filterOnePole->TauUS/1.0e6);
      break;
    case HIGHPASS:       
      // highpass is the _difference_
      return filterOnePole->X - filterOnePole->Y;
      break;
    case DIFFERENTIATOR:
      // like a highpass, but normalize
      return (filterOnePole->X - filterOnePole->Y) / (filterOnePole->TauUS / 1.0e6);
      break;
    default:
      // should never get to here, return 0 just in case
      return 0;
  }
}
  
void FilterOnePole_setToNewValue(FilterOnePole *filterOnePole, float newVal ){ 	// resets the filter to a new value
	filterOnePole->Y = filterOnePole->Ylast = filterOnePole->X = newVal;
}

void FilterOnePoleCascade_init(FilterOnePoleCascade *filterOnePoleCascade, float riseTime, float initialValue){
	FILTER_TYPE ft = LOWPASS;
	FilterOnePole_init(&filterOnePoleCascade->Pole1, ft, 1.0, 0);
	FilterOnePole_init(&filterOnePoleCascade->Pole2, ft, 1.0, 0);

	FilterOnePoleCascade_setRiseTime(filterOnePoleCascade, riseTime );
	FilterOnePoleCascade_setToNewValue(filterOnePoleCascade, initialValue );
}  // rise time to step function, 10% to 90%
  
  // rise time is 10% to 90%, for a step input
void FilterOnePoleCascade_setRiseTime(FilterOnePoleCascade *filterOnePoleCascade, float riseTime ){
  float tauScale = 3.36;      // found emperically, by running test();

  FilterOnePole_setTau(&filterOnePoleCascade->Pole1, riseTime / tauScale);
  FilterOnePole_setTau(&filterOnePoleCascade->Pole2, riseTime / tauScale);
}
  
void FilterOnePoleCascade_setToNewValue(FilterOnePoleCascade *filterOnePoleCascade, float newVal ){
  FilterOnePole_setToNewValue(&filterOnePoleCascade->Pole1, newVal );

  FilterOnePole_setToNewValue(&filterOnePoleCascade->Pole2, newVal );
}
  
float FilterOnePoleCascade_input(FilterOnePoleCascade *filterOnePoleCascade, float inVal ){
  FilterOnePole_input(&filterOnePoleCascade->Pole2, FilterOnePole_input(&filterOnePoleCascade->Pole1, inVal ));
  return FilterOnePoleCascade_output(filterOnePoleCascade);
}
  
float FilterOnePoleCascade_output(FilterOnePoleCascade *filterOnePoleCascade){
  return FilterOnePole_output(&filterOnePoleCascade->Pole2);
}

uint64_t micros(){
	return TIM2->CNT;
}

/*FilterOnePole::FilterOnePole( FILTER_TYPE ft, float fc, float initialValue ) {
  setFilter( ft, fc, initialValue );
}

void FilterOnePole::setFilter( FILTER_TYPE ft, float fc, float initialValue ) {
  FT = ft;
  setFrequency( fc );

  Y = initialValue;
  Ylast = initialValue;
  X = initialValue;

  LastUS = micros();
}

float FilterOnePole::input( float inVal ) {
  long time = micros();
  ElapsedUS = float(time - LastUS);   // cast to float here, for math
  LastUS = time;                      // update this now

  // shift the data values
  Ylast = Y;
  X = inVal;                          // this is now the most recent input value
  
  // filter value is controlled by a parameter called X
  // tau is set by the user in microseconds, but must be converted to samples here
  TauSamps = TauUS / ElapsedUS;
  
  float ampFactor;
#ifdef ARM_FLOAT
  ampFactor = expf( -1.0 / TauSamps );     // this is 1 if called quickly
#else
  ampFactor = exp( -1.0 / TauSamps );      // this is 1 if called quickly
#endif
  
  Y = (1.0-ampFactor)*X + ampFactor*Ylast;     // set the new value

  return output();
}

void FilterOnePole::setFrequency( float newFrequency ) {
  setTau( 1.0/(TWO_PI*newFrequency ) ); // τ=1/ω
}

void FilterOnePole::setTau( float newTau ) {
  TauUS = newTau * 1e6;
}

float FilterOnePole::output() {
    // figure out which button to read
  switch (FT) {
    case LOWPASS:         
      // return the last value
      return Y; 
      break;
    case INTEGRATOR:      
      // using a lowpass, but normaize
      return Y * (TauUS/1.0e6);
      break;
    case HIGHPASS:       
      // highpass is the _difference_
      return X-Y;
      break;
    case DIFFERENTIATOR:
      // like a highpass, but normalize
      return (X-Y)/(TauUS/1.0e6);
      break;
    default:
      // should never get to here, return 0 just in case
      return 0;
  }
}
// stuff for filter2 (lowpass only)
// should be able to set a separate fall time as well
FilterOnePoleCascade::FilterOnePoleCascade( float riseTime, float initialValue ) {
  setRiseTime( riseTime );
  setToNewValue( initialValue );
}

void FilterOnePoleCascade::setRiseTime( float riseTime ) {
  float tauScale = 3.36;      // found emperically, by running test();

  Pole1.setTau( riseTime / tauScale );
  Pole2.setTau( riseTime / tauScale );
}

float FilterOnePoleCascade::input( float inVal  ) {
  Pole2.input( Pole1.input( inVal ));
  return output();
}

// clears out the values in the filter
void FilterOnePoleCascade::setToNewValue( float newVal ) {
  Pole1.setToNewValue( newVal );
  Pole2.setToNewValue( newVal );
}

float FilterOnePoleCascade::output() {
  return Pole2.output();
}

*/
