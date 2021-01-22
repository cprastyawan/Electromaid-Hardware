#include "RunningStatistics.h"
#include "FloatDefine.h"

void RunningStatistics_input(RunningStatistics *runningStatistics, float inVal ){
  FilterOnePoleCascade_input(&runningStatistics->averageValue, inVal);
  FilterOnePoleCascade_input(&runningStatistics->averageSquareValue, inVal * inVal);
}
  
  // constructor
void RunningStatistics_init(RunningStatistics *runningStatistics){
	FilterOnePoleCascade_init(&runningStatistics->averageValue, 1.0, 0);
	FilterOnePoleCascade_init(&runningStatistics->averageSquareValue, 1.0, 0);

    RunningStatistics_setWindowSecs(runningStatistics, 1 );   // setup with one second average
    RunningStatistics_setInitialValue(runningStatistics, 0, 0); // start with zero
}
  
void RunningStatistics_setWindowSecs( RunningStatistics *runningStatistics, float windowSecs ){
  runningStatistics->AverageSecs = windowSecs;
  
  FilterOnePoleCascade_setRiseTime( &runningStatistics->averageValue, runningStatistics->AverageSecs );
  FilterOnePoleCascade_setRiseTime( &runningStatistics->averageSquareValue, runningStatistics->AverageSecs );
}
  
void RunningStatistics_setInitialValue(RunningStatistics *runningStatistics, float initialMean, float initialSigma){

  FilterOnePoleCascade_setToNewValue(&runningStatistics->averageValue, initialMean );
  FilterOnePoleCascade_setToNewValue(&runningStatistics->averageSquareValue, (initialMean * initialMean) + (initialSigma * initialSigma));
}
    
float RunningStatistics_mean(RunningStatistics *runningStatistics){
    return FilterOnePoleCascade_output(&runningStatistics->averageValue);
}
  
float RunningStatistics_variance(RunningStatistics *runningStatistics){
  float var = FilterOnePoleCascade_output(&runningStatistics->averageSquareValue) - (FilterOnePoleCascade_output(&runningStatistics->averageValue) * FilterOnePoleCascade_output(&runningStatistics->averageValue));
  //float var = averageSquareValue.output() - averageValue.output()*averageValue.output();
  // because of the filtering, it's possible that this could be negative ... check!
  if( var < 0 ) var = 0;
    
  return var;
}
  
float RunningStatistics_sigma(RunningStatistics *runningStatistics){
  #ifdef ARM_FLOAT
  return sqrtf(RunningStatistics_variance(runningStatistics));
#else
  return sqrt(RunningStatistics_variance(runningStatistics));
#endif
}
  
float RunningStatistics_CV(RunningStatistics *runningStatistics){
  static const float maxCV = 1e3;
  float meanTmp = RunningStatistics_mean(runningStatistics);
  
  // prevent divide by zero
  if( meanTmp == 0 ) return maxCV;
  else               return RunningStatistics_sigma(runningStatistics) / meanTmp;
}

/*RunningStatistics::RunningStatistics() {
  setWindowSecs( 1 );   // setup with one second average
  setInitialValue( 0 ); // start with zero
}

void RunningStatistics::setWindowSecs( float windowSecs ) {
  AverageSecs = windowSecs;
  
  averageValue.setRiseTime( AverageSecs );
  averageSquareValue.setRiseTime( AverageSecs );
}

void RunningStatistics::setInitialValue( float initialMean, float initialSigma ) {
  averageValue.setToNewValue( initialMean );
  averageSquareValue.setToNewValue( sq(initialMean) + sq(initialSigma ) );
}

void RunningStatistics::input( float inVal ) {
  averageValue.input(inVal);              // calculates running average
  averageSquareValue.input(inVal*inVal);  // calculates running average of square
}
  
float RunningStatistics::mean() {
  return averageValue.output();
}
  
float RunningStatistics::variance() {
  float var = averageSquareValue.output() - averageValue.output()*averageValue.output();
    
  // because of the filtering, it's possible that this could be negative ... check!
  if( var < 0 ) var = 0;
    
  return var;
}
  
float RunningStatistics::sigma() {
  
#ifdef ARM_FLOAT
  return sqrtf(variance());
#else
  return sqrt(variance());
#endif

}

float RunningStatistics::CV() {
  static const float maxCV = 1e3;
  float meanTmp = mean();
  
  // prevent divide by zero
  if( meanTmp == 0 ) return maxCV;
  else               return sigma() / meanTmp;
}*/
