#ifndef RunningStatistics_h
#define RunningStatistics_h

#include "FilterOnePole.h"

typedef struct{
  // in statistics, SigmaSqr is:
  //   σ^2 = <x^2> - <x>^2
  // averages can be taken by low-pass smoothing with a (two-pole) filter
  
  float AverageSecs;   // seconds to average over
  
  FilterOnePoleCascade averageValue;
  FilterOnePoleCascade averageSquareValue;
  
}RunningStatistics;

void RunningStatistics_input(RunningStatistics *runningStatistics, float inVal );
  
  // constructor
void RunningStatistics_init(RunningStatistics *runningStatistics);
  
void RunningStatistics_setWindowSecs( RunningStatistics *runningStatistics, float windowSecs);
  
void RunningStatistics_setInitialValue(RunningStatistics *runningStatistics, float initialMean, float initialSigma);
    
float RunningStatistics_mean(RunningStatistics *runningStatistics);
  
float RunningStatistics_variance(RunningStatistics *runningStatistics);
  
float RunningStatistics_sigma(RunningStatistics *runningStatistics);
  
float RunningStatistics_CV(RunningStatistics *runningStatistics);

#endif
