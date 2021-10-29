#include "PID.h"
#include <math.h>
#include <iostream>

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_, double min_output_, double max_output_) {

  p_error = 0;
  i_error = 0;
  d_error = 0;
  
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
  
  prev_cte = 0;
  
  min_output = min_output_;
  max_output = max_output_;
}

void PID::UpdateError(double cte) {
  // Update PID errors based on cte.
  
  p_error = cte;
  d_error = cte - prev_cte;
  i_error += cte;
    
  prev_cte = cte;
}

double PID::TotalError() {
  // Return the total error
  
  return - Kp * p_error - Kd * d_error - Ki * i_error;
}

double PID::GetValue(double cte){
  // Return new value restricted between [min_output,max_output]
  UpdateError(cte);
  return std::max(std::min(TotalError(),max_output), min_output);
}