#include "PidController.h"

PidController::PidController(float k_p, float k_i, float k_d, float max_out /*=0*/) :
  k_p_(k_p), k_i_(k_i), k_d_(k_d), max_out_(max_out)
{
}


float PidController::computeOutput(float reference, float output, float dt)
{
  // convert time to seconds
  dt *= 1000000;

  // compute error:
  float error = reference - output;
  
  // compute error derivative:
  float de_dt = (error - last_error_ ) / dt;

  // Do smoothing (numeric derivatives are noisy):
  de_dt = 0.8 * last_de_dt_ + 0.2 * de_dt;

  // compute output:
  float output = k_p_ * error + k_i_ * integrated_error_ + k_d_ * de_dt;
  
  // Check for saturation - anti-reset windup
  if (output > max_out_) {
    // clamp -- and DO NOT INTEGRATE ERROR (anti- reset windup)
    output = max_out_; 
  }
  else if (output < -max_out) 
  {
    // clamp -- and DO NOT INTEGRATE ERROR (anti- reset windup)
    output = -max_out; 
  } 
  else 
  {
    integrated_error_ += error * dt; 
  }

  // save variables
  last_error_ = error;
  last_de_dt_ = de_dt;

  return output;
}

void PidController::set_params(float k_p, float k_i, float k_d, float max_out);
{
  k_p_ = k_p;
  k_i_ = k_i;
  k_d_ = k_d;
  max_out_ = max_out;
}

// float PidController::computeOutput(float reference, float output)
// {
//   return computeOutput(reference, output, time_end_ - time_start_)
// }

// inline void PidController::timeDifference(float time_end)
// {
//   time_end_ = time_end;
// }

// inline void PidController::timeDifference(float time_start, float time_end)
// {
//   time_start_ = time_start;
//   time_end_ = time_end;
// }
