#include "PidController.h"

PidController::PidController(float k_p, float k_i, float k_d, float max_out /*=1.0*/) :
  k_p_(k_p), k_i_(k_i), k_d_(k_d), max_out_(max_out), min_out_(0.0), 
  last_error_(0.0), last_de_dt_(0.0), integrated_error_(0.0) 
{
}

// float PidController::computeOutput(float reference, float measurement, float dt, float pwm_duty)
// {
//   // convert time to seconds
//   // dt *= 1000000;

//   // compute error:
//   float error = reference - measurement;
  
//   // compute error derivative:
//   float de_dt = (error - last_error_ ) / dt;

//   // Do smoothing (numeric derivatives are noisy):
//   de_dt = 0.7 * last_de_dt_ + 0.3 * de_dt;

//   // compute output:
//   float output = k_p_ * error + k_i_ * integrated_error_ + k_d_ * de_dt;

//   // Active Low output
//   if (reference > 0.0) pwm_duty -= output;
//   else                 pwm_duty += output;

//   // Active High output
//   // if (reference > 0.0) pwm_duty += output;
//   // else                 pwm_duty -= output;

//   // Check for saturation - anti-reset windup
//   if (pwm_duty > max_out_) {
//     // clamp -- and DO NOT INTEGRATE ERROR (anti- reset windup)
//     pwm_duty = max_out_; 
//   }
//   else if (pwm_duty < min_out_) 
//   {
//     // clamp -- and DO NOT INTEGRATE ERROR (anti- reset windup)
//     pwm_duty = min_out_; 
//   } 
//   else if (error < 0.2*measurement && error > -0.2*measurement)
//   {
//     integrated_error_ += error * dt; 
//   }

//   // save variables
//   last_error_ = error;
//   last_de_dt_ = de_dt;
  
//   // PRINT_DEBUG("Ref: %d.%03d, Vel: %d.%03d Out: %d.%03d Err: %de-9 P: %de-9 I: %de-9 D: %de-9",
//   //   (int)(reference),
//   //   abs((int)(reference*1000)%1000),
//   //   (int)(measurement),
//   //   abs((int)(measurement*1000)%1000),
//   //   (int)(pwm_duty),
//   //   abs((int)(pwm_duty*1000)%1000),
//   //   (int)(error*1000000000),
//   //   (int)(k_p_*error*1000000000),
//   //   (int)(k_i_*integrated_error_*1000000000),
//   //   (int)(k_d_*de_dt*1000000000)
//   // );

//   PRINT_DEBUG("Vel: %d.%03d Err: %de-9 P: %de-9 I: %de-9 D: %de-9 Ref: %d.%03d Out: %d.%03d",
//     (int)(measurement),
//     abs((int)(measurement*1000)%1000),
//     (int)(error*1000000000),
//     (int)(k_p_*error*1000000000),
//     (int)(k_i_*integrated_error_*1000000000),
//     (int)(k_d_*de_dt*1000000000),
//     (int)(reference),
//     abs((int)(reference*1000)%1000),
//     (int)(pwm_duty),
//     abs((int)(pwm_duty*1000)%1000)
//   );

//   // PRINT_DEBUG("Error: %d", (int)(error*1000));
//   // PRINT_DEBUG("pwm_duty: %d", (int)(pwm_duty*1000));
  
//   return pwm_duty;
// }

float PidController::computeOutput(float reference, float measurement, float dt)
{
  // convert time to seconds
  // dt *= 1000000;

  reference = fabs(reference);
  measurement = fabs(measurement);

  // compute error:
  float error = reference - measurement;
  
  // compute error derivative:
  float de_dt = (error - last_error_ ) / dt;

  // Do smoothing (numeric derivatives are noisy):
  de_dt = 0.7 * last_de_dt_ + 0.3 * de_dt;

  // compute output:
  float output = k_p_ * error + k_i_ * integrated_error_ + k_d_ * de_dt;

  // output = fabs(output);

  // Check for saturation - anti-reset windup
  if (output > max_out_) {
    // clamp -- and DO NOT INTEGRATE ERROR (anti- reset windup)
    output = max_out_; 
  }
  else if (output < min_out_) 
  {
    // clamp -- and DO NOT INTEGRATE ERROR (anti- reset windup)
    output = min_out_; 
  } 
  else if (error < 0.5*measurement && error > -0.5*measurement)
  {
    integrated_error_ += error * dt; 
  }

  // save variables
  last_error_ = error;
  last_de_dt_ = de_dt;
  
   PRINT_DEBUG("Vel: %d.%03d Err: %de-9 P: %de-9 I: %de-9 D: %de-9 Ref: %d.%03d Out: %d.%03d",
     (int)(measurement),
     abs((int)(measurement*1000)%1000),
     (int)(error*1000000000),
     (int)(k_p_*error*1000000000),
     (int)(k_i_*integrated_error_*1000000000),
     (int)(k_d_*de_dt*1000000000),
     (int)(reference),
     abs((int)(reference*1000)%1000),
     (int)(output),
     abs((int)(output*1000)%1000)
   );

  // PRINT_DEBUG("Error: %d", (int)(error*1000));
  // PRINT_DEBUG("Output: %d", (int)(output*1000));
  
  return output;
}


void PidController::setParams(float k_p, float k_i, float k_d, float max_out /*=1.0*/)
{
  k_p_ = k_p;
  k_i_ = k_i;
  k_d_ = k_d;
  max_out_ = max_out;
}

