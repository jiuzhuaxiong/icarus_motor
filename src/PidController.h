#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include "mbed.h"

class PidController {

public:
  
  PidController(float k_p, float k_i, float k_d, float max_out=0);

  float computeOutput(float reference, float output, float dt);

  void timeDifference(float time_start, float time_end);
  void timeDifference(float time_end);


private:

  void reset();


  float last_error_ = 0.0; 
  float last_de_dt_ = 0.0; 
  float integrated_error_ = 0.0; 

  float k_p_; 
  float k_i_; 
  float k_d_; 

  // If 0.0, then it is considered unlimited
  float max_out_;
  float min_out_ = 0.0;

  mbed::Timer

};

#endif
