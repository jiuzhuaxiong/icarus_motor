#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include "mbed.h"
#include "wiring.h"

class PidController 
{

public:
  
  PidController(float k_p, float k_i, float k_d, float max_out=1.0);

  float computeOutput(float reference, float measurement, float dt);

  void setParams(float k_p, float k_i, float k_d, float max_out=1.0);

  inline void reset(){
    last_error_ = 0.0; 
    last_de_dt_ = 0.0; 
    integrated_error_ = 0.0; 
  }


private:

  float k_p_; 
  float k_i_; 
  float k_d_; 

  float max_out_;
  float min_out_;

  float last_error_; 
  float last_de_dt_; 
  float integrated_error_; 

  // float time_start_;
  // float time_end_;

};



#endif
