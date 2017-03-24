#include "mbed.h"
#include "rtos.h"
#include <stdlib.h>

#include "wiring.h"

// ======================================== GLOBAL CONSTANTS ========================================

class PidController {

public:
  
  PidController(float k_p, float k_i, float k_d, float max_out /*=0*/) :
    k_p_(k_p), k_i_(k_i), k_d_(k_d), max_out_(max_out), last_error_(0), last_de_dt_(0),
    integrated_error_(0) 
  {
  }

  float computeOutput(float reference, float measurement, float dt)
  {
    // convert time to seconds
    // dt *= 1000000;

    // compute error:
    float error = reference - measurement;
    
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
    else if (output < -max_out_) 
    {
      // clamp -- and DO NOT INTEGRATE ERROR (anti- reset windup)
      output = -max_out_; 
    } 
    else if (error < 0.2*measurement && error > -0.2*measurement)
    {
      integrated_error_ += error * dt; 
    }

    // save variables
    last_error_ = error;
    last_de_dt_ = de_dt;
    
    PRINT_DEBUG("P:%d, I:%d, D:%d",(int)(k_p_*error*1000000),(int)(k_i_*integrated_error_*1000000),(int)(k_d_*de_dt*1000000));

    //PRINT_DEBUG("Error: %d", (int)(error*1000));
//    PRINT_DEBUG("Output: %d", (int)(output*1000));
    
    return output;
  }

  void setParams(float k_p, float k_i, float k_d, float max_out)
  {
    k_p_ = k_p;
    k_i_ = k_i;
    k_d_ = k_d;
    max_out_ = max_out;
  }

  // void timeDifference(float time_end);


private:

  inline void reset(){
    last_error_ = 0.0; 
    last_de_dt_ = 0.0; 
    integrated_error_ = 0.0; 
  }

  float last_error_; 
  float last_de_dt_; 
  float integrated_error_; 

  float k_p_; 
  float k_i_; 
  float k_d_; 

  // If 0.0, then it is considered unlimited
  float max_out_;

  // float min_out_ = 0.0;

  // float time_start_;
  // float time_end_;

};

// ======================================== GLOBAL CONSTANTS ========================================

// Drive state to output table
const int8_t DRIVE_TABLE[] = {0x12,0x18,0x09,0x21,0x24,0x06,0x00,0x00};

// Mapping from interrupter inputs to sequential rotor states. 0x00 and 0x07 are not valid
const int8_t STATE_MAP[] = {0x07,0x05,0x03,0x04,0x01,0x00,0x02,0x08};  
// const int8_t STATE_MAP[] = {0x07,0x01,0x03,0x02,0x05,0x00,0x04,0x07}; // Alternative if phase order of input or drive is reversed

// Phase lead to make motor spin
const int8_t lead = -2;  // 2 for forwards, -2 for backwards

// Increment values for the ticks encoder
const int INC[2] = {-1, 1};

const int VEL_PERIOD = 100;     // in milliseconds

const float KP_VELOCITY_FAST = 0.021;
const float KI_VELOCITY_FAST = 0.000000005;
const float KD_VELOCITY_FAST = 0.000144;


const float KP_VELOCITY_SLOW = 0.015;;
const float KI_VELOCITY_SLOW = 0.000000002;
const float KD_VELOCITY_SLOW = 0.000024;


// ======================================== GLOBAL VARIABLES ========================================

// Value of the ticks encoder
volatile int tick = 0;  

volatile int8_t rotorState;

volatile float velocity = 0;

volatile int t_before = 0;
volatile int t_now = 0;
volatile int t_diff = 0; // revolutions/sec is 1/t_diff

volatile float pwm_duty_cycle = 0.5;
volatile int pwm_period = 400; // in microseconds

volatile float ref_vel;

volatile int rots;

volatile bool using_i1_velocity = false;

Timer t;

Thread thread_v(osPriorityNormal, 500);
Thread thread_spin(osPriorityNormal, 500);
Thread thread_vel_control;
// int pwm_on = 0.5;

// PidController vel_controller(0.01, 0.00000001, 0.1, 1.0);
PidController vel_controller(KP_VELOCITY_FAST, KI_VELOCITY_FAST, KD_VELOCITY_FAST, 1.0); //PID values from ZiglerNicholas [0.021, 0.07636363636363636, 0.0014437500000000002]
//PidController pos_controller(100.0, 0.0, 0.0 0.0 1.0);

int8_t orState = 0;    //Rotot offset at motor state 0

// ======================================== INTERRUPTS ========================================


inline void CHA_rise_isr() {
    tick += INC[CHB.read()];
}

    // Maybe faster?
    // int val = CHB.read();
    // tick += (1>>val);
    // tick -= (1>>!val);


inline void I1_rise_isr(){

    t_now = t.read_us();
    int t_diff_temp = t_now-t_before;
    if(t_diff_temp > 0.01){ // Ignore if the duration is too small (implying glitch)
        rots++;
        t_diff = t_now-t_before;
        t_before = t_now;

        if(using_i1_velocity){
            tick = rots*117;  
        }
    }
}




// ======================================== FUNCTION DEFINTIONS ========================================

//Set a given drive state
void motorOut(int8_t driveState){
    
    //Lookup the output byte from the drive state.
    int8_t driveOut = DRIVE_TABLE[driveState & 0x07];

    //Turn off Vm
    if (~driveOut & 0x02) L1H.write(1);
    if (~driveOut & 0x08) L2H.write(1);
    if (~driveOut & 0x20) L3H.write(1);

   if (~driveOut & 0x01) L1L.write(0);
   if (~driveOut & 0x04) L2L.write(0);
   if (~driveOut & 0x10) L3L.write(0);

    //Then turn on gnd
   if (driveOut & 0x01) L1L.write(1);
   if (driveOut & 0x02) L1H.write(1-pwm_duty_cycle);

   if (driveOut & 0x04) L2L.write(1);
   if (driveOut & 0x08) L2H.write(1-pwm_duty_cycle);

   if (driveOut & 0x10) L3L.write(1);
   if (driveOut & 0x20) L3H.write(1-pwm_duty_cycle);

    // and turn on Vm
}


//Convert photointerrupter inputs to a rotor state
inline int8_t readRotorState(){
    return STATE_MAP[I1 + 2*I2 + 4*I3];
}


//Basic synchronisation routine    
int8_t motorHome() {
    //Put the motor in drive state 0 and wait for it to stabilise

    L2H.write(1);
    L3H.write(1);
    L1L.write(0);
    L2L.write(0);

    L1H.write(0);
    L3L.write(1);

    bool stable = false;
    int prev_state, curr_state;

    prev_state = readRotorState();

    while(!stable){
        wait(1.0);
        curr_state = readRotorState();
        stable = (velocity == 0) && (curr_state == prev_state);
        prev_state = curr_state;
    }

    //Get the rotor state
    return readRotorState();
}

void set_pwm(int p){
    L1H.period_us(p);
    L2H.period_us(p);
    L3H.period_us(p);
}


// ======================================== THREADS ========================================
void spin(){

    //Initialise the serial port
    int8_t intState = 0;
    int8_t intStateOld = -1;
    while(1){
        intState = readRotorState();
        if (intState != intStateOld) {
            intStateOld = intState;
            motorOut((intState-orState+lead+6)%6); //+6 to make sure the remainder is positive
        }
    }
}


void velocity_thread(){
    int tick_before, tick_after;
    float result;
    while(1){
        if (velocity < 10 && velocity > -10){
          using_i1_velocity = false;
          tick_before = tick;
          Thread::wait(VEL_PERIOD);
          tick_after = tick;
          velocity = 1000.0/(VEL_PERIOD)*(tick_after-tick_before)/117.0;
        }
        else {
          using_i1_velocity = true;
          result = 1000000.0/(float)t_diff;
          velocity = result;
          Thread::wait(VEL_PERIOD);
        }

    }

}


void velocity_control_thread(){
    float vel_copy;
    while(1){
        pwm_duty_cycle = vel_controller.computeOutput(ref_vel, velocity, VEL_PERIOD*1000);
//        PRINT_DEBUG("Duty: 0.%03d",(int)(pwm_duty_cycle*1000))
        // PRINT_DEBUG("%d.%03d",((int)velocity),abs((int)(velocity*1000)%1000));
        Thread::wait(VEL_PERIOD);
    }
}


// ======================================== MAIN ========================================


int main() {
    // =============================
    // Test MAIN
    // =============================

    CHA.rise(&CHA_rise_isr);
    I1.rise(&I1_rise_isr);

    // =============================
    // Original MAIN
    // =============================

    pc.printf("Hello\n\r");

    set_pwm(pwm_period);

    //orState is subtracted from future rotor state inputs to align rotor and motor states
    
    thread_v.start(velocity_thread);

    orState = motorHome();

    // // Begin threads
    t.start();

    ref_vel = 16.0;

    if(ref_vel < 10.0){
      vel_controller.setParams(KP_VELOCITY_FAST, KI_VELOCITY_FAST, KD_VELOCITY_FAST, 1.0);
    }
    else {
      vel_controller.setParams(KP_VELOCITY_SLOW, KI_VELOCITY_SLOW, KD_VELOCITY_SLOW, 1.0);
    }

    thread_spin.start(spin);
    thread_vel_control.start(velocity_control_thread);


    while (1){
        // set_pwm(239);

        // PRINT_DEBUG("Ticks: %d",tick)
        // PRINT_DEBUG("Rots: %d",rots)
        Thread::wait(100);
        // set_pwm(213);
        // PRINT_DEBUG("Vel from Tick: %d.%03d",velocity/1000,abs(velocity%1000));
        // PRINT_DEBUG("Ticks: %d",tick)
        // Thread::wait(1000);
        // set_pwm(190);
        // PRINT_DEBUG("Vel from Tick: %d.%03d",velocity/1000,abs(velocity%1000));
        // PRINT_DEBUG("Ticks: %d",tick)
        // Thread::wait(1000);
        // set_pwm(179);
        // PRINT_DEBUG("Vel from Tick: %d.%03d",velocity/1000,abs(velocity%1000));
        // PRINT_DEBUG("Ticks: %d",tick)
        // Thread::wait(1000);
    }
}
