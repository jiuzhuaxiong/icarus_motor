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
    // de_dt = 0.5 * last_de_dt_ + 0.5 * de_dt;

    // compute output:
    float output = k_p_ * error + k_i_ * integrated_error_ + k_d_ * de_dt;

    
    // Check for saturation - anti-reset windup
    if (output > max_out_) {
      // clamp -- and DO NOT INTEGRATE ERROR (anti- reset windup)
      output = max_out_; 
    }
    else if (output < 0) 
    {
      // clamp -- and DO NOT INTEGRATE ERROR (anti- reset windup)
      output = 0; 
    } 
    else if (error < 0.2*measurement && error > -0.2*measurement)
    {
      integrated_error_ += error * dt; 
    }

    // save variables
    last_error_ = error;
    last_de_dt_ = de_dt;
    
    // PRINT_DEBUG("Vel: %d.%03d Err: %d.%03d P: %d.%03d I: %d.%09d D: %d.%09d",
    //   (int)(measurement),
    //   abs((int)(measurement*1000)%1000),
    //   (int)(error),
    //   abs((int)(error*1000)%1000),
    //   (int)(k_p_*error),
    //   abs((int)(k_p_*error*1000)%1000),
    //   (int)(k_i_*integrated_error_),
    //   abs((int)(k_i_*integrated_error_*1000000000)%1000000000),
    //   (int)(k_d_*de_dt),
    //   abs((int)(k_d_*de_dt*1000000000)%1000000000)
    // );

    PRINT_DEBUG("Vel: %d.%03d Err: %de-9 P: %de-9 I: %de-9 D: %de-9",
      (int)(measurement),
      abs((int)(measurement*1000)%1000),
      (int)(error*1000000000),
      (int)(k_p_*error*1000000000),
      (int)(k_i_*integrated_error_*1000000000),
      (int)(k_d_*de_dt*1000000000)
    );

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
  float min_out_ = 0.0;

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

const int VEL_PERIOD = 30;     // in milliseconds

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
volatile int t_diff = 10000000000; // revolutions/sec is 1/t_diff
volatile int t_diff_temp=0;

volatile float pwm_duty_cycle = 0.5;
volatile int pwm_period = 400; // in microseconds

volatile float ref_vel;

volatile int rots;

volatile float R, V;  // Command line arguments
volatile bool r_updated, v_updated;  // True if R, V were updated during the last command

Timer t;

Thread thread_v(osPriorityNormal, 500);
Thread thread_spin(osPriorityNormal, 500);
Thread thread_vel_control;
//Thread thread_parser(osPriorityNormal, 500);
Thread thread_parser(osPriorityNormal, 500);
// int pwm_on = 0.5;

// PidController vel_controller(0.01, 0.00000001, 0.1, 1.0);
PidController vel_controller(0.013, 0.03, 0.000742, 1.0); //PID values from ZiglerNicholas [0.021, 0.07636363636363636, 0.0014437500000000002]
//PidController pos_controller(100.0, 0.0, 0.0 0.0 1.0);
// PidController vel_controller(0.018, 0.109, 0.000742, 1.0); //PID values from ZiglerNicholas [0.021, 0.07636363636363636, 0.0014437500000000002]

int8_t orState = 0;    //Rotot offset at motor state 0

// ======================================== INTERRUPTS ========================================


inline void CHA_rise_isr() {
    tick += INC[CHB.read()];
    // PRINT_DEBUG("Tick: %d", tick);
}

    // Maybe faster?
    // int val = CHB.read();
    // tick += (1>>val);
    // tick -= (1>>!val);

inline void I1_rise_isr(){
    t_now = t.read_us();
    t_diff_temp = t_now-t_before;
    if(t_diff_temp > 10000){ // Ignore if the duration is too small (implying glitch)
      t_diff = t_diff_temp;
      t_before = t_now;
      rots++;
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
    PRINT_DEBUG("motor home");

    L2H.write(1);
    PRINT_DEBUG("L2H Off");
    L3H.write(1);
    PRINT_DEBUG("L3H Off");
    L1L.write(0);
    PRINT_DEBUG("L1L Off");
    L2L.write(0);
    PRINT_DEBUG("L2L Off");


    L1H.write(0);
    PRINT_DEBUG("L1H On");
    L3L.write(1);
    PRINT_DEBUG("L3L On");


    PRINT_DEBUG("Rotate to home state");

    bool stable = false;
    int prev_state, curr_state;

    PRINT_DEBUG("Reading rotor state");
    prev_state = readRotorState();
    PRINT_DEBUG("Read rotor state");

    while(!stable){
        PRINT_DEBUG("Waiting to stabilise");
        wait(1.0);
        curr_state = readRotorState();
        stable = (curr_state == prev_state);
        // stable = (curr_state == 0) && (curr_state == prev_state);

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
    int8_t intStateOld = 0;
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
          tick_before = tick;
          Thread::wait(VEL_PERIOD);
          tick_after = tick;
          velocity = 1000.0/(VEL_PERIOD)*(tick_after-tick_before)/117.0;
        }
        else {
          velocity = 1000000.0/(float)t_diff;
          Thread::wait(VEL_PERIOD);
        }

    }

}


void velocity_control_thread(){
    while(1){
        pwm_duty_cycle = vel_controller.computeOutput(ref_vel, velocity, (float)VEL_PERIOD/1000.0);
        // PRINT_DEBUG("Duty: 0.%03d",(int)(pwm_duty_cycle*1000))
        Thread::wait(VEL_PERIOD);
    }
}

// ======================================== PARSER ========================================

void parse_input_thread(){
    float r, v;
    bool cmd, r_cmd, v_cmd;
    cmd = parseNote(input, n, d, s);
    cmd = parseCmd(input, r, v, r_cmd, v_cmd);

}


bool parseCmd(char* in, float& r, float& v, bool& r_cmd, bool& v_cmd,){

    char buf_r[7] = {0};
    char buf_v[7] = {0};
    if((in[0] == 'R') && (strchr(in,'V') != NULL)){ // R and V command
        
        int pos;
        for(int i=0;i<=strlen(in);i++){
            if(in[i] == 'V'){
                pos = i;
            }
        }

        for(int i=1;i<pos;i++){
            buf_r[i-1] = in[i];
        }
        for(int i=pos+1;i<strlen(in);i++){
            buf_v[i-pos-1] = in[i];
        }
        printf("buf_r: %s, buf_v: %s", buf_r, buf_v);

        r = atof(buf_r);
        v = atof(buf_v);

        if (r>999.99 || r<-999.99){
            return false;
        }
        if (v>999.99 || v<-999.99){
            return false;
        }

        if(v < 0){
            v = 0 - v;
        }
        r_cmd = true;
        v_cmd = true;
        
        return true;
    }
    if(in[0] == 'V'){ // Only V command
        for(int i=1;i<=strlen(in);i++){
            buf_v[i-1] = in[i];
        }
        printf("buf_v: %s\r\n", buf_v);
        v = atof(buf_v);

        if (v>999.99 || v<-999.99){
            return false;
        }

        r_cmd = false;
        v_cmd = true;

        return true;
    }
    else if(in[0] == 'R'){ // Only R command
        for(int i=1;i<=strlen(in);i++){
            buf_r[i-1] = in[i];
        }
        printf("buf_r: %s\r\n", buf_r);
        r = atof(buf_r);

        if (r>999.99 || r<-999.99){
            return false;
        }

        r_cmd = true;
        v_cmd = false;

        return true;
    }
    return false;   // Used for invalid input commands, deal with at higher level
}


bool parseNote(char* in, int* note, int* duration, int& size){
    int notes[] = {142, 127, 239, 213, 190, 179, 159}; // A B C D E F G
    int sharps[] = {134, 127, 225, 201, 179, 169, 150}; // A# B C# D# F F# G#
    int flats[] = {150, 134, 245, 225, 201, 190, 190}; //A^ B^ C^ D^ E^ E G^

    if(in[0] == 'T'){
        int i=1;
        int j=0;
        while(i<strlen(in)){
            if(in[i+1] == '#'){
                int idx = in[i] & 0x0F;
                note[j] = sharps[idx-1];
                duration[j] = int(in[i+2]-'0');
                i+=3;
                j+=1;
            }
            else if(in[i+1] == '^'){
                int idx = in[i] & 0x0F;
                note[j] = flats[idx-1];
                duration[j] = int(in[i+2]-'0');
                i+=3;
                j+=1;
            }
            else{
                int idx = in[i] & 0x0F;
                printf("Idx: %d\n", idx);
                note[j] = notes[idx-1];
                duration[j] = int(in[i+1]-'0');
                i+=2;
                j+=1;
            }
        }
        printf("Size: %d\n", j);
        size = j;
        return true;
    }
    return false;
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

    PRINT_DEBUG("Setting PWM");
    set_pwm(pwm_period);

    //orState is subtracted from future rotor state inputs to align rotor and motor states
    

    PRINT_DEBUG("Starting velocity thread");

    thread_v.start(velocity_thread);


    PRINT_DEBUG("Synchronising state");
    orState = motorHome();


    PRINT_DEBUG("Starting timer");
    // // Begin threads
    t.start();

    ref_vel = 20.0;

    // if(ref_vel < 10.0){
    //   vel_controller.setParams(KP_VELOCITY_FAST, KI_VELOCITY_FAST, KD_VELOCITY_FAST, 1.0);
    // }
    // else {
    //   vel_controller.setParams(KP_VELOCITY_SLOW, KI_VELOCITY_SLOW, KD_VELOCITY_SLOW, 1.0);
    // }

    thread_spin.start(spin);
    thread_vel_control.start(velocity_control_thread);


    while (1){
        // set_pwm(239);
        // PRINT_DEBUG("Tick: %d", tick);
        // Thread::wait(100);
        // PRINT_DEBUG("Vel:%d.%03d",(int)(velocity),abs((int)(velocity*1000)%1000));
        // PRINT_DEBUG("Ticks: %d",tick);
        // PRINT_DEBUG("Rots: %d",rots)
        Thread::wait(100);
    }
}
