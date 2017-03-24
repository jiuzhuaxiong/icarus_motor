#include "mbed.h"
#include "rtos.h"
#include <stdlib.h>

#include "wiring.h"

// ======================================== GLOBAL CONSTANTS ========================================

class PidController {

public:
  
  PidController(float k_p, float k_i, float k_d, float max_out=1.0) :
    k_p_(k_p), k_i_(k_i), k_d_(k_d), max_out_(max_out), min_out_(0.0), 
    last_error_(0.0), last_de_dt_(0.0), integrated_error_(0.0) 
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
    de_dt = 0.7 * last_de_dt_ + 0.3 * de_dt;

    // compute output:
    float output = k_p_ * error + k_i_ * integrated_error_ + k_d_ * de_dt;

    
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
    else if (error < 0.2*measurement && error > -0.2*measurement)
    {
      integrated_error_ += error * dt; 
    }

    // save variables
    last_error_ = error;
    last_de_dt_ = de_dt;
    
    PRINT_DEBUG("Vel: %d.%03d Err: %de-9 P: %de-9 I: %de-9 D: %de-9",
      (int)(measurement),
      abs((int)(measurement*1000)%1000),
      (int)(error*1000000000),
      (int)(k_p_*error*1000000000),
      (int)(k_i_*integrated_error_*1000000000),
      (int)(k_d_*de_dt*1000000000)
    );

    // PRINT_DEBUG("Error: %d", (int)(error*1000));
    // PRINT_DEBUG("Output: %d", (int)(output*1000));
    
    return output;
  }

  void setParams(float k_p, float k_i, float k_d, float max_out=1.0)
  {
    k_p_ = k_p;
    k_i_ = k_i;
    k_d_ = k_d;
    max_out_ = max_out;
  }

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


const float VEL_THRESH = 10.0;

// CONTROLLER PARAMETERS
const float KP_VELOCITY_FAST = 0.013;
const float KI_VELOCITY_FAST = 0.01;
const float KD_VELOCITY_FAST = 0.000742;


const float KP_VELOCITY_SLOW = 0.015;;
const float KI_VELOCITY_SLOW = 0.000000002;
const float KD_VELOCITY_SLOW = 0.000024;

// NOTES DATA
const uint8_t notes[7] = {142, 127, 239, 213, 190, 179, 159}; // A B C D E F G
const uint8_t sharps[7] = {134, 127, 225, 201, 179, 169, 150}; // A# B C# D# F F# G#
const uint8_t flats[7] = {150, 134, 245, 225, 201, 190, 190}; //A^ B^ C^ D^ E^ E G^



// ======================================== GLOBAL VARIABLES ========================================

// Value of the ticks encoder
volatile int tick = 0;  

volatile int8_t rotorState;

volatile float velocity = 0;

volatile int t_before = 0;
volatile int t_now = 0;
volatile int t_diff = 100000000; // revolutions/sec is 1/t_diff
volatile int t_diff_temp=0;

volatile float pwm_duty_cycle = 1;
volatile int pwm_period = 400; // in microseconds

volatile float ref_vel;

volatile int rots;

volatile float R = 0;
volatile float V = 0;  // Command line arguments
volatile bool r_cmd = false, v_cmd = false, n_cmd = false;  // True if R, V were updated during the last command
volatile uint8_t* N;
volatile uint8_t* D;
volatile int8_t melody_size=0; // Size of N and D

Timer t;

Thread thread_v(osPriorityNormal, 500);
Thread thread_spin(osPriorityNormal, 500);
Thread thread_vel_control;
//Thread thread_music;

// Thread thread_parser(osPriorityNormal, 700);
// int pwm_on = 0.5;

// PidController vel_controller(0.01, 0.00000001, 0.1);
//PID values from ZiglerNicholas [0.021, 0.07636363636363636, 0.0014437500000000002]
PidController vel_controller(KP_VELOCITY_FAST, KI_VELOCITY_FAST, KD_VELOCITY_FAST); 
//PidController pos_controller(100.0, 0.0, 0.0 0.0 1.0);
// PidController vel_controller(0.018, 0.109, 0.000742); //PID values from ZiglerNicholas [0.021, 0.07636363636363636, 0.0014437500000000002]



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

inline void I1_isr_rise(){
    if(I2){
        t_now = t.read_us();
        t_diff_temp = t_now-t_before;
        if(t_diff_temp > 10000){ // Ignore if the duration is too small (implying glitch)
          t_diff = t_diff_temp;
          t_before = t_now;
          rots++;
        }
    }
}

inline void I1_isr_fall(){
    if(I2){
        t_now = t.read_us();
        t_diff_temp = t_now-t_before;
        if(t_diff_temp > 10000){ // Ignore if the duration is too small (implying glitch)
          t_diff = -t_diff_temp;
          t_before = t_now;
          rots++;
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
    float curr_velocity = 0;
    int tick_before, tick_after;
    while(1){
        if (velocity < VEL_THRESH && velocity > -VEL_THRESH){
            tick_before = tick;
            Thread::wait(VEL_PERIOD);
            tick_after = tick;
            curr_velocity = 1000.0/(VEL_PERIOD)*(tick_after-tick_before)/117.0;
            velocity = 0.2*curr_velocity +0.8*velocity;

        }
        else {
            curr_velocity = 1000000.0/(float)t_diff; // 1 revolutions * 10^6 pecoseconds
            velocity = 0.2*curr_velocity +0.8*velocity;
            Thread::wait(VEL_PERIOD);
        }

    }

}


void velocity_control_thread(){
    while(1){
        pwm_duty_cycle = vel_controller.computeOutput(V, velocity, (float)VEL_PERIOD/1000.0);
        // PRINT_DEBUG("Duty: 0.%03d",(int)(pwm_duty_cycle*1000))
        Thread::wait(VEL_PERIOD);
    }
}

// ======================================== PARSER ========================================

bool parseCmd(char* in, float& r, float& v, bool& r_cmd, bool& v_cmd){
    char buf_r[7] = {0};
    char buf_v[7] = {0};
    char* v_pos = strchr(in,'V');
    
    char *r_val_start, *v_val_start;
    int r_val_len, v_val_len;
    
    if((in[0] == 'R') && (v_pos != NULL)){ // R and V command
        r_val_start = in + 1;
        r_val_len = v_pos - r_val_start;
        v_val_len = (in + strlen(in)) - (v_pos+1);

        r_cmd = true;
        v_cmd = true;
    }
    // Only R command
    else if(in[0] == 'R'){ 
        r_val_start = in + 1;
        r_val_len = (in + strlen(in)) - r_val_start;
 
        r_cmd = true;
        v_cmd = false;
    }
    // Only V command
    else if(in[0] == 'V'){ 
        v_val_start = in + 1;
        v_val_len = (in + strlen(in)) - v_val_start;
        
        r_cmd = false;
        v_cmd = true;
    }

    // Used for invalid input commands, deal with at higher level
    if(!r_cmd && !v_cmd) return false;

    if(r_cmd){
        // Copy the R value in buf_r
        memcpy( buf_r, r_val_start, r_val_len);
        r = atof(buf_r);
        //if (r>999.99 || r<-999.99)  return false;
    }

    if(v_cmd){
        // Copy the V value in buf_v
        memcpy( buf_v, v_val_start, v_val_len);
        v = atof(buf_v);
        // Make the v positive if r was also specified
        if(v < 0.0 && r_cmd)     v = 0.0 - v;

        //if (v>999.99 || v<-999.99)  return false;
    }

    return true;   
}


bool parseNote(char* in, uint8_t* note, uint8_t* duration, int8_t& size){

    if(in[0] == 'T'){
        int8_t i=1;
        int8_t j=0;
        while(i<strlen(in)){
            if(in[i+1] == '#'){
                uint8_t idx = in[i] & 0x0F;
                note[j] = sharps[idx-1];
                duration[j] = uint8_t(in[i+2]-'0');
                i+=3;
                j+=1;
            }
            else if(in[i+1] == '^'){
                uint8_t idx = in[i] & 0x0F;
                note[j] = flats[idx-1];
                duration[j] = uint8_t(in[i+2]-'0');
                i+=3;
                j+=1;
            }
            else{
                uint8_t idx = in[i] & 0x0F;
                note[j] = notes[idx-1];
                duration[j] = uint8_t(in[i+1]-'0');
                i+=2;
                j+=1;
            }
        }
        //pc.printf("Size: %d\n", j);
        size = j;
        return true;
    }
    return false;
}


void parse_input_thread(){
    char input[49];
    float r_tmp, v_tmp;
    uint8_t n[16];
    uint8_t d[16];
    int8_t s = 0;
    //bool cmd, r_cmd, v_cmd;
    //bool r_updated, v_updated, n_updated;
    bool r_updated, v_updated;

    // Make the pointers point to the array with the notes
    N = n;
    D = d;

    while(1){
        r_updated = false;
        v_updated = false;

        pc.scanf("%s", input);

        if ( parseNote(input, n, d, s) ){
            // N = n;
            // D = d;
            n_cmd = true;
            for(int i=0; i<s; i++){
                PRINT_DEBUG("Note: %u, Duration: %u", N[i], D[i]);
            }
        }
        else if( parseCmd(input, r_tmp, v_tmp, r_updated, v_updated) ){
            n_cmd = false;
            r_cmd = r_updated;
            v_cmd = v_updated;

            PRINT_DEBUG("R: %de-3 V: %de-3",
              (int)(R*1000),
              (int)(V*1000)
            );
        }      
        
        // Notes command:    n_cmd=true
        // Rotation command: r_cmd = true
        // Velocity command: v_cmd = true
        // Autotune command: r_cmd=false, v_cmd=False, n_cmd=false
        // States - n_cmd=true 

        terminateControlThreads();        

        // Update reference values - does not matter if not meaningful: will be ignored by controlOutput()
        R = r_tmp;
        V = v_tmp;
        SIZE = s;

        // Start the threads to control the behaviour
        controlOutput();
    }
}

void terminateControlThreads(){
    // Stop the velocity control threads
    thread_spin.terminate();
    thread_vel_control.terminate();

    // Reset the controllers for the next run
    vel_controller.reset();
    pos_controller.reset();
}

void controlOutput(){
    // Music Command
    if (n_cmd) {
        PRINT_DEBUG("Notes not working yet");
        // Play music
        thread_spin.start(spin);
        thread_music.start(play_music_thread);
    }

    else{
        // Autotune command
        if(!v_cmd && !r_cmd){
            PRINT_DEBUG("Autotune not working yet");
        }

        // Position/Velocity command
        else{
            // Update controller parameters
            if (v_cmd) {
                if(V < VEL_THRESH)  vel_controller.setParams(KP_VELOCITY_FAST, KI_VELOCITY_FAST, KD_VELOCITY_FAST);
                else                vel_controller.setParams(KP_VELOCITY_SLOW, KI_VELOCITY_SLOW, KD_VELOCITY_SLOW);

                // Start velocity control thread
                thread_vel_control.start(velocity_control_thread);
            }

            if(r_cmd){
                PRINT_DEBUG("Position control not working yet");
                // if(R < VEL_THRESH)  pos_controller.setParams(KP_VELOCITY_FAST, KI_VELOCITY_FAST, KD_VELOCITY_FAST);
                // else                pos_controller.setParams(KP_VELOCITY_SLOW, KI_VELOCITY_SLOW, KD_VELOCITY_SLOW);
            }

            // Start the thread that spins the motor
            thread_spin.start(spin);

        }
    }
}

void play_music_thread(){
    pwm_duty_cycle = 0.5;
    while(1){
        for(int i=0; i<melody_size; i++){
            set_pwm(N[i]);
            Thread::wait(D[i]);
            // PRINT_DEBUG("Note: %u, Duration: %u", N[i], D[i]);
        }
    }
}


// ======================================== MAIN ========================================


int main() {
    // =============================
    // Test MAIN
    // =============================

    CHA.rise(&CHA_rise_isr);
    I1.rise(&I1_isr_rise);
    I1.fall(&I1_isr_fall);


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

    // ref_vel = 20.0;
    // thread_spin.start(spin);
    // thread_vel_control.start(velocity_control_thread);

    while (1){
        parse_input_thread();

        // set_pwm(239);
        // PRINT_DEBUG("Tick: %d", tick);
        // Thread::wait(100);
        // PRINT_DEBUG("Vel:%d.%03d",(int)(velocity),abs((int)(velocity*1000)%1000));
        // PRINT_DEBUG("Ticks: %d",tick);
        // PRINT_DEBUG("Rots: %d",rots)
        Thread::wait(100);
    }
}
