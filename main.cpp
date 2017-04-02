#include "mbed.h"
#include "rtos.h"
#include <stdlib.h>

#include "parse.h"
#include "globals.h"
#include "PidController.h"
#include "wiring.h"

// ======================================== INTERRUPTS ========================================


inline void CHA_rise_isr() {
    tick += INC[CHB.read()];
    tick_adjust_mutex.lock();
    tick_adjust += INC[CHB.read()];
    tick_adjust_mutex.unlock();
    // PRINT_DEBUG("Tick: %d", tick);
}

    // Maybe faster?
    // int val = CHB.read();
    // tick += (1>>val);
    // tick -= (1>>!val);


inline void I1_isr_rise(){
    if(I2){

        if(!tick_offset){
            tick_offset=(tick%117+117)%117;
        }

        t_now_fall = t.read_us();
        t_diff_temp = t_now_fall-t_before_fall;
        if(t_diff_temp > 10000){ // Ignore if the duration is too small (implying glitch)
          t_diff = t_diff_temp;
          t_before_fall = t_now_fall;
          rots++;
            tick_adjust_mutex.lock();
            tick_adjust = ((rots-1)*117)+tick_offset;
            tick_adjust_mutex.unlock();

        }   

    }
}

inline void I1_isr_fall(){
    if(I2){

        if(!tick_offset){
            tick_offset=(tick%117+117)%117;
            // rots++;
        }
        t_now_rise = t.read_us();
        t_diff_temp = t_now_rise-t_before_rise;

        if(t_diff_temp > 10000){ // Ignore if the duration is too small (implying glitch)
            t_diff = -t_diff_temp;
            t_before_rise = t_now_rise;
            rots--;
            tick_adjust_mutex.lock();
            tick_adjust = (rots*117)+tick_offset;
            tick_adjust_mutex.unlock();
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
//    if (driveOut & 0x02) L1H.write(pwm_duty_cycle);

    if (driveOut & 0x04) L2L.write(1);
     if (driveOut & 0x08) L2H.write(1-pwm_duty_cycle);
//    if (driveOut & 0x08) L2H.write(pwm_duty_cycle);

    if (driveOut & 0x10) L3L.write(1);
     if (driveOut & 0x20) L3H.write(1-pwm_duty_cycle);
//    if (driveOut & 0x20) L3H.write(pwm_duty_cycle);

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
        stable = (tick_diff == 0) && (curr_state == prev_state);
        // stable = (curr_state == 0) && (curr_state == prev_state);

        prev_state = curr_state;
    }

    rots = 0;
    rotations = 0;
    tick = 0;

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

void tick_diff_thread(){
    int tick_before;
    while(1){
        tick_before = tick;
        Thread::wait(VEL_PERIOD);
        tick_diff = tick-tick_before;
    }
}

void rotations_thread(){
    while(1){
        rotations = (float)tick_adjust/117.0;
        Thread::wait(VEL_PERIOD);
    }
}


void velocity_measure_thread(){
    float curr_velocity = 0;
    while(1){
        // If getting within 35 ticks/VEL_PERIOD, use ticks for velocity
        if (tick_diff < TICK_DIFF_THRESH && tick_diff > -TICK_DIFF_THRESH){ 
            curr_velocity = 1000.0/(VEL_PERIOD)*(tick_diff)/117.0;
        }
        // Else use I1 rotation counter for velocity
        else {
            curr_velocity = 1000000.0/(float)t_diff; // 1 revolutions * 10^6 pecoseconds
        }
        velocity = 0.2*curr_velocity +0.8*velocity;
        Thread::wait(VEL_PERIOD);
    }

    // float curr_velocity = 0;
    // while(1){
    //     if (velocity < VEL_THRESH && velocity > -VEL_THRESH){
    //         curr_velocity = 1000.0/(VEL_PERIOD)*(tick_diff)/117.0;
    //         velocity = 0.2*curr_velocity +0.8*velocity;
    //     }
    //     else {
    //         curr_velocity = 1000000.0/(float)t_diff; // 1 revolutions * 10^6 pecoseconds
    //         velocity = 0.2*curr_velocity +0.8*velocity;
    //     }
    //     Thread::wait(VEL_PERIOD);

    // }
}


void velocity_control_thread(){
    while(1){
        pwm_duty_cycle = vel_controller.computeOutput(V, velocity, (float)VEL_PERIOD/1000.0);
        // PRINT_DEBUG("Duty: 0.%03d",(int)(pwm_duty_cycle*1000))
        Thread::wait(VEL_PERIOD);
    }
}


void play_music_thread(){
    pwm_duty_cycle = 0.5;
    while(1){
        for(int i=0; i<melody_size; i++){
            set_pwm(N[i]);
            Thread::wait(uint16_t(D[i])*1000);
            // PRINT_DEBUG("Note: %u, Duration: %u", N[i], D[i]);
        }
    }
}


// ======================================== CONTROL FUNCTIONS ========================================


void terminateControlThreads(){
    // Stop the velocity control threads
    thread_spin.terminate();
    thread_vel_control.terminate();

    thread_v.terminate();
    thread_r.terminate();

    // Reset the controllers for the next run
    vel_controller.reset();
    // pos_controller.reset();
}


void controlOutput(){
    motorHome();

    // Music Command
    if (n_cmd) {
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
                if(V > -VEL_THRESH && V < VEL_THRESH)  
                    vel_controller.setParams(KP_VELOCITY_SLOW, KI_VELOCITY_SLOW, KD_VELOCITY_SLOW);
                else                                    
                    vel_controller.setParams(KP_VELOCITY_FAST, KI_VELOCITY_FAST, KD_VELOCITY_FAST);

                if (V < 0.0)    lead = -2;
                else            lead = 2;

                PRINT_DEBUG("LEAD %d", lead);
                // Start velocity control thread
                thread_v.start(velocity_measure_thread);
                thread_vel_control.start(velocity_control_thread);
            }

            if(r_cmd){
                PRINT_DEBUG("Position control not working yet");
                // if(R < VEL_THRESH)  pos_controller.setParams(KP_VELOCITY_FAST, KI_VELOCITY_FAST, KD_VELOCITY_FAST);
                // else                pos_controller.setParams(KP_VELOCITY_SLOW, KI_VELOCITY_SLOW, KD_VELOCITY_SLOW);
                
                thread_r.start(rotations_thread);
            }

            // Start the thread that spins the motor
            thread_spin.start(spin);
        }
    }
}


void parseInput(){
    bool command;
    float r_tmp, v_tmp;
    uint8_t n[16], d[16];
    int8_t s = 0;
    bool r_updated, v_updated;

    // Make the pointers point to the array with the notes
    N = n;
    D = d;

    while(1){
        command = false;
        
        while (!command){
            if (pc.readable()){
                input[in_idx] = pc.getc();
                if (input[in_idx] == '\r' || input[in_idx] == '\n'){
                    input[in_idx] = '\0';
                    command = true;
                    in_idx = 0;
                } 
                else ++in_idx;
            }
            // PRINT_DEBUG("%d.%03d",(int)rotations,(int)(rotations*1000)%1000);
            Thread::wait(100);
        }

        r_updated = false;
        v_updated = false;

        if ( parseNote(input, n, d, s) ){
            n_cmd = true;
        }
        else if( parseCmd(input, r_tmp, v_tmp, r_updated, v_updated) ){
            n_cmd = false;
            r_cmd = r_updated;
            v_cmd = v_updated;
        }      
        
        // Notes command:    n_cmd=true
        // Rotation command: r_cmd = true
        // Velocity command: v_cmd = true
        // Autotune command: r_cmd=false, v_cmd=false, n_cmd=false
        // States - n_cmd=true 

        terminateControlThreads();        

        // Update reference values - does not matter if not meaningful: will be ignored by controlOutput()
        R = r_tmp;
        V = v_tmp;
        melody_size = s;

        // Print the new command
        if (n_cmd){
            for(int i=0; i<s; ++i){
                PRINT_DEBUG("Note: %u, Duration: %u", N[i], D[i]);
            }
        }
        else{
            PRINT_DEBUG("R:%d.%03d V:%d.%03d", (int)(R), abs((int)(R*1000)%1000), (int)(V), abs((int)(V*1000)%1000));
        }

        // Start the threads to control the behaviour
        controlOutput();
    }
}



// ======================================== MAIN ========================================


int main() {
    pc.printf("Hello\n\r");

    PRINT_DEBUG("Setting PWM");
    set_pwm(pwm_period);

    // PRINT_DEBUG("Starting velocity thread");

    PRINT_DEBUG("Synchronising state");


    CHA.rise(&CHA_rise_isr);
    I1.rise(&I1_isr_rise);
    I1.fall(&I1_isr_fall);

    thread_diff.start(tick_diff_thread);

    orState = motorHome();

    // thread_v.start(velocity_measure_thread); 
    // thread_r.start(rotations_thread);

    PRINT_DEBUG("Starting timer");
    t.start();

    // V = 8.0;
    // thread_vel_control.start(velocity_control_thread);
//    thread_spin.start(spin);

    // Run a while loop trying to parse     
    parseInput();
    // ANDREW'S DEBUG SECTION


    while (1){
         // PRINT_DEBUG("Rot: %d.%03d",(int)rotations,(int)(rotations*1000)%1000);
//        PRINT_DEBUG("Rots: %d, Ticks: %d",rots,tick);
        // PRINT_DEBUG("Vel: %d.%03d",(int)velocity,(int)(velocity*1000)%1000);
        Thread::wait(100);
    }

}
