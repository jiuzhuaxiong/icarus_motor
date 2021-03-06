#include <stdlib.h>

#include "mbed.h"
#include "rtos.h"

#include "parse.h"
#include "globals.h"
#include "PidController.h"
#include "wiring.h"


// ======================================== INTERRUPTS ========================================


inline void isrRiseCHA() 
{
    tick += INC[CHB.read()];
    tick_adjust_mutex.lock();
    tick_adjust += INC[CHB.read()];
    tick_adjust_mutex.unlock();
}


inline void isrRiseI1()
{
    if(I2){

        if(!tick_offset){
            tick_offset=(tick%117+117)%117;
        }

        t_now_fall = t.read_us();
        t_diff_temp = t_now_fall-t_before_fall;

            // Ignore if the duration is too small (implying glitch)
        if(t_diff_temp > 10000){ 
            t_diff = t_diff_temp;
            t_before_fall = t_now_fall;
            rots++;
            tick_adjust_mutex.lock();
            tick_adjust = ((rots-1)*117)+tick_offset;
            tick_adjust_mutex.unlock();

        }   

    }
}

inline void isrFallI1()
{
    if(I2){

        if(!tick_offset)   tick_offset=(tick%117+117)%117;

        t_now_rise = t.read_us();
        t_diff_temp = t_now_rise-t_before_rise;

        // Ignore if the duration is too small (implying glitch)
        if(t_diff_temp > 10000){ 
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
void motorOut(int8_t driveState)
{
    
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
    // Active Low - i.e. output of 1 means no speed, 0 means highest speed
    if (driveOut & 0x01) L1L.write(1);
    if (driveOut & 0x02) L1H.write(1-pwm_duty_cycle);

    if (driveOut & 0x04) L2L.write(1);
    if (driveOut & 0x08) L2H.write(1-pwm_duty_cycle);

    if (driveOut & 0x10) L3L.write(1);
    if (driveOut & 0x20) L3H.write(1-pwm_duty_cycle);

}


//Convert photointerrupter inputs to a rotor state
inline int8_t readRotorState()
{
    return STATE_MAP[I1 + 2*I2 + 4*I3];
}


//Basic synchronisation routine    
int8_t motorHome() {
    PRINT_DEBUG("motor home");
    lead = 2;

    L2H.write(1);
    L3H.write(1);
    L1L.write(0);
    L2L.write(0);


    L1H.write(0);
    L3L.write(1);


    PRINT_DEBUG("Rotate to home state");

    bool stable = false;
    int prev_state, curr_state;

    // PRINT_DEBUG("Reading rotor state");
    prev_state = readRotorState();
    // PRINT_DEBUG("Read rotor state");

    while(!stable){
        PRINT_DEBUG("Waiting to stabilise");
        wait(1.0);
        curr_state = readRotorState();
        stable = (tick_diff == 0) && (curr_state == prev_state);

        prev_state = curr_state;
    }

    rots = 0;
    rotations = 0;
    tick = 0;

    //Get the rotor state
    return readRotorState();
}


void setPWM(int p)
{
    L1H.period_us(p);
    L2H.period_us(p);
    L3H.period_us(p);
}


// ======================================== THREADS ========================================

void spin()
{
    //Initialise the serial port
    int8_t intState = 0;
    int8_t intStateOld = 0;
    while(1){
        intState = readRotorState();
        if (intState != intStateOld) {
            intStateOld = intState;
            motorOut((intState-origin_state+lead+6)%6); //+6 to make sure the remainder is positive
        }
    }
}

void tickDiffThread()
{
    int tick_before;
    while(1){
        tick_before = tick;
        Thread::wait(VEL_PERIOD);
        tick_diff = tick-tick_before;
    }
}

void rotationsThread()
{
    while(1){
        rotations = ((float)tick_adjust)/117.0;
        Thread::wait(VEL_PERIOD);
    }
}


void velocityMeasureThread()
{
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
}


void velocityControlThread()
{
    while(1){
        pwm_duty_cycle = vel_controller.computeOutput(V, velocity, (float)VEL_PERIOD/1000.0);
        // PRINT_DEBUG("Duty: 0.%03d",(int)(pwm_duty_cycle*1000))
        Thread::wait(VEL_PERIOD);
    }
}


void positionControlThread()
{
    float out;   
    while(1){
        out = pos_controller.computeOutput(R, rotations, (float)VEL_PERIOD/1000.0);
        if (out < 0.0){
            if (R > 0.0) lead = -1;
            else         lead = 1;
            out = -out;
        }
        else {
            if (R > 0.0) lead = 2;
            else         lead = -2;
        }

        pwm_duty_cycle = out;
        Thread::wait(VEL_PERIOD);
    }
}


void playMusicThread()
{
    pwm_duty_cycle = 0.5;
    while(1){
        for(int i=0; i<melody_size; i++){
            setPWM(N[i]);
            Thread::wait(uint16_t(D[i])*1000);
            // PRINT_DEBUG("Note: %u, Duration: %u", N[i], D[i]);
        }
    }
}


// ======================================== CONTROL FUNCTIONS ========================================

/**
 *  Terminate all threads 
 */
void terminateControlThreads()
{
    // Stop the control threads
    thread_spin.terminate();
    thread_vel_control.terminate();
    thread_pos_control.terminate();

    // Stop the music thread
    thread_music.terminate();

    // Stop the measurement threads
    thread_vel_measure.terminate();
    thread_pos_measure.terminate();

    // Reset the controllers for the next run
    vel_controller.reset();
    pos_controller.reset();
}


/**
 * Start threads relevant only to the received serial input 
 */
void controlOutput()
{
    motorHome();

    // Music Command
    if (n_cmd) {
        thread_spin.start(spin);
        thread_music.start(playMusicThread);
    }

    else{
        // Autotune command
        if(!v_cmd && !r_cmd){
            PRINT_DEBUG("Autotune not working yet");
        }

        // Position/Velocity command
        else{
            // Set to the default pwm period
            setPWM(PWM_PERIOD);

            // Update controller parameters
            if (v_cmd) {
                if(V > -VEL_THRESH && V < VEL_THRESH)  
                    vel_controller.setParams(KP_VELOCITY_SLOW, KI_VELOCITY_SLOW, KD_VELOCITY_SLOW);
                else                                    
                    vel_controller.setParams(KP_VELOCITY_FAST, KI_VELOCITY_FAST, KD_VELOCITY_FAST);

                if (V < 0.0)    lead = -2;
                else            lead = 2;

                // Start velocity control thread
                thread_vel_measure.start(velocityMeasureThread);
                thread_vel_control.start(velocityControlThread);
            }

            if(r_cmd){
                if (R < 0.0)    lead = -2;
                else            lead = 2;
                
                thread_pos_measure.start(rotationsThread);
                thread_pos_control.start(positionControlThread);
            }

            // Start the thread that spins the motor
            thread_spin.start(spin);
        }
    }
}



// ======================================== MAIN ========================================


int main() {
    pc.printf("Hello\n\r");

    PRINT_DEBUG("Setting PWM");
    setPWM(PWM_PERIOD);

    PRINT_DEBUG("Synchronising state");

    // Attach interrupts
    CHA.rise(&isrRiseCHA);
    I1.rise(&isrRiseI1);
    I1.fall(&isrFallI1);

    // Start the thread that counts the ticks
    thread_diff.start(tickDiffThread);

    // Get the origin state
    origin_state = motorHome();


    // PRINT_DEBUG("Starting timer");
    t.start();

    bool command, r_updated, v_updated;
    float r_tmp, v_tmp;
    uint8_t n[16], d[16];
    int8_t size = 0;

    // Make the pointers point to the array with the notes
    N = n;
    D = d;

    while(1){
        command = false;
        
        // Remember serial inputs until you see a terminating character
        while (!command){
            if (pc.readable()){
                input[in_idx] = pc.getc();

                pc.printf("%c", input[in_idx]);

                if (input[in_idx] == '\r' || input[in_idx] == '\n'){
                    input[in_idx] = '\0';
                
                    pc.printf("\n\r");
                
                    command = true;
                    in_idx = 0;
                } 
                else ++in_idx;
            }
 
            Thread::wait(PARSER_WAIT);
        }

        r_updated = false;
        v_updated = false;

        // Check if it is a music command
        if ( parseNote(input, n, d, size) ){
            n_cmd = true;
        }
        // Check if it is a velocity/position command
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
        melody_size = size;

        // Print the new command
        if (n_cmd){
            for(int i=0; i<size; ++i){
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
