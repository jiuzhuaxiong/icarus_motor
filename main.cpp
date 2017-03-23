#include "mbed.h"
#include "rtos.h"

#include "wiring.h"


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

// ======================================== GLOBAL VARIABLES ========================================

// Value of the ticks encoder
volatile int tick = 0;  

volatile int8_t rotorState;

volatile int velocity = 0;

volatile int t_before = 0;
volatile int t_now = 0;
volatile int t_diff = 0; // revolutions/sec is 1/t_diff

volatile float pwm_duty_cycle = 0.5;
volatile int pwm_period = 1000; // in microseconds

Timer t;

float period = 0.5;

Thread thread_v;

// int pwm_on = 0.5;

// ======================================== FUNCTION DEFINTIONS ========================================

//Set a given drive state
void motorOut(int8_t driveState){
    
    //Lookup the output byte from the drive state.
    int8_t driveOut = DRIVE_TABLE[driveState & 0x07];

    //Turn off Vm
    if (~driveOut & 0x02) L1H.write(1);
    if (~driveOut & 0x08) L2H.write(1);
    if (~driveOut & 0x20) L3H.write(1);
    //Turn off gnd
    if (~driveOut & 0x01) L1L = 0;
    if (~driveOut & 0x04) L2L = 0;
    if (~driveOut & 0x10) L3L = 0;
    
    //Then turn on gnd
    // if (driveOut & 0x01) L1L = 1;
    // if (driveOut & 0x04) L2L = 1;
    // if (driveOut & 0x10) L3L = 1;
    // Optimised version
    L1L = (driveOut & 0x01);
    L2L = (driveOut & 0x04)>>2;
    L3L = (driveOut & 0x10)>>4;

    // and turn on Vm
    // if (driveOut & 0x02)>>1 L1H.write(1-PWM_DUTY_CYCLE);
    // if (driveOut & 0x08) L2H.write(1-PWM_DUTY_CYCLE);
    // if (driveOut & 0x20) L3H.write(1-PWM_DUTY_CYCLE);
    // Might be more optimised? Need timing analysis
    L1H.write(1-pwm_duty_cycle*((driveOut & 0x02)>>1));
    L2H.write(1-pwm_duty_cycle*((driveOut & 0x08)>>3));
    L3H.write(1-pwm_duty_cycle*((driveOut & 0x20)>>5));

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
    L1L = 0;
    L2L = 0;
    L3L = 1;
    L1H.write(0);

    wait(1.0);
    //Get the rotor state
    return readRotorState();
}


inline void CHA_rise_isr() {
    tick -= INC[CHB.read()];
    // PRINT_DEBUG("Tick: %d",tick)

    // Maybe faster?
    // int val = CHB.read();
    // tick += (1>>val);
    // tick -= (1>>!val);
}


void velocity_thread(){
    int tick_before, tick_after;
    while(1){
        tick_before = tick;
        Thread::wait(100);
        tick_after = tick;
        velocity = 10000*(tick_before-tick_after)/117;
    }
}

void I1_rise_isr(){
    t_now = t.read_us();
    t_diff = t_now-t_before;
    t_before = t_now;
}

void spin(){
    int8_t orState = 0;    //Rotot offset at motor state 0
    
    //Initialise the serial port
    int8_t intState = 0;
    int8_t intStateOld = 0;

    //Run the motor synchronisation
    orState = motorHome();
    pc.printf("Rotor origin: %x\n\r",orState);

    while(1){
        intState = readRotorState();
        if (intState != intStateOld) {
            intStateOld = intState;
            motorOut((intState-orState+lead+6)%6); //+6 to make sure the remainder is positive
        }
    }
}


Thread thread_spin;
//Main
int main() {
    // =============================
    // Test MAIN
    // =============================

    // motorOut(0);
    // wait(1.0);


     I1.rise(&I1_rise_isr);
    CHA.rise(&CHA_rise_isr);
    // CHA.fall(&CHA_fall_isr);
    // CHB.rise(&CHB_rise_isr);
    // CHB.fall(&CHB_fall_isr);

    // //Poll the rotor state and set the motor outputs accordingly to spin the motor
    // while (1) {
    //     loop();
    // }


    // =============================
    // Original MAIN
    // =============================

    pc.printf("Hello\n\r");

    L1H.period_us(pwm_period);
    L2H.period_us(pwm_period);
    L3H.period_us(pwm_period);

    //orState is subtracted from future rotor state inputs to align rotor and motor states
    
    // Begin threads
    t.start();
    thread_spin.start(spin);
    thread_v.start(velocity_thread);


    while (1){
        // Main thread
         PRINT_DEBUG("Vel from I1: %d.%d",(1000000/t_diff),(1000000000/t_diff)%1000);
        PRINT_DEBUG("Vel from Tick: %d.%d",velocity/1000,velocity%1000);
        Thread::wait(500);
    }

    //Poll the rotor state and set the motor outputs accordingly to spin the motor
    // while (1) {
    //     intState = readRotorState();
    //     if (intState != intStateOld) {
    //         intStateOld = intState;
    //         motorOut((intState-orState+lead+6)%6); //+6 to make sure the remainder is positive
    //     }
    // }

}





