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

volatile Timer t;

volatile int8_t rotorState;

volatile float velocity = 0;

float period = 0.5;

Thread thread_v;

// int pwm_on = 0.5;

// ======================================== FUNCTION DEFINTIONS ========================================

//Set a given drive state
void motorOut(int8_t driveState){
    
    //Lookup the output byte from the drive state.
    int8_t driveOut = DRIVE_TABLE[driveState & 0x07];

    PRINT_DEBUG("motorOut: %x",driveState);

    //Turn off first
    if (~driveOut & 0x01) L1L = 0;
    if (~driveOut & 0x02) L1H = 1;
    if (~driveOut & 0x04) L2L = 0;
    if (~driveOut & 0x08) L2H = 1;
    if (~driveOut & 0x10) L3L = 0;
    if (~driveOut & 0x20) L3H = 1;
    
    //Then turn on
    if (driveOut & 0x01) L1L = 1;
    if (driveOut & 0x02) L1H = 0;
    if (driveOut & 0x04) L2L = 1;
    if (driveOut & 0x08) L2H = 0;
    if (driveOut & 0x10) L3L = 1;
    if (driveOut & 0x20) L3H = 0;  

    PRINT_DEBUG("[%d,%d,%d,%d,%d,%d]",(int)L1L,(int)L1H,(int)L2L,(int)L2H,(int)L3L,(int)L3H);    
}

//Convert photointerrupter inputs to a rotor state
inline int8_t readRotorState(){
    return STATE_MAP[I1 + 2*I2 + 4*I3];
}

//Basic synchronisation routine    
int8_t motorHome() {
    //Put the motor in drive state 0 and wait for it to stabilise
    motorOut(0);
    wait(1.0);
    //Get the rotor state
    return readRotorState();
}


inline void CHA_rise_isr() {
    tick -= INC[CHB.read()];
    // PRINT_DEBUG("Tick: %d",tick)
}

// inline void CHA_fall_isr() {
//     tick += INC[CHB.read()];
// }

// inline void CHB_rise_isr() {
//     tick += INC[CHA.read()];
// }

// inline void CHB_fall_isr() {
//     tick -= INC[CHA.read()];
//     // Maybe faster?
//     // int val = CHA.read();
//     // tick += (1>>val);
//     // tick -= (1>>!val);
// }

// inline void I1_rise_isr() {
// }
// inline void I1_fall_isr() {

// }

// inline void I2_rise_isr() {

// }
// inline void I2_fall_isr() {

// }

// inline void I3_rise_isr() {


// }
// inline void I3_fall_isr() {

// }


void velocity_thread(){
    int t_before, t_after;
    while(true){
        t_before = tick;
        Thread::wait(100);
        t_after = tick;
        velocity = 10.0*(float)(t_before-t_after)/117.0;
    }
}


inline void loop(){

    for(int i=0; i<6; i++){
        motorOut(i);
        wait(period);
        // wait(pwm_on*period/6);
        // motorOut(6);
        // wait(0.5);
        // wait((1-pwm_on)*period/6);  
    }
}


//Main
int main() {
    // =============================
    // Test MAIN
    // =============================

    // motorOut(0);
    // wait(1.0);


    CHA.rise(&CHA_rise_isr);
    thread_v.start(velocity_thread);
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
    int8_t orState = 0;    //Rotot offset at motor state 0
    
    //Initialise the serial port
    int8_t intState = 0;
    int8_t intStateOld = 0;
    pc.printf("Hello\n\r");

    //Run the motor synchronisation
    orState = motorHome();
    pc.printf("Rotor origin: %x\n\r",orState);
    //orState is subtracted from future rotor state inputs to align rotor and motor states
    
    //Poll the rotor state and set the motor outputs accordingly to spin the motor
    while (1) {
        PRINT_DEBUG("before:%d,after:%d!",(int)(velocity*1000));

        // pc.printf("Velocity: %d\n\r",(int)(velocity*1000));

        Thread::wait(500);

        // intState = readRotorState();
        // if (intState != intStateOld) {
        //     intStateOld = intState;
        //     motorOut((intState-orState+lead+6)%6); //+6 to make sure the remainder is positive
        // }
    }

}





