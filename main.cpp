#include "mbed.h"
#include "rtos.h"

//Photointerrupter input pins
#define I1pin D2
#define I2pin D11
#define I3pin D12

//Incremental encoder input pins
#define CHApin D7
#define CHBpin D8  

//Motor Drive output pins   //Mask in output byte
#define L1Lpin D4           //0x01   PROBLEM? (Left, 2nd from bottom)
#define L1Hpin D5           //0x02   PROBLEM? Or good
#define L2Lpin D3           //0x04
#define L2Hpin D6           //0x08   PROBLEM? (Left, 4th from bottom)
#define L3Lpin D9           //0x10
#define L3Hpin D10          //0x20

//Mapping from sequential drive states to motor phase outputs
/*
State   L1  L2  L3
0       H   -   L
1       -   H   L
2       L   H   -
3       L   -   H
4       -   L   H
5       H   L   -
6       -   -   -
7       -   -   -
*/
//Drive state to output table
const int8_t driveTable[] = {0x12,0x18,0x09,0x21,0x24,0x06,0x00,0x00};

//Mapping from interrupter inputs to sequential rotor states. 0x00 and 0x07 are not valid
const int8_t stateMap[] = {0x07,0x05,0x03,0x04,0x01,0x00,0x02,0x08};  
//const int8_t stateMap[] = {0x07,0x01,0x03,0x02,0x05,0x00,0x04,0x07}; //Alternative if phase order of input or drive is reversed

//Phase lead to make motor spin
const int8_t lead = -2;  //2 for forwards, -2 for backwards

//Status LED
DigitalOut led1(LED1);

//Photointerrupter inputs
DigitalIn I1(I1pin);
DigitalIn I2(I2pin);
DigitalIn I3(I3pin);

//Quadrature inputs
InterruptIn CHA(CHApin);
InterruptIn CHB(CHBpin);

//Motor Drive outputs
DigitalOut L1L(L1Lpin);
DigitalOut L1H(L1Hpin);
DigitalOut L2L(L2Lpin);
DigitalOut L2H(L2Hpin);
DigitalOut L3L(L3Lpin);
DigitalOut L3H(L3Hpin);

//Serial output
Serial pc(SERIAL_TX, SERIAL_RX);

//Set a given drive state
void motorOut(int8_t driveState){
    
    //Lookup the output byte from the drive state.
    int8_t driveOut = driveTable[driveState & 0x07];

    pc.printf("motorOut: %x\n\r",driveState);

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

    pc.printf("[%d,%d,%d,%d,%d,%d]\n\r",(int)L1L,(int)L1H,(int)L2L,(int)L2H,(int)L3L,(int)L3H);    
}

    //Convert photointerrupter inputs to a rotor state
inline int8_t readRotorState(){
    return stateMap[I1 + 2*I2 + 4*I3];
}

//Basic synchronisation routine    
int8_t motorHome() {
    //Put the motor in drive state 0 and wait for it to stabilise
    motorOut(0);
    wait(1.0);
    
    //Get the rotor state
    return readRotorState();
}
   

int tick = 0;

Timer t;

int inc[2] = {-1,1};

inline void CHA_rise_isr() {
    tick -= inc[CHB.read()];
}

inline void CHA_fall_isr() {
    tick += inc[CHB.read()];
}

inline void CHB_rise_isr() {
    tick += inc[CHA.read()];
}

inline void CHB_fall_isr() {
    tick -= inc[CHA.read()];
    // Maybe faster?
    // int val = CHA.read();
    // tick += (1>>val);
    // tick -= (1>>!val);
}


int period = 0.5;
// int pwm_on = 0.5;

inline void loop(){

    for(int i=0; i<6; i++){
        motorOut(i);
        wait(period);
        // wait(pwm_on*period/6);
        // motorOut(6);
        // wait(0.5);
        // wait((1-pwm_on)*period/6);  
    }
    // motorOut(2);
    // wait(1);
    // motorOut(1);
    // wait(1);
    // motorOut(0);
    // wait(1);
    // motorOut(5);
    // wait(1);
    // motorOut(4);
    // wait(1);  
    // motorOut(3);
    // wait(1);

}


//Main
int main() {
    // int8_t orState = 0;    //Rotot offset at motor state 0
    
    // //Initialise the serial port
    // int8_t intState = 0;
    // int8_t intStateOld = 0;
    // int8_t motorspin = 0;
    // pc.printf("Hello\n\r");
    
    // //Run the motor synchronisation
    // orState = motorHome();
    // pc.printf("Rotor origin: %x\n\r",orState);
    // //orState is subtracted from future rotor state inputs to align rotor and motor states

    motorOut(0);
    wait(1.0);


    CHA.rise(&CHA_rise_isr);
    CHA.fall(&CHA_fall_isr);
    CHB.rise(&CHB_rise_isr);
    CHB.fall(&CHB_fall_isr);

    //Poll the rotor state and set the motor outputs accordingly to spin the motor
    while (1) {
        loop();
    }
}





