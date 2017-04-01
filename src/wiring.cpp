#include "wiring.h"

// Status LED
DigitalOut led1(LED1);

// Photointerrupter inputs
InterruptIn I1(I1pin);
InterruptIn I2(I2pin);
InterruptIn I3(I3pin);

// Quadrature inputs
InterruptIn CHA(CHApin);
DigitalIn CHB(CHBpin);

// Motor Drive outputs
DigitalOut L1L(L1Lpin);
PwmOut L1H(L1Hpin);
DigitalOut L2L(L2Lpin);
PwmOut L2H(L2Hpin);
DigitalOut L3L(L3Lpin);
PwmOut L3H(L3Hpin);

// Serial output
Serial pc(SERIAL_TX, SERIAL_RX);


