#ifndef WIRING_H
#define WIRING_H

// Photointerrupter input pins
#define I1pin D2
#define I2pin D11
#define I3pin D12

// Incremental encoder input pins
#define CHApin D7
#define CHBpin D8  

// Motor Drive output pins   // Mask in output byte
#define L1Lpin D4           // 0x01   PROBLEM? (Left, 2nd from bottom)
#define L1Hpin D5           // 0x02   PROBLEM? Or good
#define L2Lpin D3           // 0x04
#define L2Hpin D6           // 0x08   PROBLEM? (Left, 4th from bottom)
#define L3Lpin D9           // 0x10
#define L3Hpin D10          // 0x20

// Mapping from sequential drive states to motor phase outputs
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

// Debug logger
#define DEBUG_LEVEL 1

#if DEBUG_LEVEL    
#define PRINT_DEBUG(...)    printf(__VA_ARGS__);\
                            printf("\n\r");
#else
#define PRINT_DEBUG(...)                         
#endif


#endif


