#ifndef WIRING_H
#define WIRING_H

#include "mbed.h"

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
extern DigitalOut led1;

// Photointerrupter inputs
extern InterruptIn I1;
extern InterruptIn I2;
extern InterruptIn I3;

// Quadrature inputs
extern InterruptIn CHA;
extern DigitalIn CHB;

// Motor Drive outputs
extern DigitalOut L1L;
extern PwmOut L1H;
extern DigitalOut L2L;
extern PwmOut L2H;
extern DigitalOut L3L;
extern PwmOut L3H;

// Serial output
extern Serial pc;

// Debug logger
#define DEBUG_LEVEL 1

#if DEBUG_LEVEL    
#define PRINT_DEBUG(...)    printf(__VA_ARGS__);\
                            printf("\n\r");
#else
#define PRINT_DEBUG(...)                         
#endif


#endif
