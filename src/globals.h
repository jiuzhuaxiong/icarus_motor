#ifndef GLOBALS_H
#define GLOBALS_H

#include "mbed.h"
#include "PidController.h"

// ======================================== GLOBAL CONSTANTS ========================================

// Drive state to output table
const int8_t DRIVE_TABLE[] = {0x12,0x18,0x09,0x21,0x24,0x06,0x00,0x00};

// Mapping from interrupter inputs to sequential rotor states. 0x00 and 0x07 are not valid
const int8_t STATE_MAP[] = {0x07,0x05,0x03,0x04,0x01,0x00,0x02,0x08};  
// const int8_t STATE_MAP[] = {0x07,0x01,0x03,0x02,0x05,0x00,0x04,0x07}; // Alternative if phase order of input or drive is reversed

// Phase lead to make motor spin

// Increment values for the ticks encoder
const int8_t INC[2] = {1, -1};

const uint8_t VEL_PERIOD = 30;     // in milliseconds


const float VEL_THRESH = 10.0;
const int8_t TICK_DIFF_THRESH = 35;

// CONTROLLER PARAMETERS
//PID values from ZiglerNicholas [0.021, 0.07636363636363636, 0.0014437500000000002]
// const float KP_VELOCITY_FAST = 0.013;
// const float KI_VELOCITY_FAST = 0.014;
// const float KD_VELOCITY_FAST = 0.000742;
const float KP_VELOCITY_FAST = 0.036; //0.036
const float KI_VELOCITY_FAST = 0.035952;//0.136752
const float KD_VELOCITY_FAST = 0.00236925;//0.00236925

const float KP_VELOCITY_SLOW = 0.015;;
const float KI_VELOCITY_SLOW = 0.000000002;
const float KD_VELOCITY_SLOW = 0.000024;

// const float KP_POS = 0.0025; //0.0025
// const float KI_POS = 0.0011; //0.0555
// const float KD_POS = 0.00028125; //0.000028125
const float KP_POS = 0.036; //0.036
const float KI_POS = 0.035952;//0.136752
const float KD_POS = 0.00236925;//0.00236925


// NOTES DATA
const uint8_t NOTES[7] = {142, 127, 239, 213, 190, 179, 159}; // A B C D E F G
const uint8_t SHARPS[7] = {134, 127, 225, 201, 179, 169, 150}; // A# B C# D# F F# G#
const uint8_t FLATS[7] = {150, 134, 245, 225, 201, 190, 190}; //A^ B^ C^ D^ E^ E G^

const uint16_t PWM_PERIOD = 2000; // in microseconds

// Acceleration must be in rot/VEL_PERIOD, not rot/s 
// const float ACCELERATION = 16.0;
const float ACCELERATION = 20.0;
const float DECCELERATION = 0.5;
const float MAX_SPEED = 30.0;


// ======================================== GLOBAL VARIABLES ========================================


volatile float deccelerate_threshold;
volatile float last_vel_target = 0.0;

volatile float velocity     = 0;
volatile float rotations    = 0;

volatile int t_before_rise  = 0;
volatile int t_now_rise     = 0;
volatile int t_before_fall  = 0;
volatile int t_now_fall     = 0;
volatile int t_diff         = 2147482647; // revolutions/sec is 1/t_diff


volatile float pwm_duty_cycle  = 1;

// Value of the ticks encoder
volatile int tick = 0;  
volatile int rots = 0;
volatile int tick_offset = 0;
volatile int tick_adjust = 0;
volatile int tick_diff;
volatile int t_diff_temp=0;
Mutex tick_adjust_mutex;

// Rotation reference
volatile float R = 0;
// Velocity reference
volatile float V = 0;  
volatile bool r_cmd = false, v_cmd = false, n_cmd = false;  // True if R, V were updated during the last command
volatile uint8_t* N;
volatile uint8_t* D;
volatile int8_t melody_size = 0; // Size of N and D

// Input buffer
char input[49];
// Index for input buffer
int8_t in_idx = 0;

volatile int8_t lead = -2;  // 2 for forwards, -2 for backwards

Timer t;

int8_t origin_state = 0;    //Rotor offset at motor state 0


// ======================================== THREADS ========================================

Thread thread_diff(osPriorityNormal, 300);
Thread thread_pos_measure(osPriorityNormal, 500);
Thread thread_vel_measure(osPriorityNormal, 500);
Thread thread_spin(osPriorityNormal, 500);
Thread thread_vel_control(osPriorityNormal, 600);   // Actual used stack size is 528 
Thread thread_pos_control(osPriorityNormal, 700);
Thread thread_music(osPriorityNormal, 500);     // Verify stack size


// ======================================== CONTROLLERS ========================================

PidController vel_controller(KP_VELOCITY_FAST,  KI_VELOCITY_FAST, KD_VELOCITY_FAST); 
PidController pos_controller(KP_POS,            KI_POS,           KD_POS);
// PidController pos_controller(KP_POS,            KI_POS,           KD_POS, -1.0, 1.0);

//PID values from ZiglerNicholas [0.021, 0.07636363636363636, 0.0014437500000000002]
// PidController vel_controller(0.018, 0.109, 0.000742); 



#endif


