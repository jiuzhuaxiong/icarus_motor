# icarus_motor
Embedded motor driver for Imperial Embedded Systems Course (Team Icarus)

Melody, Rotation and Velocity commands are supported. Each can be used independently.
Please note that the motor will start spinning when a melody is played.

After a command is executed, the motor will stabilise and return to the "home" position in preparation for the next command. This also occurs during motor initialisation at the beginning of the program.

To turn on debug messages whilst running the motor, go to wiring.h and set the value of DEBUG_LEVEL to 1. However, bear in mind that serial commands cannot get processed while the motor is running because of printing taking up processing time

# Microcontroller

NUCLEO-F303K8
https://developer.mbed.org/platforms/ST-Nucleo-F303K8/


# RTOS
https://docs.mbed.com/docs/mbed-os-api-reference/en/latest/APIs/tasks/rtos/
