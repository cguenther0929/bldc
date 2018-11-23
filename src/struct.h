/******************************************************************************
*   FILE: struct.h
*
*   PURPOSE: Structures used throughout the code base are defined here.  
*
*   DEVICE: PPIC18F66K22
*
*   COMPILER: Microchip XC8 v1.32
*
*   IDE: MPLAB X v3.45
*
*   TODO:  
*
*   NOTE:
*
******************************************************************************/
#ifndef __STRUCT_H_
#define __STRUCT_H_

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include "config.h"             //Project specific header file

extern struct GlobalInformation         //  Structure to pass information that is shared among modules
{
    uint8_t     comu_state;             // Holder for commutate state of motor
    uint8_t     closed_loop_state;           // To describe the intermediate state while looking for the next commutate state
    uint8_t     motor_run_mode;         // Container to track the run mode
    uint16_t    motor_rpm;              //Container for track motor RPM in revs per minute
    // uint16_t    motor_start_timer;      // Used in start mode and increments on 10ms boundaries 
    
    uint16_t    tick10ms;               // Increases every 50mili tick1000monds 
    uint16_t    tick100ms;              // Used to keep track of passing deci-tick1000monds
    uint16_t    tick1000ms;             // Use this to tick tick1000monds -- might not be used for audio interface board

} GlobalInformation;


#endif