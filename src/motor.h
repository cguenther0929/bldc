/******************************************************************************
*   FILE: pwm.h
*
*   PURPOSE:  Header file for PWM souce file.  
*
*   DEVICE: PIC18F66K22
*
*   COMPILER: Microchip XC8 v1.10
*
*   IDE: MPLAB X v1.60
*
*   TODO:
*
*   NOTE:
*
******************************************************************************/

#ifndef _H_MOTOR_H
#define _H_MOTOR_H

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include "config.h"
#include "struct.h"
#include "main.h"



void TestCommutate(void);       /* Delete or comment*/

void MotAlignment(void);        /* need to comment */

void PerformCommutate( void );  /* need to comment */

void UpdateMotorOutputs ( void );   /* need to comment*/

void OpenLoopStart ( void );        /* todo need to comment*/

void UpdateComuState( void );       /* TODO need to comment */

#endif