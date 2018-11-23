/******************************************************************************
*   FILE: adc.h
*
*   PURPOSE:  ADC header file.  This file will define all routines and
*           values that will be used by the ADC module.
*
*   DEVICE: PIC18LF6620
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

#ifndef __H_ADC_H
#define __H_ADC_H

#include <xc.h>         //Part specific header file
#include "config.h"     //Project specific header file
#include "math.h"

#define REF4D096        1
#define REF2D048        2
#define EXTREF          3
#define INTREF          4
#define AVDD            5


/********************************************************
*FUNCTION: void EnableA2D( void )
*PURPOSE: Turn on the A2D module
*PRECONDITION: A2D may be off
*POSTCONDITION: A2D module now on
*RETURN: Nothing
********************************************************/
void EnableA2D( void );

/********************************************************
*FUNCTION: uint8_t ReadA2D( uint8_t8_t CHAN, bool FILTER)
*PURPOSE: Read that ADC at the desired channel.
*   Take 16 samples and average
*   Function allows user to take a snapshot or 16 reading
*   average.
*PRECONDITION: Associated IO pins must be set properly
*POSTCONDITION: Value is read from the desired channel
*RETURN: 10 bit ADC reading from channel (little endin)
********************************************************/
uint16_t ReadA2D( uint8_t CHAN, bool FILTER );

/********************************************************
*FUNCTION: void InitA2D(bool format, uint8_t acqtime, uint8_t conversion_clk)
*PURPOSE: Initialize the analog to digital converter
*PRECONDITION: A2D not initialized
*POSTCONDITION: A2D is initialized
*RETURN: Nothing
********************************************************/
void InitA2D(bool format, uint8_t acqtime, uint8_t prescaler);

/********************************************************
*FUNCTION: EnableAnalogCh
*PURPOSE: To enable a particular pin to read an analog voltage
*PRECONDITION: This will work best if ANCON0 and ANCON1 are first
            cleared such that all other pins will function as 
            digital I/O
*POSTCONDITION: Analog channel defined by "Chan" will now
            read the analog voltage value present at the pin
*RETURN: Nothing
********************************************************/
void EnableAnalogCh(uint8_t Chan);

/********************************************************
*FUNCTION: void AnRefSel(uint8_t PosRef, uint8_t NegRef)
*PURPOSE: Allows the user a quick way to set up the references
        for the module.  Need to define both the positive and
        negative references.
*PRECONDITION: A2D references could be at default
*POSTCONDITION: A2D references now defined
*RETURN: Nothing
********************************************************/
void AnalogRefSel(uint8_t PosRef, uint8_t NegRef);

#endif
/* END OF FILE */