/******************************************************************************
*   FILE: timer.h
*
*   PURPOSE: Header file for timer.c
*
*   DEVICE: PIC18F66K22
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

#ifndef __TIMER_H_
#define __TIMER_H_

#include <xc.h>                 //Part specific header file
#include <stdint.h>
#include <stdbool.h>
#include "config.h"             //Project specific header file

/********************************************************
*FUNCTION: void Timer0Init(bool interrupts, uint8_t prescaler, uint8_t clksource )
*PURPOSE: To initialize timer 0.  Note: function set up so that
        timer will operate in 16 bit mode.
*PRECONDITION: Timer 1 is not initialized
*POSTCONDITION: Timer 1 is ready to use
*RETURN: Nothing
********************************************************/
void Timer0Init(uint8_t interrupts, uint16_t prescaler, uint8_t clksource ) ;

/********************************************************
*FUNCTION: void Timer0On( void )
*PURPOSE: To turn ON timer 0
*PRECONDITION: Timer 1 could be OFF
*POSTCONDITION: Timer 1 is now ON
*RETURN: Nothing
********************************************************/
void Timer0On( void );

/********************************************************
*FUNCTION: void Timer1Init( uint8_t interrupts, uint8_t prescaler, bool clksource )
*PURPOSE: To initialize timer 1. When reading from timer one 
        in 16bit mode, we must read from the LOW byte first. 
        When we write to the timer, we must write to the LOW byte
        and then to the HIGH byte.  
*PRECONDITION: Timer 1 not initialized
*POSTCONDITION: Timer 1 now initialized
*RETURN: Nothing
********************************************************/
void Timer1Init( uint8_t interrupts, uint8_t prescaler, uint8_t clksource );

/********************************************************
*FUNCTION: void Timer1On(uint8_t RegHigh, uint8_t RegLow)
*PURPOSE: To turn ON timer 1
*PRECONDITION: Timer 1 is not running
*POSTCONDITION: Timer 1 now running with overflow period 
                defined by RegHigh and RegLow (and prescaler)
*RETURN: Nothing
********************************************************/
void Timer1On(uint8_t RegHigh, uint8_t RegLow);

/********************************************************
*FUNCTION: void Timer1Off( void )
*PURPOSE: To turn OFF timer 1
*PRECONDITION: Timer 1 is running
*POSTCONDITION: Timer 1 now stopped
*RETURN: Nothing
********************************************************/
void Timer1Off( void );

/********************************************************
*FUNCTION: uint16_t Timer1CtrVal( void )
*PURPOSE: To return the contents (16 bits) of timer 1's register
*PRECONDITION: Timer1 likely initialized
*POSTCONDITION: Timer1's register value (16 bit) is returned
*RETURN: Timer1's register value (16 bit) is returned
********************************************************/
uint16_t Timer1CtrVal( void );

/********************************************************
*FUNCTION: void Timer1CtrVal( uint16_t CtrRegVal )
*PURPOSE: Pass in 16b value and load timer 1's High/Low Register
*PRECONDITION: Timer1 likely initialized
*POSTCONDITION: Timer1's high/low registers are loaded
*RETURN: Nothing
********************************************************/
void Timer1On16b(uint16_t CtrRegVal);

/********************************************************
*FUNCTION: void Timer2Init( bool interrupts, uint8_t prescaler, uint8_t postscaler )
*PURPOSE: To initialize timer 2
*PRECONDITION: Timer 2 not initialized
*POSTCONDITION: Timer 2 now initialized
*RETURN: Nothing
********************************************************/
void Timer2Init(bool interrupts, uint8_t prescaler, uint8_t postscaler);

/********************************************************
*FUNCTION: void Timer2On(uint8_t period)
*PURPOSE: To turn ON timer 2
*PRECONDITION: Timer 2 is not running
*POSTCONDITION: Timer 2 now running with overflow period 
                defined by period register and pre/post scaler 
*RETURN: Nothing
********************************************************/
void Timer2On(uint8_t period);

/********************************************************
*FUNCTION: void Timer2Off( void )
*PURPOSE: To turn OFF timer 2
*PRECONDITION: Timer 2 could be running 
*POSTCONDITION: Timer 2 disabled 
*RETURN: Nothing
********************************************************/
void Timer2Off( void );

/********************************************************
*FUNCTION: void Timer3Init( bool interrupts, uint8_t prescaler, bool clksource )
*PURPOSE: To initialize timer 3
*PRECONDITION: Timer 3 not initialized
*POSTCONDITION: Timer 3 now initialized
*RETURN: Nothing
********************************************************/
void Timer3Init( bool interrupts, uint8_t prescaler, bool clksource );

/********************************************************
*FUNCTION: void Timer3On(uint8_t RegHigh, uint8_t RegLow)
*PURPOSE: To turn ON timer 3
*PRECONDITION: Timer 3 is not running
*POSTCONDITION: Timer 3 now running with overflow period 
                defined by RegHigh and RegLow (and prescaler)
*RETURN: Nothing
********************************************************/
void Timer3On(uint8_t RegHigh, uint8_t RegLow);

/********************************************************
*FUNCTION: void Timer3Off( void )
*PURPOSE: To turn OFF timer 3
*PRECONDITION: Timer 3 is running
*POSTCONDITION: Timer 3 now stopped
*RETURN: Nothing
********************************************************/
void Timer3Off( void );

/********************************************************
*FUNCTION: void Timer4Init( bool interrupts, uint8_t prescaler, uint8_t postscaler )
*PURPOSE: To initialize timer 4
*PRECONDITION: Timer 4 not initialized
*POSTCONDITION: Timer 4 now initialized
*RETURN: Nothing
********************************************************/
void Timer4Init( bool interrupts, uint8_t prescaler, uint8_t clksource );

/********************************************************
*FUNCTION: void Timer4On(uint8_t period)
*PURPOSE: To turn ON timer 4.  The perior register will 
        count up from 0x00 until the count value equals
        the user-defined period value.  At this point,
        the interrupt is triggered and the timer value 
        automatically reset.  
*PRECONDITION: Timer 4 is not running
*POSTCONDITION: Timer 4 now running with overflow period 
                defined by period register and pre/post scaler
*RETURN: Nothing
********************************************************/
void Timer4On( uint8_t period );

/********************************************************
*FUNCTION: void Timer4Off( void )
*PURPOSE: To turn OFF timer 2
*PRECONDITION: Timer 2 could be running 
*POSTCONDITION: Timer 2 disabled 
*RETURN: Nothing
********************************************************/
void Timer4Off( void );

#endif
/* END OF FILE */