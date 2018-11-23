/******************************************************************************
*   FILE: isr.h
*
*   PURPOSE: Header file for isr.c
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

#ifndef __ISR_H_
#define __ISR_H_

#include <xc.h>                 //Part specific header file
#include <stdint.h>
#include <stdbool.h>
#include "config.h"             //Project specific header file
#include "main.h"
#include "struct.h"

/********************************************************
*FUNCTION: void Init_Interrupts( void )
*PURPOSE: This function is responsible for initializing
    interrupt features
*PRECONDITION: Desired interrupts not yet supports
*POSTCONDITION: Desired interrupts now enabled
*RETURN: Nothing
********************************************************/
void Init_Interrupts( void );

/********************************************************
*FUNCTION: void interrupt low_priority main_isr( void )
*PURPOSE: This is the low priority interrupt that will handle
    the majority of the interrupts generated.  Examples would
    be the UART and timer.  These are non-critical interrupts
*PRECONDITION: Interrupt flag not set.
*POSTCONDITION: Interrupt respond to and flag cleared
*RETURN: Nothing
********************************************************/
void interrupt low_priority main_isr( void );

/********************************************************
*FUNCTION: void interrupt high_priority edges_isr( void )
*PURPOSE: This is where the vector will point when we detect
    and edge on one of the TACH signal inputs
*PRECONDITION: Interrupt flag not set.
*POSTCONDITION: Interrupt respond to and flag cleared
*RETURN: Nothing
********************************************************/
void interrupt high_priority edges_isr( void );

/********************************************************
*FUNCTION: void DisableInterrupts( void )
*PURPOSE: Will prevent interrupts from firing
*PRECONDITION: Interrupts may be enabled.
*POSTCONDITION: Interrupts now disabled.
*RETURN: Nothing
********************************************************/
void DisableInterrupts( void );

/********************************************************
*FUNCTION: void EnableInterrupts( void )
*PURPOSE: Enable high and low priority interrupts
*PRECONDITION: Interrupts may be disabled.
*POSTCONDITION: Interrupts now enabled.
*RETURN: Nothing
********************************************************/
void EnableInterrupts( void );

/********************************************************
*FUNCTION: void Events10ms(void)
*PURPOSE: Runs on a 1 millisecond interrupt boundary.  
*       This routine shall be kept slim!
*PRECONDITION: Interrupts and time base should be setup
*POSTCONDITION: Action taken on a 1ms boundary
*RETURN: Nothing
********************************************************/
void Events10ms(void);  

/********************************************************
*FUNCTION: void Events100ms(void)
*PURPOSE: Runs on a 100 millisecond interrupt boundary.  
*       This routine shall be kept slim!
*PRECONDITION: Interrupts and time base should be setup
*POSTCONDITION: Action taken on a 100ms boundary
*RETURN: Nothing
********************************************************/
void Events100ms(void); 

/********************************************************
*FUNCTION: void Events1000ms(void)
*PURPOSE: Runs on a 1000 millisecond interrupt boundary.  
*       This routine shall be kept slim!
*PRECONDITION: Interrupts and time base should be setup
*POSTCONDITION: Action taken on a 1000ms boundary
*RETURN: Nothing
********************************************************/
void Events1000ms(void);       

/********************************************************
*FUNCTION: void PORTBINTSetup( uint8_t channel )
*PURPOSE: Initialize interrupts (i.e. INT0)
*PRECONDITION: Interrupts must be enabled.  Not that defineing
* as high priority will require the flag be moved into the 
* appropriate ISR function.  
*POSTCONDITION: INTx interrupts now setup 
*RETURN: Nothing
********************************************************/
void PORTBINTSetup( uint8_t channel, bool edge_rising, bool highpri );     

#endif
/* END OF FILE */