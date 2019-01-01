/******************************************************************************
*   FILE: isr.c
*
*   PURPOSE: Interrupt service routines are contained in this file.  This file contains
*           algorithms that pertain to the interrupt service routine.
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

#include "isr.h"                //Include the header file for this module.

struct GlobalInformation gblinfo;

void Init_Interrupts( void ) {

    IPEN = 1;           //This allows interrupts to be defined a priority (either high or low) -- see page 103/380
    GIEH = 1;           //Enable all interrupts with high priority
    GIEL = 1;           //Enable all interrupts with low priority

}

void PORTBINTSetup( uint8_t channel, bool edge_rising, bool highpri ) {
	switch (channel) {
        case 0: 
            //No option to set priority on this interrupt -- it is always high!
            TRISB0 = input;
            INT0IF = 0;     //Clear this flag before enabling interrupt 
            INT0IE = 1;     //Enable the INT0 interrupt
            (edge_rising == true)?(INTEDG0 = 1):(INTEDG0 = 1);
        break;
        
        case 1:
            TRISB1 = input;
            INT1IF = 0;     //Clear this flag before enabling interrupt 
            INT1IE = 1;     //Enable INT1 interrupt 
            (highpri == true) ? (INT1IP = 1):(INT1IP = 0);
            (edge_rising == true)?(INTEDG1 = 1):(INTEDG1 = 1);
        break;
        
        case 2:
            TRISB2 = input;
            INT2IF = 0;     //Clear this flag before enabling interrupt 
            INT2IE = 1;     //Enable the INT2 interrupt
            (highpri == true) ? (INT2IP = 1):(INT2IP = 0);
            (edge_rising == true)?(INTEDG2 = 1):(INTEDG2 = 1);
        break;

        default:
        
        break;

    }

}

// void interrupt high_priority edges_isr( void ) {     
__interrupt (high_priority) void edges_isr( void ) {     
    
    if(INTCONbits.INT0IF){
        INTCONbits.INT0IF = 0;
    }
    
    // if(INTCON3bits.INT1IF){
    //     INTCON3bits.INT1IF = 0;     //Clear the interrupt flag
    //     INTCON3bits.INT1IE = 0;     //Clear the interrupt flag
    // }
    
    if(INTCON3bits.INT2IF){
        INTCON3bits.INT2IF = 0;     //Clear the interrupt flag
    }

    if(INTCON3bits.INT3IF) {
        INTCON3bits.INT3IF = 0;     //Clear the interrupt flag
    }

}

// void interrupt low_priority main_isr( void ) {
__interrupt (low_priority) void main_isr( void ) {

    uint8_t temp = 0;
    
    if(TMR0IF){                                     //Timer 1 interrupt
        TMR0H = TMR0HIGH;                        //Load the high register for the timer -- looking for 1/100 of a tick1000ms
        TMR0L = TMR0LOW;                        //Load the low register for the timer
        
        Events10ms();
        
        if(gblinfo.tick10ms >= 9) {
            gblinfo.tick10ms = 0;               //Reset centi-tick1000monds
            Events100ms();
            if(gblinfo.tick100ms >= 9) {         //Once Second Reached
                gblinfo.tick100ms = 0;           //Reset 100 milliseconds ounter
                Events1000ms();                 //Look at events that are to happen every 1s
                if(gblinfo.tick1000ms >= 59)                     //We've ticked away one minute, so reset
                    gblinfo.tick1000ms = 0;                      //Reset seconds counter
                else
                    gblinfo.tick1000ms += 1;                     //Increment seconds counter
            }
            else {
                 gblinfo.tick100ms += 1;                         //Increment 100 millisecond timer 
            }
        }
        else {
            gblinfo.tick10ms += 1;                               //Increment 1 millisecond timer
        }

        TMR0IF = 0;                         //Software is responsible for clearing this flag
    }   /* END IF FOR TMR1IF */
    
    // if(PIR2bits.TMR3IF){
        // PIR2bits.TMR3IF = 0;                     //Clear the interrupt flag  
    // } /* END IF FOR TMR3IF */


    if(INT1IF){
        INTCON3bits.INT1IF = 0;     //Clear the interrupt flag
        // if(RB1 == 1){         //TODO remove this junk
        //     gblinfo.wakeedge = true;
        // }
    }
    
    // if(INTCON3bits.INT2IF){
    //     INTCON3bits.INT2IF = 0;     //Clear the interrupt flag
    // }

    // if(INTCON3bits.INT3IF) {
    //     INTCON3bits.INT3IF = 0;     //Clear the interrupt flag
    // }
    
} /* END void interrupt low_priority main_isr( void ) */

void Events10ms(void) {                  //Keep routine slim!
}

void Events100ms(void) {                //Keep routine slim!
    
}

void Events1000ms(void) {
    HEALTH_LED = ~HEALTH_LED;
}


void DisableInterrupts( void ) {
    GIEH = 0;           //Disable high priority interrupts
    GIEL = 0;           //Disable low priority interrupts
}

void EnableInterrupts( void ) {
    GIEH = 1;           //Enable high priority interrupts
    GIEL = 1;           //Enable low priority interrupts
}