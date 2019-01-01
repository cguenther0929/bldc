/******************************************************************************
*   FILE: timer.c
*
*   PURPOSE: Source file containing all timer-related routines.  
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

#include "timer.h"              //Include the header file for this module
#include "struct.h"             //Include the structure header file for global information

struct GlobalInformation gblinfo;

void Timer0On( void ){
    #ifndef TMR0HIGH                    //Assume TMR0LOW is also not defined
        TMR0H = 254;                   //Load the high register for the timer (16MHz and prescaler of 64)
        TMR0L = 191;                   	//Load the low register for the timer   (16MHz and prescaler of 64)
    #else
        TMR0H = TMR0HIGH;                   	//Load the high register for the timer (16MHz and prescaler of 64)
        TMR0L = TMR0LOW;                   	//Load the low register for the timer   (16MHz and prescaler of 64)
    #endif
    TMR0ON = 1;                 	//Set the bit to turn on the timer
}

void Timer0Init(uint8_t interrupts, uint16_t prescaler, uint8_t clksource ) {
    T0CONbits.T08BIT = 0;             //We want this timer to run in 16bit mode.
    
    switch(interrupts){
        case 0:
            INTCONbits.TMR0IE = 0;     //Do not cause an interrupt
            break;
        case 1:
            INTCONbits.TMR0IE = 1;     //Cause an interrupt
            TMR0IP = 0;                //We want this to be a low priority interrupt
            break;
        default:
            INTCONbits.TMR0IE = 0;     //Default is do not cause an interrupt
            break;

    }
    
    switch (prescaler){     //Set the prescaler
        case 1:         //User wished not to use prescaler.  Will run FOSC/4
            T0CONbits.PSA = 1;
            break;
        case 2:
            T0CONbits.PSA = 0;
            T0CONbits.T0PS = 0;
            break;
        case 4:
            T0CONbits.PSA = 0;
            T0CONbits.T0PS = 1;
            break;
        case 8:
            T0CONbits.PSA = 0;
            T0CONbits.T0PS = 2;
            break;
        case 16:
            T0CONbits.PSA = 0;
            T0CONbits.T0PS = 3;
            break;
        case 32:
            T0CONbits.PSA = 0;
            T0CONbits.T0PS = 4;
            break;
        case 64:
            T0CONbits.PSA = 0;
            T0CONbits.T0PS = 5;
            break;
        case 128:
            T0CONbits.PSA = 0;
            T0CONbits.T0PS = 6;
            break;
        case 256:
            T0CONbits.PSA = 0;
            T0CONbits.T0PS = 7;
            break;
        default:                //Will not use the prescaler
            T0CONbits.PSA = 1;
            break;
    }
    
    switch(clksource){
        case 0:             //Use the internal instruction clock (FOSC/4)
            T0CONbits.T0CS = 0;
            break;
        case 1:         //use external clock on T0CKI pin
            T0CONbits.T0CS = 1;
            T0CONbits.T0SE = 1;               //Increment timer on low to high input (when clock source is T0CKI) 
            break;
        default:
            T0CONbits.T0CS = 0;  //Default is to use internal clock source
            break;
    }
}

//TODO  Remove this note -- this will be the timer to count ticks between commutate and crossover
void Timer1Init( bool interrupts, uint8_t prescaler, bool clksource ) {
    
    T1CONbits.RD16 = 1;               //Allow 16 bits to be written to the timer register at at once.  
    switch(interrupts){
        case 0:
            PIE1bits.TMR1IE = 0;     //Do not cause an interrupt
            break;
        case 1:
            PIR1bits.TMR1IF = 0;     //Clear the flag to be safe
            PIE1bits.TMR1IE = 1;     //Enable the interrupt
            IPR1bits.TMR1IP = 0;     //We want this to be a low priority interrupt
            break;
        default:
            PIE1bits.TMR1IE = 0;     //Default is do not cause an interrupt
            break;

    }
    
    switch (prescaler){     //Set the prescaler
        case 1:         //User wished not to use prescaler.  Will run FOSC/4
            T1CKPS1 = 0;
            T1CKPS0 = 0; 
            break;
        case 2:
            T1CKPS1 = 0;
            T1CKPS0 = 1;
            break;
        case 4:
            T1CKPS1 = 1;
            T1CKPS0 = 0;
            break;
        case 8:
            T1CKPS1 = 1;
            T1CKPS0 = 1;
            break;
        default:                //Will not use the prescaler
            T1CKPS1 = 0;
            T1CKPS0 = 0;
            break;
      }

    switch(clksource){
        case 0:             //Use the internal instruction clock (FOSC/4)
            TMR1CS1 = 0; TMR1CS0 = 0;
            break;
        case 1:         //Timer cloCK is system clock (Fosc)
            TMR1CS1 = 0; TMR1CS0 = 1;
            break;
        case 2:
            TMR1CS1 = 1; TMR1CS0 = 0;  //Clock source is either  from a pin or oscillator (depends on SOSCI/SOSCO bit settings)
            break;
        default:
            TMR1CS1 = 0; TMR1CS0 = 0;  //Default is to use internal clock source
            break;
    }

}

void Timer1On(uint8_t RegHigh, uint8_t RegLow){
    TMR1H = RegHigh;                   //Load the low register for the timer
    TMR1L = RegLow;                   //Load the high register for the timer
    T1CONbits.TMR1ON = 1;                     //Set the bit to turn on the timer
}

void Timer1Off( void ){
    TMR1ON = 0;                     //Turn the timer off
}

void Timer2On( uint8_t period ){
    PR2 = period;						//Define the period for the timer.  
    TMR2ON = 1;                 	//Set the bit to turn on the timer
}

void Timer2Off( void ){
    TMR2ON = 0;                 	//Set the bit to turn on the timer
}

void Timer2Init( bool interrupts, uint8_t prescaler, uint8_t postscaler) {
    switch(interrupts){
        case 0:
            TMR2IE = 0;     //Do not cause an interrupt
            break;
        case 1:
            TMR2IE = 1;     //Cause an interrupt
            TMR2IP = 0;     //We want this to be a low priority interrupt
            break;
        default:
            TMR2IE = 0;     //Default is do not cause an interrupt
            break;
    };

    switch (prescaler){     //Set the prescaler
        case 1:         	//Prescaler of 1
            T2CKPS1=0;T2CKPS0=0;
            break;
        case 4:
            T2CKPS1=0;T2CKPS0=1;
            break;
        case 16:
            T2CKPS1=1;T2CKPS0=1;
            break;
        default:                //Will not use the prescaler
            T2CKPS1=0;T2CKPS0=0;
            break;
      };

	if(postscaler <= 16){
		postscaler -= 1;  			//This seeting is 0-based, so subtract one.  
		T2CONbits.T2OUTPS = (uint8_t)(postscaler & 0x0F);  //T2OUTPS is four bits wide
	}

}

void Timer3Init( bool interrupts, uint8_t prescaler, bool clksource ) {

    /* Some additional notes on this timer.  TXSYNC bit/functions are not supported here.  If such 
    features are desired, or this timer/counter operates from a source other than fosc/4, then code
    will need to be put in place for support.  By default RD16=0, which means the 16 bit read/write 
    operations are performed in two separate 8 bit operations */
    
    T3CONbits.RD16 = 1;     //Allow 16 bits to be written to the timer register at at once.  
    switch(interrupts){
        case 0:
            TMR3IE = 0;     //Do not cause an interrupt
            break;
        case 1:
            TMR3IF = 0;     //Clear the flag to be safe
            TMR3IE = 1;     //Cause an interrupt
            TMR3IP = 1;     //We want this to be a high priority interrupt
            break;
        default:
            TMR3IE = 0;     //Default is do not cause an interrupt
            break;

    }

    switch (prescaler){     //Set the prescaler
        case 1:         //User wished not to use prescaler.  Will run FOSC/4
            T3CKPS1 = 0;
            T3CKPS0 = 0; 
            break;
        case 2:
            T3CKPS1 = 0;
            T3CKPS0 = 1;
            break;
        case 4:
            T3CKPS1 = 1;
            T3CKPS0 = 0;
            break;
        case 8:
            T3CKPS1 = 1;
            T3CKPS0 = 1;
            break;
        default:                //Will not use the prescaler
            T3CKPS1 = 0;
            T3CKPS0 = 0;
            break;
      }

    switch(clksource){
        case 0:             //Use the internal instruction clock (FOSC/4)
            TMR3CS1 = 0; TMR3CS0 = 0;
            break;
        case 1:         //Timer input depends on SOSCEN bits
            TMR3CS1 = 1; TMR3CS0 = 0;
            break;
        case 2:         //Use non-divided system clock (FOSC)
            TMR3CS1 = 0; TMR3CS0 = 1;
            break;
        default:
             TMR3CS1 = 0; TMR3CS0 = 0;  //Default is to use internal clock source (FOSC/4)
             break;
    }

}

void Timer3On(uint8_t RegHigh, uint8_t RegLow){
    TMR3H = RegHigh;                   //Load the low register for the timer
    TMR3L = RegLow;                   //Load the high register for the timer
    TMR3ON = 1;                     //Set the bit to turn on the timer
}

void Timer3Off( void ){
    TMR3ON = 0;                     //Turn the timer off
}

void Timer4Init(  bool interrupts, uint8_t prescaler, uint8_t postscaler ) {
	    switch(interrupts){
        case 0:
            TMR4IE = 0;     //Do not cause an interrupt
            break;
        case 1:
            TMR4IE = 1;     //Cause an interrupt
            TMR4IP = 0;     //We want this to be a low priority interrupt
            break;
        default:
            TMR4IE = 0;     //Default is do not cause an interrupt
            break;
    };

    switch (prescaler){     //Set the prescaler
        case 1:         	//Prescaler of 1
            T4CKPS1=0;T4CKPS0=0;
            break;
        case 4:
            T4CKPS1=0;T4CKPS0=1;
            break;
        case 16:
            T4CKPS1=1;T4CKPS0=1;
            break;
        default:                //Will not use the prescaler
            T4CKPS1=0;T4CKPS0=0;
            break;
      };

	if(postscaler <= 16){
		postscaler -= 1;  			//This seeting is 0-based, so subtract one.  
		T4CONbits.T4OUTPS = (uint8_t)(postscaler & 0x0F);  //T2OUTPS is four bits wide
	}
}

void Timer4On( uint8_t period ){
    PR4 = period;					//Define the period for the timer.  
    TMR4ON = 1;                 	//Set the bit to turn on the timer
}

void Timer4Off( void ){
    TMR4ON = 0;                 	//Set the bit to turn on the timer
}