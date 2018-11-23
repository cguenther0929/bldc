/******************************************************************************
*   FILE: pwm.c
*
*   PURPOSE: Handles features related to PWM modules.
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

#include "pwm.h"


void PWM1Init(uint8_t timer){
	CCP1CONbits.CCP1M = 0xC;				//Binary value of 0xb1100 -- PWM mode: P1A, P1C, P1B, and P1D are active-high
	CCP1CONbits.P1M = 0x0;					//Single output: P1A, P1C, P1B, and P1D are controlled by steering
	switch (timer){
		case TIMER2:
			CCPTMRS0bits.C1TSEL = 0;		//Define ECCP1 PWM to run from timer 2
		break;
		case TIMER4:
			CCPTMRS0bits.C1TSEL = 1;		//Define ECCP1 PWM to run from timer 4
		break;
		case TIMER6:
			CCPTMRS0bits.C1TSEL = 2;		//Define ECCP1 PWM to run from timer 6
		break;
		case TIMER8:
			CCPTMRS0bits.C1TSEL = 3;		//Define ECCP1 PWM to run from timer 8
		break;
		case TIMER10:
			CCPTMRS0bits.C1TSEL = 4;		//Define ECCP1 PWM to run from timer 10
		break;
		default:
			CCPTMRS0bits.C1TSEL = 0;		//Define ECCP1 PWM to run from timer 2
		break;
	}
}

void PWM1EnableOuts(uint8_t outpin){
	switch (outpin) {
		case 'a':
			PSTR1CONbits.STRA=1;			//P1A pin has the PWM waveform with polarity control from CCP3M<1:0>
		break;
		case 'b':
			PSTR1CONbits.STRB=1;			//P1B pin has the PWM waveform with polarity control from CCP3M<1:0>
		break;	
		case 'c':
			PSTR1CONbits.STRC=1;			//P1C pin has the PWM waveform with polarity control from CCP3M<1:0>
		break;
		case 'd':
			PSTR1CONbits.STRD=1;			//P1D pin has the PWM waveform with polarity control from CCP3M<1:0>
		break;
		default:
			PSTR1CONbits.STRA=0;			//Output disabled -- will remain a port pin
			PSTR1CONbits.STRB=0;            //Output disabled -- will remain a port pin
			PSTR1CONbits.STRC=0;            //Output disabled -- will remain a port pin
			PSTR1CONbits.STRD=0;			//Output disabled -- will remain a port pin
		break;
	}
	
}

void PWM1_SetDutyCycle(uint16_t dutycycle){
	/*
    The datasheet confuses the reader by mislabelling the duty cycle cycle calculation
    on page 256 of 550.  This formula should be called pulse width, in which pulse width
    is defined by (1/Actual Frequency) * Desire duty cycle.  If we work out the equation,
    we end up with CCPxL:CCPxCON Setting = Desired Duty / (PWM Frequency * 100 * TOSC * Timer Prescaler)
    So the processor does not need to calculate the number each time, PWM Frequency * 100 * TOSC * Timer Prescaler is
    pre-calculated. The 100 value is in there so that we can pass an integer to the formula (i.e 50 for 50% opposed to 0.5)
    For this application, the frequency will be known at compile time, thus we can allow the compiler to 
    calculate (PWM Frequency * 100 * TOSC * Timer Prescaler) to save the processor from having to do 
    this math.  It will then only be required to calculate CCPxL:CCPxCON Setting = Desired Duty / Derived Constant,
    where "Derived Constant" is the value calculated at compile time.  See PWM.h for more information
    */
    
    uint16_t reg_value;
    uint16_t lowerbits;
	uint16_t upperbits;
    
    reg_value = (uint16_t)((float)(dutycycle/PWM_Duty_Cycle_Constant)); 
	
	lowerbits = (uint16_t)(reg_value & 0x0003);			//Just want the lower two bits
	upperbits = (uint16_t)((reg_value >> 2) & 0x00FF);	//Now grab the upper 8 bits
	CCP1CONbits.DC1B = lowerbits;						//Bits 1 and 0 of duty cycle value
	CCPR1L=upperbits;									//Bits [9:2] of duty cycle value
}

void PWM2Init(uint8_t timer){
	CCP2CONbits.CCP2M = 0xC;				//Binary value of 0xb1100 -- PWM mode: P2A, P2C, P2B, and P2D are active-high
	CCP2CONbits.P2M = 0x0;					//Single output: P2A, P2C, P2B, and P2D are controlled by steering

	switch(timer){
		case TIMER2:
			CCPTMRS0bits.C2TSEL = 0;		//Define ECCP1 PWM to run from timer 2
		break;
		case TIMER4:
			CCPTMRS0bits.C2TSEL = 1;		//Define ECCP2 PWM to run from timer 4
		break;
		case TIMER6:
			CCPTMRS0bits.C2TSEL = 2;		//Define ECCP2 PWM to run from timer 6
		break;
		case TIMER8:
			CCPTMRS0bits.C2TSEL = 3;		//Define ECCP2 PWM to run from timer 8
		break;
		case TIMER10:
			CCPTMRS0bits.C2TSEL = 4;		//Define ECCP2 PWM to run from timer 10
		break;
		default:
			CCPTMRS0bits.C2TSEL = 0;		//Define ECCP2 PWM to run from timer 2
		break;
	}
}

void PWM2EnableOuts(uint8_t outpin){
	switch (outpin) {
		case 'a':
			PSTR2CONbits.STRA=1;			//P1A pin has the PWM waveform with polarity control from CCP3M<1:0>
		break;
		case 'b':
			PSTR2CONbits.STRB=1;			//P1B pin has the PWM waveform with polarity control from CCP3M<1:0>
		break;	
		case 'c':
			PSTR2CONbits.STRC=1;			//P1C pin has the PWM waveform with polarity control from CCP3M<1:0>
		break;
		case 'd':
			PSTR2CONbits.STRD=1;			//P1D pin has the PWM waveform with polarity control from CCP3M<1:0>
		break;
		default:
			PSTR2CONbits.STRA=0;			//Output disabled -- will remain a port pin
			PSTR2CONbits.STRB=0;            //Output disabled -- will remain a port pin
			PSTR2CONbits.STRC=0;            //Output disabled -- will remain a port pin
			PSTR2CONbits.STRD=0;			//Output disabled -- will remain a port pin
		break;
	}
	
}

void PWM2_SetDutyCycle(uint16_t dutycycle){
	uint16_t reg_value;
    uint16_t lowerbits;
	uint16_t upperbits;
    
	reg_value = (uint16_t)((float)(dutycycle/PWM_Duty_Cycle_Constant)); 
    	
	lowerbits = (uint16_t)(reg_value & 0x0003);			//Just want the lower two bits
	upperbits = (uint16_t)((reg_value >> 2) & 0x00FF);	//Now grab the upper 8 bits
	CCP2CONbits.DC2B = lowerbits;					//Bits 1 and 0 of duty cycle value
	CCPR2L=upperbits;								//Bits [9:2] of duty cycle value
}