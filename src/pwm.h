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

#ifndef _H_PWM_H
#define _H_PWM_H

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include "config.h"
#include "struct.h"
#include "uart.h"
#include "math.h"

/* The following preprocessor calculations have been proven to work.  Let the float values be floats since
the compiler appears to typecast and truncate properly when necessary.  Let the preprocessor know we want floats
by including decimals in our constants (i.e. 1.0 vs 1) */

#define Processor_Oscillator_Freq   32768000            //User modified value
#define PWM_Frequency               45000               //User modified value
#define PWM_Timer_Prescaler         4                   //User modified value
#define Processor_TOSC              (1.0/Processor_Oscillator_Freq)
#define PWM_Duty_Cycle_Constant     (PWM_Frequency*100.0*Processor_TOSC*PWM_Timer_Prescaler)               //Math correct -- compared to datasheet 266/550
#define PWM_Timer_PR_Value          ((1.0/(PWM_Frequency*4.0*Processor_TOSC*PWM_Timer_Prescaler))-1.0)     //Math correct -- compare to datasheet 266/550

/********************************************************
*FUNCTION: void PWM1Init(uint8_t timer, uint16_t dutycycle)
*PURPOSE: To initialize the enhanced PWM 1 module 
*PRECONDITION: PWM 1 module has not been configured.  Will
*		also need clock resources
*POSTCONDITION: Enhanced PWM 1 module now initialized.  Appropriate
*		timer will need to be started for correct output.  Also
*		make sure config bit is set so that ECCP1 is available
*		on the proper pin
*RETURN: Nothing
********************************************************/
void PWM1Init(uint8_t timer);

/********************************************************
*FUNCTION: void PWM1EnableOuts(uint8_t output)
*PURPOSE: To enable a particualr PWM output pin
*PRECONDITION: PWM1Init should first be run.  
*POSTCONDITION: Specified output pin will now carry PWM signal.  
*		Appropriate timer must be configured and running.  
*RETURN: Nothing
********************************************************/
void PWM1EnableOuts(uint8_t outpin);

/********************************************************
*FUNCTION: void PWM1_SetDutyCycle(uint16_t dutycycle)
*PURPOSE: To load the duty cycle registers for PWM module 1
*PRECONDITION: PWM1Init should first be run.  Also, the proper
*		output drive should be enabled via PWM1EnableOuts
*POSTCONDITION: Duty cycle now entered and running  
*RETURN: Nothing
********************************************************/
void PWM1_SetDutyCycle(uint16_t dutycycle);

/********************************************************
*FUNCTION: void PWM2Init(uint8_t timer, uint16_t dutycycle)
*PURPOSE: To initialize the enhanced PWM 2 module 
*PRECONDITION: PWM 2 module has not been configured.  Will
*		also need clock resources
*POSTCONDITION: Enhanced PWM 2 module now initialized.  Appropriate
*		timer will need to be started for correct output.  Also
*		make sure config bit is set so that ECCP2 is available
*		on the proper pin
*RETURN: Nothing
********************************************************/
void PWM2Init(uint8_t timer);

/********************************************************
*FUNCTION: void PWM2EnableOuts(uint8_t output)
*PURPOSE: To enable a particualr PWM output pin
*PRECONDITION: PWM2Init should first be run.  
*POSTCONDITION: Specified output pin will now carry PWM signal.  
*		Appropriate timer must be configured and running.  
*RETURN: Nothing
********************************************************/
void PWM2EnableOuts(uint8_t outpin);

/********************************************************
*FUNCTION: void PWM2_SetDutyCycle(uint16_t dutycycle)
*PURPOSE: To load the duty cycle registers for PWM module 2
*PRECONDITION: PWM2Init should first be run.  Also, the proper
*		output drive should be enabled via PWM1EnableOuts
*POSTCONDITION: Duty cycle now entered and running  
*RETURN: Nothing
********************************************************/
void PWM2_SetDutyCycle(uint16_t dutycycle);

#endif	