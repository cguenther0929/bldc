/******************************************************************************
*   FILE: config.h
*
*   PURPOSE: Configuration file specific to the processor being used and the 
*           underlying hardware. 
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
#ifndef __CONFIG_H_
#define __CONFIG_H_

//#include "type.h"       //Needed for defined functions
#include <stdint.h>
#include <stdbool.h>
#include <xc.h>

/* PREPROCESSOR CALCULATION TO DETERMINE TIMER4'S INCREMENT RATE */
#define T4_Osc_Freq                     7372800                    //Oscillator used for MCU
#define T4_prescaler                    16.0
#define T4_postscaler                   16.0
#define T4_FOSCBY4                      (T4_Osc_Freq/4.0)
#define T4_SCALER                       (T4_prescaler*T4_postscaler)
#define T4_OSC                          (T4_FOSCBY4/T4_SCALER)

/* PREPROCESSOR CALCULATION FOR MOTOR RPM CALCULATIONS 
    To determine the necessary number of timer ticks between commutation switches 
    we use the following equation 65535 - ((60/(RPM*42))*(Timer Increment Rate))
*/
#define MOTOR_POLES                     14
#define POLE_PAIRS                      (MOTOR_POLES/2)
#define INTS_PER_REV                    (6*POLE_PAIRS)
#define TICKS_SCALER                    ((60.0*T4_OSC)/INTS_PER_REV)   //Pre-processor calculation for determineing oscillator ticks

/* DEFINE CODE VERSION NUMBER */
#define MAJVER              0x00
#define MINVER              0x00
#define BUGVER              0x01

/* DEFINES FOR MC PINS */
#define APH_CROSSED         PORTBbits.RB0           // Digital input indicating A phase crossed
#define BPH_CROSSED         PORTBbits.RB1           // Digital input indicating B phase crossed
#define CPH_CROSSED         PORTBbits.RB2           // Digital input indicating C phase crossed

#define MOT_PWM             LATCbits.LATC2          // Motor PWM

#define AHI_DRV             LATEbits.LATE0          // Control inputs for high-side/low-side drivers
#define BHI_DRV             LATEbits.LATE1
#define CHI_DRV             LATEbits.LATE2
#define ALO_DRV             LATEbits.LATE3
#define BLO_DRV             LATEbits.LATE4
#define CLO_DRV             LATEbits.LATE5

/* DEFINES FOR LED PINS */
#define COMU_LED            LATCbits.LATC3          //For driving the commutate LED
#define HEALTH_LED          LATCbits.LATC4          //For driving health LED

#define AHI_LED             LATFbits.LATF7          //Pins for driving commutate state indicators             
#define BHI_LED             LATGbits.LATG0
#define CHI_LED             LATGbits.LATG1
#define ALO_LED             LATGbits.LATG2
#define BLO_LED             LATGbits.LATG3
#define CLO_LED             LATGbits.LATG4

/* DEFINES FOR PUSH BUTTONS */
#define PB1                 PORTDbits.RD0            //Inputs from buttons
#define PB2                 PORTDbits.RD1
#define PB3                 PORTDbits.RD2

/* DEFINES FOR ANALOG SENSE */
#define MOT_CURRENT         4           //Analog channel for reading motor current

/* REGISTER VALUES FOR 10MS TIME BASE */         
#define TMR0HIGH            251         //32.768MHz with 256 prescaler and an interrupt every 10ms // 7.3728MHz with 16 prescaler at 10ms until the 32.768MHz oscillator shows up
#define TMR0LOW             127        


/* DEFINE VARIOUS PIN FUNCITONS */
#define output              0           //Define the output pin direction setting
#define input               1

/* INTUTIVE TIMER NAMES */
#define TIMER1              1
#define TIMER2              2
#define TIMER3              3
#define TIMER4              4
#define TIMER5              5
#define TIMER6              6
#define TIMER7              7
#define TIMER8              8
#define TIMER9              9   
#define TIMER10             10

// #define MtrTmrInit          Timer1Init
// #define MtrTmrOff           Timer1Off
// #define MtrTmrOn            Timer1On

/* MOTOR COMMUTATE STATES */
#define DRV_NONE            0
#define DRV_B2A             1
#define DRV_C2A             2
#define DRV_C2B             3
#define DRV_A2B             4
#define DRV_A2C             5
#define DRV_B2C             6

/* INTERMEDIATE COMMUTATE STATE */
#define CLOSED_LOOP_IDLE    1           // Closed loop has not been enabled 
#define JUST_SWITCHED       2           // A commutation switch just happened, and we're not looking for XOVER yet
#define LOOKING_FOR_XOVER   3           // Looking for XOVER, application should be very time sensitive
#define WAITING_FOR_SW      4           // XOVER detected, now counting 2x timer value for commutation switch

/* MOTOR RUN MODE */
#define MOTOR_OFF           1
#define OPEN_LOOP_START     2
#define NORMAL_RUN          3

/* VARIOUS MOTOR PARAMETERS */
#define OL_RAMP_RATE        20          //Increase the RPM value by this much every 500us 
#define OL_MAX_RPM          1000        //When this RPM value is achieved in OL start, transition to closed-loop run
#define START_RPM           100         //Open loop start task will use this as the starting RPM -- ramps up from here
#define START_DUTY          10          //Duty cycle percent for when starting motor

/* LED CONTROL */
#define ledon               0
#define ledoff              1

#endif