/******************************************************************************
*   FILE: adc.c
*
*   PURPOSE: Contains the routines that will read from the MCU's internal
*           ADC.
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

#include "adc.h"        //Include the header file for this module

void InitA2D(bool format, uint8_t acqtime, uint8_t prescaler) {

    switch (format) {
        case 0:
            ADFM = 0;       //Result is left justified EX: ADRESH = [Bit11, ... Bit4]  + [Bit3, Bit2, Bit1, Bit0, ADSGN, ADSGN, ADSGN, ADSGN]
            break;
        case 1:
            ADFM = 1;       //Result is right justified EX: ADRESH = [ADSGN, ADSGN, ADSGN, ADSGN, Bit11, ... Bit8] + [Bit7, ... Bit0]
            break;
    };
    
    switch (acqtime){
        case 0:
            ACQT2 = 0; ACQT1 = 0; ACQT0 = 0;
            break;
        case 2:
            ACQT2 = 0; ACQT1 = 0; ACQT0 = 1;
            break;
        case 4:
            ACQT2 = 0; ACQT1 = 1; ACQT0 = 0;
            break;
        case 6:
            ACQT2 = 0; ACQT1 = 1; ACQT0 = 1;
            break;    
        case 8:
            ACQT2 = 1; ACQT1 = 0; ACQT0 = 0;
            break;    
        case 12:
            ACQT2 = 1; ACQT1 = 0; ACQT0 = 1;
            break; 
        case 16:
            ACQT2 = 1; ACQT1 = 1; ACQT0 = 0;
            break; 
        case 20:
            ACQT2 = 1; ACQT1 = 1; ACQT0 = 1;
            break; 
        default:
            ACQT2 = 1; ACQT1 = 1; ACQT0 = 1;            //Safe approach is to give plenty of time
            break;
            
    }

    switch (prescaler) {
        case 2:             //FRC divided by 2 for A2D clock
            ADCS2 = 0;
            ADCS1 = 0;
            ADCS0 = 0;
            break;
        case 4:             //FRC divided by 4 for A2D clock
            ADCS2 = 1;
            ADCS1 = 0;
            ADCS0 = 0;
            break;
        case 8:             //FRC divided by 8 for A2D clock
            ADCS2 = 0;
            ADCS1 = 0;
            ADCS0 = 1;
            break;
        case 16:            //FRC divided by 16 for A2D clock
            ADCS2 = 1;
            ADCS1 = 0;
            ADCS0 = 1;
            break;
        case 32:            //FRC divided by 32 for A2D clock
            ADCS2 = 0;
            ADCS1 = 1;
            ADCS0 = 0;
            break;
        case 64:            //FRC divided by 64 for A2D clock
            ADCS2 = 1;
            ADCS1 = 1;
            ADCS0 = 0;
            break;
        default:            //FRC is dedicated RC oscillator
            ADCS2 = 0;
            ADCS1 = 1;
            ADCS0 = 1;
            break;
    };

}

void AnalogRefSel(uint8_t PosRef, uint8_t NegRef) {

    switch (PosRef) {
        case REF4D096:
            VCFG1 = 1; VCFG0 = 1;       //Internal 4.096 Reference
            break;
        
        case REF2D048:
            VCFG1 = 1; VCFG0 = 0;       //Internal 2.048 Reference
            break;
        
        case EXTREF:
            VCFG1 = 0; VCFG0 = 1;       //User wants external positive reference 
            break;
        
        case INTREF:
            VCFG1 = VCFG0 = 0;          //Reference is AVDD pin 
            break;

        default:
            VCFG1 = VCFG0 = 0;          //This way we know the ADC will (somewhat) work
        break;

    }
    
    (NegRef == EXTREF)?(VNCFG=1):(VNCFG=0);     //VNCFG = 1 = External VREF pin is used; VNCFG = 0 = External AVSS pin is used    

}

void EnableA2D( void ){
    ADON = 1;               //Turn the A2D module on!
}

void EnableAnalogCh(uint8_t Chan){
    switch (Chan){
        case 0:
            TRISA0 = input;
            ANSEL0 = 1;
            break;
        case 1:
            TRISA1 = input;
            ANSEL1 = 1;
            break;
        case 2:
            TRISA2 = input;
            ANSEL2 = 1;
            break;
        case 3:
            TRISA3 = input;
            ANSEL3 = 1;
            break;
        case 4:
            TRISA5 = input;
            ANSEL4 = 1;
            break;
        case 5:
            TRISF7 = input;
            ANSEL5 = 1;
            break;
        case 6:
            TRISF1 = input;
            ANSEL6 = 1;
            break;
        case 7:
            TRISF2 = input;
            ANSEL7 = 1;
            break;
        case 8:
            TRISF3 = input;
            ANSEL8 = 1;
            break;
        default:
            break;
            
    }
}

uint16_t ReadA2D( uint8_t CHAN, bool FILTER ) {

    uint8_t i;
    uint8_t j;
    uint8_t k;
    uint16_t avgarray[16];      //This is the array to store reading for later averaging
    uint16_t temp = 0;
    uint32_t sum = 0;
    uint16_t ADCH = 0;          //Use this to hold onto the high ADC VAL
    uint16_t ADCL = 0;          //Use this to hold onto the  ADC VAL

    ADON = 1;           //Turn on the ADC module

    switch(CHAN) {              //Set the correct CHS bits to read the value from the A2D -- then wait the acquisition time
            case 0: CHS3 = 0; CHS2 = 0; CHS1 = 0; CHS0 = 0; break;
            case 1: CHS3 = 0; CHS2 = 0; CHS1 = 0; CHS0 = 1; break;
            case 2: CHS3 = 0; CHS2 = 0; CHS1 = 1; CHS0 = 0; break;
            case 3: CHS3 = 0; CHS2 = 0; CHS1 = 1; CHS0 = 1; break;
            case 4: CHS3 = 0; CHS2 = 1; CHS1 = 0; CHS0 = 0; break;
            case 5: CHS3 = 0; CHS2 = 1; CHS1 = 0; CHS0 = 1; break;
            case 6: CHS3 = 0; CHS2 = 1; CHS1 = 1; CHS0 = 0; break;
            case 7: CHS3 = 0; CHS2 = 1; CHS1 = 1; CHS0 = 1; break;
            case 8: CHS3 = 1; CHS2 = 0; CHS1 = 0; CHS0 = 0; break;
            case 9: CHS3 = 1; CHS2 = 0; CHS1 = 0; CHS0 = 1; break;
            case 10: CHS3 = 1; CHS2 = 0; CHS1 = 1; CHS0 = 0; break;
            case 11: CHS3 = 1; CHS2 = 0; CHS1 = 1; CHS0 = 1; break;
            case 12: CHS3 = 1; CHS2 = 1; CHS1 = 0; CHS0 = 0; break;
            case 13: CHS3 = 1; CHS2 = 1; CHS1 = 0; CHS0 = 1; break;
            case 14: CHS3 = 1; CHS2 = 1; CHS1 = 1; CHS0 = 0; break;
            case 15: CHS3 = 1; CHS2 = 1; CHS1 = 1; CHS0 = 1; break;

            default: CHS3 = 0;  CHS2 = 0; CHS1 = 0; CHS0 = 0; break;
    }

    if(!FILTER) {                   //Here the user just wishes to take a snapshot (no averaging)
            GO = 1;                 //Start the conversion -- CHOLD is disconnected
            while (GO == 1);        //Wait here for the conversion to finish
            ADCH = ADRESH;          //Get the high value
            ADCH = ADCH << 8;       //Shift this to the upper byte of the 16 bit integer
            ADCH = ADCH & 0xFF00;   //Do this just to be safe.

            ADCL = ADRESL;          //Store the lower portion of the ADC reading
            temp = ADCH + ADCL;     //Build the 16 bit number
            return(temp);           //Return the high and low 10 bit number (little endian)

    }
    else {
            for(i = 0; i < 16; i++) {                       //Fill the average array
                    GO = 1;                                 //Start the conversion -- CHOLD is disconnected
                    while (GO == 1);                        //Wait here for the conversion to finish
                    ADCH = ADRESH;                          //store the upper portion of the ADC reading
                    ADCH = ADCH << 8;                       //Shift this value into the upper 8bits of the 16 bit value
                    ADCH = ADCH & 0xFF00;                   //Do this just to be safe
                
                    ADCL = ADRESL;                          //Store the lower 8 bits of the ADC reading
                    temp = ADCH + ADCL;                     //Build the 16 bit number
                
                    avgarray[i] = temp;                     //Put the value in the array
            }               
            sum = 0;                                        //Make sure this value is cleared
            for(i = 0; i < 16; i++) {                       //Get the average from the 16 values
                    sum = (uint32_t)(sum + avgarray[i]);        //Sum up all of the values
            }
            sum = (uint32_t)(sum / 16);   //Divide by the number of samples. -- could just bit shift here, but compiler most likely interprets this as a shift.
            temp = (uint16_t)(sum & 0xFFFF);

            return(temp);        //Return the average number (little endian)
    }

}
