/******************************************************************************
*   FILE: uart.h
*
*   PURPOSE:  UART header file.  Contains UART routines for the MCU.
*               Target device is a PIC16F15xx 8 bit MCU.
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

#ifndef __H_UART_H
#define __H_UART_H

#include <xc.h>         //Part specific header file
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>  
#include "config.h"     //Project specific header file
#include <stdio.h>      
#include <string.h>     //Needed mainly for memcpy function
#include <stdlib.h>

extern struct UARTMembers {
    uint8_t rxchar;                   //Support this so that a PC can calibrate the unit
	uint8_t rxbuf[50];
};

/* RX HANDLING SIZE DEFINITION */
#define max_buffer          15         //Max buffer is 16 characters (0 to 15) 
#define TX_TMOUT            4000       //Max time to wait on TRMT1 (50bits at 115200 = 434.1uS).  DelayCount = TimeValue * (7.3728e6)    NOTE not assumeing FOSC/4

/* CONTROL CHARACTERS */
#define LF			1
#define CR			2

/********************************************************
*FUNCTION: void SetUpEUSART( void )
*PURPOSE: Set up the EUSART
*PRECONDITION: EUSART is not setup
*POSTCONDITION: EUSART is now ready to use
*RETURN: Nothing
********************************************************/
void SetUpUART( void );

/********************************************************
*FUNCTION: void UARTString (const char * y, uint8_t action )
*PURPOSE: Send a string out the UART
*PRECONDITION: None
*POSTCONDITION: String now sent out the UART
*RETURN: Nothing
********************************************************/
void UARTString (const char *y, uint8_t action );

/********************************************************
*FUNCTION: int IntegerInput(const char * y, uint8_t action)
*PURPOSE: User enteres a string of ascii characters and theseReceive
		are then converted to an integer number. 
*PRECONDITION: Uart must be initialized along with 
				uart RX interrupts.  
*POSTCONDITION: User allowed to enter a number 
*RETURN: Integer value of ascii string entered.  
********************************************************/
int IntegerInput(const char * y, uint8_t action);

//TODO: Need to comment
uint8_t BinaryInput(const char * y, uint8_t action);

/********************************************************
*FUNCTION: BYTE GetNumberByte( void )
*PURPOSE: Recieve numerical input from user (value 0 to 255).
			Values larger than 255 will not be accepted.
*PRECONDITION: Uart must be initialized along with 
				uart RX interrupts.  
*POSTCONDITION: User ascii number uint8_t is converted to a number.
*RETURN: Number that user entered (BYTE is max size)
********************************************************/
BYTE GetNumberByte( void );

/********************************************************
*FUNCTION: void TXmessage(const char * y)
*PURPOSE: Send message string to PTE PC. Specially developed
        for the PTE project, thus this function will not be
        relevant for any other projects.  
*PRECONDITION: UART must be configured
*POSTCONDITION: Message sent to PTE PC using defined protocol
*RETURN: Nothing
********************************************************/
void TXmessage(const char *y);

/********************************************************
*FUNCTION: void UARTRead( void )
*PURPOSE: Read the BYTE stored in the UART buffer
*PRECONDITION: None
*POSTCONDITION: BYTE now read from the UART buffer
*RETURN: Nothing
********************************************************/
void UARTRead( void );

/********************************************************
*FUNCTION: void PrintUnsignedDecimal (uint16_t number, uint8_t action)
*PURPOSE: Print an unsigned decimal value
*PRECONDITION: None
*POSTCONDITION: Unsigned decimal value should be printed
*RETURN: Nothing
********************************************************/
void PrintUnsignedDecimal (uint16_t number, uint8_t action);

/********************************************************
*FUNCTION: void PrintLargeDecimal (DWORD number, uint8_t action)
*PURPOSE: Print large number (up to 8 digits)
*PRECONDITION: UART must be configured
*POSTCONDITION: Unsigned decimal value should be printed
*RETURN: Nothing
********************************************************/
void PrintLargeDecimal (DWORD number, uint8_t action);

/********************************************************
*FUNCTION: void PrintBinInt (uint16_t number, uint8_t action)
*PURPOSE: Print a binary integer to the console (16 bits)
*PRECONDITION: None
*POSTCONDITION: Binary integer number printed to the console
*RETURN: Nothing
********************************************************/
void PrintBinInt (uint16_t number, uint8_t action);

/********************************************************
*FUNCTION: void PrintBinChar(uint16_t number, uint8_t action)
*PURPOSE: Print a binary number to the console (8 bits)
*PRECONDITION: None
*POSTCONDITION: Binary number now printed to console
*RETURN: Nothing
********************************************************/
void PrintBinChar(uint16_t number, uint8_t action);

/********************************************************
*FUNCTION: void ClearCursorUp( void )
*PURPOSE: To clear the screen from the cursor up
*PRECONDITION: None
*POSTCONDITION: Screen now cleared from the cursor on up
*RETURN: Nothing
********************************************************/
void ClearCursorUp( void );

/********************************************************
*FUNCTION: void ClearScreen( void )
*PURPOSE: To clear the console
*PRECONDITION: None
*POSTCONDITION: Screen is now cleared
*RETURN: Nothing
********************************************************/
void ClearScreen( void );

/********************************************************
*FUNCTION: void CursorTopLeft( void )
*PURPOSE: To place the cursor at the top left of the console
*PRECONDITION: None
*POSTCONDITION: Cursor now at the top left of the screen
*RETURN: Nothing
********************************************************/
void CursorTopLeft( void );

/********************************************************
*FUNCTION: ResetTerminal( void )
*PURPOSE: To reset the terminal.  This will clear everything
*PRECONDITION: None
*POSTCONDITION: The screen is not reset and completely cleared
*RETURN: Nothing
********************************************************/
void ResetTerminal( void );

/********************************************************
*FUNCTION: void PrintFloat (float number, uint8_t action)
*PURPOSE: Print a float number to the console
*PRECONDITION: None
*POSTCONDITION: Float number now printed to the console
*RETURN: Nothing
********************************************************/
void PrintFloat (float number, uint8_t action);

/********************************************************
*FUNCTION: void ClearLine( void )
*PURPOSE: Clear the current line that the cursor is in
*PRECONDITION: Line may be filled with characters
*POSTCONDITION: Line that cursor is on is now cleared
*RETURN: Nothing
********************************************************/
void ClearLine( void );

/********************************************************
*FUNCTION: void PrintChar( uint8_t inchar, uint8_t action )
*PURPOSE: Print a single character to the console
*PRECONDITION: None
*POSTCONDITION: Single character now printed to the console
*RETURN: Nothing
********************************************************/
void PrintChar( uint8_t inchar, uint8_t action );

/********************************************************
*FUNCTION: void Space( BYTE spaces )
*PURPOSE: In terminal, insert line feeds. Qty defined by 
*			spaces.
*PRECONDITION: None
*POSTCONDITION: Line feeds will be inserted.  
*RETURN: Nothing
********************************************************/
void Space( BYTE spaces );

/********************************************************
*FUNCTION: void TXWait( void )
*PURPOSE: Wait for the transmit buffer to empty or timeout 
        which ever happens first.  
*PRECONDITION: UART shall be setup.
*POSTCONDITION: Transmit buffer should be empty
*RETURN: Nothing
********************************************************/
void TXWait( void );

#endif
/* END OF FILE */