/******************************************************************************
*   FILE: main.c
*
*   PURPOSE: Main source
*
*   DEVICE: PIC18F66K22
*
*   COMPILER: Microchip XC8 v1.32
*
*   IDE: MPLAB X v3.45
*
*   TODO:  When reading from timer one in 16bit mode, we must read from the LOW byte first!
*
*   NOTE:
*
******************************************************************************/

#include "main.h" //Include header file associated with main.c

// CONFIG1L
#pragma config RETEN = OFF      // VREG Sleep Enable bit (Disabled - Controlled by SRETEN bit)
#pragma config INTOSCSEL = HIGH // LF-INTOSC Low-power Enable bit (LF-INTOSC in High-power mode during Sleep)
#pragma config SOSCSEL = DIG    // SOSC Power Selection and mode Configuration bits (Digital (SCLKI) mode)
#pragma config XINST = OFF      // Extended Instruction Set (Disabled)

// CONFIG1H
#pragma config FOSC = EC3       // TODO Update this description Oscillator (Internal RC oscillator) with output on OSC2.  p45/550.  default IRCF<2:0> = 110 = 8MHz (on pin 8MHz/4 = 2MHz)
#pragma config PLLCFG = OFF     // PLL x4 Enable bit (Disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor (Disabled)
#pragma config IESO = OFF       // Internal External Oscillator Switch Over Mode (Disabled)

// CONFIG2L
#pragma config PWRTEN = ON      // Power Up Timer (Enabled)
#pragma config BOREN = SBORDIS  // Brown Out Detect (Enabled in hardware, SBOREN disabled)
#pragma config BORV = 1         // Brown-out Reset Voltage bits (2.7V)
#pragma config BORPWR = ZPBORMV // BORMV Power level (ZPBORMV instead of BORMV is selected)

// CONFIG2H
#pragma config WDTEN = OFF      // Watchdog Timer Disabled in HW
#pragma config WDTPS = 1024     // Watchdog Postscaler (1:1024)

// CONFIG3L
#pragma config RTCOSC = SOSCREF // RTCC Clock Select (RTCC uses SOSC)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 Mux (RC1)
#pragma config MSSPMSK = MSK7   // MSSP address masking (7 Bit address masking mode)
#pragma config MCLRE = ON       // Master Clear Enable (MCLR Enabled, RG5 Disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Overflow Reset (Enabled)
#pragma config BBSIZ = BB2K     // Boot Block Size (2K word Boot Block size)

// CONFIG5L
#pragma config CP0 = OFF        // Code Protect 00800-03FFF (Disabled)
#pragma config CP1 = OFF        // Code Protect 04000-07FFF (Disabled)
#pragma config CP2 = OFF        // Code Protect 08000-0BFFF (Disabled)
#pragma config CP3 = OFF        // Code Protect 0C000-0FFFF (Disabled)

// CONFIG5H
#pragma config CPB = OFF        // Code Protect Boot (Disabled)
#pragma config CPD = OFF        // Data EE Read Protect (Disabled)

// CONFIG6L
#pragma config WRT0 = OFF       // Table Write Protect 00800-03FFF (Disabled)
#pragma config WRT1 = OFF       // Table Write Protect 04000-07FFF (Disabled)
#pragma config WRT2 = OFF       // Table Write Protect 08000-0BFFF (Disabled)
#pragma config WRT3 = OFF       // Table Write Protect 0C000-0FFFF (Disabled)

// CONFIG6H
#pragma config WRTC = OFF       // Config. Write Protect (Disabled)
#pragma config WRTB = OFF       // Table Write Protect Boot (Disabled)
#pragma config WRTD = OFF       // Data EE Write Protect (Disabled)

// CONFIG7L
#pragma config EBRT0 = OFF      // Table Read Protect 00800-03FFF (Disabled)
#pragma config EBRT1 = OFF      // Table Read Protect 04000-07FFF (Disabled)
#pragma config EBRT2 = OFF      // Table Read Protect 08000-0BFFF (Disabled)
#pragma config EBRT3 = OFF      // Table Read Protect 0C000-0FFFF (Disabled)

// CONFIG7H
#pragma config EBRTB = OFF      // Table Read Protect Boot (Disabled)

struct GlobalInformation gblinfo;

void main()
{
    uint16_t i;
    SetUp();
    while (true) {
        if(PB1 == 0){
            TestCommutate();
            tick100msDelay(10);
        }
        if(PB2 == 0) {
            OpenLoopStart();
            ClosedLoopRun();
            
            tick100msDelay(10);
        }
    }

} //END Main()

void SetUp(void)
{
    /* PIN DIRECTIONS FOR ANALOG SELECT */
    TRISA5 = input;
    
    /* PIN DIRECTION FOR LEDS */
    TRISC3 = output;                    //Output for commutate LED
    TRISC4 = output;                    //Health LED

    TRISF7 = output;                    //Drives LED for A HI indicator
    TRISG0 = output;                    //Drives LED for B HI indicator
    TRISG1 = output;                    //Drives LED for C HI indicator
    TRISG2 = output;                    //Drives LED for A LO indicator
    TRISG3 = output;                    //Drives LED for B LO indicator
    TRISG4 = output;                    //Drives LED for C LO indicator

    /* PIN DIRECTION FOR MOTOR DRIVER */
    TRISB0 = input;                     //Digital input for A phase crossed 
    TRISB1 = input;                     //Digital input for B phase crossed
    TRISB2 = input;                     //Digital input for C phase crossed
    TRISC2 = output;                    //Motor PWM output

    TRISE0 = output;                    // A HI Gate Driver control pin           
    TRISE1 = output;                    // B HI Gate Driver control pin 
    TRISE2 = output;                    // C HI Gate Driver control pin 
    AHI_DRV = BHI_DRV = CHI_DRV = 0;    // For safety, turn these off now
    
    TRISE3 = output;                    // A LO Gate driver control pin
    TRISE4 = output;                    // B LO Gate driver control pin
    TRISE5 = output;                    // C LO Gate driver control pin
    ALO_DRV = BLO_DRV = CLO_DRV = 0;    // For safety, turn these off now

    /* PIN DIRECTION FOR PUSH BUTTONS */
    TRISD0 = input;                     // Push Button 1 input
    TRISD1 = input;                     // Push Button 2 input
    TRISD2 = input;                     // Push Button 3 input   
    
    /* INITIAL CONDITION OF COMMUTATE LEDS */
    AHI_LED = ledoff;
    BHI_LED = ledoff;
    CHI_LED = ledoff;
    ALO_LED = ledoff;
    BLO_LED = ledoff;
    CLO_LED = ledoff; 

    Init_Interrupts();                  //Set up interrupts  

    AnalogRefSel(EXTREF, EXTREF);       //User internal 2.048V reference and External VREF pin for negative reference -- page 216/380
    InitA2D(1, 2, 32);                  //Set up AD (Justification, Acq Time (TAD), Prescaler) ==> (Right, 16 TAD, RC Oscillator) -- page 361/550

    gblinfo.tick10ms = 0;       // Initialize 100s of a tick1000mond counter to 0
    gblinfo.tick100ms = 0;      // Initialize 100s of a tick1000mond counter to 0
    gblinfo.tick1000ms = 0;     // Seconds counter

    gblinfo.comu_state          = DRV_NONE;             //Initial motor driver state
    gblinfo.motor_run_mode      = MOTOR_OFF;            //Initial motor run state
    gblinfo.closed_loop_state   = CLOSED_LOOP_IDLE;     // Initial state for intermediate commutate state 
    gblinfo.motor_rpm           = START_RPM;
    // gblinfo.motor_start_timer   = 0;   //TODO remove this line

    /* DISABLE ANALOG CHANNELS */
    ANCON0 = 0x00;          // Analog channels 7-0 are configured for digital inputs. p.363     
    ANCON1 = 0x00;          // Analog channel 10-8 are configred for digital inputs. p.364
    EnableAnalogCh(4);      // Channel for current sense

    /* SETUP TIMER FOR PWM GENERATION */
    PWM1Init(TIMER2);				            //Initialize PWM 1 -- ARGS: Timer to use (options are 2, 4, 6, 8 and 10)
	PWM1EnableOuts('x');		                //ARGS: Output pin (ex 'a', 'b', 'c', or 'd').  'x' will leave the pin as digital I/O for now
	Timer2Init(0,PWM_Timer_Prescaler,1);  		//Args: Interrupts, Prescaler, Postscaler
	Timer2On(PWM_Timer_PR_Value);				//Args: Period register unsigned char

    /* SETUP TIMER FOR OPEN LOOP STARTING */
    Timer4Init(0,T4_prescaler,T4_postscaler);        //ARGS: No Interrupts; Prescaler of 16; Postscaler of 16
    Timer4Off();
    
    /* SETUP TIMER FOR COMMUTATE POINTS */
    Timer1Init(0,8,0);  // 0 = Do not cause interrupts, 8 = prescaler value (max value), 0 = clock source (set to internal Fosc/4)
    Timer1Off();
    
    /* TIMER FOR APPLICATION INTERRUPTS */
    Timer0Init(1, 256, 0); //ARGS: interrupts = yes, prescaler = 256, clksource = FOSC/4
    Timer0On();             
}

void tick100msDelay(uint16_t ticks)
{
    uint16_t i = 0;
    uint16_t tick = 0; //Used to lock time value
    for (i = ticks; i > 0; i--)
    {
        tick = gblinfo.tick100ms;
        while (tick == gblinfo.tick100ms)
            ; //Wait for time to wrap around (in one half tick1000mond)
    }
}

void tick10msDelay(uint16_t ticks)
{
    uint16_t i = 0;
    uint16_t tick = 0; //Used to lock time value
    for (i = ticks; i > 0; i--)
    {
        tick = gblinfo.tick10ms;
        while (tick == gblinfo.tick10ms); //Wait for time to wrap around (in one half tick1000mond)
    }
}
/* END OF FILE */