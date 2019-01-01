/******************************************************************************
*   FILE: motor.c
*
*   PURPOSE: Contains all motor-related tasks.  
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

#include "motor.h" //Include header file associated with motor.c

struct GlobalInformation gblinfo;

void MotAlignment( void ) {
    AHI_DRV = BHI_DRV = CHI_DRV = ALO_DRV = BLO_DRV = CLO_DRV = 0; // Reset FET outputs, and some deadband
    
    //Drive the B->A phase for a short duration then hit the lowside FETs to align the motor
    BHI_DRV = 1;
    ALO_DRV = 1;
    tick10msDelay(1);
    BHI_DRV = 0;
    BLO_DRV = 1;
    tick10msDelay(1);
    BLO_DRV = 0;
    ALO_DRV = 0;
    
    AHI_DRV = BHI_DRV = CHI_DRV = ALO_DRV = BLO_DRV = CLO_DRV = 0; // Reset FET outputs, and some deadband
    tick10msDelay(1);
}

void OpenLoopStart ( void ) {     
    uint8_t     t4_period;
    uint8_t     duty_cycle;  
    uint16_t    i;                          // Using counter 
    bool        inc_duty_flag = true;       // Prevent rapid attempts at increasing duty cycle when RPM conditions are met
    
    gblinfo.motor_rpm = START_RPM;
    duty_cycle = START_DUTY;                // Increase the duty_cycle as the RPM increases

    PWM1EnableOuts('a');                    // P1A pin has output pin
    PWM1_SetDutyCycle(START_DUTY);          // Will have to see if this is a reasonable value
    MotAlignment();
    gblinfo.motor_run_mode = OPEN_LOOP_START;

    gblinfo.comu_state = DRV_B2A;           // Define initial commutate state
    UpdateMotorOutputs();

    // delay_tick = 3307;                      // Derived from Excel calculator for 10RPM start
    // delay_reduction = 300;

    // for(i=0;i<500;i++) {
    //     for(j=0;j<delay_tick;j++);
    //     // UpdateComuState();
    //     // UpdateMotorOutputs();
    //     AHI_LED = ~AHI_LED;             // TODO DEBUG ONLY
    // }

    // while (delay_tick > 30) {
    //     for(i=0;i<42;i++) {
    //         for(j=0;j<delay_tick;j++);
    //         UpdateComuState();
    //         UpdateMotorOutputs();
    //         AHI_LED = ~AHI_LED;             // TODO DEBUG ONLY
    //     }
    //     delay_tick -= delay_reduction;
    // }

    // for(i=0;i<5000;i++) {                   // Run at this speed for a while
    //     for(j=0;j<delay_tick;j++);
    //     UpdateComuState();
    //     UpdateMotorOutputs();
    //     AHI_LED = ~AHI_LED;                 // TODO DEBUG ONLY
    // }

    // while(gblinfo.motor_rpm <= OL_MAX_RPM){  //In loop until we hit the "started" RPM
    //     tmr1_reg_setting = 54015;
    //     tmr1_reg_hi = (uint8_t)(tmr1_reg_setting / 256);
    //     tmr1_reg_lo = (uint8_t)(tmr1_reg_setting % 256);

    //     TMR1IE = 0;
    //     TMR1IF = 0;
    //     Timer1On(tmr1_reg_hi,tmr1_reg_lo);                                          //register values set for 20ms
    //     t4_period = (uint8_t)(TICKS_SCALER / gblinfo.motor_rpm);    //Define interrupts_per_second for commutation 
        
    //     while(TMR1IF == 0){         //Run at this RPM until 500us timer expires
    //         Timer4On(t4_period);       
    //         while(TMR4IF == 0);
    //         TMR4IF = 0;
    //         AHI_LED = ~AHI_LED;             //TODO DEBUG ONLY
    //         UpdateComuState();
    //         UpdateMotorOutputs();
    //     }
        
    //     BHI_LED = ~BHI_LED;
    //     gblinfo.motor_rpm += OL_RAMP_RATE;
    //     if(gblinfo.motor_rpm >= 1000)
    //         PWM1_SetDutyCycle(20);
    // }

    // rpm_ramp_rate = OL_RAMP_RATE;
    
    while(gblinfo.motor_rpm <= OL_MAX_RPM){  //In loop until we hit the "started" RPM

        t4_period = (uint8_t)(TICKS_SCALER / gblinfo.motor_rpm);    //Define interrupts_per_second for commutation 
        
        for(i=0;i<10;i++) {
            Timer4On(t4_period);       
            while(TMR4IF == 0);
            TMR4IF = 0;
            // AHI_LED = ~AHI_LED;             // TODO DEBUG ONLY
            UpdateComuState();
            UpdateMotorOutputs();
        }
        
        gblinfo.motor_rpm += OL_RAMP_RATE;

        if(gblinfo.motor_rpm >= 200 && gblinfo.motor_rpm < 400 && inc_duty_flag) {
            // ALO_LED = ledon;
            duty_cycle += 2;
            PWM1_SetDutyCycle(duty_cycle);
            inc_duty_flag = false;
        }
        
        else if(gblinfo.motor_rpm >= 400 && gblinfo.motor_rpm < 600 && !inc_duty_flag) {
            // ALO_LED = ledon;
            duty_cycle += 2;
            PWM1_SetDutyCycle(duty_cycle);
            inc_duty_flag = true;
        }
        
        else if(gblinfo.motor_rpm >= 600 && gblinfo.motor_rpm < 800 && inc_duty_flag) {
            // BLO_LED = ledon;
            duty_cycle += 2;
            PWM1_SetDutyCycle(duty_cycle);
            inc_duty_flag = false;
        }
        
        else if(gblinfo.motor_rpm >= 800 && gblinfo.motor_rpm < 1000 && !inc_duty_flag) {
            // BLO_LED = ledon;
            duty_cycle += 2;
            PWM1_SetDutyCycle(duty_cycle);
            inc_duty_flag = true;
        }
        
        else if(gblinfo.motor_rpm >= 1000 && inc_duty_flag) {
            // CLO_LED = ledon;
            duty_cycle += 2;
            PWM1_SetDutyCycle(duty_cycle);
            inc_duty_flag = false;
        }
    }

    tick100msDelay(50);         //  Small delay so the last state remains driven.  
    gblinfo.motor_run_mode = NORMAL_RUN;        //Motor is now in normal run mode
    AHI_DRV = BHI_DRV = CHI_DRV = ALO_DRV = BLO_DRV = CLO_DRV = 0; // TODO debugging only!
    TMR4IF = 0;
    Timer4Off();

}

void UpdateComuState( void ) {
    //Based on the current state, increment to the next state
    switch (gblinfo.comu_state){
        case DRV_B2A:
            gblinfo.comu_state = DRV_C2A;
            break;
        case DRV_C2A:
            gblinfo.comu_state = DRV_C2B;
            break;
        case DRV_C2B:
            gblinfo.comu_state = DRV_A2B;
            break;
        case DRV_A2B:
            gblinfo.comu_state = DRV_A2C;
            break;
        case DRV_A2C:
            gblinfo.comu_state = DRV_B2C;
            break;
        case DRV_B2C:
            gblinfo.comu_state = DRV_B2A;
            break;
        default:
            gblinfo.comu_state = DRV_NONE;

    }

}

void UpdateMotorOutputs ( void ){
    AHI_DRV = BHI_DRV = CHI_DRV = ALO_DRV = BLO_DRV = CLO_DRV = 0; // Reset FET outputs, and some deadband
    AHI_LED = BHI_LED = CHI_LED = ALO_LED = BLO_LED = CLO_LED = ledoff; // Reset FET outputs, and some deadband
    
    switch(gblinfo.comu_state){
        case DRV_B2A:
            BHI_DRV = 1; BHI_LED = ledon;
            ALO_DRV = 1; ALO_LED = ledon;
            break;
        case DRV_C2A:
            CHI_DRV = 1; CHI_LED = ledon;
            ALO_DRV = 1; ALO_LED = ledon;
            break;
        case DRV_C2B:
            CHI_DRV = 1; CHI_LED = ledon;
            BLO_DRV = 1; BLO_LED = ledon;
            break;
        case DRV_A2B:
            AHI_DRV = 1; AHI_LED = ledon;   
            BLO_DRV = 1; BLO_LED = ledon;
            break;
        case DRV_A2C:
            AHI_DRV = 1; AHI_LED = ledon;
            CLO_DRV = 1; CLO_LED = ledon;
            break;
        case DRV_B2C:
            BHI_DRV = 1; BHI_LED = ledon;
            CLO_DRV = 1; CLO_LED = ledon;
        default:
            AHI_DRV = BHI_DRV = CHI_DRV = ALO_DRV = BLO_DRV = CLO_DRV = 0; // Reset FET outputs, and some deadband

    }
}

void TestCommutate(void) {
    uint16_t i;
    AHI_DRV = ALO_DRV = BHI_DRV = BLO_DRV = CHI_DRV = CLO_DRV = 0;
    PWM1EnableOuts('a');        //P1A pin has output pin
    PWM1_SetDutyCycle(25);      //Enter value in percent
    tick10msDelay(1);
    MotAlignment();
    
    for(i=0;i<7;i++) {
        BHI_DRV = 1; BHI_LED = ledon;               //B -> A
        ALO_DRV = 1; ALO_LED = ledon;                
        tick10msDelay(10);
        AHI_LED = ALO_LED = BHI_LED = BLO_LED = CHI_LED = CLO_LED = ledoff;
        AHI_DRV = ALO_DRV = BHI_DRV = BLO_DRV = CHI_DRV = CLO_DRV = 0;
        NOP();

        CHI_DRV = 1; CHI_LED = ledon;               //C -> A
        ALO_DRV = 1; ALO_LED = ledon;               
        tick10msDelay(10);
        AHI_DRV = ALO_DRV = BHI_DRV = BLO_DRV = CHI_DRV = CLO_DRV = 0;
        AHI_LED = ALO_LED = BHI_LED = BLO_LED = CHI_LED = CLO_LED = ledoff;
        NOP();
        
        CHI_DRV = 1; CHI_LED = ledon;                //C -> B
        BLO_DRV = 1; BLO_LED = ledon;               
        tick10msDelay(10);
        AHI_DRV = ALO_DRV = BHI_DRV = BLO_DRV = CHI_DRV = CLO_DRV = 0;
        AHI_LED = ALO_LED = BHI_LED = BLO_LED = CHI_LED = CLO_LED = ledoff;
        NOP();
        
        AHI_DRV = 1; AHI_LED = ledon;                //A -> B
        BLO_DRV = 1; BLO_LED = ledon;               
        tick10msDelay(10);
        AHI_DRV = ALO_DRV = BHI_DRV = BLO_DRV = CHI_DRV = CLO_DRV = 0;
        AHI_LED = ALO_LED = BHI_LED = BLO_LED = CHI_LED = CLO_LED = ledoff;
        NOP();
        
        AHI_DRV = 1; AHI_LED = ledon;                //A -> C
        CLO_DRV = 1; CLO_LED = ledon;               
        tick10msDelay(10);
        AHI_DRV = ALO_DRV = BHI_DRV = BLO_DRV = CHI_DRV = CLO_DRV = 0;
        AHI_LED = ALO_LED = BHI_LED = BLO_LED = CHI_LED = CLO_LED = ledoff;
        NOP();
        
        BHI_DRV = 1; BHI_LED = ledon;                //B -> C
        CLO_DRV = 1; CLO_LED = ledon;
        tick10msDelay(10);
        AHI_DRV = ALO_DRV = BHI_DRV = BLO_DRV = CHI_DRV = CLO_DRV = 0;
        AHI_LED = ALO_LED = BHI_LED = BLO_LED = CHI_LED = CLO_LED = ledoff;
        NOP();
    }
    
    AHI_DRV = ALO_DRV = BHI_DRV = BLO_DRV = CHI_DRV = CLO_DRV = 0;
    AHI_LED = ALO_LED = BHI_LED = BLO_LED = CHI_LED = CLO_LED = ledoff;
}