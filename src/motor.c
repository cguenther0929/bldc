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
    uint16_t    j;                          // Using counter 
    bool        inc_duty_flag = true;       // Prevent rapid attempts at increasing duty cycle when RPM conditions are met
    
    gblinfo.motor_rpm = START_RPM;
    duty_cycle = START_DUTY;                // Increase the duty_cycle as the RPM increases

    PWM1EnableOuts('a');                    // P1A pin has output pin
    PWM1_SetDutyCycle(START_DUTY);          // Will have to see if this is a reasonable value
    MotAlignment();
    gblinfo.motor_run_mode = OPEN_LOOP_START;

    // COMU_LED = ledon;                       // Active low signal, so pull low

    gblinfo.comu_state = DRV_B2A;           // Define initial commutate state
    UpdateMotorOutputs();

    while(gblinfo.motor_rpm <= OL_MAX_RPM){  //In loop until we hit the "started" RPM

        t4_period = (uint8_t)(TICKS_SCALER / gblinfo.motor_rpm);    //Define interrupts_per_second for commutation 
        
        for(i=0;i<10;i++) {
            Timer4On(t4_period);       
            while(TMR4IF == 0);
            TMR4IF = 0;
            UpdateComuState();
            UpdateMotorOutputs();
        }
        
        gblinfo.motor_rpm += OL_RAMP_RATE;

        if(gblinfo.motor_rpm >= 200 && gblinfo.motor_rpm < 400 && inc_duty_flag) {
            duty_cycle += 2;
            PWM1_SetDutyCycle(duty_cycle);
            inc_duty_flag = false;
        }
        
        else if(gblinfo.motor_rpm >= 400 && gblinfo.motor_rpm < 600 && !inc_duty_flag) {
            duty_cycle += 2;
            PWM1_SetDutyCycle(duty_cycle);
            inc_duty_flag = true;
        }
        
        else if(gblinfo.motor_rpm >= 600 && gblinfo.motor_rpm < 800 && inc_duty_flag) {
            duty_cycle += 3;
            PWM1_SetDutyCycle(duty_cycle);
            inc_duty_flag = false;
        }
        
        else if(gblinfo.motor_rpm >= 800 && gblinfo.motor_rpm < 1000 && !inc_duty_flag) {
            duty_cycle += 3;
            PWM1_SetDutyCycle(duty_cycle);
            inc_duty_flag = true;
        }
        
        else if(gblinfo.motor_rpm >= 1000  && gblinfo.motor_rpm < 1300 && inc_duty_flag) {
            duty_cycle += 4;
            PWM1_SetDutyCycle(duty_cycle);
            inc_duty_flag = false;
        }
        
        else if(gblinfo.motor_rpm >= 1400  && gblinfo.motor_rpm < 1600 && !inc_duty_flag) {
            duty_cycle += 4;
            PWM1_SetDutyCycle(duty_cycle);
            inc_duty_flag = true;
        }
        
        else if(gblinfo.motor_rpm >= 1700 && inc_duty_flag) {
            duty_cycle += 4;
            PWM1_SetDutyCycle(duty_cycle);
            inc_duty_flag = false;
        }
    }

    // Since it's a 14 pole motor, one revolution = 7*6 or 42 interrupts
    // TODO can we remove the following?  The foolowing was put in for debugging only.
    // i = 0;      //Reset the counter
    // while(true){
    //     Timer4On(t4_period);       
    //     while(TMR4IF == 0);
    //     TMR4IF = 0;
    //     UpdateComuState();
    //     UpdateMotorOutputs();
        
    //     if(i > 420 && gblinfo.comu_state == DRV_B2C){
    //         COMU_LED = ledoff;          
    //         // for(j=0;j<483;j++);       // Delay for approximately how long to drive previous state for 2000 RMP (derived empirically)
    //         for(j=0;j<1500;j++);       // Delay for approximately how long to drive previous state for 2000 RMP (derived empirically)
    //         break;
    //     }
    //     i++;
    // }
    
    // AHI_DRV = BHI_DRV = CHI_DRV = ALO_DRV = BLO_DRV = CLO_DRV = 0;          // May want to remove this line
    // AHI_LED = BHI_LED = CHI_LED = ALO_LED = BLO_LED = CLO_LED = ledoff;     // May want to remove this line
    
    gblinfo.motor_run_mode = NORMAL_RUN;        //Motor is now in normal run mode
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
    AHI_LED = BHI_LED = CHI_LED = ALO_LED = BLO_LED = CLO_LED = ledoff; 
    
    switch(gblinfo.comu_state){
        case DRV_B2A:                       // Previous step was B -> C. New step B -> A  
            AHI_DRV = CHI_DRV = 0;
            CLO_DRV = BLO_DRV = 0;
            
            BHI_DRV = 1; BHI_LED = ledon;   
            NOP();
            NOP();
            ALO_DRV = 1; ALO_LED = ledon;
            
            break;
        case DRV_C2A:                       // Previous step was B -> A. New step C -> A
            AHI_DRV = BHI_DRV = 0;
            CLO_DRV = BLO_DRV = 0;          
            
            ALO_DRV = 1; ALO_LED = ledon;   
            NOP();
            NOP();
            CHI_DRV = 1; CHI_LED = ledon;
            
            break;
        case DRV_C2B:                       // Previous step was C -> A.  New step C -> B
            AHI_DRV = BHI_DRV = 0;
            ALO_DRV = CLO_DRV = 0;          
            
            CHI_DRV = 1; CHI_LED = ledon;
            NOP();
            NOP();
            BLO_DRV = 1; BLO_LED = ledon;
            
            break;
        case DRV_A2B:                       // Previous step was C -> B. New step A -> B
            BHI_DRV = CHI_DRV =0;
            ALO_DRV = CLO_DRV = 0;
            
            BLO_DRV = 1; BLO_LED = ledon;
            NOP();
            NOP();
            AHI_DRV = 1; AHI_LED = ledon;   
            
            break;
        case DRV_A2C:                       // Previous step was A -> B. New step A -> C
            BHI_DRV = CHI_DRV = 0;
            ALO_DRV = BLO_DRV = 0;
            
            AHI_DRV = 1; AHI_LED = ledon;
            NOP();
            NOP();
            CLO_DRV = 1; CLO_LED = ledon;
            
            break;
        case DRV_B2C:                       // Previous step was A -> C. New step B -> C
            AHI_DRV = CHI_DRV = 0;
            ALO_DRV = BLO_DRV = 0;
            
            CLO_DRV = 1; CLO_LED = ledon;
            NOP();
            NOP();
            BHI_DRV = 1; BHI_LED = ledon;
            
            break;
        default:
            AHI_DRV = BHI_DRV = CHI_DRV = ALO_DRV = BLO_DRV = CLO_DRV = 0;          
            AHI_LED = BHI_LED = CHI_LED = ALO_LED = BLO_LED = CLO_LED = ledoff;
            break;

    }
    Timer1On(0,0);      //Start counting after a commutate switch.  
}

void ClosedLoopRun( void ) {
    uint16_t ctrval             = 0x0000;
    uint16_t ct_reg_val         = 0x0000;
    uint16_t rntime             = 0x0000;
    unsigned long pwm_width     = 0x00000000;  //Use large counters here in hopes to make math easier for processor
    unsigned long xovr_width    = 0x00000000;
    unsigned long target_width  = 0x00000000;

    DisableInterrupts();            //TODO debugging only 
    for(rntime=0;rntime<4200;rntime++){

        switch (gblinfo.comu_state) {
            case DRV_B2A:               // XOVR is Low and undriven phase is C.  Need to monitor at begining of pulse.
                while(true){
                    pwm_width = 0;
                    xovr_width = 0;
                    while(PWM_STATE){
                        COMU_LED = ~COMU_LED;
                        if(!CPH_CROSSED){
                            pwm_width++;
                            xovr_width++;
                        }
                        else
                        {
                            pwm_width++;
                        }
                    }

                    target_width = (pwm_width * TARGET_XOVR_PRCNT);         // Multiply by target percent (x10, so 900 for 90%)
                    target_width = (target_width >> 10);                                    // Divide by ~ 100*10 (2^10 = 1024)
                    
                    /* See if the midway point of the commutate switch has been found */
                    if(xovr_width >= target_width){
                        ct_reg_val = 0xFFFF - Timer1CtrVal();
                        Timer1On16b(ct_reg_val);
                        
                        /* Wait for the switch point to come around */
                        while(TMR1IF);
                        UpdateComuState();
                        UpdateMotorOutputs();
                        break;
                    }
                }

            break;
            
            case DRV_C2A:           // XOVR is HIGH and undriven phase is B.  Need to monitor near end of pulse
                while(true){
                    pwm_width = 0;
                    xovr_width = 0;
                    while(PWM_STATE){
                        COMU_LED = ~COMU_LED;
                        if(BPH_CROSSED){
                            pwm_width++;
                            xovr_width++;
                        }
                        else
                        {
                            pwm_width++;
                        }
                    }

                    target_width = (pwm_width * TARGET_XOVR_PRCNT);         // Multiply by target percent (x10, so 900 for 90%)
                    target_width = (target_width >> 10);                                    // Divide by ~ 100*10 (2^10 = 1024)
                    // target_width >>= 10;                                    // Divide by ~ 100*10 (2^10 = 1024)
                    
                    /* See if the midway point of the commutate switch has been found */
                    if(xovr_width >= target_width){
                        ct_reg_val = 0xFFFF - Timer1CtrVal();
                        Timer1On16b(ct_reg_val);
                        
                        /* Wait for the switch point to come around */
                        while(TMR1IF);
                        UpdateComuState();
                        UpdateMotorOutputs();
                        break;
                    }
                }
                
            break;
            
            case DRV_C2B:           // XOVR is LOW and undriven phase is A.  Need to monitor near begining of pulse
                while(true){
                    pwm_width = 0;
                    xovr_width = 0;
                    while(PWM_STATE){
                        COMU_LED = ~COMU_LED;
                        if(!APH_CROSSED){
                            pwm_width++;
                            xovr_width++;
                        }
                        else
                        {
                            pwm_width++;
                        }
                    }

                    target_width = (pwm_width * TARGET_XOVR_PRCNT);         // Multiply by target percent (x10, so 900 for 90%)
                    target_width = (target_width >> 10);                                    // Divide by ~ 100*10 (2^10 = 1024)
                    // target_width >>= 10;                                    // Divide by ~ 100*10 (2^10 = 1024)
                    
                    /* See if the midway point of the commutate switch has been found */
                    if(xovr_width >= target_width){
                        ct_reg_val = 0xFFFF - Timer1CtrVal();
                        Timer1On16b(ct_reg_val);
                        
                        /* Wait for the switch point to come around */
                        while(TMR1IF);
                        UpdateComuState();
                        UpdateMotorOutputs();
                        break;
                    }
                }
            
            break;

            case DRV_A2B:           // XOVR is HIGH and undriven phase is C.  Need to monitor near end of pulse
                while(true){
                    pwm_width = 0;
                    xovr_width = 0;
                    while(PWM_STATE){
                        COMU_LED = ~COMU_LED;
                        if(CPH_CROSSED){
                            pwm_width++;
                            xovr_width++;
                        }
                        else
                        {
                            pwm_width++;
                        }
                    }

                    target_width = (pwm_width * TARGET_XOVR_PRCNT);         // Multiply by target percent (x10, so 900 for 90%)
                    target_width = (target_width >> 10);                                    // Divide by ~ 100*10 (2^10 = 1024)
                    // target_width >>= 10;                                    // Divide by ~ 100*10 (2^10 = 1024)
                    
                    /* See if the midway point of the commutate switch has been found */
                    if(xovr_width >= target_width){
                        ct_reg_val = 0xFFFF - Timer1CtrVal();
                        Timer1On16b(ct_reg_val);
                        
                        /* Wait for the switch point to come around */
                        while(TMR1IF);
                        UpdateComuState();
                        UpdateMotorOutputs();
                        break;
                    }
                }
                
            break;

            case DRV_A2C:           // XOVR is LOW and undriven phase is B.  Need to monitor near begining of pulse
                while(true){
                    pwm_width = 0;
                    xovr_width = 0;
                    while(PWM_STATE){
                        COMU_LED = ~COMU_LED;
                        if(!BPH_CROSSED){
                            pwm_width++;
                            xovr_width++;
                        }
                        else
                        {
                            pwm_width++;
                        }
                    }

                    target_width = (pwm_width * TARGET_XOVR_PRCNT);         // Multiply by target percent (x10, so 900 for 90%)
                    target_width = (target_width >> 10);                                    // Divide by ~ 100*10 (2^10 = 1024)
                    // target_width >>= 10;                                    // Divide by ~ 100*10 (2^10 = 1024)
                    
                    /* See if the midway point of the commutate switch has been found */
                    if(xovr_width >= target_width){
                        ct_reg_val = 0xFFFF - Timer1CtrVal();
                        Timer1On16b(ct_reg_val);
                        
                        /* Wait for the switch point to come around */
                        while(TMR1IF);
                        UpdateComuState();
                        UpdateMotorOutputs();
                        break;
                    }
                }
                
            break;

            case DRV_B2C:               // XOVR is HIGH and undriven phase is A.  Need to monitor near end of pulse
                while(true){
                    pwm_width = 0;
                    xovr_width = 0;
                    while(PWM_STATE){
                        COMU_LED = ~COMU_LED;
                        if(APH_CROSSED){
                            pwm_width++;
                            xovr_width++;
                        }
                        else
                        {
                            pwm_width++;
                        }
                    }

                    target_width = (pwm_width * TARGET_XOVR_PRCNT);         // Multiply by target percent (x10, so 900 for 90%)
                    target_width = (target_width >> 10);                                    // Divide by ~ 100*10 (2^10 = 1024)
                    
                    /* See if the midway point of the commutate switch has been found */
                    if(xovr_width >= target_width){
                        ct_reg_val = 0xFFFF - Timer1CtrVal();
                        Timer1On16b(ct_reg_val);
                        
                        /* Wait for the switch point to come around */
                        while(TMR1IF);
                        UpdateComuState();
                        UpdateMotorOutputs();
                        break;
                    }
                }
            break;
        }
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