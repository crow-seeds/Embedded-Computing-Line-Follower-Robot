// Write me!
#include "PWM.h"
#include "driverlib.h"
#include <stdint.h>

#define TIMERA1_PERIOD 32000

    //set up the PWM configs for both the left and right motors
    Timer_A_PWMConfig config1 =
    {
            TIMER_A_CLOCKSOURCE_SMCLK,
            TIMER_A_CLOCKSOURCE_DIVIDER_1,
            20,
            TIMER_A_CAPTURECOMPARE_REGISTER_3,
            TIMER_A_OUTPUTMODE_RESET_SET,
            20
    };

    Timer_A_PWMConfig config2 =
    {
            TIMER_A_CLOCKSOURCE_SMCLK,
            TIMER_A_CLOCKSOURCE_DIVIDER_1,
            20,
            TIMER_A_CAPTURECOMPARE_REGISTER_4,
            TIMER_A_OUTPUTMODE_RESET_SET,
            20
    };

//initializes the PWM stuff, takes in period and duty (as a percentage, changes speed of motor)
void PWM_Init(uint16_t period, uint16_t duty3, uint16_t duty4){

    config1.timerPeriod = period;
    config2.timerPeriod = period;
    config1.dutyCycle = (duty3*period)/100; //converts duty cycle percentage to % of period
    config2.dutyCycle = (duty4*period)/100;

    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN6 | GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_Timer_A_generatePWM(TIMER_A0_BASE, &config1); //sets up as PWM, generates pwm signal
    MAP_Timer_A_generatePWM(TIMER_A0_BASE, &config2);
}


//both these methods change the duty cycle of the left and right motor
void PWM_Duty_Right(uint16_t duty1){
    config1.dutyCycle = (duty1*config1.timerPeriod)/100;
    MAP_Timer_A_generatePWM(TIMER_A0_BASE, &config1);
}

void PWM_Duty_Left(uint16_t duty1){
    config2.dutyCycle = (duty1*config2.timerPeriod)/100;
    MAP_Timer_A_generatePWM(TIMER_A0_BASE, &config2);
}
