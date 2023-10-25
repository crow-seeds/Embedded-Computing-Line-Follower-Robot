/*
 * InterruptsMain.c
 */
/*
#include "Bump.h"
#include "Clock.h"
#include "UART0.h"
#include "LaunchPad.h"
#include <stdint.h>

volatile uint8_t reading;

void Hello_World_Example(void){
    Clock_Init();
    Bump_Init();
    UART0_Init();
    reading = 0;
    while(1){
        reading = Bump_Read();
        UART0_OutUDec(reading);
        UART0_OutString("\r");
        Clock_Delay1ms(1000);
    }
}

void Interrupts_Warm_Up(void){
    Clock_Init(); //initalize clock, bump, and UART0 stuff
    Bump_Init();
    UART0_Init();
    reading = 0; //set reading as 0 so i can view it in expressions tab for debugging
    while(1){
        reading = Bump_Read(); //set reading variable as the bump read data for debugging
        UART0_OutUDec(reading); //print it out through the UART terminal thing
        UART0_OutString(" ");
        Clock_Delay1ms(1000); //delay 1 second
    }
}

void toggleLedFor2Seconds(uint8_t bumpSensors){
    LaunchPad_LED(1);
    Clock_Delay1ms(2000);
    LaunchPad_LED(0);
}

void testingThing(uint8_t bumpSensors){
    printf("peepee");
}

void Interrupts_Race_Day(void){
    Clock_Init(); //initializes clock, doesnt do anything for race day
    Bump_Interrupt_Init(&toggleLedFor2Seconds); //sets the function that gets called during interrupt to toggle the red led for 2 seconds
    LaunchPad_Init();
    while(1){

    }
}

int main(void){
    //Un-commment only the function you are useing
    printf("test");
    //Hello_World_Example();
    //Interrupts_Warm_Up();
    Interrupts_Race_Day();
}

*/
