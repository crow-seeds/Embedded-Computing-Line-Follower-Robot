// Zach Hicks

/* DriverLib Includes */
#include "driverlib.h"

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>

/* Project Includes */
#include "Clock.h"

// Initialize HFXT oscillator in the power state allowing for maximum operating frequency of 48 MHz
void HFXT_Init(void){
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_PJ, GPIO_PIN3 | GPIO_PIN2, GPIO_PRIMARY_MODULE_FUNCTION); // set PJ pins 2 and 3 to HFXT in/out mode
    PCM_setCoreVoltageLevel(PCM_VCORE1);    // enable higher power state to allow 48 MHZ operation
    FlashCtl_setWaitState(FLASH_BANK0, 3);  // wait state for flash controller, must be called when changing clock frequency ranges
    FlashCtl_setWaitState(FLASH_BANK1, 3);
    CS_startHFXT(false);
}

#define CLK_FREQ    48000000    // 48 MHz crystal

void Clock_Init(){
    CS_setExternalClockSourceFrequency(0, CLK_FREQ);    // set frequency for HFXT crystal to 48 MHz
    HFXT_Init();    // starts the HFXT crystal
    CS_initClockSignal(CS_MCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_1);     // init MCLK at 48 MHz
    CS_initClockSignal(CS_SMCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_4);    // init SMCLK at 12 MHz, 1/4 of MCLK
//    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P4, GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION); // output MCLK to P4.3
//    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P7, GPIO_PIN0, GPIO_PRIMARY_MODULE_FUNCTION); // output SMCLK to P7.0


}

// ------------Clock_Delay1us------------
// Simple delay function which delays about n microseconds.
// Inputs: n, number of us to wait
// Outputs: none
void Clock_Delay1us(uint32_t n){
  n = (382*n)/100;; // 1 us, tuned at 48 MHz
  while(n){
    n--;
  }
}

// delay function
// which delays about 6*ulCount cycles
// ulCount=8000 => 1ms = (8000 loops)*(6 cycles/loop)*(20.83 ns/cycle)
  //Code Composer Studio Code
void delay(unsigned long ulCount){
  __asm (  "pdloop:  subs    r0, #1\n"
      "    bne    pdloop\n");
}

// ------------Clock_Delay1ms------------
// Simple delay function which delays about n milliseconds.
// Inputs: n, number of msec to wait
// Outputs: none
void Clock_Delay1ms(uint32_t n){
  while(n){
    delay(48000000/9162);   // 1 msec, tuned at 48 MHz
    n--;
  }
}

