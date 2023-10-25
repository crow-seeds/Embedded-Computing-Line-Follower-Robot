#include <stdint.h>
/*
 * Reflectance.c
 * Provides functions to
 * 1. Initialize Pololu reflectance sensor
 * 2. Read Pololu reflectance sensor
 * 3. Determine robot position
 * 4. Determine robot status
 *
 */

#include "driverlib.h"

#include "Clock.h"
#include "Reflectance.h"

#define ALLBITS  0xFF
#define BITSHIFT 0x01

//------------Reflectance_Init------------
// Initialize sensor array to GPIO, set LEDs (P5.3 and P9.2)
// as output and sensors (P7.0-P7.7) as output
// Input: none
// Output: none
void Reflectance_Init(void){
    //Initialize even LED on P5.3 as GPIO output
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN3);

    //Initialize odd LED on P9.2 as GPIO output
    GPIO_setAsOutputPin(GPIO_PORT_P9, GPIO_PIN2);

    //Initialize reflectance sensor as GPIO output
    GPIO_setAsOutputPin(GPIO_PORT_P7, PIN_ALL8);
}

//------------Reflectance_Read------------
// Read reflectance sensor
// Input: the delay time in us
// Output: result the sensor readings, bit 0 corresponds
//         to the rightmost sensor, bit 7 to the leftmost sensor
uint8_t Reflectance_Read(uint32_t time){
    uint8_t result;

    //Turn on LEDs
    GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN3);
    GPIO_setOutputHighOnPin(GPIO_PORT_P9, GPIO_PIN2);

    //Charge reflectance sensor capacitors
    GPIO_setAsOutputPin(GPIO_PORT_P7, PIN_ALL8);
    GPIO_setOutputHighOnPin(GPIO_PORT_P7, PIN_ALL8);

    //Wait 10us for capacitors to charge
    Clock_Delay1us(10);
    //Set reflectance sensor as input
    GPIO_setAsInputPin(GPIO_PORT_P7, PIN_ALL8);
    //Wait @param time
    Clock_Delay1us(time);
    //Read reflectance sensor values
    //Assign to result
    result = P7->IN;
    //Turn off LEDs
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN3);
    GPIO_setOutputLowOnPin(GPIO_PORT_P9, GPIO_PIN2);
    //Return result, the sensor readings
    return result;
}

//------------Reflectance_Center------------
// Determine robot's status over the line
// Input: the delay time in us
// Output: result the robot location status (LOST/RIGHT/LEFT/ON LINE)
uint8_t Reflectance_Center(uint32_t time){
    uint8_t left, right, result, data;
    //Get reflectance read data
    //Assign result to data
    data = Reflectance_Read(time);
    //Get value from left center sensor
    left = (data >> 4) & 0x01;
    //Get value from right center sensor
    right = (data >> 3) & 0x01;
    /*
     * Check robot status truth table
     * INPUT (L,R) | OUTPUT
     * ------------|------------
     *      11     | ON_LINE (3)
     *      10     | LEFT    (2)
     *      01     | RIGHT   (1)
     *      00     | LOST    (0)
     */
    if(left == 1 && right == 1) {
        result = ON_LINE;
    }
    else if(left == 1 && right == 0) {
            result = LEFT;
    }
    else if(left == 0 && right == 1) {
        result = RIGHT;
    }
    else if(left == 0 && right == 0) {
        result = LOST; // off road
    }
    //Return result, the robot status
    return result;
}

//------------Reflectance_Position------------
// Determine robot's status over the line
// Input: the collected sensor data
// Output: the position value between +345 (most left)
//         to -345 (most right)
int32_t Reflectance_Position(uint8_t data){
    //Initialize weight vector
    //L--->R
    int16_t weight[8] = {334, 238, 142, 48, -48, -142, -238, -334};
    //Initialize bit vector to hold sensor data
    uint8_t bits[8] = {};
    //Initialize other necessary variables
    int16_t num = 0;
    int16_t result = 0;
    uint8_t denom = 0;
    uint8_t i;
    //Fill bit vector with sensor data
    for(i=0; i<8; i++) {
        bits[i] = (data >> i) & BITSHIFT;
    }
    //Num = sum(bit_reading*weight)
    //Denom = sum(bit readings)
    for(i=0; i<8; i++) {
        num += bits[i] * weight[i];
        denom += bits[i];
    }
    if(denom != 0){
        result = num / denom;
    }
    //Return result, the robot position
    return result;
}
