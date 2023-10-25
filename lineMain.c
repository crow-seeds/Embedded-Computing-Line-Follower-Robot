#include "driverlib.h"
#include <stdint.h>
#include <stdbool.h>
#include "Clock.h"
#include "PWM.h"
#include "Motor.h"
#include "Nokia5110.h"
#include "opt3001.h"
#include "typedef.h"

//define words for the reflectance states so i can read it in code easier
#define ON_LINE 3
#define LEFT    2
#define RIGHT   1
#define LOST    0
#define HARDLEFT 4
#define HARDRIGHT 5

#define BITSHIFT 0x01

//port and pin definitions for bump ports so i can read easier in code
#define BUMP_PORT   GPIO_PORT_P4
#define BUMP0       GPIO_PIN0   // P4.0
#define BUMP1       GPIO_PIN2   // P4.2
#define BUMP2       GPIO_PIN3   // P4.3
#define BUMP3       GPIO_PIN5   // P4.5
#define BUMP4       GPIO_PIN6   // P4.6
#define BUMP5       GPIO_PIN7   // P4.7
#define BUMP_PINS   (GPIO_PIN0 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7)

//  Left Brake Signal
#define BRAKE_L_PIN                 6
#define BRAKE_L_BIT                 (0x01 << BRAKE_L_PIN)
#define BRAKE_L_PORT                P8
#define SET_BRAKE_L_AS_AN_OUTPUT     BRAKE_L_PORT->DIR |= BRAKE_L_BIT
#define TURN_ON_BRAKE_L         BRAKE_L_PORT->OUT |= BRAKE_L_BIT
#define TURN_OFF_BRAKE_L          BRAKE_L_PORT->OUT &= ~BRAKE_L_BIT
#define TOGGLE_BRAKE_L            BRAKE_L_PORT->OUT ^= BRAKE_L_BIT

//  Right Brake Signal
#define BRAKE_R_PIN                 7
#define BRAKE_R_BIT                 (0x01 << BRAKE_R_PIN)
#define BRAKE_R_PORT                P8
#define SET_BRAKE_R_AS_AN_OUTPUT     BRAKE_R_PORT->DIR |= BRAKE_R_BIT
#define TURN_ON_BRAKE_R         BRAKE_R_PORT->OUT |= BRAKE_R_BIT
#define TURN_OFF_BRAKE_R          BRAKE_R_PORT->OUT &= ~BRAKE_R_BIT
#define TOGGLE_BRAKE_R            BRAKE_R_PORT->OUT ^= BRAKE_R_BIT

//  Left Front Signal
#define FRONT_L_PIN                 0
#define FRONT_L_BIT                 (0x01 << FRONT_L_PIN)
#define FRONT_L_PORT                P8
#define SET_FRONT_L_AS_AN_OUTPUT     FRONT_L_PORT->DIR |= FRONT_L_BIT
#define TURN_ON_FRONT_L         FRONT_L_PORT->OUT |= FRONT_L_BIT
#define TURN_OFF_FRONT_L          FRONT_L_PORT->OUT &= ~FRONT_L_BIT
#define TOGGLE_FRONT_L            FRONT_L_PORT->OUT ^= FRONT_L_BIT

//  Right Front Signal
#define FRONT_R_PIN                 5
#define FRONT_R_BIT                 (0x01 << FRONT_R_PIN)
#define FRONT_R_PORT                P8
#define SET_FRONT_R_AS_AN_OUTPUT     FRONT_R_PORT->DIR |= FRONT_R_BIT
#define TURN_ON_FRONT_R         FRONT_R_PORT->OUT |= FRONT_R_BIT
#define TURN_OFF_FRONT_R          FRONT_R_PORT->OUT &= ~FRONT_R_BIT
#define TOGGLE_FRONT_R            FRONT_R_PORT->OUT ^= FRONT_R_BIT

// Pushbutton S1
#define PUSHBUTTON_S1_PIN                       1
#define PUSHBUTTON_S1_BIT                       (0x01 << PUSHBUTTON_S1_PIN)
#define PUSHBUTTON_S1_PORT                      P1
#define SET_PUSHBUTTON_S1_TO_AN_INPUT           PUSHBUTTON_S1_PORT->DIR &= ~PUSHBUTTON_S1_BIT
#define ENABLE_PULL_UP_PULL_DOWN_RESISTORS_S1   PUSHBUTTON_S1_PORT->REN |= PUSHBUTTON_S1_BIT
#define SELECT_PULL_UP_RESISTORS_S1             PUSHBUTTON_S1_PORT->OUT |= PUSHBUTTON_S1_BIT

// Pushbutton S2
#define PUSHBUTTON_S2_PIN                       4
#define PUSHBUTTON_S2_BIT                       (0x01 << PUSHBUTTON_S2_PIN)
#define PUSHBUTTON_S2_PORT                      P1
#define SET_PUSHBUTTON_S2_TO_AN_INPUT           PUSHBUTTON_S2_PORT->DIR &= ~PUSHBUTTON_S2_BIT
#define ENABLE_PULL_UP_PULL_DOWN_RESISTORS_S2   PUSHBUTTON_S2_PORT->REN |= PUSHBUTTON_S2_BIT
#define SELECT_PULL_UP_RESISTORS_S2             PUSHBUTTON_S2_PORT->OUT |= PUSHBUTTON_S2_BIT


//variables and various debug stuff
int counter = 0;
uint8_t lightData = 0;
uint8_t rawLightData = 0;
uint8_t bumpData = 0;
uint8_t debugExpression = 0;

// OPT3001
uint16_t lightRawData;
float    convertedLux;
bool succ;

//speed
float speed = 40;

//speed modifier
float speedModifier = 1;

//states for the finite state machine
typedef enum {
    Initial,
    ReadData,
    Left,
    Center,
    Right,
    Lost,
    Obstruction,
    HardLeft,
    HardRight
} FSMState;

typedef enum {
    Initial2,
    ReadData2,
    Normal_Operation,
    Take_It_Slow,
    Stop
} FSMState2;

FSMState CurrentState = Initial;
FSMState2 CurrentState2 = Initial2;

//acceleration data
struct bmi160_accel_t {
    s16 x;/**<accel X  data*/
    s16 y;/**<accel Y  data*/
    s16 z;/**<accel Z  data*/
};

struct bmi160_accel_t acc;

//initialize the timer, every 1ms it changes the fsm
void TimerInit(void){
    const Timer_A_UpModeConfig upConfig = { //sets the config
        TIMER_A_CLOCKSOURCE_SMCLK, //use smclk for timer
        TIMER_A_CLOCKSOURCE_DIVIDER_24, //sets divider to 24 since 12MHz/500kHz is 24
        499, //sets the period to the parameter
        TIMER_A_TAIE_INTERRUPT_DISABLE, //all of these are default settings
        TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE,
        TIMER_A_DO_CLEAR
    };
    Timer_A_configureUpMode(TIMER_A1_BASE, &upConfig); //configures up mode
    Interrupt_enableInterrupt(INT_TA1_0); //sets the interrupt to TA1_0
    Timer_A_clearTimer(TIMER_A1_BASE); //clears timer
    Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE); //starts timer
}


//takes capacitors as outputs, and then waits
void Reflectance_Start(void){
    GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN3);
    GPIO_setOutputHighOnPin(GPIO_PORT_P9, GPIO_PIN2);
    GPIO_setAsOutputPin(GPIO_PORT_P7, PIN_ALL8);
    GPIO_setOutputHighOnPin(GPIO_PORT_P7, PIN_ALL8);
    //Wait 10us for capacitors to charge
    Clock_Delay1us(10);
    //Set reflectance sensor as input
    GPIO_setAsInputPin(GPIO_PORT_P7, PIN_ALL8);
}
//changes LightData variable for use in fsm
void Reflectance_End(void){
    rawLightData = P7->IN;
    //Turn off LEDs
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN3);
    GPIO_setOutputLowOnPin(GPIO_PORT_P9, GPIO_PIN2);

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
        bits[i] = (rawLightData >> i) & BITSHIFT;
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

    if(denom == 0){
        lightData = LOST;
    }
    else if(result <= 48 && result >= -48) {
        lightData = ON_LINE;
    }
    else if(result <= 200 && result > 48) {
        lightData = LEFT;
    }
    else if(result >= -200 && result < -48) {
        lightData = RIGHT;
    }
    else if(result > 200) {
        lightData = HARDLEFT;
    }
    else if(result < -200){
        lightData = HARDRIGHT;
    }
}

//reads the bump sensors, changes bump data to a representation of the bump sensors being pressed
void Bump_Read_Line(void){
    uint8_t pin0 = GPIO_getInputPinValue(BUMP_PORT, BUMP0); //gets the input values of the bump sensors and shifts them for each one so i can add them up and have each sensor represent a bit
    uint8_t pin1 = GPIO_getInputPinValue(BUMP_PORT, BUMP1) << 1;
    uint8_t pin2 = GPIO_getInputPinValue(BUMP_PORT, BUMP2) << 2;
    uint8_t pin3 = GPIO_getInputPinValue(BUMP_PORT, BUMP3) << 3;
    uint8_t pin4 = GPIO_getInputPinValue(BUMP_PORT, BUMP4) << 4; //shifts them different amounts so each sensor is a different bit
    uint8_t pin5 = GPIO_getInputPinValue(BUMP_PORT, BUMP5) << 5;
    uint8_t result = pin0+pin1+pin2+pin3+pin4+pin5; //adds them to get them into a singular binary string
    bumpData = ~result & 0b111111; //negates them since they are pull up resistors, bit masks so that the unused bits (6 and 7) are not negated
}

//initialize every single thing
void Everything_Init(void){
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4, BUMP_PINS); //initialize bump stuff
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN3); //initialize reflectance stuff
    GPIO_setAsOutputPin(GPIO_PORT_P9, GPIO_PIN2);
    GPIO_setAsOutputPin(GPIO_PORT_P7, PIN_ALL8);
    SET_BRAKE_L_AS_AN_OUTPUT; //initialize LEDs
    SET_BRAKE_R_AS_AN_OUTPUT;
    SET_FRONT_L_AS_AN_OUTPUT;
    SET_FRONT_R_AS_AN_OUTPUT;
    ENABLE_PULL_UP_PULL_DOWN_RESISTORS_S1; //initalize buttons
    ENABLE_PULL_UP_PULL_DOWN_RESISTORS_S2;
    SELECT_PULL_UP_RESISTORS_S1;
    SELECT_PULL_UP_RESISTORS_S2;
    SET_PUSHBUTTON_S1_TO_AN_INPUT;
    SET_PUSHBUTTON_S2_TO_AN_INPUT;
    TimerInit(); //initialize timer
    Motor_Init();
    Nokia5110_Init();
    MAP_Interrupt_enableMaster();
    //LaunchPad_Init();
    initI2C();
    sensorOpt3001Init();
    sensorOpt3001Enable(true);
    bmi160_initialize_sensor();
    bmi160_config_running_mode(6);
    bmi160_accel_foc_trigger_xyz(0x03, 0x03, 0x01, 0, 0, 16000);
    bmi160_set_foc_gyro_enable(0x01, 0, 0, 0);
}

void nextStateFunction(void){ //dictates what state the fsm is in from the sensor data
    switch(CurrentState){
        case Initial:
            if(GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN4) == 0){
                CurrentState = ReadData;
            }
            break;
        case ReadData:
            if(GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN1) == 0){
                CurrentState = Initial;
            }else if(bumpData != 0){
                CurrentState = Obstruction;
            }else if(lightData == ON_LINE){
                CurrentState = Center;
            }else if(lightData == LEFT){
                CurrentState = Left;
            }else if(lightData == RIGHT){
                CurrentState = Right;
            }else if(lightData == LOST){
                CurrentState = Lost;
            }else if(lightData == HARDLEFT){
                CurrentState = HardLeft;
            }else if(lightData == HARDRIGHT){
                CurrentState = HardRight;
            }
            break;
        case Obstruction:
            CurrentState = ReadData;
            break;
        case Center:
            CurrentState = ReadData;
            break;
        case Left:
            CurrentState = ReadData;
            break;
        case Right:
            CurrentState = ReadData;
            break;
        case HardLeft:
            CurrentState = ReadData;
            break;
        case HardRight:
            CurrentState = ReadData;
            break;
        case Lost:
            CurrentState = ReadData;
            break;
    }
}

char* indicator = "";


void nextStateFunction2(void){
    switch(CurrentState2){
        case Initial2:
            if(GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN4) == 0){
                CurrentState2 = ReadData2;
            }
            break;
        case ReadData2:
            if(acc.z > 18000){
                CurrentState2 = Stop;
                indicator = " Picked Up";
            }else if(convertedLux < 60){
                CurrentState2 = Stop;
                indicator = " Too Dark";
            }else if (convertedLux < 200){
                CurrentState2 = Take_It_Slow;
            }else{
                CurrentState2 = Normal_Operation;
            }
            break;
        case Normal_Operation:
            CurrentState2 = ReadData2;
            break;
        case Take_It_Slow:
            CurrentState2 = ReadData2;
            break;
        case Stop:
            if(GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN4) == 0){
                CurrentState2 = ReadData2;
                indicator = "";
            }
            break;
    }
}

void outputFunction(void){ //changes stuff based on the fsm state
    switch(CurrentState){
        case Initial:
            Motor_Stop(); //stop motors on initial state
            TURN_OFF_BRAKE_R;
            TURN_OFF_BRAKE_L;
            TURN_OFF_FRONT_R;
            TURN_OFF_FRONT_L;
            break;
        case ReadData:
            Nokia5110_Clear();
            Nokia5110_SetCursor(0,0);
            break;
        case Obstruction:
            Nokia5110_OutString("Obstruction");
            Motor_Stop(); //stops when hitting an obstruction
            Clock_Delay1ms(10);
            TURN_ON_BRAKE_R;
            TURN_ON_BRAKE_L;
            TURN_ON_FRONT_R;
            TURN_ON_FRONT_L;
            break;
        case Center:
            Nokia5110_OutString("Center");
            Motor_Forward(speed * speedModifier, speed * speedModifier); //moves forward when forward on a line
            Clock_Delay1ms(10);
            TURN_ON_BRAKE_R;
            TURN_ON_BRAKE_L;
            TURN_OFF_FRONT_R;
            TURN_OFF_FRONT_L;
            break;
        case Left:
            Nokia5110_OutString("Left");
            Motor_Forward(speed * speedModifier, speed * speedModifier * .5); //moves right when offset to the left
            Clock_Delay1ms(10);
            TURN_OFF_BRAKE_R;
            TURN_ON_BRAKE_L;
            TURN_OFF_FRONT_R;
            TURN_OFF_FRONT_L;
            break;
        case Right:
            Nokia5110_OutString("Right");
            Motor_Forward(speed * speedModifier * .5, speed * speedModifier); //moves left when offset to the right
            Clock_Delay1ms(10);
            TURN_ON_BRAKE_R;
            TURN_OFF_BRAKE_L;
            TURN_OFF_FRONT_R;
            TURN_OFF_FRONT_L;
            break;
        case HardLeft:
            Nokia5110_OutString("Hard Left");
            Motor_Left(speed * 1.5 * speedModifier, speed * 1.5 * speedModifier); //moves right when offset to the left
            Clock_Delay1ms(10);
            TURN_OFF_BRAKE_R;
            TURN_ON_BRAKE_L;
            TURN_OFF_FRONT_R;
            TURN_OFF_FRONT_L;
            break;
        case HardRight:
            Nokia5110_OutString("Hard Right");
            Motor_Right(speed * 1.5 * speedModifier, speed * 1.5 * speedModifier); //moves left when offset to the right
            Clock_Delay1ms(10);
            TURN_ON_BRAKE_R;
            TURN_OFF_BRAKE_L;
            TURN_OFF_FRONT_R;
            TURN_OFF_FRONT_L;
            break;
        case Lost:
            Nokia5110_OutString("Lost");
            Motor_Backward(speed * speedModifier, speed * speedModifier);
            Clock_Delay1ms(10);
            TURN_OFF_BRAKE_R;
            TURN_OFF_BRAKE_L;
            TURN_ON_FRONT_R;
            TURN_ON_FRONT_L;
            break;
    }
}

void outputFunction2(void){
    switch(CurrentState2){
            case Initial2:
                speedModifier = 1;
                break;
            case ReadData2:
                Nokia5110_Clear();
                break;
            case Normal_Operation:
                //Nokia5110_OutString(" Normal");
                speedModifier = 1;
                break;
            case Take_It_Slow:
                //Nokia5110_OutString(" Slow");
                speedModifier = 0.5;
                break;
            case Stop:
                Nokia5110_OutString(indicator);
                speedModifier = 0;
                break;

    }
}

void TA1_0_IRQHandler(void){ //changes the fsm state on every interrupt (1ms)
    counter++;
    if(counter == 1){
        Reflectance_Start(); //every 10 interrupts, does the reflectance and bump stuff
    }else if(counter == 2){
        Reflectance_End();
    }else if(counter == 3){
        Bump_Read_Line();
    }else if(counter == 4){
        nextStateFunction();
        nextStateFunction2();
        outputFunction();
        outputFunction2();
    }
    else if(counter == 10){
        counter = 0;
    }
    Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);
}


int main(void)
{
    Clock_Init(); //initalize everything
    Everything_Init();
    Interrupt_enableMaster();

    while(1){
        sensorOpt3001Read(&lightRawData);
        sensorOpt3001Convert(lightRawData, &convertedLux);
        bmi160_read_accel_xyz(&acc);
        Clock_Delay1ms(1);
    }


}

