/*******************************************************************************
 * Programmer: Shwetha Niddodi                                                 *
 * Class: CptS 466                                                             *
 * Lab Project: Project 3                                                      *
 * Date: 03/02/2014                                                            *
 *                                                                             *
 * Description:  This project captures the temperature from Pmod Tmp2 using    *
 *               I2C and displays current and average temperature on Pmod CLS  *
 *               using SPI. It is continuation of Project 2                    *
 ******************************************************************************/
 // #include all necessary standard and user-defined libraries
#include <peripheral/incap.h>	//For capturing input signals using IC2 and IC3
#include <peripheral/outcompare.h> //Enable OC2 and OC3 for motor running
#include "system_config.h"
#include "motor_control.h"
#include "timer_util.h"
#include "pmodACL.h"
#include "FreeRTOS.h"
#include "timers.h"

#define REDUCTION_MOTOR        19.0
#define WHEEL_DIAMETER         2.56
#define INCHES_2_FEET          (1/12.0)
#define PI                     3.14159
/* 1 inch = 1.57828E-5 miles/ 1 sec = (1/3600) hr*/
#define INCHESPERSEC_2_MPH     0.05681808
#define SA_PER_REVOLUTION      58
#define LEVEL1_MIN              10.0
#define LEVEL1_MAX              11.0
#define LEVEL2_MIN              15.0
#define LEVEL2_MAX              18.0
#define ANGLE_LEVEL2_MIN        5.0
#define ANGLE_LEVEL2_MAX        7.0
#define PULSE_WIDTH           (0.75* (PWM_PERIOD+1))
#define PULSE_WIDTH_MAX       (0.85*(PWM_PERIOD+2))
#define PULSE_WIDTH_MIN       (0.7*(PWM_PERIOD+2))
#define MSEC_2_SEC              0.001

extern unsigned long long totalMotorRunCounter;
extern unsigned long long timer2Counter;
unsigned long currentLeftRotationCntr = 0;
unsigned long currentRightRotationCntr = 0;
unsigned int countRight = 0;
unsigned int countLeft = 0;
double motorDistance = 0;
double motorSpeed = 0;
BOOL isMotorLeftRestart = FALSE;
BOOL isMotorRightRestart = FALSE;
BOOL isTimer2Reset = TRUE;
unsigned int rightPulseWidth = PULSE_WIDTH;
unsigned int leftPulseWidth = PULSE_WIDTH;
unsigned long saveRightRotationCntr = 0;
BOOL isSwitch1Pressed = FALSE;
BOOL isSwitch2Pressed = FALSE;
BOOL isEnableAutoCorrect = TRUE;
BOOL isTimerNeeded = TRUE;
double cornerDistance = 0;

extern TimerHandle_t motorTimerHandle;
extern TimerHandle_t motorAfterDelayTimerHandle;
extern TimerHandle_t timerCounterHandle;
extern TimerHandle_t displayStatsHandle;

void setup_OC2(unsigned char direction);
void setup_OC3(unsigned char direction);
void displayTime(unsigned int IC2, unsigned int IC3);
void adjust_dutycycle_motorLeft(unsigned int value);
void adjust_dutycycle_motorRight(unsigned int value);
void calculate_motor_speed(double* speedMph,double* distanceFeet);

/*************************************************************
 * Function:          setup_OC2                              *
 * Date Created:      02/22/2014                             *
 * Date Last Modified:                                       *
 * Description: This function is used for generating output  *
 *              waveform with required dutycyle (80%)        *
 * Input parameters: direction: direction of motor           *
 *                   (clockwise or anti clockwise            *
 * Returns:          none                                    *
 * Usages:           Used to run the motor                   *
 * Preconditions:    Timer 2 is set up                       *
 * Postconditions: Output waveform is generated and the right*
 *                 motor started spinning                    *
 *************************************************************/
void setup_OC2(unsigned char direction)
{
    if(direction)
    {
        PORTSetBits( IOPORT_D, BIT_7 ); //set h bridge dir
    }
    else
    {
        PORTClearBits( IOPORT_D, BIT_7 ); //set h bridge dir
    }
    delay(1);
    /* The right most arguments of the OpenOC1 call represent the duty cycle
    of the output waveform */
    OpenOC2( OC_ON | OC_TIMER_MODE16 | OC_TIMER2_SRC | OC_IDLE_STOP | OC_PWM_FAULT_PIN_DISABLE,
            PULSE_WIDTH, PULSE_WIDTH);
    rightPulseWidth = PULSE_WIDTH;
}

/*************************************************************
 * Function:          setup_OC3                              *
 * Date Created:      02/22/2014                             *
 * Date Last Modified:                                       *
 * Description: This function is used for generating output  *
 *              waveform with required dutycyle (80%)        *
 * Input parameters: direction: direction of motor           *
 *                   (clockwise or anti clockwise            *
 * Returns:          none                                    *
 * Usages:           Used to run the motor                   *
 * Preconditions:    Timer 2 is set up                       *
 * Postconditions: Output waveform is generated and the left *
 *                 motor started spinning                    *
 *************************************************************/
void setup_OC3(unsigned char direction)
{
    if(direction)
    {
        PORTSetBits( IOPORT_D, BIT_6 ); //set h bridge dir
    }
    else
    {
        PORTClearBits( IOPORT_D, BIT_6 ); //set h bridge dir
    }
    OpenOC3( OC_ON | OC_TIMER_MODE16 | OC_TIMER2_SRC | OC_IDLE_STOP | OC_PWM_FAULT_PIN_DISABLE,
            PULSE_WIDTH, PULSE_WIDTH);
    leftPulseWidth = PULSE_WIDTH;
}

/*************************************************************
 * Function:          set_hbridge                            *
 * Date Created:      02/22/2014                             *
 * Date Last Modified:                                       *
 * Description:     This function sets up h-bridges for the  *
 *                  2 motors                                 *
 *                                                           *
 * Input parameters: none                                    *
 * Returns:          none                                    *
 * Usages:           Used to run the motors                  *
 * Preconditions:                                            *
 * Postconditions:   The direction and enable pins are set as*
 *                   output pins and SA pin is set as input  *
 *                   pin                                     *
 *************************************************************/
void set_hbridge(void)
{
    PORTSetPinsDigitalIn( IOPORT_D, BIT_9|BIT_10 ); // Triggers the interrupt, and hence motor will spin
    PORTSetPinsDigitalOut( IOPORT_D, BIT_7 ); //Dir pin
    PORTSetPinsDigitalOut( IOPORT_D, BIT_1 ); //Enable pin
    PORTClearBits (IOPORT_D, BIT_1); // Make sure no waveform is outputted to Enable pin
    PORTSetPinsDigitalOut( IOPORT_D, BIT_6 ); //Dir pin
    PORTSetPinsDigitalOut( IOPORT_D, BIT_2 ); //Enable pin
    PORTClearBits (IOPORT_D, BIT_2); // Make sure no waveform is outputted to Enable pin
}

/*************************************************************
 * Function:          setup_switches                         *
 * Date Created:      02/22/2014                             *
 * Date Last Modified:                                       *
 * Description:      This function will set up the pins as   *
 *                   input pins for switch 1 and 2           *
 *                                                           *
 * Input parameters: none                                    *
 * Returns:          none                                    *
 * Usages:           Used for triggers for switch 1 and 2    *
 * Preconditions:                                            *
 * Postconditions:   Pin (RA 14, 15) are set as input pins   *
 *                                                           *
 *************************************************************/
void setup_switches(void)
{
   // TRISA = 0x0000;//Bit 14, 15 are set as input for SW1 and SW2
   // PORTA = 0x0000;//PortA is cleared for switches
    PORTSetPinsDigitalIn( IOPORT_A, BIT_14|BIT_15 ); // Triggers the interrupt, and hence motor will spin
    PORTClearBits (IOPORT_A, BIT_14|BIT_15); // Make sure no waveform is outputted to Enable pin
}

/*************************************************************
 * Function:          setup_IC2                              *
 * Date Created:      02/22/2014                             *
 * Date Last Modified:02/22/2014                             *
 * Description: This function is for setting up IC2 which    *
 *              gets triggered for every rising and falling  *
 *              edge of the input pulse                      *
 *              capturing SA pulses of the right motor       *
 *                                                           *
 * Input parameters: none                                    *
 * Returns:          none                                    *
 * Usages:           It is used for capturing SA pulses of   *
 *                   the right motor                         *
 * Preconditions:                                            *
 * Postconditions: Input pulses are captured for every rising*
 *                 and falling edge                          *
 *************************************************************/
void setup_IC2(void)
{
    OpenCapture2( IC_EVERY_EDGE | IC_INT_1CAPTURE | IC_TIMER2_SRC | IC_FEDGE_RISE | IC_ON );

    //OpenCapture2( IC_EVERY_FALL_EDGE | IC_INT_1CAPTURE | IC_TIMER2_SRC | IC_ON );
}


/*************************************************************
 * Function:          setup_IC3                              *
 * Date Created:      02/22/2014                             *
 * Date Last Modified:02/22/2014                             *
 * Description: This function is for setting up IC3 which    *
 *              gets triggered for every rising and falling  *
 *              edge of the input pulse                      *
 *              capturing SA pulses of the left motor        *
 *                                                           *
 * Input parameters: none                                    *
 * Returns:          none                                    *
 * Usages:           It is used for capturing SA pulses of   *
 *                   the left motor                          *
 * Preconditions:                                            *
 * Postconditions: Input pulses are captured for every rising*
 *                 and falling edge                          *
 *************************************************************/
void setup_IC3(void)
{
    OpenCapture3( IC_EVERY_EDGE | IC_INT_1CAPTURE | IC_FEDGE_RISE | IC_TIMER2_SRC | IC_ON );
    //OpenCapture3( IC_EVERY_FALL_EDGE | IC_INT_1CAPTURE | IC_FEDGE_RISE | IC_TIMER2_SRC | IC_ON );
}


/*************************************************************
 * Function:          configure_interrupts                   *
 * Date Created:      02/22/2014                             *
 * Date Last Modified:                                       *
 * Description:   The function is used for configuring       *
 *                external interrupts and input capture      *
 *                interrupts                                 *
 *                                                           *
 * Input parameters: none                                    *
 * Returns:          none                                    *
 * Usages:          Used for getting interrupt for switch 1,2*
 *                   interrupts and input capture interrupts *
 * Preconditions:                                            *
 * Postconditions:   Port B pins (10 - 13) are configured as *
 *                   output pins    2                         *
 *************************************************************/
void configure_interrupts(void)
{
  

    INTEnableSystemMultiVectoredInt ();

    // JF01 - RA14 - INT3 - SW1
    // JF02 - RA15 - INT4 - SW2
    //Configure external interrupt for pmodACL INT1
    ConfigINT1 (EXT_INT_PRI_7 | FALLING_EDGE_INT | EXT_INT_ENABLE);
    PmodACLGetIntSource(SPI_CHANNEL2);
    PmodACLSetIntMap(SPI_CHANNEL2, 1);
    PmodACLSetIntEnable(SPI_CHANNEL2,PMODACL_BIT_INT_MAP_DATA_READY);

    //Configure external interrupts (3 and 4) for switch (1 and 2)
    ConfigINT3 (EXT_INT_PRI_4 | RISING_EDGE_INT | EXT_INT_ENABLE);
    ConfigINT4 (EXT_INT_PRI_4 | RISING_EDGE_INT | EXT_INT_ENABLE);
  
    // Configure interrupt for input capture (2 and 3) to find number of pulses
    // per rotation
    ConfigIntCapture2(IC_INT_ON | IC_INT_PRIOR_7 |IC_INT_SUB_PRIOR_3);
    ConfigIntCapture3(IC_INT_ON | IC_INT_PRIOR_7 |IC_INT_SUB_PRIOR_3);
    INTEnableInterrupts (); // Enable system wide interrupts
}

/*************************************************************
 * Function:          state_machine                          *
 * Date Created:      10/26/2014                             *
 * Date Last Modified:                                       *
 * Description:                                              *
 *                                                           *
 * Input parameters: none                                    *
 * Returns:          none                                    *
 * Usages:           Used to set up LEDs                     *
 * Preconditions:                                            *
 * Postconditions:   Port B pins (10 - 13) are configured as *
 *                   output pins                             *
 *************************************************************/
void __ISR(_EXTERNAL_3_VECTOR, IPL4AUTO) INT3Handler(void)
{
    unsigned int switch1 = 0;
    int fail = 0;
    switch1 = mPORTARead();
    switch1 = (switch1 & 0x4000) >> 14;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    mINT3ClearIntFlag (); // Clear interrupt
    if(switch1)
    {
        isSwitch1Pressed = TRUE;
        //Turn on Timer 2
        if(xTimerStartFromISR(timerCounterHandle, &xHigherPriorityTaskWoken) != pdPASS){
            fail = 1;
        }
        //Turn on OCR 2
        setup_OC2(1);
        //Turn on OCR 3
        setup_OC3(0);
        //xTimerStart(displayStatsHandle, 0);
        if(xTimerStartFromISR(motorTimerHandle, &xHigherPriorityTaskWoken) != pdPASS){
            fail = 1;
        }

    }
}

/*************************************************************
 * Function:          state_machine                          *
 * Date Created:      10/26/2014                             *
 * Date Last Modified:                                       *
 * Description:                                              *
 *                                                           *
 * Input parameters: none                                    *
 * Returns:          none                                    *
 * Usages:           Used to set up LEDs                     *
 * Preconditions:                                            *
 * Postconditions:   Port B pins (10 - 13) are configured as *
 *                   output pins                             *
 *************************************************************/
void __ISR(_EXTERNAL_4_VECTOR, IPL7AUTO) INT4Handler(void)
{
    unsigned int switch2 = mPORTARead() & 0x8000;
    switch2 = switch2 >> 15;

    mINT4ClearIntFlag (); // Clear interrupt

    if(switch2)
    {
        isSwitch2Pressed = TRUE;
            //Turn on Timer 2
        xTimerStart(timerCounterHandle, 0);
        //Turn on OCR 2
        setup_OC2(1);
        //Turn on OCR 3
        setup_OC3(0);
        xTimerStart(displayStatsHandle, 0);
       xTimerStart(motorTimerHandle, 0);
    }
}

/*************************************************************
 * Function:          IC2_IntHandler                         *
 * Date Created:      02/22/2014                             *
 * Date Last Modified:02/22/2014                             *
 * Description: This interrupt handler captures the SA pulses*
 *              of right motor. It calculates the time       *
 *              differene between succesful pulses. This is  *
 *              needed for auto correction of speed of motors*
 *                                                           *
 * Input parameters: no                                      *
 * ne                                                        *
 * Returns:          none                                    *
 * Usages:           Used to control the duty cyle applied   *
 *                   to both the motors in order to make the *
 *                   robot move in straight line             *
 * Preconditions:                                            *
 * Postconditions:   It provides the time difference between *
 *                   successful motor pulses                 *
 *************************************************************/
void __ISR(_INPUT_CAPTURE_2_VECTOR, IPL7AUTO) IC2_IntHandler()
{
    static BOOL fallEdgeFlag = 0;
    static BOOL firstTime = 1;
    static unsigned long long savedBlkNr = 0;
    static unsigned int savedCapture = 0;
    unsigned int currentCapture = 0;
    unsigned long long currentBlkNr = 0;

    mIC2ClearIntFlag();

    if(isEnableAutoCorrect)
    {
        if(isMotorLeftRestart)
        {
            isMotorLeftRestart = FALSE;
            savedBlkNr = 0;
            savedCapture = 0;
            fallEdgeFlag = 0;
            firstTime = 1;
            currentRightRotationCntr = 0;
        }

        currentRightRotationCntr++;
        if(fallEdgeFlag)
        {
            fallEdgeFlag = FALSE;
        }
        else
        {
            fallEdgeFlag = TRUE;
            if(firstTime)
            {
                savedBlkNr = timer2Counter;
                savedCapture = ReadTimer2();
                firstTime = FALSE;
            }
            else
            {
                firstTime = TRUE;
                currentBlkNr = timer2Counter;
                currentCapture = ReadTimer2();

                if(currentBlkNr == savedBlkNr)
                {
                    countRight = currentCapture - savedCapture;
                }
                else if(currentBlkNr > savedBlkNr)
                {
    countRight = ((currentBlkNr - savedBlkNr)*PWM_PERIOD)+currentCapture - savedCapture;
                }
            }
        }
    }
}

/*************************************************************
 * Function:          IC3_IntHandler                         *
 * Date Created:      02/22/2014                             *
 * Date Last Modified:02/22/2014                             *
 * Description: This interrupt handler captures the SA pulses*
 *              of right motor. It calculates the time       *
 *              differene between succesful pulses. It       *
 *              compares this value with that of right motor.*
 *              The difference is used for duty cycle        *
 *              adjustment                                   *
 *                                                           *
 * Input parameters: no                                      *
 * ne                                                        *
 * Returns:          none                                    *
 * Usages:           Used to control the duty cyle applied   *
 *                   to both the motors in order to make the *
 *                   robot move in straight line             *
 * Preconditions:                                            *
 * Postconditions:   It provides the time difference between *
 *                   successful motors                       *
 *************************************************************/
void __ISR(_INPUT_CAPTURE_3_VECTOR, IPL7AUTO) IC3_IntHandler()
{
    static BOOL fallEdgeFlag = 0;
    static BOOL firstTime = 1;
    static unsigned long long savedBlkNr = 0;
    static unsigned int savedCapture = 0;
    unsigned int currentCapture = 0;
    unsigned long long currentBlkNr = 0;

    mIC3ClearIntFlag();

    if(isEnableAutoCorrect)
    {
        if(isMotorRightRestart)
        {
            isMotorRightRestart = FALSE;
            savedBlkNr = 0;
            savedCapture = 0;
            fallEdgeFlag = 0;
            firstTime = 1;
            currentRightRotationCntr = 0;
        }
        currentLeftRotationCntr++;
        if(fallEdgeFlag)
        {
            fallEdgeFlag = FALSE;
        }
        else
        {
            fallEdgeFlag = TRUE;
            if(firstTime)
            {
                savedBlkNr = timer2Counter;
                //savedCapture = mIC3ReadCapture();
                savedCapture = ReadTimer2();
                firstTime = FALSE;
            }
            else
            {
                firstTime = TRUE;
                currentBlkNr = timer2Counter;
                currentCapture = ReadTimer2();

                if(currentBlkNr == savedBlkNr)
                {
                    countLeft = currentCapture - savedCapture;
                }
                else if(currentBlkNr > savedBlkNr)
                {
    countLeft = ((currentBlkNr - savedBlkNr)*PWM_PERIOD)+currentCapture - savedCapture;
                }

                if(countLeft > countRight)
                {
                       adjust_dutycycle_motorLeft(2);
                       adjust_dutycycle_motorRight(-1);
                }
                else if(countLeft < countRight)
                {
                       adjust_dutycycle_motorRight(2);
                       adjust_dutycycle_motorLeft(-1);
                }
                else
                {
                    asm("nop");
                }
            }
        }
    }
}


/*************************************************************
 * Function:          calculate_motor_speed                  *
 * Date Created:      02/22/2014                             *
 * Date Last Modified:02/22/2014                             *
 * Description: This function calculates the motor speed in  *
 *              mph and distance in feet                     *
 *                                                           *
 * Input parameters: speedMph - output parameter for speed   *
 *                   distanceFeet - output parameter for     *
 *                   distance                                *
 * Returns:          none                                    *
 * Usages:           Used to display speed and distance      *
 * Preconditions:                                            *
 * Postconditions:   Speed and distance are returned         *
 *                                                           *
 *************************************************************/
void calculate_motor_speed(double* speedMph,double* distanceFeet)
{
    unsigned int rotationCntr = 0;
    double distanceInch = 0;
    double speedInchPerSec = 0;

    rotationCntr = currentRightRotationCntr - saveRightRotationCntr;

    distanceInch = (rotationCntr*WHEEL_DIAMETER*PI/SA_PER_REVOLUTION)/2;
    speedInchPerSec = distanceInch/TIME_STEP;
    *distanceFeet = distanceInch*INCHES_2_FEET;
    *speedMph = speedInchPerSec*INCHESPERSEC_2_MPH;
    saveRightRotationCntr = currentRightRotationCntr;

    return;
}

/*************************************************************
 * Function:          adjustDutyCycleMotorRight              *
 * Date Created:      02/22/2014                             *
 * Date Last Modified:02/22/2014                             *
 * Description: This function will adjust the duty cyle of   *
 *              right motor.                                  *
 * Input parameters: value - incremental pulse width value   *
 *                                                           *
 * Returns:          none                                    *
 * Usages:           Used to adjust the speed of left motor  *
 *                                                           *
 * Preconditions: The left motor is running slowly compared  *
 *                to right motor                             *
 * Postconditions: The duty cycle is adjusted                *
 *************************************************************/
void adjust_dutycycle_motorRight(unsigned int value)
{
    if(value == 1)
    {
        if((rightPulseWidth+value) < PULSE_WIDTH_MAX)
        {
            //PORTClearBits( IOPORT_D, BIT_1 ); //set h bridge enable
            //delay(1);
            SetDCOC2PWM(rightPulseWidth+ value);//OC2RS = rightPulseWidth+ value;
            rightPulseWidth = rightPulseWidth+value;
        }
    }
    else
    {
        if((rightPulseWidth+value) > PULSE_WIDTH_MIN)
        {
            //PORTClearBits( IOPORT_D, BIT_1 ); //set h bridge enable
            //delay(1);
            SetDCOC2PWM(rightPulseWidth+value);//OC2RS = rightPulseWidth+ value;
            rightPulseWidth = rightPulseWidth+value;
        }
    }
}


/*************************************************************
 * Function:          adjust_dutycycle_motorLeft             *
 * Date Created:      02/22/2014                             *
 * Date Last Modified:02/22/2014                             *
 * Description: This function will adjust the duty cyle of   *
 *              left motor.                                  *
 * Input parameters: value - incremental pulse width value   *
 *                                                           *
 * Returns:          none                                    *
 * Usages:           Used to adjust the speed of left motor  *
 *                                                           *
 * Preconditions: The left motor is running slowly compared  *
 *                to right motor                             *
 * Postconditions: The duty cycle is adjusted                *
 *************************************************************/
void adjust_dutycycle_motorLeft(unsigned int value)
{
    if(value == 1)
    {
        if((leftPulseWidth+value) < PULSE_WIDTH_MAX)
        {
            //PORTClearBits( IOPORT_D, BIT_1 ); //set h bridge enable
            //delay(1);
            SetDCOC2PWM(leftPulseWidth+ value);//OC2RS = rightPulseWidth+ value;
            leftPulseWidth = leftPulseWidth+value;
        }
    }
    else if(value == -1)
    {
        if((leftPulseWidth+value) > PULSE_WIDTH_MIN)
        {
            //PORTClearBits( IOPORT_D, BIT_1 ); //set h bridge enable
            //delay(1);
            SetDCOC2PWM(leftPulseWidth-value);//OC2RS = rightPulseWidth+ value;
            leftPulseWidth = leftPulseWidth+value;
        }
    }
}


/*************************************************************
 * Function:          straightLineControl                    *
 * Date Created:      02/22/2014                             *
 * Date Last Modified:02/22/2014                             *
 * Description: This function will control the motor for the *
 *              Case1: motor moving in straight line. It     *
 *             calculates the motor speed and distance. After*
 *             distance of 10 feet, it stops for few msec and*
 *             restarts for a distance of 15 feet            *
 *              left motor.                                  *
 * Input parameters: none                                    *
 *                                                           *
 * Returns:          none                                    *
 * Usages:           Used to calculate speed and distance and*
 *                   to control the motor action for case 1  *
 *                                                           *
 * Preconditions: Switch 1 is pressed and motor is running   *
 *                                                           *
 * Postconditions: The robot moves for 10 feet and stops for *
 *                 few msec and then again moves for 15 feet *
 *************************************************************/
void straight_line_control(void)
{
    double distance = 0;
    static BOOL isSecondRun = FALSE;

    calculate_motor_speed(&motorSpeed,&distance);
    motorDistance += distance;

    if((!isSecondRun) &&
           ((motorDistance >= LEVEL1_MIN) && (motorDistance <= LEVEL2_MAX)))
    {
        motorDistance = 0;
        motorSpeed = 0;
        isSecondRun = TRUE;
        isTimerNeeded = FALSE;
        saveRightRotationCntr = 0;
        isEnableAutoCorrect = FALSE;

        PORTClearBits( IOPORT_D, BIT_1 ); //set h bridge enable
        delay(1);
        //Close OC2 and OC3 and stop timer 2
        CloseOC2();
        delay(1);
        PORTClearBits( IOPORT_D, BIT_2 ); //set h bridge enable
        delay(1);
        CloseOC3();
        CloseTimer2();

        timer2Counter = 0;
        isMotorLeftRestart = TRUE;
        isMotorRightRestart = TRUE;
        //xTimerStart(motorTimerDelayHandle, 0);
    }

    if((isSecondRun) &&
            ((motorDistance >= LEVEL2_MIN) && (motorDistance <= LEVEL2_MAX)))
    {
        motorSpeed = 0;
        motorDistance = 0;
        saveRightRotationCntr = 0;
        PORTClearBits( IOPORT_D, BIT_1 ); //set h bridge dir
        delay(1);
        CloseOC2();
        PORTClearBits( IOPORT_D, BIT_2 ); //set h bridge dir
        delay(1);
        CloseOC3();
        CloseTimer2();
        CloseTimer4();
        isTimerNeeded = TRUE;

        timer2Counter = 0;
        isMotorLeftRestart = TRUE;
        isMotorRightRestart = TRUE;

        isSwitch1Pressed = FALSE;
        isSecondRun = FALSE;
    }
}

/*************************************************************
 * Function:          right_angle_control                    *
 * Date Created:      02/22/2014                             *
 * Date Last Modified:02/22/2014                             *
 * Description: This function will control the motor for the *
 *            Case2: motor moving in straight line for 5 feet*
 *            making right angle turn and moving for another5*
 *            feet.It calculates the motor speed and distance*
 *                                                           *
 * Input parameters: none                                    *
 *                                                           *
 * Returns:          none                                    *
 * Usages:           Used to calculate speed and distance and*
 *                   to control the motor action for case 2  *
 *                                                           *
 * Preconditions: Switch 2 is pressed and motor is running   *
 *                                                           *
 * Postconditions: The robot moves for 5 feet and makes      *
 *                 right turn and then again moves for 5 feet*
 *************************************************************/
void right_angle_control(void)
{
    double distance = 0;
    static BOOL isFirstRun = TRUE;

    calculate_motor_speed(&motorSpeed,&distance);
    motorDistance += distance;

    if((isFirstRun) &&
    ((motorDistance > ANGLE_LEVEL2_MIN) && (motorDistance <= ANGLE_LEVEL2_MAX)))
    {
            //cornerDistance = motorDistance;
            motorDistance = 0;
            motorSpeed = 0;
            isTimerNeeded = FALSE;
            saveRightRotationCntr = 0;
            timer2Counter = 0;
            isMotorLeftRestart = TRUE;
            isMotorRightRestart = TRUE;
            isEnableAutoCorrect = FALSE;
            //Reverse the direction of left motor to make right turn
            PORTClearBits( IOPORT_D, BIT_2 ); //clear h bridge 2 enable
            delay(10);

            PORTSetBits( IOPORT_D, BIT_6 ); //set h bridge 1 dir (in reverse direction)
            delay(10);

            //xTimerStart(motorTimerDelayHandle, 0);
            isFirstRun = FALSE;
    }
    if((!isFirstRun) &&
             ((motorDistance > ANGLE_LEVEL2_MIN) && (motorDistance <= ANGLE_LEVEL2_MAX)))
    {
            motorDistance = 0;
            motorSpeed = 0;
            saveRightRotationCntr = 0;
            timer2Counter = 0;
            isMotorLeftRestart = TRUE;
            isMotorRightRestart = TRUE;
            PORTClearBits( IOPORT_D, BIT_1 ); //clear h bridge 1 enable
            CloseOC2();//Stop sending PWM signal through OC3
            PORTClearBits( IOPORT_D, BIT_2 ); //clear h bridge 2 enable
            CloseOC3(); //Stop sending PWM signal through OC3
            CloseTimer2();
            CloseTimer4();
            isSwitch2Pressed = FALSE;
            isTimerNeeded = FALSE;
            isFirstRun = TRUE;
    }
}

/*************************************************************
 * Function:          controlMotor                           *
 * Date Created:      02/22/2014                             *
 * Date Last Modified:02/22/2014                             *
 * Description: This function will control the motor action  *
 *              for case 1 and case 2                        *
 *              left motor.                                  *
 * Input parameters: none                                    *
 *                                                           *
 * Returns:          none                                    *
 * Usages:           Used to control the motor action        *
 *                                                           *
 * Preconditions: Switch 1 or Switch 2 is pressed and timer 4*
 *                is started                                 *
 * Postconditions: motor actions are controll                *
 *************************************************************/
void control_motor(void)
{
    if(isSwitch1Pressed && isTimerNeeded)
    {
        straight_line_control();
    }
    else if(isSwitch2Pressed && isTimerNeeded)
    {
        right_angle_control();
    }
}

/*************************************************************
 * Function:          displayStatistics                      *
 * Date Created:      02/22/2014                             *
 * Date Last Modified:02/22/2014                             *
 * Description: This function will send the speed, distance, *
 *              temperature and total time the motors have   *
 *              been running to the CLS for display every sec*
 *              Average temperature since last reset is      *
 *              displayed every 5 seconds                    *
 * Input parameters: none                                    *
 *                                                           *
 * Returns:          none                                    *
 * Usages:           Used to display the required statistics *
 *                                                           *
 * Preconditions: SPI2, I2C is configured and Switch 1 or 2  *
 *                is pressed                                 *
 * Postconditions: The Statistics are displayed              *
 *************************************************************/
void display_statistics(void)
{
    float temp = 0.0;
    float tempAvg = 0.0;
    double totalTime = 0.0;

    static unsigned char fiveSecCounter = 0;

    //displayCLS(motorSpeed, motorDistance);
    fiveSecCounter++;
    getCurrentTemperature(&temp);
    totalTime = totalMotorRunCounter*TIME_STEP*MSEC_2_SEC;
    if(fiveSecCounter >= 5)
    {
        getAverageTemperature(&tempAvg);
        fiveSecCounter = 0;
    }
    //display_SPI2(motorSpeed, motorDistance, totalTime, temp, tempAvg);
    display_CLS(motorSpeed, motorDistance, totalTime, temp, tempAvg);
}

/*************************************************************
 * Function:          controlMotorAfterDelay                 *
 * Date Created:      02/22/2014                             *
 * Date Last Modified:02/22/2014                             *
 * Description: This function will restart the motors after  *
 *              few msecs for Case 1. It is configure left   *
 *              motor to run in forward direction after few  *
 *              msec for case 2                              *
 * Input parameters: none                                    *
 *                                                           *
 * Returns:      none                                        *
 * Usages:      Used to restart the motors for Case 1 or     *
 *              to set the direction of left motor in forward*
 *              direction                                    *
 *                                                           *
 * Preconditions: Switch1 and 10 feet distance is completed  *
 *                or Switch2 is pressed and 5 feet distance  *
 *                is completed                               *
 *                                                           *
 * Postconditions: Motor control is managed                  *
 *************************************************************/
void control_motor_after_delay(void)
{
    static char counter = 0;

    //Increment time counter
    counter++;

    if(isSwitch1Pressed)
    {
        if(counter == 16)
        {
            //5 seconds is up, restart motor
            //xTimerStart(timerCounterHandle, 0);
            setup_OC2(1);
            setup_OC3(0);

            isTimerNeeded = TRUE;
            isEnableAutoCorrect = TRUE;
            //Stop timer 5
            CloseTimer5();
            counter = 0;
        }
    }
    else
    {
        if(counter == 6)
        {
            //If 5 seconds is up, reverse the direction left motor.
            PORTClearBits( IOPORT_D, BIT_2 ); //clear h bridge 2 enable
            delay(10);
            PORTClearBits( IOPORT_D, BIT_6 ); //set h bridge 1 dir (as anti clockwise)
            delay(10);
            CloseTimer5();
            isTimerNeeded = TRUE;
            isEnableAutoCorrect = TRUE;
            counter = 0;
        }
   }
}

