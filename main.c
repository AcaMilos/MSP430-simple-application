/* Project 39 - Parking Sensor Implementation
 *
 * Author:   Aleksandar Milosevic 658/17
 * Version:  V1
 * Date:     29.05.2022.
 * Platform: MSP430F5529LP
 * External peripherals:
 *      - Ultrasonic Ranging Module HC-SR04
 *      - LED
 *      - two 7-segment display
 *
 * MCU internal peripherals:
 *      - Parallel port pins  -> for HC-SR04, LED4, SW1, UART, 7-segment display
 *      - Timer A0 (CCR2)     -> for lightening LED4
 *      - Timer A2 (CCR0)     -> for 7-segment display multiplexing
 *      - Timer B0 (CCR6)     -> for sensor triggering
 *      - Timer A1 (CCR1)     -> for ultrasonic signal travel time counting
 *      - USCI A1 (UARTmode)  -> for sending result to USB
 *
 * -----------------------------------Project Summary---------------------------------------------
 *
 * Ultrasonic sensor (controlled from MCU) measure distance and MCU is showing it on 7-segment
 * display and light up LED LD4 (lighting mostly when distance is shorter, and lighting less often
 * when distance is far). Also user can press switch S1 (located on red board - LaunchPad) and
 * measurement result (distance measured in decimeters) will be sent using UART to USB connection
 * of LaunchPad.
 *
 * -----------------------------------------------------------------------------------------------
 *
 * */

#include <msp430.h> 
#include <stdint.h>
#include "functions.h"


/* Global variables */
volatile uint8_t EchoReceived = 0;          // Signal that sensor finished measuring
volatile uint16_t responseTime = 0;         // Time between sensor triggering until he started measuring
volatile uint16_t travellTime = 0;          // Time that ultrasonic signal spent in traveling from transimtter to receiver
volatile uint8_t distance = 0;              // Distance measured in decimeters
volatile uint8_t digitFirst = 0;            // First digit for showing on 7-segment display
volatile uint8_t digitSec = 0;              // Second digit for showing on 7-segment display
volatile uint8_t distanceBCD = 0;           // Distance in DM but in BCD format
volatile uint8_t UARTrequest = 0;           // Indicator that button SW1 is pressed and measured distance should be passed through UART
volatile uint8_t activeLED = LED4;          //
volatile uint8_t LEDoption = CHANGE_PWM_DUTY_CYCLE;    //
volatile uint8_t S1pressed = 0;             //
volatile uint8_t S2pressed = 0;             //
volatile uint8_t S3pressed = 0;             //
volatile uint16_t debounceTimeS1 = 0;       // Time for debouncing SW1 when he is pressed
volatile uint16_t debounceTimeS2 = 0;       // Time for debouncing SW2 when he is pressed
volatile uint16_t debounceTimeS3 = 0;       // Time for debouncing SW3 when he is pressed
volatile uint8_t P1IFG_temp = 0;            //


/* Functions declarations */
void TaskCalculateResult(void);
void TaskSendToUart(void);
void TaskChangeLED(void);
void TaskChangeLEDmodule(void);
void StartScheduler(void);


/* -------------------------------------TaskCalculateResult--------------------------------------------
 * Takes travel time measured with counter TA1 and calculates distance in decimeters and Duty Cycle.
 * Also this task is parsing digits of distance to BCD format so it can be shown on display.
 * After that Duty Cycle of LED3 light is changed on updated value.
 */
void TaskCalculateResult(void){

    distance = travellTime * (SOUND_SPEED / TWO_WAY_PATH);

    /* Reset flag*/
    EchoReceived = 0;

    /* Parse digits of distance to prepare to be shown on 7-segment display */
    distanceBCD = HexaToBCD(distance);
    digitFirst = (distanceBCD >> 4) & 0x0F;
    digitSec = distanceBCD & 0x0F;

    /* Update LED PWM*/
    UpdatePWM(activeLED, distance, LEDoption);
}


/* ----------------------------------------TaskSendToUart----------------------------------------------
 * Firstly is doing debounce and after that is sending first digit of distance measured in decimeters
 * to UART, and second digit will be sent from UART Interrupt Routine
 */
void TaskSendToUart(void){

    while(1){
        /* If measurement Result are received, jump to processing them in task TaskCalculateResult*/
        if(EchoReceived) return;
        // Software debounce
        debounceTimeS1++;
        if(debounceTimeS1 > 20000){
            /* Check if the pin is still pressed*/
            if(P2IN & BIT1){
                SendDigitToUART(digitFirst);
                /* debounceTimeS1 and S1pressed are reseting in UART ISR after all process of sending
                 * measured value is over
                 * */
                break;
            }
            else {
                /* If it was a glitch (not a real button S1 press) -> reset all variables and prepare
                 * them for next potential press*/
                debounceTimeS1 = 0;
                S1pressed = 0;
                P2IFG &= ~BIT1;     // clear flag
                P2IE |= BIT1;      // enable interrupt
            }
        }
    }
    return;
}


/* ----------------------------------------TaskChangeLED----------------------------------------------
 *
 */
void TaskChangeLED(void){

    while(1){
        /* If measurement Result are received, jump to processing them in task TaskCalculateResult*/
        if(EchoReceived) return;
        // Software debounce
        debounceTimeS2++;
        if(debounceTimeS2 > 20000){
            /* Check if the pin is still pressed*/
            if(P1IN & BIT1){
                /* Change active LED */
                if(activeLED == LED3) activeLED = LED4;
                else activeLED = LED3;
                ChangeLED(activeLED, distance, LEDoption);
            }
            // Reseting S2pressed flag and counter debounceTimeS2
            S2pressed = 0;
            debounceTimeS2 = 0;
            P1IFG &= ~BIT1;     // clear flag
            P1IE  |= BIT1;     // enable interrupt on SW1
        }
    }
}

/* ----------------------------------------TaskChangeLEDmodule----------------------------------------
 *
 */
void TaskChangeLEDmodule(void){

    while(1){
        /* If measurement Result are received, jump to processing them in task TaskCalculateResult*/
        if(EchoReceived) return;
        // Software debounce
        debounceTimeS3++;
        if(debounceTimeS3 > 20000){
            /* Check if the pin is still pressed*/
            if(P1IN & BIT4){
                /* Change option of LED type lighting when obstacle is closer and closer */
                if(LEDoption == CHANGE_PWM_DUTY_CYCLE) {
                    LEDoption = CHANGE_PWM_PERIOD;
                    /* Turn off LED2 */
                    P4OUT &= ~BIT7;
                    /* Turn on LED1 */
                    P1OUT |= BIT0;
                }
                else {
                    LEDoption = CHANGE_PWM_DUTY_CYCLE;
                    /* Turn off LED1 */
                    P1OUT &= ~BIT0;
                    /* Turn on LED2 */
                    P4OUT |= BIT7;
                }
            }
            S3pressed = 0;
            debounceTimeS3 = 0;
            P1IFG &= ~BIT4;     // clear flag
            P1IE  |= BIT4;      // enable interrupt on SW1
        }
    }
}

/*  -----------------------------------------Scheduler--------------------------------------------------
 *  Manages the Tasks and give them processor time when some of them is ready to be run.
 */
void StartScheduler(void){
    while(1){
        /* Checking conditions before giving processor time to some of tasks */
        if(EchoReceived == 1){
            TaskCalculateResult();
        }
        if(S1pressed == 1){
            TaskSendToUart();
        }
        if(S2pressed == 1){
            TaskChangeLED();
        }
        if(S3pressed == 1){
            TaskChangeLEDmodule();
        }
    }
}

/**
 * main.c
 */
int main(void)
{

    /**
     * Hardware Setup
     */

    /* Stop watchdog timer */
    WDTCTL = WDTPW | WDTHOLD;

    /* Pins setup */
    ConfigurePins();

    /* Timer A0 setup -> LED3 (P1.3 - used for generating PWM on LED3) */
    ConfigureLED_PWM_Timer();

    /* Timer A2 setup -> DISPLAY MULTIPLEXING (used for multiplexing 7-segment display) */
    ConfigureDisplayMuxTimer();

    /* Timer B0 setup -> TRIG pin (P3.6 - used for triggering measure) */
    ConfigureTriggerTimer();

    /* Timer A1 setup -> ECHO pin (P2.0 - used for reading sensor's measurement result) */
    ConfigureMeasureTimer();

    /* UART configuration */
    ConfigureUART();



    /* Enabling Global Interrupt*/
    __enable_interrupt();



    /* -------------------------Starting Scheduler---------------------------
     * Program is starting Scheduler and should never exit from it if all
     * program is work correctly
     * */
    StartScheduler();

}



/**
 * Interrupt Routines
 */

/* PORT1 interrupt routine */
void __attribute__ ((interrupt(PORT1_VECTOR))) P1ISR (void)
{
    P1IFG_temp = P1IFG;

    if (P1IFG_temp & BIT1)
    {
        P1IFG &= ~BIT1;     // clear flag
        P1IE  &= ~BIT1;     // disable interrupt on SW1
        S2pressed = 1;
    }

    if(P1IFG_temp & BIT4)
    {
        P1IFG &= ~BIT4;     // clear flag
        P1IE  &= ~BIT4;     // disable interrupt on SW1
        S3pressed = 1;
    }

    return;
}


/* Timer A1 interrupt routine */
void __attribute__ ((interrupt(TIMER1_A1_VECTOR))) TA1ISR (void){

    /* Read value from TA1CCR1 and storing it in sensorResult variable */
    if((TA1IV == TA1IV_TACCR1) && ((P2IN & BIT0) == 0x00)){
        // If falling edge -> Sensor has finished, save result that TimerA1 has counted
        travellTime = TA1R - responseTime;
        EchoReceived = 1;       // Setting a flag that result is received
    }
    else{
        // If rising edge -> Sensor gave first response that he started measuring, record time that passed until that
        responseTime = TA1R;
    }

    return;
}

/* Timer A2 interrupt routine */
void __attribute__ ((interrupt(TIMER2_A0_VECTOR))) TA2ISR (void){

    static uint8_t current_digit = 0;

    /* algorithm:
     * - turn off previous display (SEL signal)
     * - set a..g for current display
     * - activate current display
     */
    if (current_digit == 1)
    {
        P6OUT |= BIT4;          // turn off SEL2
        WriteLed(digitFirst);   // define seg a..g
        P7OUT &= ~BIT0;         // turn on SEL1
    }
    else if (current_digit == 0)
    {
        P7OUT |= BIT0;          // turn off SEL2
        WriteLed(digitSec);     // define seg a..g
        P6OUT &= ~BIT4;         // turn on SEL1
    }
    current_digit = (current_digit + 1) & 0x01;

    return;
}
