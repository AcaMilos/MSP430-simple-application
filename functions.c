/**
 * @file function.c
 * @brief Implementation of function used to write to 7seg display
 *
 * @date 2017
 * @author Strahinja Jankovic (jankovics@etf.bg.ac.rs)
 * @author Marija Bezulj (meja@etf.bg.ac.rs)
 *
 * @version [2.0 - 04/2021] Hardware change
 * @version [1.0 - 03/2017] Initial version
 */

#include <functions.h>
#include <msp430.h>

/**
 * Table of settings for a-g lines for appropriate digit
 */
const unsigned int segtab2[] = {
        0x48,
        0x40,
        0x08,
        0x40,
        0x40,
        0x40,
        0x48,
        0x40,
        0x48,
        0x40
};

const unsigned int segtab3[] = {
        0x80,
        0x00,
        0x80,
        0x80,
        0x00,
        0x80,
        0x80,
        0x80,
        0x80,
        0x80
};

const unsigned int segtab4[] = {
        0x09,
        0x08,
        0x08,
        0x08,
        0x09,
        0x01,
        0x01,
        0x08,
        0x09,
        0x09
};

const unsigned int segtab8[] = {
        0x02,
        0x00,
        0x06,
        0x06,
        0x04,
        0x06,
        0x06,
        0x00,
        0x06,
        0x06
};

/*
 * --------------------------------Functions Definitions----------------------------------
 *
 * */


void WriteLed(unsigned int digit)
{

    P2OUT |= BIT6 | BIT3;
    P2OUT &= ~segtab2[digit];

    P3OUT |= BIT7;
    P3OUT &= ~segtab3[digit];

    P4OUT |= BIT3 | BIT0;
    P4OUT &= ~segtab4[digit];

    P8OUT |= BIT2 | BIT1;
    P8OUT &= ~segtab8[digit];
}


void SendDigitToUART(unsigned int digit){
    UCA1TXBUF = DIGIT2ASCII(digit);      // send second digit
}


unsigned int HexaToBCD(unsigned int distanceInDM){
    unsigned int shift = 0;
    unsigned int bcdResult = 0;

    while(distanceInDM){
        bcdResult |= (distanceInDM % 10) << (shift++ << 2);
        distanceInDM /= 10;
    }

    return bcdResult;
}


extern void ChangeLED(unsigned int activeLED, unsigned int distance, unsigned int LEDoption){

    if(distance > MAX_DISTANCE) distance = MAX_DISTANCE;

    /* Change currently active LED */
    switch(activeLED){
    case LED3:
        /* Pin P1.2 -> connecting to LED3 */
        P1SEL |= BIT2;      // Selected TA0 CCR2 on this pin
        P1DIR |= BIT2;      // Selected to be output
        /* Pin P1.3 -> disconnecting of LED4 */
        P1SEL &= ~BIT3;
        P1DIR &= ~BIT3;
        break;
    case LED4:
        /* Pin P1.2 -> disconnecting of LED3 */
        P1SEL &= ~BIT2;
        P1DIR &= ~BIT2;
        /* Pin P1.3 -> connecting to LED4 */
        P1SEL |= BIT3;      // Selected TA0 CCR2 on this pin
        P1DIR |= BIT3;      // Selected to be output
        break;
    default:
        break;
    }

    return;
}


extern void UpdatePWM(unsigned int activeLED, unsigned int distance, unsigned int LEDoption){

    unsigned int newPeriod = 0, dutyCycle;

    /* Change LED PWM Duty Cycle */
    if(LEDoption == CHANGE_PWM_DUTY_CYCLE){
        switch(activeLED){
        case LED3:
            /* Calculate new PWM parameters */
            newPeriod = MAX_PWM_PERIOD;
            dutyCycle = (distance * (MAX_PWM_PERIOD/MAX_DISTANCE)) & 0xFFFF;
            /* Timer A0 reconfigure settings */
            TA0CCR0 = newPeriod;                // write new Period
            TA0CCR1 = dutyCycle;                // write new DutyCycle
            break;
        case LED4:
            /* Calculate new PWM parameters */
            newPeriod = MAX_PWM_PERIOD;
            dutyCycle = (distance * (MAX_PWM_PERIOD/MAX_DISTANCE)) & 0xFFFF;
            /* Timer A0 reconfigure settings */
            TA0CCR0 = newPeriod;                // write new Period
            TA0CCR2 = dutyCycle;                // write new DutyCycle
            break;
        default:
            break;
        }
    }
    /* Change LED PWM Period */
    else{
        switch(activeLED){
        case LED3:
            /* Calculate new PWM parameters */
            newPeriod = (MIN_PWM_PERIOD + distance * (MAX_PWM_PERIOD/MAX_DISTANCE)) & 0xFFFF;
            dutyCycle = (newPeriod >> 1) & 0xFFFF;
            /* Timer A0 reconfigure settings */
            TA0CCR0 = newPeriod;                // write new Period
            TA0CCR1 = dutyCycle;                // write new DutyCycle
            break;
        case LED4:
            /* Calculate new PWM parameters */
            newPeriod = (MIN_PWM_PERIOD + distance * (MAX_PWM_PERIOD/MAX_DISTANCE)) & 0xFFFF;
            dutyCycle = (newPeriod >> 1) & 0xFFFF;
            /* Timer A0 reconfigure settings */
            TA0CCR0 = newPeriod;                // write new Period
            TA0CCR2 = dutyCycle;                // write new DutyCycle
            break;
        default:
            break;
        }
    }
    return;
}


void ConfigurePins(void){
    /* Pins setup */
    /* Pin 7.0 -> connected to DISP1 */
    P7DIR |= BIT0;      // set P7.0 as out (SEL1)
    P7OUT |= BIT0;      // disable display 1
    /* Pin 6.4 -> connected to DISP2 */
    P6DIR |= BIT4;      // set P6.4 as out (SEL2)
    P6OUT |= BIT4;      // disable display 2
    /* Pins P2.3, P2.6, P3.7, P4.0, P4.3, P8.1, P8.2 connected to segments a, b, c, d, e, f and g */
    P2DIR |= BIT6 | BIT3;       // configure P2.3 and P2.6 as out
    P3DIR |= BIT7;              // configure P3.7 as out
    P4DIR |= BIT3 | BIT0;       // configure P4.0 and P4.3 as out
    P8DIR |= BIT2 | BIT1;       // configure P8.1 and P8.2 as out
    /* Pin P3.6 -> connected to sensor TRIG pin (used for triggering ultrasonic ranging sensor) */
    P3SEL |= BIT6;      // Selected TB0 CCR6 on this pin
    P3DIR |= BIT6;      // Selected to be output
    /* Pin P2.0 -> connected to sensor ECHO pin (used to read value from ultrasonic ranging sensor) */
    P2SEL |= BIT0;      // Selected TA1 CCR1 on this pin
    P2DIR &= ~BIT0;     // Selected to be input
    /* Pin P1.3 -> connected to LED4 */
    P1SEL |= BIT3;      // Selected TA0 CCR2 on this pin
    P1DIR |= BIT3;      // Selected to be output
    /* Pin P2.1 -> Switch S1 */
    P2REN |= BIT1;      // enable pull up/down
    P2OUT |= BIT1;      // set pull up
    P2DIR &= ~BIT1;     // configure P2.1 as in
    P2IES |= BIT1;      // interrupt on falling edge
    P2IFG &= ~BIT1;     // clear flag
    P2IE  |= BIT1;      // enable interrupt
    /* Pin P1.1 -> Switch S2 */
    P1REN |= BIT1;      // enable pull up/down
    P1OUT |= BIT1;      // set pull up
    P1DIR &= ~BIT1;     // configure P2.1 as in
    P1IES |= BIT1;      // interrupt on falling edge
    P1IFG &= ~BIT1;     // clear flag
    P1IE  |= BIT1;      // enable interrupt
    /* Pin P1.4 -> Switch S3 */
    P1REN |= BIT4;      // enable pull up/down
    P1OUT |= BIT4;      // set pull up
    P1DIR &= ~BIT4;     // configure P2.1 as in
    P1IES |= BIT4;      // interrupt on falling edge
    P1IFG &= ~BIT4;     // clear flag
    P1IE  |= BIT4;      // enable interrupt
    /* Pins P4.4 and P4.5 -> UART */
    P4SEL |= BIT4 | BIT5;       // configure P4.4 and P4.5 for UART
    /* Pin P1.0 -> LED 1*/
    P1OUT &= ~BIT0;     // clear P1.0
    P1DIR |= BIT0;      // configure P1.0 as out
    /* Pin P4.7 -> LED 2*/
    P4OUT &= ~BIT7;     // clear P4.7
    P4DIR |= BIT7;      // configure P4.7 as out
    P4OUT |= BIT7;      // configure LED2 ON on start (start mode is CHANGE_PWM_DUTY_CYCLE)
}

void ConfigureLED_PWM_Timer(void){
    /* Timer A0 setup -> LED3 (P1.3 - used for generating PWM on LED3) */
    /* Timer A0 Capture/Compare Register 0 */
    TA0CCR0 = MAX_PWM_PERIOD;           // Timer A0 max value to count to it
    /* Timer A0 Capture/Compare Control Register 1 */
    TA0CCTL1 |= CM_0 | OUTMOD_3;        // Compare Mode, TA0CCR1 Output Mode: Set-Reset
    /* Timer A0 Capture/Compare Control Register 2 */
    TA0CCTL2 |= CM_0 | OUTMOD_3;        // Compare Mode, TA0CCR1 Output Mode: Set-Reset
    /* Timer A0 Capture/Compare Register 1 */
    TA0CCR1 = (MAX_PWM_PERIOD >> 1);         // Duty-cycle is 0 on program starting
    /* Timer A0 Capture/Compare Register 2 */
    TA0CCR2 = (MAX_PWM_PERIOD >> 1);         // Duty-cycle is 0 on program starting
    /* Starting Timer A0 */
    TA0CTL |= TASSEL__ACLK + MC__UP;    // Clocking by clock ACLK and count up to value stored in TA0CCR0
}

void ConfigureDisplayMuxTimer(void){
    /* Timer A2 setup -> DISPLAY MULTIPLEXING (used for multiplexing 7-segment display) */
    /* Timer A2 Capture/Compare Register 0 */
    TA2CCR0 = DISP_MUX_PERIOD;          // Timer A0 max value to count to it
    /* Timer A2 Capture/Compare Control Register 0 */
    TA2CCTL0 |= CCIE;            // Compare Mode, enable interrupt from TA2CCR0
    /* Starting Timer A2 */
    TA2CTL |= TASSEL__ACLK + MC__UP;    // Clocking by clock ACLK and count up to value stored in TA2CCR0
}

void ConfigureTriggerTimer(void){
    /* Timer B0 setup -> TRIG pin (P3.6 - used for triggering measure) */
    /* Timer B0 Capture/Compare Register 0 */
    TBCCR0 = TRIGGER_PERIOD;           // Timer B0 max value to count to it
    /* Timer B0 Capture/Compare Control Register 6 */
    TBCCTL6 |= OUTMOD_3;               // TA0CCR1 Output Mode: Set-Reset
    /* Timer B0 Capture/Compare Register 6 */
    TBCCR6 = TRIGGER_PULSE_WIDTH;      // Generating 10us pulse (logic '1') on TB0CCR1 output
    /* Starting Timer B0 */
    TBCTL |= TBSSEL__SMCLK + MC__UP;   // Clocking by clock SMCLK and count up to value stored in TB0CCR0
}

void ConfigureMeasureTimer(void){
    /* Timer A1 setup -> ECHO pin (P2.0 - used for reading sensor's measurement result) */
    /* Timer A1 Capture/Compare Control Register 1 */
    TA1CCTL1 |= CM_3 | SCS | CAP | CCIE;    // Capture Mode (on falling edge), P2.0 selected as input
    /* Starting Timer A1 */
    TA1CTL |= TASSEL__SMCLK | MC__CONTINUOUS;
}

void ConfigureUART(void){
    /* UART configuration */
    UCA1CTL1 |= UCSWRST;            // enter sw reset
    UCA1CTL0 = UCSPB;               // two stop bits
    UCA1CTL1 |= UCSSEL__ACLK;       // select ACLK as clk source
    UCA1BRW = BR9600;               // same as UCA1BR0 = 3; UCA1BR1 = 0
    UCA1MCTL |= UCBRS_3 + UCBRF_0;  // configure 9600 bps
    UCA1CTL1 &= ~UCSWRST;           // leave sw reset
    UCA1IE |= UCTXIE;               // enable TX interrupt
}


extern void WriteLed(unsigned int digit);

extern void SendDigitToUART(unsigned int digit);

extern unsigned int HexaToBCD(unsigned int distanceInDM);

extern void ChangeLED(unsigned int activeLED, unsigned int distance, unsigned int LEDoption);

extern void UpdatePWM(unsigned int activeLED, unsigned int distance, unsigned int LEDoption);

extern void ConfigurePins(void);

extern void ConfigureLED_PWM_Timer(void);

extern void ConfigureDisplayMuxTimer(void);

extern void ConfigureTriggerTimer(void);

extern void ConfigureMeasureTimer(void);

extern void ConfigureUART(void);
