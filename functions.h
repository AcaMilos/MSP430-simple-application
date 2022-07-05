/*
 * function.h
 *
 *  Created on: 27 May 2022
 *      Author: ACA
 */

#ifndef FUNCTIONS_H_
#define FUNCTIONS_H_


/* Macros */
#define MAX_PWM_PERIOD         32768        // with ACLK = 32768Hz it will be: PWM_PERIOD = 1s
#define MIN_PWM_PERIOD          5000        // with ACLK = 32768Hz it will be: PWM_PERIOD ~ 122ms
#define DISP_MUX_PERIOD          163        // with ACLK = 32768Hz it will be ~5ms
#define TRIGGER_PERIOD        0xFFFF        // with SMCLK = 10MHz it will be: TRIGGER_PERIOD ~ 65ms
#define TRIGGER_PULSE_WIDTH       10        // with SMCLK = 10MHz it will be: TRIGGER_PULSE_WIDTH = 10us
#define SOUND_SPEED                0.0034   // sound speed in [decimeters/microsecond]
#define TWO_WAY_PATH               2        // ultrasonic signal is travelling from transmitter to obstacle and back to
                                            // receiver we have double time of signal travelling from sensor to obstacle,
                                            // and because of that it is necessary to divide measured time with 2
#define MAX_DISTANCE           40           // Maximum distance that sensor can measure is 40 decimeters
#define BR9600                  3           // Baud rate used for UART
#define CHANGE_PWM_PERIOD       1           //
#define CHANGE_PWM_DUTY_CYCLE   2           //
#define LED3                    3           //
#define LED4                    4           //



/** macro to convert digit to ASCII code */
#define DIGIT2ASCII(x)      (x + '0')


/**
 * @brief Function used to write to 7seg display
 * @param digit - value 0-9 to be displayed
 *
 * Function writes data a-g on PORT6.
 * It is assumed that appropriate 7seg display is enabled.
 */
extern void WriteLed(unsigned int digit);

/* Function that sends digit to UART interface*/
extern void SendDigitToUART(unsigned int digit);

/* Function that parse two number digits (Converts numbers form hexadecimal format into BCD format) */
extern unsigned int HexaToBCD(unsigned int distanceInDM);

/**/
extern void ChangeLED(unsigned int activeLED, unsigned int distance, unsigned int LEDoption);

/**/
extern void UpdatePWM(unsigned int activeLED, unsigned int distance, unsigned int LEDoption);

/* Function that set up pins for this project*/
extern void ConfigurePins(void);

/* Timer A0 setup -> LED3 (P1.3 - used for generating PWM on LED3) */
extern void ConfigureLED_PWM_Timer(void);

/* Timer A2 setup -> DISPLAY MULTIPLEXING (used for multiplexing 7-segment display) */
extern void ConfigureDisplayMuxTimer(void);

/* Timer B0 setup -> TRIG pin (P3.6 - used for triggering measure) */
extern void ConfigureTriggerTimer(void);

/* Timer A1 setup -> ECHO pin (P2.0 - used for reading sensor's measurement result) */
extern void ConfigureMeasureTimer(void);

/* Function that configure USCI_A1 registers for this project*/
extern void ConfigureUART(void);


#endif /* FUNCTIONS_H_ */
