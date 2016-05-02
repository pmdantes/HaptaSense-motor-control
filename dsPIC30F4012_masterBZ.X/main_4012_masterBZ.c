/*
 * File:   main.c (Master Code)
 * Author: PaoloDantes & BrandonZhang
 *
 * Created on April 17, 2015, 2:23 PM
 */

#include <stdio.h>
#include <stdlib.h>
#include "config.h"
#include "p30FXXXX.h"



// IO definitions
#define LEDRED  LATEbits.LATE3 // RED
#define LEDYLW  LATEbits.LATE4 // YELLOW
#define LEDGRN  LATEbits.LATE5 // GREEN
#define TRISRED  TRISEbits.TRISE3 // YELLOW
#define TRISYLW  TRISEbits.TRISE4 // YELLOW
#define TRISGRN  TRISEbits.TRISE5 // GREEN


// CAN Operation Modes
#define CONFIG_MODE     4
#define LISTEN_ALL      7
#define LISTEN_ONLY     3
#define LOOPBACK        2
#define DISABLE         1
#define NORMAL          0

// CAN bit timing
#define FOSC        20000000 // 20Mhz Crystal
#define FCY         FOSC/4
#define BITRATE     1000000  // 100kbps
#define NTQ         16      // Amount of Tq per bit time
#define BRP_VAL     (((4*FCY)/(2*NTQ*BITRATE))-1) // refer to pg. 693 of Family Reference

// Define PID controls
#define PID_KP  2.0
#define PID_KD  0.1
#define PID_KI  0
#define PID_TI  0
#define PID_TD  0
#define PID_TS  10
#define PID_N   10

// Define PWM constants
#define PWM_FREQUENCY       16000
#define PWM_PRESCALER       0       // 0=1:1 1=1:4 2=1:16 3=1:64
#define PWM_COUNTS_PERIOD   (FCY/PWM_FREQUENCY)-1

// Define PWM PID controller constants
#define PID_PWM_FREQUENCY   16000.0
#define PID_FOSC            20000000.0
#define PID_FCY             PID_FOSC/4.0
#define PID_PWM_COUNTS_PERIOD   (FCY/PWM_FREQUENCY)-1.0

// pid_t type
typedef struct {
    float Kp, Kd, Ki, T;
    unsigned short N; // derivative filter parameter (3-20)
    float i, ilast; // integral --> NOT USED IN THIS VERSION. FOR FUTURE IMPLEMENTATION
    float y, ylast; // output
    float d, dlast; // derivative term
    float u; // u=uc-y
    float e, elast; // error
} pid_t;
pid_t mypid;

// Misc. variables

//InData0 initialization (InData0[0] = POSCNT(Master), InData0[1] = POSCNT(Slave))
unsigned int InData0[4] = {0, 30000, 0, 0};  
unsigned int targetPos = 30000; //Motor initial Target position

//CAN BUS initialization
void InitCan(void) {
    // Initializing CAN Module Control Register
    C1CTRLbits.REQOP = CONFIG_MODE; // 4 = Configuration mode
    C1CTRLbits.CANCAP = 1; // Enable CAN capture
    C1CTRLbits.CSIDL = 0; // 0 = Continue CAN module op in idle mode
    C1CTRLbits.CANCKS = 0; // 1: Fcan=Fcy 0: Fcan=4Fcy
    C1CFG1bits.SJW = 0; // Synchronized jump width is 1xTq
    C1CFG1bits.BRP = 1; // Baud rate prescaler = 20 (CAN baud rate of 100kHz) BRP = 1 means maximum CAN Bus Baud rate (should be 1mhz)
    C1CFG2bits.SEG2PHTS = 1; // 1=Freely Programmable 0=Maximum of SEG1PH or 3Tq's whichever is greater
    C1CFG2bits.PRSEG = 1; // Propagation Segment = 2Tq
    C1CFG2bits.SEG1PH = 6; // Phase Buffer Segment 1 = 7Tq
    C1CFG2bits.SEG2PH = 5; // Phase Buffer Segment 2 = 6Tq
    C1CFG2bits.SAM = 1; // 1=Bus line sampled 3 times 0=Bus line sampled once

    // Initializing CAN interrupt
    C1INTF = 0; // Reset all CAN interrupts
    IFS1bits.C1IF = 0; // Reset Interrupt flag status register
    C1INTE = 0x00FF; // Enable all CAN interrupt sources
    IEC1bits.C1IE = 1; // Enable CAN1 Interrupt

    /*---------------------------------------------------------
     *  CONFIGURE RECEIVER REGISTERS, FILTERS AND MASKS
     *---------------------------------------------------------*/

    // Configure CAN Module Receive Buffer Register
    C1RX0CONbits.DBEN = 0; // Buffer 0 does not overflow in buffer 1

    // Initializing Acceptance Mask Register
    C1RXM0SID = 0x1FFD;

    // Initializing Message Acceptance filter
    C1RXF0SID = 0x0AA8; // 0x0FA

    /*---------------------------------------------------------
     *  CONFIGURE RECEIVER REGISTERS, FILTERS AND MASKS
     *---------------------------------------------------------*/

    // Configure CAN Module Transmit Buffer Register
    C1TX0CONbits.TXPRI = 2; // 2 = High intermediate message priority

    // Initializing Transmit SID
    C1TX0SID = 0X50A8;
    C1TX0DLCbits.DLC = 8; // Data length is 8bytes

    C1CTRLbits.REQOP = NORMAL;
    while (C1CTRLbits.OPMODE != NORMAL); //Wait for CAN1 mode change from Configuration Mode to Loopback mode
}

//QEI Initialization
void InitQEI(void) {
    ADPCFG |= 0x0038; // RB3, RB4, RB5 configured to digital pin
    QEICONbits.QEIM = 0; // Disable QEI module
    QEICONbits.CNTERR = 0; // Clear any count errors
    QEICONbits.QEISIDL = 0; // Continue operation during sleep
    QEICONbits.SWPAB = 0; // QEA and QEB not swapped
    QEICONbits.PCDOUT = 0; // Normal I/O pin operation
    QEICONbits.POSRES = 1; // Index pulse resets position counter
    QEICONbits.TQCS = 0; // Internal clock source (Fcy) = 2Mhz
    DFLTCONbits.CEID = 1; // Count error interrupts disabled
    DFLTCONbits.QEOUT = 1; // Digital filters output enabled for QEn pins
    DFLTCONbits.QECK = 2; // 1:4 clock divide for digital filter for QEn
                            // FILTER_DIV = (MIPS*FILTERED_PULSE)/3
                            //  ==> 5MHz*5usec/3 = 3.33 --> 2
    POSCNT = 30000; // Reset position counter
    MAXCNT = 0xFFFF; // 65,535
    QEICONbits.QEIM = 7; // X4 mode with position counter reset by
                         // 6 - index pulse
                         // 7 - match (MAXCNT)
    return;
}

//PWM initialization
void InitPwm(void) {
    PTCONbits.PTEN = 0; // Disable PWM timerbase
    PTCONbits.PTCKPS = PWM_PRESCALER; // prescaler
    PTCONbits.PTOPS = 0; // 1:1 postscaler
    PTCONbits.PTMOD = 0; // free running mode
    PWMCON1bits.PMOD1 = 1; // PWM in independent mode
    PWMCON1bits.PMOD2 = 1; // PWM in independent mode
    PWMCON1bits.PEN1L = 1; // PWM1L (PIN 24) output controlled by PWM
    PWMCON1bits.PEN2L = 1; // PWM2L (PIN 26) output controlled by PWM
    PTMR = 0; // PWM counter value start at 0
    PTPER = PWM_COUNTS_PERIOD; // Set PWM period
    return;
}

//Timer 1 initialization (Motor PWM output (Haptic Feedback))
void InitTmr1(void) {
    TMR1 = 0; // Reset timer counter
    T1CONbits.TON = 0; // Turn off timer 1
    T1CONbits.TSIDL = 0; // Continue operation during sleep
    T1CONbits.TGATE = 0; // Gated timer accumulation disabled
    T1CONbits.TCS = 0; // Use Tcy as source clock
    T1CONbits.TCKPS = 0; // Tcy/1 as input clock
    PR1 = 5000;        // (5000 = 500hz)
    IFS0bits.T1IF = 0; // Clear timer 1 interrupt flag
    IEC0bits.T1IE = 1; // Enable timer 1 interrupts
    IPC0bits.T1IP = 7; // Enable timer 1 interrupts
    return;
}

//Timer 2 initialization (CAN bus Transfer)
void InitTmr2(void)
{
   TMR2 = 0;                // Reset timer counter
   T2CONbits.TON = 0;       // Turn off timer 2
   T2CONbits.TSIDL = 0;     // Continue operation during sleep
   T2CONbits.TGATE = 0;     // Gated timer accumulation disabled
   T2CONbits.TCS = 0;       // Use Tcy as source clock
   T2CONbits.TCKPS = 0;     // Tcy/1 as input clock
   PR2 = 10000;              // Interrupt period = 10ms
   IFS0bits.T2IF = 0;       // Clear timer 2 interrupt flag
   IEC0bits.T2IE = 1;       // Enable timer 2 interrupts
   IPC1bits.T2IP = 7;       // Enable timer 2 interrupts
   return;
}

int main() {
    //ISR initializations
    InitQEI();
    InitPwm();
    InitTmr1();
    InitTmr2();
    InitCan();


    //LED PORT initializations
    TRISRED = 0; // PORTE output
    TRISYLW = 0; // PORTE output
    TRISGRN = 0;

    //set all leds to be off initially
    LEDRED = 0;
    LEDYLW = 0;
    LEDGRN = 0;

    // Enable PWM Module
    PTCONbits.PTEN = 1;
    
    //set initial QEI value
    POSCNT = 30000; // This prevents under and overflow of the POSCNT register

    // Enable CAN module
    C1CTRLbits.REQOP = NORMAL;
    while (C1CTRLbits.OPMODE != NORMAL);

    //Turn on timer 1 (Motor PWM Calculation (Haptic Feedback))
    T1CONbits.TON = 1;

    //Turn on timer 2 (CAN bus message transmission)
    T2CONbits.TON = 1;

    while (1) {


    } //while
} // main


// Can Bus interrupt
void __attribute__((interrupt, no_auto_psv)) _C1Interrupt(void) {
    IFS1bits.C1IF = 0; // Clear interrupt flag

    if (C1INTFbits.TX0IF) {
        C1INTFbits.TX0IF = 0; //clear CAN bus transfer interrupt flag

    }

    // If a message was received sucessfully
    if (C1INTFbits.RX0IF) {
        C1INTFbits.RX0IF = 0; //clear CAN bus receive interrupt flag

        //Move the recieve data from Buffers to InData
        InData0[0] = C1RX0B1; //POSCNT (Master)
        InData0[1] = C1RX0B2; //POSCNT (Slave)
        InData0[2] = C1RX0B3; //ADCValue (Slave) (Currently Not in Use)
        InData0[3] = C1RX0B4; //Sends Confirmation receive flag for Slave

        C1RX0CONbits.RXFUL = 0; //clear CAN bus receive full bit
    }
}


//Timer 1 for outputing Haptic Feedback
void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void)
{
    IFS0bits.T1IF = 0; // Clear timer 1 interrupt flag
    volatile float error = 0.0;

      //Motor PWM calculations (Provides Haptic Feedback (Absolute Position Control))
      //Calculates Error (float) and then uses error data to produce PWM
      //error is calculated using the difference between InData0[1] (Slave position) and the current position

      //Motor moves CW for haptic feedback (turn on red LED)
    if (InData0[1] > POSCNT) {
        error = (float) (InData0[1] - POSCNT);
        PDC1 = (unsigned int) ((1.0)*(error));
        PDC2 = 0;
        LEDGRN = 0;
        LEDRED = 1;

      //Motor moves CCW for haptic feedback (turn on green LED)
    } else if (InData0[1] < POSCNT) {
        error = (float) (POSCNT - InData0[1]);
        LEDGRN = 1;
        LEDRED = 0;
        PDC1 = 0;
        PDC2 = (unsigned int) (1.0)*(error);

      //NO PWM output to motor
    } else {
        PDC1 = 0;
        PDC2 = 0;
        LEDRED = 0;
        LEDGRN = 0;
    }

}

//Timer for Sending CAN bus data
void __attribute__((interrupt, no_auto_psv)) _T2Interrupt(void)
{
    IFS0bits.T2IF = 0; // Clear timer 2 interrupt flag
    C1TX0B1 = POSCNT; // Send Master position Data
    C1TX0B4 = 1; // PIC ID for UART (1 = Master, 2 = Slave)
    C1TX0CONbits.TXREQ = 1;
    while (C1TX0CONbits.TXREQ != 0);

}
