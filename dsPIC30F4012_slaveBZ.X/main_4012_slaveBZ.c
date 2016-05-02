/*
 * File:   main.c (Slave Code)
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
#define PID_PWM_FREQUENCY       16000.0
#define PID_FOSC            20000000.0
#define PID_FCY             PID_FOSC/4.0
#define PID_PWM_COUNTS_PERIOD   (FCY/PWM_FREQUENCY)-1.0


// pid_t type
typedef struct {
    float Kp, Kd, Ki, T;
    unsigned short N;       // derivative filter parameter (3-20)
    float i, ilast;         // integral --> NOT USED IN THIS VERSION. FOR FUTURE IMPLEMENTATION
    float y, ylast;         // output
    float d, dlast;         // derivative term
    float u;                // u=uc-y
    float e, elast;            // error
} pid_t;
pid_t mypid;


// Misc. variables

//InData0 initialization (InData0[0] = POSCNT(Master), InData0[1] = POSCNT(Slave))
unsigned int InData0[4] = {30000, 0, 0, 0};
unsigned int ADCValue0 = 0;
unsigned int targetPos = 30000;
unsigned char iRecievedMsg =0;

//CAN BUS initialization
void InitCan(void) {
    // Initializing CAN Module Control Register
    C1CTRLbits.REQOP = CONFIG_MODE; // 4 = Configuration mode
    C1CTRLbits.CANCAP = 1; // Enable CAN capture
    C1CTRLbits.CSIDL = 0; // 0 = Continue CAN module op in idle mode
    C1CTRLbits.CANCKS = 0; // 1: Fcan=Fcy 0: Fcan=4Fcy
    C1CFG1bits.SJW = 0; // Synchronized jump width is 1xTq
    C1CFG1bits.BRP = 1; // Baud rate prescaler = 20 (CAN baud rate of 100kHz
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
void InitQEI(void)
{
    ADPCFG |= 0x0038;           // RB3, RB4, RB5 configured to digital pin
    QEICONbits.QEIM = 0;        // Disable QEI module
    QEICONbits.CNTERR = 0;      // Clear any count errors
    QEICONbits.QEISIDL = 0;     // Continue operation during sleep
    QEICONbits.SWPAB = 0;       // QEA and QEB not swapped
    QEICONbits.PCDOUT = 0;      // Normal I/O pin operation
    QEICONbits.POSRES = 1;      // Index pulse resets position counter
    QEICONbits.TQCS = 0;        // Internal clock source (Fcy) = 2Mhz
    DFLTCONbits.CEID = 1;       // Count error interrupts disabled
    DFLTCONbits.QEOUT = 1;      // Digital filters output enabled for QEn pins
    DFLTCONbits.QECK = 2;       // 1:4 clock divide for digital filter for QEn
                                // FILTER_DIV = (MIPS*FILTERED_PULSE)/3
                                //  ==> 5MHz*5usec/3 = 3.33 --> 2
    POSCNT = 30000;                 // Reset position counter
    MAXCNT = 0xFFFF; // 65,535
    QEICONbits.QEIM = 7;        // X4 mode with position counter reset by
                                // 6 - index pulse
                                // 7 - match (MAXCNT)
    return;
}

//PWM initialization
void InitPwm(void)
{
    PTCONbits.PTEN = 0;                 // Disable PWM timerbase
    PTCONbits.PTCKPS = PWM_PRESCALER;   // prescaler
    PTCONbits.PTOPS = 0;                // 1:1 postscaler
    PTCONbits.PTMOD = 0;                // free running mode
    PWMCON1bits.PMOD1 = 1;              // PWM in independent mode
    PWMCON1bits.PMOD2 = 1;              // PWM in independent mode
    PWMCON1bits.PEN1L = 1;              // PWM1L (PIN 24) output controlled by PWM
    PWMCON1bits.PEN2L = 1;              // PWM2L (PIN 26) output controlled by PWM
    PTMR = 0;                           // PWM counter value start at 0
    PTPER = PWM_COUNTS_PERIOD;          // Set PWM period
    return;
}

//Uart initialization
void InitUart(){
    U1MODEbits.UARTEN = 0;      // UART is disabled
    U1MODEbits.USIDL = 0;       // Continue operation in Idle Mode
    U1MODEbits.ALTIO = 1;       // UART communicates using U1ATX and U1ARX (pins 11&12)
    U1MODEbits.WAKE = 1;        // Enable wake-up on Start bit detec durign sleep mode
    U1MODEbits.PDSEL = 0;       // 8-bit data, no parity
    U1MODEbits.STSEL = 0;       // 2 stop bits

    U1STAbits.UTXISEL = 0;      // Interrupt when TX buffer has one character empty
    U1STAbits.UTXBRK = 0;       // U1TX pin operates normally
    U1STAbits.URXISEL = 0;      // Interrupt when word moves from REG to RX buffer
    U1STAbits.ADDEN = 0;        // Address detect mode disabled


    U1BRG =  7;           // p.507 of family reference
                                // 38400 baud rate for FOSC = 20MHz

    IFS0bits.U1TXIF = 0;        // Clear U1TX interrupt
    IFS0bits.U1RXIF = 0;        // Clear U1RX interrupt
    IPC2bits.U1TXIP = 5;        // U1TX interrupt 5 priority
    IPC2bits.U1RXIP = 5;        // U1RX interrupt 5 priority
    IEC0bits.U1TXIE = 1;        // Enable U1TX interrupt
    IEC0bits.U1RXIE = 1;        // Enable U1RX interrupt

    U1MODEbits.LPBACK = 0;      // Enable loopback mode
    U1MODEbits.UARTEN = 1;      // UART is enabled
    U1STAbits.UTXEN = 1;        // U1TX pin enabled

}

//ADC Initialization
void InitAdc(void) {
    ADPCFG &= ~0x0007; // Sets  PB1 and PB2 as analog pin (clear bits)
    ADCON1bits.ADON = 0; // A/D converter module off
    ADCON1bits.ADSIDL = 0; // Continue module operation in Idle mode
    ADCON1bits.SIMSAM = 0; // Samples multiple channels individually in sequence
    ADCON1bits.FORM = 0; // Integer (DOUT = 0000 00dd dddd dddd)
    ADCON1bits.SSRC = 7; // Internal counter ends sampling and starts conversion (auto convert)
    ADCON1bits.ASAM = 1; // Sampling in a channel begins when the conversion finishes (Auto-sets SAMP bit)

    ADCON2bits.VCFG = 7; // Set AVss and AVdd as A/D VrefH and VrefL
    ADCON2bits.CSCNA = 0;
    ADCON2bits.BUFM = 0; // Buffer configured as one 16-word buffer ADCBUF(15...0)
    ADCON2bits.CHPS = 0; // Convert CH0
    ADCON2bits.SMPI = 0; // Interrupts at the completion of conversion for each sample/convert sequence

    ADCON3bits.ADRC = 0; // Clock derived from system clock
    ADCON3bits.ADCS = 31; // A/D Conversion clock select = 32*Tcy
    ADCON3bits.SAMC = 31; // 31*Tad Auto-Sample time

    ADCHSbits.CH0SA = 1; // Channel 0 positive input is AN2
    ADCHSbits.CH0NA = 0; // Channel 0 negative input is Vref-

    // ADC Interrupt Initialization
    IFS0bits.ADIF = 0; // Clear timer 1 interrupt flag
    IEC0bits.ADIE = 1; // Enable timer 1 interrupts
    IPC2bits.ADIP = 5; // ADC interrupt priority 5 (default 4)
    return;
}

//Timer 1 initialization (Motor PWM output)
void InitTmr1(void)
{
   TMR1 = 0;                // Reset timer counter
   T1CONbits.TON = 0;       // Turn off timer 1
   T1CONbits.TSIDL = 0;     // Continue operation during sleep
   T1CONbits.TGATE = 0;     // Gated timer accumulation disabled
   T1CONbits.TCS = 0;       // Use Tcy as source clock
   T1CONbits.TCKPS = 0;     // Tcy/1 as input clock
   PR1 = 1000;              // (1000 before = 2.5k)
   IFS0bits.T1IF = 0;       // Clear timer 1 interrupt flag
   IEC0bits.T1IE = 1;       // Enable timer 1 interrupts
   IPC0bits.T1IP = 7;       // Enable timer 1 interrupts
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
   PR2 = 5000;              // (5000 = 500hz)
   IFS0bits.T2IF = 0;       // Clear timer 2 interrupt flag
   IEC0bits.T2IE = 1;       // Enable timer 2 interrupts
   IPC1bits.T2IP = 7;       // Enable timer 2 interrupts
   return;
}

//PID initialization function
void InitPid(pid_t *p, float kp, float kd, float ki, float T, unsigned short N, float il, float yl, float dl, float el)
{
    p->Kp = kp;
    p->Kd = kd;
    p->Ki = ki;
    p->T = T;
    p->N = N;
    p->ilast = il;
    p->ylast = yl;
    p->dlast = dl;
    p->elast = el;
}

//PID calcuation function as well motor PWM output
void CalcPid(pid_t *mypid)
{
    volatile float pidOutDutyCycle;
    volatile int target = InData0[0]; //Sets InData0[0] (Master position) as the target
    volatile int motorPos = (int)POSCNT;
    volatile float error = 0.0;

    //Calculates error through Targeted position and current motor position (Float)
    error = (float)(target - motorPos);

    //PID Output duty cycle based on error (Float)
    pidOutDutyCycle = (mypid->Kp*error);

    //Motor PWM Saturation
    if (pidOutDutyCycle >= 997.0) pidOutDutyCycle = 997.0;
    if (pidOutDutyCycle <= -997.0) pidOutDutyCycle = -997.0;

    //Motor CCW Operation
    if (pidOutDutyCycle < 0.0){
        PDC1 = 0;
        PDC2  = (unsigned int)((-1.0)*(pidOutDutyCycle));
    }
    //Motor CW Operation
    else {
        PDC1 = (unsigned int)(1.0)*(pidOutDutyCycle);
        PDC2  = 0;
    }
    return;
}

int main() {
    //ISR initializations
    InitAdc();
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

    // Initialize PID
    InitPid(&mypid, PID_KP, PID_KD, PID_KI, PID_TS, PID_N, 0.0, 0.0, 0.0, 0.0);
    
    //set initial QEI value
    POSCNT = 30000; // This prevents under and overflow of the POSCNT register

    //turn  on timer 1 (for Motor Control)
    T1CONbits.TON = 1;

    // Enable ADC Module
    ADCON1bits.ADON = 1; // A/D converter module on

    // Enable CAN module
    C1CTRLbits.REQOP = NORMAL;
    while (C1CTRLbits.OPMODE != NORMAL);

    //Turn on timer 2 (CAN bus message transmission)
    T2CONbits.TON = 1;
     

    while (1) {


    } //while
} // main

//ADC Interrupt
void __attribute__((interrupt, no_auto_psv)) _ADCInterrupt(void) {
    IFS0bits.ADIF = 0;//turn off ADC interrupt flag

    //Testing ADCValue0 data with LEDS when ADCValue0 has a value LEDRED is on
    if(ADCValue0 >= 1) {
        LEDRED = 1;
    } else {
        LEDRED = 0;
    }

    //Trigger Receive message to send data through CAN BUS
    iRecievedMsg = 1;

}

// Can Bus interrupt
void __attribute__((interrupt, no_auto_psv)) _C1Interrupt(void) {
    IFS1bits.C1IF = 0; // Clear interrupt flag

    if (C1INTFbits.TX0IF) {
        C1INTFbits.TX0IF = 0; //clear CAN bus transfer interrupt flag

    }

    if (C1INTFbits.RX0IF) {
        C1INTFbits.RX0IF = 0; //clear CAN bus receive interrupt flag

        //Move the recieve data from Buffers to InData
        InData0[0] = C1RX0B1; //POSCNT (Master)
        InData0[1] = C1RX0B2; //POSCNT (Slave)
        InData0[2] = C1RX0B3; //ADCValue (Slave) (Currently Not in Use)
        InData0[3] = C1RX0B4; // PIC ID for UART (1 = Master, 2 = Slave)
 
        C1RX0CONbits.RXFUL = 0; // clear CAN bus receive full bit
    }
}

//Timer 1 for outputing Motor PWM and PID Calculations
void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void)
{
    IFS0bits.T1IF = 0;   // Clear timer 1 interrupt flag
    CalcPid(&mypid);
    
}

//Timer for Sending CAN bus data
void __attribute__((interrupt, no_auto_psv)) _T2Interrupt(void)
{
    IFS0bits.T2IF = 0;   // Clear timer 2 interrupt flag

    //When Receive message flag is high send CAN Bus data
    if(iRecievedMsg == 1){
        C1TX0B2 = POSCNT; // Send Slave position Data
        C1TX0B4 = 2; // PIC ID for UART (1 = Master, 2 = Slave)
        C1TX0CONbits.TXREQ = 1;
        while (C1TX0CONbits.TXREQ != 0);
        iRecievedMsg = 0; //Clear Receive message flag
        }
    
}
