//CODE FOR THE ADD IN BOARD
/*
 * File:   add_in.c
 * Author: mered
 *
 * Created on January 9, 2024, 4:06 PM
 */

// FOSCSEL
#pragma config FNOSC = FRC              // Oscillator Source Selection (Internal Fast RC (FRC))
#pragma config IESO = ON                // Two-speed Oscillator Start-up Enable bit (Start up device with FRC, then switch to user-selected oscillator source)

// FOSC
#pragma config POSCMD = NONE            // Primary Oscillator Mode Select bits (Primary Oscillator disabled)
#pragma config OSCIOFNC = OFF           // OSC2 Pin Function bit (OSC2 is clock output)
#pragma config FCKSM = CSECMD           // Clock Switching Mode bits (Clock switching is enabled,Fail-safe Clock Monitor is disabled)
#pragma config PLLKEN = ON              // PLL Lock Status Control (PLL lock signal will be used to disable PLL clock output if lock is lost)
#pragma config XTCFG = G3               // XT Config (24-32 MHz crystals)
#pragma config XTBST = ENABLE           // XT Boost (Boost the kick-start)

// FWDT
#pragma config RWDTPS = PS2147483648    // Run Mode Watchdog Timer Post Scaler select bits (1:2147483648)
#pragma config RCLKSEL = LPRC           // Watchdog Timer Clock Select bits (Always use LPRC)
#pragma config WINDIS = ON              // Watchdog Timer Window Enable bit (Watchdog Timer operates in Non-Window mode)
#pragma config WDTWIN = WIN25           // Watchdog Timer Window Select bits (WDT Window is 25% of WDT period)
#pragma config SWDTPS = PS2147483648    // Sleep Mode Watchdog Timer Post Scaler select bits (1:2147483648)
#pragma config FWDTEN = ON_SW           // Watchdog Timer Enable bit (WDT controlled via SW, use WDTCON.ON bit)

// FPOR
#pragma config BISTDIS = DISABLED       // Memory BIST Feature Disable (mBIST on reset feature disabled)

// FICD
#pragma config ICS = PGD1               // ICD Communication Channel Select bits (Communicate on PGC3 and PGD3)
#pragma config JTAGEN = OFF             // JTAG Enable bit (JTAG is disabled)
#pragma config NOBTSWP = DISABLED       // BOOTSWP instruction disable bit (BOOTSWP instruction is disabled)

// FDEVOPT
#pragma config ALTI2C1 = OFF            // Alternate I2C1 Pin bit (I2C1 mapped to SDA1/SCL1 pins)
#pragma config ALTI2C2 = OFF            // Alternate I2C2 Pin bit (I2C2 mapped to SDA2/SCL2 pins)
#pragma config ALTI2C3 = OFF            // Alternate I2C3 Pin bit (I2C3 mapped to SDA3/SCL3 pins)
#pragma config SMBEN = SMBUS            // SM Bus Enable (SMBus input threshold is enabled)
#pragma config SPI2PIN = PPS            // SPI2 Pin Select bit (SPI2 uses I/O remap (PPS) pins)

// FALTREG
#pragma config CTXT1 = OFF              // Specifies Interrupt Priority Level (IPL) Associated to Alternate Working Register 1 bits (Not Assigned)
#pragma config CTXT2 = OFF              // Specifies Interrupt Priority Level (IPL) Associated to Alternate Working Register 2 bits (Not Assigned)
#pragma config CTXT3 = OFF              // Specifies Interrupt Priority Level (IPL) Associated to Alternate Working Register 3 bits (Not Assigned)
#pragma config CTXT4 = OFF              // Specifies Interrupt Priority Level (IPL) Associated to Alternate Working Register 4 bits (Not Assigned)

#define FCY 60000000

//CLOCK DEFINITIONS
#define POSTD1 2
#define POSTD2 1
#define PRE_PLL 1
#define FBD_PLL 60


//ADD IN FUNCTIONS AND ISRs
void init_Timer1(void);
void __attribute__((interrupt, no_auto_psv)) _ADCAN1Interrupt(void);
void __attribute__((interrupt, no_auto_psv)) _ADCAN0Interrupt(void);
void __attribute__((interrupt, no_auto_psv)) _SI2C2Interrupt(void);
void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void);

#include <xc.h>
#include "libpic30.h"
#include "real_adc_fnc.h"
#include "real_i2c_client_fnc.h"

int main(void) {
    
    //SETUP CLOCK
    // CPU clock is 8MHz*60/1/1/2/2/2 = 60MIPS
    _POST1DIV = POSTD1; // :2
    _POST2DIV = POSTD2; // :1
    _PLLPRE = PRE_PLL; // :1
    PLLFBD = FBD_PLL; // x60
    __builtin_write_OSCCONH(1); // 1=FRCPLL
    __builtin_write_OSCCONL(1);
    while (_COSC != 1);
    while (OSCCONbits.LOCK != 1);
    
    //INITIATE TIMER, ADC, AND I2C
    init_ADC();
    init_Timer1();
    i2c_client_init();    
    
    while(1){
        

        
    }
    
    return 0;
}


// ADC AN0 ISR
void __attribute__((interrupt, no_auto_psv)) _ADCAN0Interrupt(void)
{
    dataAN0 = ADCBUF0; // read conversion result
    voltage0 = (float)dataAN0 * (float)(3.3/(float)4096); //Convert digital to voltage value
    _ADCAN0IF = 0; // clear interrupt flag
}

void __attribute__((interrupt, no_auto_psv)) _ADCAN1Interrupt(void) {
    dataAN1 = ADCBUF1;
    voltage1 = (float)dataAN1 * (float)(3.3/(float)4096); //Convert digital to voltage value
            
    //Clear interrupt flag
    _ADCAN1IF = 0;
}

void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void) {
    ADCON3Lbits.SWCTRG = 1;//Enable ADC Read
    IFS0bits.T1IF = 0; //Clear flag
}

void __attribute__((interrupt, no_auto_psv)) _SI2C2Interrupt(void) {
    int temp;
    int rbf;
    int rw;
    int da;
    int cov;
    
    rbf = I2CSTAT.RBF;
    rw = I2CSTAT.R_W;
    da = I2CSTAT.D_A;
    cov = I2CSTAT.I2COV;
    
   
    if(I2CSTAT.I2COV == 1){
        temp = I2CRCV;
    }
    else if(I2CSTAT.R_W == 0 && I2CSTAT.D_A == 1)			
    {
        while(I2CSTAT.RBF == 0);
        register_r = I2CRCV;
        I2CCONL.SCLREL = 1;//real send ack (free SCL)
    }
    else if (I2CSTAT.R_W == 1)	
    {
        if(register_r == 0x12){
            send_data();
        }
        else{
            register_r = I2CRCV;
            I2CCONL.SCLREL = 1;//real send ack (free SCL) 
        }
        
    }
    else
    {
        if(I2CSTAT.RBF == 1){
            temp = I2CRCV;
        }
        I2CCONL.SCLREL = 1;//send ack for address match
    }
    
    SI2C_F = 0; //clear interrupt flag for SI2C1
}

void init_Timer1(void) {
    
    T1CONbits.TON = 1;
    T1CONbits.TCKPS = 3;
    T1CONbits.TCS = 0;
    T1CONbits.TSYNC = 0;

    IPC0bits.T1IP = 1;
    IEC0bits.T1IE = 1;
    PR1 = 0xFFFFFFFF; //2048 clock cycle rollover
}
