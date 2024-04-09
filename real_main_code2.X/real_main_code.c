/* 
* File:   main_code.c
 * Author: mered
 *
 * Created on February 18, 2024, 10:46 AM
 */

//I2C CONFIGURATION SETTINGS
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

#include <xc.h>
#include <p33CK256MP502.h>
#include "libpic30.h"
#include "string.h"
#include "real_ble_fnc.h"
#include "real_config.h"
#include "real_i2c_fnc.h"
#include "real_adc_fnc.h"
#include "real_imu.h"
#include "real_fuel_gauge.h"
#include <stdio.h>
#include <stdlib.h>
 
 
//INERTIAL MEASUREMENT UNIT REGISTER MAP
//IMU address: 0x68
//Accelerometer: DATA8-DATA13 (0x0C-0x11)
//Gyroscope: DATA14-DATA19 (0x12-0x17)

//FUEL GAUGE INFORMATION
//Fuel Gauge address: 0x36

//ADD IN PCB INFORMATION
//Add in address: 0x18
//Add in FSR register: 0x12

//SENSOR PCB INFORMATION
//Sensor address: 0x28
//Add in FSR register: 0x14

//CLOCK DEFINITIONS
#define POSTD1 2
#define POSTD2 1
#define PRE_PLL 1
#define FBD_PLL 60

//PIN DEFINITIONS
#define BATTERY_INT_PIN 38
#define BLE_INT_PIN 43
#define BLE_LED_PIN LATBbits.LATB11
#define BLE_LED_PIN_SET TRISBbits.TRISB11
#define PWR_PIN_SET TRISAbits.TRISA4
#define PWR_PIN     LATAbits.LATA4


//MISC DEFINITIONS
#define ADD_W    0x30
#define ADD_R    0x31
#define ADD_DATA 0x12
#define SENSOR_W 0x50
#define SENSOR_R 0x51
#define SENSOR_DATA 0x14
#define rxFlag IFS0bits.U1RXIF

//program variables
volatile int dataAN0;
volatile int dataAN3;
volatile float fsr_o5;
volatile float fsr_o6;

//FLAGS
volatile int BLE_LED_ON = 0;
volatile int READ_FSRS = 0;
volatile int READ_BATTERY = 0;
volatile int READ_IMU = 0;
volatile int FSR_COUNT = 0;
volatile int BATTERY_COUNT = 0;
volatile int IMU_COUNT = 0;
volatile int SEND_DATA = 0;

//SENSOR DATA
volatile float acceleration[] = {0,0,0}; //acceleration data from the IMU
volatile float angular_velocity[] = {0,0,0}; //gyroscope data from the IMU
volatile float FSRs[] = {0,0,0,0,0,0,0,0}; //FSR data
volatile float battery_voltage;
volatile float battery_soc;
volatile float config;

//INTERRUPT SERVICE ROUTINES
void __attribute__((interrupt, no_auto_psv)) _ADCAN1Interrupt(void);
void __attribute__((interrupt, no_auto_psv)) _ADCAN0Interrupt(void);
void __attribute__((interrupt, no_auto_psv)) _MI2C3Interrupt(void);
void __attribute__((interrupt, no_auto_psv)) _INT1Interrupt(void);
void __attribute__((interrupt, no_auto_psv)) _INT2Interrupt(void);
void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void);
void _ISR _U1RXInterrupt(void);

//INITIALIZATION FUNCTIONS
void init_pins(void);
void init_extInt1(void);
void init_extInt2(void);
void init_Timer1(void);
void uart_Setup(void);
void init_extInt(void);

//MISC FUNCTIONS
void read_AddIn(void);
void read_Sensor(void);
void read_sensor_data(void);
void read_IMU_data(void);
void send_Data(void);

//MAIN CODE--------------------------------------------------------------------

int main(){        
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
    
    //INTIALIZE INTERRUPTS AND FUNCTIONS
    i2c_init();
    config_imu();
    init_Timer1();
    init_ADC();
    uart_Setup(); 
    
    init_pins();
    //init_extInt();
    
    config_fuelgauge();
    
    // Send AT commands on a button press
    while(1){
        BLE_LED_PIN = 1;
        if(READ_BATTERY == 1){
            READ_BATTERY = 0;
            battery_soc = read_battery_soc();
            config = read_config();
        }
        if(READ_IMU == 1){
            READ_IMU = 0;
            read_IMU_data();
        }
        if(READ_FSRS == 1){
            READ_FSRS = 0;
            read_AddIn();
            read_Sensor();
        }
        
        if(SEND_DATA == 1){
            send_Data();
            SEND_DATA = 0;
        }
    }
}


//INTERRUPT SERVICE ROUTINES
void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void) {
    IFS0bits.T1IF = 0; //Clear flag
    
    FSR_COUNT = FSR_COUNT + 1; //increment the read counter
    IMU_COUNT = IMU_COUNT + 1;
    BATTERY_COUNT = BATTERY_COUNT + 1;
    
        
    ADCON3Lbits.SWCTRG = 1;//Enable ADC Read
    ADCON5Hbits.SHRCIE = 1; //Enable shared ADC core read
    
    if(FSR_COUNT == 2){
        FSR_COUNT = 0;
        READ_FSRS = 1;
    }
    if(IMU_COUNT == 1){
        SEND_DATA = 1;
        IMU_COUNT = 0;
        READ_IMU = 1;
    }
    if(BATTERY_COUNT == 700){
        BATTERY_COUNT = 0;
        READ_BATTERY = 1;
    }
}

void read_IMU_data(void){
    //IMU DATA
    read_imu();
    acceleration[0] = imu_data[0]/16384;
    acceleration[1] = imu_data[1]/16384;
    acceleration[2] = imu_data[2]/16384;
    angular_velocity[0] = imu_data[3];
    angular_velocity[1] = imu_data[4];
    angular_velocity[2] = imu_data[5];
    
}

void read_AddIn(void){
    float fsr7;
    float fsr8;
    
    
    i2c_start();
    i2c_send(ADD_W);
    i2c_send(ADD_DATA);
    i2c_rstart();
    received = 0;
    i2c_read(ADD_R);
    I2CCONL.RCEN = 1;
    while(I2CCONL.RCEN == 1);
    fsr7 = DATA_R;
    I2CCONL.ACKDT = 0;
    i2c_ack(); 
  
            
    I2CCONL.RCEN = 1;
    while(I2CCONL.RCEN == 1);
    fsr8 = DATA_R;
    I2CCONL.ACKDT = 1;
    i2c_ack();
    i2c_stop();
               
    FSRs[6] = fsr7;
    FSRs[7] = fsr8;
}

void read_Sensor(void){
    float fsr1;
    float fsr2;
    float fsr3;
    float fsr4;
    
    
    i2c_start();
    i2c_send(SENSOR_W);
    i2c_send(SENSOR_DATA);
    i2c_rstart();
    received = 0;
    i2c_read(SENSOR_R);
    I2CCONL.RCEN = 1;
    while(I2CCONL.RCEN == 1);
    fsr1 = DATA_R;
    I2CCONL.ACKDT = 0;
    i2c_ack(); 
  
    //__delay_ms(1);
            
    I2CCONL.RCEN = 1;
    while(I2CCONL.RCEN == 1);
    fsr2 = DATA_R;
    i2c_ack();

    //__delay_ms(1);
            
    I2CCONL.RCEN = 1;
    while(I2CCONL.RCEN == 1);
    fsr3 = DATA_R;
    i2c_ack();
               
    //__delay_ms(1);
            
    I2CCONL.RCEN = 1;
    while(I2CCONL.RCEN == 1);
    fsr4 = DATA_R;
    I2CCONL.ACKDT = 1;
    i2c_ack();
    i2c_stop();
               
               
    FSRs[0] = (float)fsr1 * (float)(3.3/(float)4096);;
    FSRs[1] = (float)fsr2 * (float)(3.3/(float)4096);
    FSRs[2] = (float)fsr3 * (float)(3.3/(float)4096);
    FSRs[3] = (float)fsr4 * (float)(3.3/(float)4096);
}

void send_Data(void){

    char data_string[10] = {0,0,0,0,0,0,0,0,0,0};
    
    // SEND FSR DATA FIRST
    for (int i=0; i<8; i++){
        sprintf(data_string, "%.7f", FSRs[i]);
        for (int j=0; j<strlen(data_string); j++){
            U1TXREG = data_string[j]; // Send character
            while(U1STAHbits.UTXBE == 0){

            }
        }
    }
        
    // IMU DATA NEXT
    // acceleration data first
    for (int i=0; i<6; i++){
        sprintf(data_string, "%.7f", acceleration[i]);
        for (int j=0; j<strlen(data_string); j++){
            U1TXREG = data_string[j]; // Send character
            while(U1STAHbits.UTXBE == 0){

            }
        }
    }
    
    // now gyroscope data
    for (int i=0; i<6; i++){
        sprintf(data_string, "%.7f", angular_velocity[i]);
        for (int j=0; j<strlen(data_string); j++){
            U1TXREG = data_string[j]; // Send character
            while(U1STAHbits.UTXBE == 0){

            }
        }
    }
    
    // SEND BATTERY VOLTAGE NOW
    sprintf(data_string, "%.7f", battery_soc);
        for (int j=0; j<strlen(data_string); j++){
            U1TXREG = data_string[j]; // Send character
            while(U1STAHbits.UTXBE == 0){

            }
        }
    
    U1TXREG = 0x0D; // Send carriage return command
    while(U1STAHbits.UTXBE == 0){

    }

    U1TXREG = 0x0A; // Send line feed command
    while(U1STAHbits.UTXBE == 0){  
    }
}

//INTERRUPT SERVICE ROUTINES

void __attribute__((interrupt, no_auto_psv)) _INT2Interrupt(void) {
    //BATTERY IS LOW, SHUT IT OFFFFFFFFF
    
    PWR_PIN = 0;
    __delay_ms(10);
    PWR_PIN = 1; //power pin is high, shut off battery! 
    //clear interrupt flag
    config_fuelgauge();
    IFS1bits.INT2IF = 0;    //Reset INT2 interrupt flag
}

void __attribute__((interrupt, no_auto_psv)) _MI2C3Interrupt(void) {
    //lower the send flag! a send has completed
    if(I2CSTAT.R_W == 0){
        sending = 0;
    }
    if(I2CSTAT.R_W == 1){
        received = 1;
    }
    if(I2CSTAT.P == 1){
        stopped = 1;
    }
    if(I2CSTAT.S == 1){
        started = 1;
    }
    
    //lower the interrupt flag
    MI2C_F = 0;
}

void __attribute__((interrupt, no_auto_psv)) _ADCAN0Interrupt(void)
{
    dataAN0 = ADCBUF0; // read conversion result
    fsr_o6 = (float)dataAN0 * (float)(3.3/(float)4096); //Convert digital to voltage value
    _ADCAN0IF = 0; // clear interrupt flag
    
    FSRs[5] = fsr_o6;
}

void __attribute__((interrupt, no_auto_psv)) _ADCAN3Interrupt(void) {
    dataAN3 = ADCBUF3;
    fsr_o5 = (float)dataAN3 * (float)(3.3/(float)4096); //Convert digital to voltage value
            
    //Clear interrupt flag
    IFS5bits.ADCAN3IF = 0;
    
    FSRs[4] = fsr_o5;
}

//INITIALIZATION FUNCTIONS
void init_Timer1(void) {
    
    T1CONbits.TON = 1;
    T1CONbits.TCKPS = 3;
    T1CONbits.TCS = 0;
    T1CONbits.TSYNC = 0;

    IPC0bits.T1IP = 2;
    IEC0bits.T1IE = 1;
    PR1 = 0x0900; //go back to 0x0900 for speedy I2C
}

void init_pins(void){
    PWR_PIN_SET = 0; //is an output
    PWR_PIN = 1; // pin out is high
    
    BLE_LED_PIN_SET = 0; //is an output
    BLE_LED_PIN = 1; //pin out is 0
}

void init_extInt(void){    
    
    //SET UP EXTERNAL INTTERUPT 2
    INTCON2bits.INT1EP = 1; //interrupt on falling edge
    RPINR1bits.INT2R = BATTERY_INT_PIN; //set pin RP38
    IFS1bits.INT2IF = 0;    //Reset INT2 interrupt flag 
    IPC5bits.INT2IP = 1; //set priority 3
    IEC1bits.INT2IE = 1;  //enable INT2
    
}

void uart_Setup(void){
    
    ANSELBbits.ANSELB0 = 0;
    ANSELBbits.ANSELB1 = 0;
    
    TRISBbits.TRISB0 = 0;               // set RB0 to be an output
    TRISBbits.TRISB13 = 0;              // set RB13 to be an output
    
    LATBbits.LATB13 = 0;                // disable ESP
//    __delay_ms(2000);
//    LATBbits.LATB0 = 1; // set GPIO0 on ESP high for normal boot mode
    
    // Map the WIFI/BT pins to UART1
    // Note that the schematic is backwards for the dsPIC: 
    // The dsPIC TX pin is labeled as the RX_BLE pin (this is from the perspective of the ESP32)
    U1MODEbits.UTXEN = 0;               // Disable transmit bit
    U1MODEbits.UARTEN = 0;              // Disable UART
    __builtin_write_RPCON(0x0000);      // Unlock Registers
    RPINR18bits.U1RXR = 37;             // Map UART1 RX pin
    RPOR2bits.RP36R = 1;                // Map UART1 TX pin
//    RPINR18bits.U1RXR = 36;             // Map UART1 RX pin
//    RPOR2bits.RP37R = 1;                // Map UART1 TX pin
    __builtin_write_RPCON(0x0800);      // Lock Registers
    
    // Set up UART & baud rate
    U1MODEHbits.BCLKSEL = 0;            // FOSC/2 (Fp)
    U1MODEHbits.BCLKMOD = 0;            // use legacy divide-by-x counter for baud rate generation
    U1MODEbits.BRGH = 1;                // 0 is divide by 16, 1 is divide by 4
    U1BRGbits.BRG = 129; 
    U1MODEbits.MOD = 0;                 // Set to be asynchronous 8-bit UART
    U1MODEHbits.STSEL = 0;              // Set # of stop bits to 1 

//    // Disable then enable ESP to make sure GPIO0 is high on start
//    LATBbits.LATB13 = 1;                // enable ESP 
    
    // Enable UART
    U1MODEbits.UARTEN = 1;              // Enable UART
    __delay_ms(4000);                   // Recommended delay after enabling UART
    U1MODEbits.UTXEN = 1;               // Enable transmit bit
    U1MODEbits.URXEN = 1;               // Enable receive bit     
    
}