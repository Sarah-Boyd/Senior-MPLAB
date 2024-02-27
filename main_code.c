/* 
* File:   main_code.c
 * Author: mered
 *
 * Created on February 18, 2024, 10:46 AM
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
#pragma config ICS = PGD2               // ICD Communication Channel Select bits (Communicate on PGC3 and PGD3)
#pragma config JTAGEN = OFF             // JTAG Enable bit (JTAG is disabled)
#pragma config NOBTSWP = DISABLED       // BOOTSWP instruction disable bit (BOOTSWP instruction is disabled)

// FDEVOPT
#pragma config ALTI2C1 = OFF            // Alternate I2C1 Pin bit (I2C1 mapped to SDA1/SCL1 pins)
#pragma config ALTI2C2 = OFF            // Alternate I2C2 Pin bit (I2C2 mapped to SDA2/SCL2 pins)
#pragma config ALTI2C3 = ON           // Alternate I2C3 Pin bit (I2C3 mapped to SDA3/SCL3 pins)
#pragma config SMBEN = SMBUS            // SM Bus Enable (SMBus input threshold is enabled)
#pragma config SPI2PIN = PPS            // SPI2 Pin Select bit (SPI2 uses I/O remap (PPS) pins)

// FALTREG
#pragma config CTXT1 = OFF              // Specifies Interrupt Priority Level (IPL) Associated to Alternate Working Register 1 bits (Not Assigned)
#pragma config CTXT2 = OFF              // Specifies Interrupt Priority Level (IPL) Associated to Alternate Working Register 2 bits (Not Assigned)
#pragma config CTXT3 = OFF              // Specifies Interrupt Priority Level (IPL) Associated to Alternate Working Register 3 bits (Not Assigned)
#pragma config CTXT4 = OFF              // Specifies Interrupt Priority Level (IPL) Associated to Alternate Working Register 4 bits (Not Assigned)

#define FCY 60000000

#include <xc.h>
#include "libpic30.h"
#include "config.h"
#include "i2c_fnc.h"
#include "adc_fnc.h"
#include "imu_fnc.h"
#include "fuel_fnc.h"
 
 
//INERTIAL MEASUREMENT UNIT REGISTER MAP
//IMU address: 0x68
//Accelerometer: DATA8-DATA13 (0x0C-0x11)
//Gyroscope: DATA14-DATA19 (0x12-0x17)

//FUEL GAUGE INFORMATION
//Fuel Gauge address: 0x68

//ADD IN PCB INFORMATION
//Add in address: 0x18
//Add in FSR register: 0x30

//CLOCK DEFINITIONS
#define POSTD1 2
#define POSTD2 1
#define PRE_PLL 1
#define FBD_PLL 60

//PIN DEFINITIONS
#define BATTERY_INT_PIN 47 //INT_2 (will need to be 38 for real board)
#define BLE_INT_PIN 60 // PWM_2 (will need to be 43 for real board)
#define BLE_LED_PIN LATCbits.LATC7 //RST_1 (LATAbits.LATA2)
#define BLE_LED_PIN_SET TRISCbits.TRISC7 //(TRISAbits.TRISA2)
#define PWR_PIN_SET TRISBbits.TRISB2
#define PWR_PIN     LATBbits.LATB2


//MISC DEFINITIONS
#define ADD_W    0x30
#define ADD_R    0x31
#define ADD_DATA1 0x12
#define ADD_DATA2 0x14


//program variables
volatile int dataAN0;
volatile int dataAN1;
volatile int dataAN17;
volatile float voltage1;
volatile float voltage2;
volatile float voltage3;

//FLAGS
volatile int BLE_LED_ON = 0;
volatile int READ_SENSORS = 0;
volatile int READ_BATTERY = 0;

//SENSOR DATA
volatile float force_fsr1; //FSR1 on the Main PCB
volatile float force_fsr2; //FSR2 on the Main PCB
volatile float acceleration[] = {0,0,0}; //acceleration data from the IMU
volatile float angular_velocity[] = {0,0,0}; //gyroscope data from the IMU
volatile float fsr7_8[] = {0,0}; //FSR1 and 2 on the Add_In PCB
volatile float battery_voltage;
volatile float battery_soc;

//INTERRUPT SERVICE ROUTINES
void __attribute__((interrupt, no_auto_psv)) _ADCAN1Interrupt(void);
void __attribute__((interrupt, no_auto_psv)) _ADCAN0Interrupt(void);
void __attribute__((interrupt, no_auto_psv)) _MI2C3Interrupt(void);
void __attribute__((interrupt, no_auto_psv)) _INT1Interrupt(void);
void __attribute__((interrupt, no_auto_psv)) _INT2Interrupt(void);
void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void);

//INITIALIZATION FUNCTIONS
void init_pins(void);
void init_extInt1(void);
void init_extInt2(void);
void init_Timer1(void);

//MISC FUNCTIONS
float read_AddIn();
void read_sensor_data();

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
    
    //setup button
    ANSELCbits.ANSELC0 = 0; //C0 is set as digital, this is AN_1
    TRISCbits.TRISC0 = 1; //C0 is set as a digital input
    
    //INTIALIZE INTERRUPTS AND FUNCTIONS
    init_Timer1();
    i2c_init();
    init_ADC();
    
    //init_pins();
    //init_extInt();
    
    
    __delay_ms(150);
    
    //INITIALIZE IMU AND FUEL GAUGE
    //config_imu();
//    power_on_reset();
//    config_fuelgauge();
    
    while (1) {
        if (PORTCbits.RC0 == 0){ // Button is pressed
            //imu_data(0x0C);
            
//            b_p = read_percent();
//            b_v = read_voltage();
//            read_config();
            
            
        }
    }
}


//INTERRUPT SERVICE ROUTINES
void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void) {
    IFS0bits.T1IF = 0; //Clear flag
    
    READ_SENSORS = READ_SENSORS + 1; //increment the read counter
    READ_BATTERY = READ_BATTERY + 1;
    
        
    ADCON3Lbits.SWCTRG = 1;//Enable ADC Read
    ADCON5Hbits.SHRCIE = 1; //Enable shared ADC core read
    
    if(READ_SENSORS == 4){
        READ_SENSORS = 0;
        read_sensor_data();
    }
    
    if(READ_BATTERY == 20){
        battery_read();
    }

}

void read_sensor_data(void){
    //IMU DATA
      int data[] = {0,0,0,0,0,0};
//    data = read_imu(data);
//    acceleration[0] = data[0];
//    acceleration[1] = data[1];
//    acceleration[2] = data[2];
//    angular_velocity[0] = data[0];
//    angular_velocity[1] = data[1];
//    angular_velocity[2] = data[2];
    
    //FSR DATA - ADD IN PCB
    read_AddIn();
    
    
    //FSR DATA - SENSOR PCB
    
}

float read_AddIn(void){
    float fsr7;
    float fsr8;
    i2c_start();
    i2c_send(ADD_W);
    i2c_send(ADD_DATA1);
    i2c_rstart();
    received = 0;
    i2c_read(ADD_R);
    I2CCONL.RCEN = 1;
    while(I2CCONL.RCEN == 1);
    fsr7 = DATA_R;
    I2CCONL.ACKDT = 1;
    i2c_ack(); 
  
    __delay_ms(10)
            
    I2CCONL.RCEN = 1;
    while(I2CCONL.RCEN == 1);
    fsr7 = DATA_R;
    I2CCONL.ACKDT = 0;
    i2c_ack();
    i2c_stop();
           
//    i2c_start();
//    i2c_send(ADD_W);
//    i2c_send(ADD_DATA2);
//    i2c_rstart();
//    received = 0;
//    i2c_read(ADD_R);
//    I2CCONL.RCEN = 1;
//    while(I2CCONL.RCEN == 1);
//    fsr7 = DATA_R;
//    I2CCONL.ACKDT = 1;
//    i2c_ack(); 
//    i2c_stop();

    
    fsr7_8[0] = fsr7;
    fsr7_8[1] = fsr8;
}

void battery_read(void){
    battery_voltage = read_battery_voltage();
    battery_soc = read_battery_soc();
}

void __attribute__((interrupt, no_auto_psv)) _INT1Interrupt(void) {
    //BLE BUTTON TO INITIATE BLE CONNECTION
    if (BLE_LED_ON == 1){
       BLE_LED_PIN = 0;
       BLE_LED_ON = 0;
    }
    else{
       BLE_LED_PIN = 1; 
       BLE_LED_ON = 1;
    }
        
 
    IFS0bits.INT1IF = 0;    //Reset INT1 interrupt flag
}

void __attribute__((interrupt, no_auto_psv)) _INT2Interrupt(void) {
    //BATTERY IS LOW, SHUT IT OFFFFFFFFF
    
    PWR_PIN = 0;
    __delay_ms(10);
    PWR_PIN = 1; //power pin is high, shut off battery!    
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
    voltage2 = (float)dataAN0 * (float)(3.3/(float)4096); //Convert digital to voltage value
    _ADCAN0IF = 0; // clear interrupt flag
}

void __attribute__((interrupt, no_auto_psv)) _ADCAN1Interrupt(void) {
    dataAN1 = ADCBUF1;
    voltage1 = (float)dataAN1 * (float)(3.3/(float)4096); //Convert digital to voltage value
            
    //Clear interrupt flag
    _ADCAN1IF = 0;
}
//
//void __attribute__((interrupt, no_auto_psv)) _ADCAN17Interrupt(void)
//{
//    dataAN17 = ADCBUF17; // read conversion result
//    voltage3 = (float)dataAN17 * (float)(3.3/(float)4096); //Convert digital to voltage value
//    IFS6bits.ADCAN17IF = 0; // clear interrupt flag
//}


void init_Timer1(void) {
    
    T1CONbits.TON = 1;
    T1CONbits.TCKPS = 0;
    T1CONbits.TCS = 0;
    T1CONbits.TSYNC = 0;

    IPC0bits.T1IP = 1;
    IEC0bits.T1IE = 1;
    PR1 = 0xFFFF; //2048 clock cycle rollover
}

void init_pins(){
    PWR_PIN_SET = 0; //is an output
    PWR_PIN = 1; // pin out is 0
    
    BLE_LED_PIN_SET = 0; //is an output
    BLE_LED_PIN = 0; //pin out is 0
}

void init_extInt(){
    //SET UP EXTERNAL INTTERUPT 2
    INTCON2bits.INT1EP = 0; //interrupt on falling edge
    RPINR1bits.INT2R = BATTERY_INT_PIN; //set pin RP47 (INT_2)
    IFS1bits.INT2IF = 0;    //Reset INT2 interrupt flag 
    IPC5bits.INT2IP = 1; //set priority 2
    IEC1bits.INT2IE = 1;  //enable INT2
    
    //SET UP EXTERNAL INTERRUOT 1
    INTCON2bits.INT1EP = 1; //interrupt on rising edge
    RPINR0bits.INT1R = BLE_INT_PIN; //set pin RP47 (INT_2)
    IFS0bits.INT1IF = 0;    //Reset INT1 interrupt flag 
    IPC3bits.INT1IP = 1; //set priority 2
    IEC0bits.INT1IE = 1;  //enable INT1
}