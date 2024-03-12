//ADC FUNCTIONS AND CODE


//ADC DEFINITIONS
#define AN1 ANSELAbits.ANSELA0
#define AN1_pin TRISAbits.TRISA0
#define AN3 ANSELAbits.ANSELA3
#define AN3_pin TRISAbits.TRISA3
#define AN4 ANSELAbits.ANSELA4
#define AN4_pin TRISAbits.TRISA4
#define AN5 ANSELBbits.ANSELB0
#define AN5_pin TRISBbits.TRISB0

//ADC VARIABLES
volatile int dataAN0;
volatile int dataAN1;
volatile int dataAN2;
volatile int dataAN3;
volatile float voltage0;
volatile float voltage1;
volatile float voltage2;
volatile float voltage3;

void init_ADC(void);
void EnableADC(void);

void init_ADC(void) {
    //set pins to ADC inputs
    
    AN1 = 1; //AN1 is analog
    AN1_pin = 1; //AN1 is input
    AN3 = 1;
    AN3_pin = 1;
    AN4 = 1;
    AN4_pin = 1;
    AN5 = 1;
    AN5_pin = 1;

    //configure the common ADC clock
    ADCON3Hbits.CLKSEL = 0;
    ADCON3Hbits.CLKDIV = 0; //no clock division (1:1)
    
    //configure the ADC core clocks
    ADCORE0Hbits.ADCS = 0; //clock divider (1:2)
    ADCORE1Hbits.ADCS = 0; //clock divider (1:2)
    ADCON2Lbits.SHRADCS = 0; //clock divider (1:2)
    
    //configure the ADC reference source
    ADCON3Lbits.REFSEL = 0; //AVdd is the voltage reference
    
    //set output format
    ADCON1Hbits.FORM = 0; //integer format
    
    //select input format
    ADMOD0Lbits.SIGN0 = 0; //unsigned
    ADMOD0Lbits.DIFF0 = 0; //single ended
    
    //set the ADC resolution
    ADCORE1Hbits.RES = 3; //12 bit resolution (dedicated core)
    ADCON1Hbits.SHRRES = 3; //12 bit resolution (shared core)
    
    //Enable ADC
    EnableADC();

    ADIELbits.IE0 = 1;
    ADIELbits.IE3 = 1;
    ADIELbits.IE4 = 1;
    ADIELbits.IE5 = 1;
    
    _ADCAN0IF = 0; // clear interrupt flag for AN0
    _ADCAN0IE = 1; // enable interrupt for AN0
    IFS5bits.ADCAN3IF = 0;
    IEC5bits.ADCAN3IE = 1;
    IFS5bits.ADCAN4IF = 0;
    IEC5bits.ADCAN4IE = 1;
    IFS6bits.ADCAN5IF = 0;
    IEC6bits.ADCAN5IE = 1;

    //set analog output triggers
    ADTRIG0Lbits.TRGSRC0 = 1; //Enable AN0 trigger
    ADTRIG0Hbits.TRGSRC3 = 1;
    ADTRIG1Lbits.TRGSRC4 = 1;
    ADTRIG1Lbits.TRGSRC5 = 1;
}

void EnableADC(void) {
    ADCON5Hbits.WARMTIME = 15; //set initialization time to maximum
    ADCON1Lbits.ADON = 1; //turn on the ADC module
    
    // Turn on and configure core 0
    ADCON5Lbits.C0PWR = 1;
    while(ADCON5Lbits.C0RDY == 0);
    ADCON3Hbits.C0EN = 1;
    
    //Turn on and configure core 1
    ADCON5Lbits.C1PWR = 1;
    while(ADCON5Lbits.C1RDY == 0);
    ADCON3Hbits.C1EN = 1;

    //Turn on and configure shared core
    ADCON5Lbits.SHRPWR = 1;
    // Wait when the shared core is ready for operation
    while(ADCON5Lbits.SHRRDY == 0);
    // Turn on digital power to enable triggers to the shared core
    ADCON3Hbits.SHREN = 1;
}