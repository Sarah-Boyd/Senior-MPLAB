
#define AN1 ANSELBbits.ANSELB2
#define AN1_pin TRISBbits.TRISB2
#define AN3 ANSELCbits.ANSELC6
#define AN3_pin TRISCbits.TRISC6

void EnableADC(void);
void init_ADC(void);

void init_ADC(void) {
    //set pins to ADC inputs
    
    AN1 = 1;//B2, CS_1, AN1 is analog
    AN1_pin = 1; //B2, CS_1, AN1 is input
    AN3 = 1;//C6, SCK_2, AN17 is analog
    AN3_pin = 1; //C6, SCK_2, AN17 is input
    
    //configure the common ADC clock
    ADCON3Hbits.CLKSEL = 0;
    ADCON3Hbits.CLKDIV = 0; //no clock division (1:1)
    
    //configure the ADC core clocks
    ADCORE0Hbits.ADCS = 0; //clock divider (1:2)
    ADCORE1Hbits.ADCS = 0; //clock divider (1:2)
    //ADCON2Lbits.SHRADCS = 0; //clock divider (1:2)
    
    //configure the ADC reference source
    ADCON3Lbits.REFSEL = 0; //AVdd is the voltage reference
    
    //set output format
    ADCON1Hbits.FORM = 0; //integer format
    
    //select input format
    ADMOD0Lbits.SIGN0 = 0; //unsigned
    ADMOD0Lbits.DIFF0 = 0; //single ended
    ADMOD0Lbits.SIGN1 = 0; //unsigned
    ADMOD0Lbits.DIFF1 = 0; //single ended
    
    //set the ADC resolution
    ADCORE1Hbits.RES = 3; //12 bit resolution (dedicated core)
    ADCON1Hbits.SHRRES = 3; //12 bit resolution (shared core)
    
    //Enable ADC
    EnableADC();

    ADIELbits.IE0 = 1;
    ADIELbits.IE1 = 1;
//    ADIEHbits.IE17 = 1;
    
    _ADCAN0IF = 0; // clear interrupt flag for AN0
    _ADCAN0IE = 1; // enable interrupt for AN0
    _ADCAN1IF = 0; // clear interrupt flag for AN1
    _ADCAN1IE = 1; // enable interrupt for AN1
//    IFS6bits.ADCAN17IF = 0; //clear interrupt flag for AN17
//    IEC6bits.ADCAN17IE = 1; //enable interrupt for AN17

    //set analog output triggers
    ADTRIG0Lbits.TRGSRC0 = 1; //Enable AN0 trigger
    ADTRIG0Lbits.TRGSRC1 = 1; //Enable AN1 trigger
//    ADTRIG4Lbits.TRGSRC17 = 1; //Enable AN17 trigger
    
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