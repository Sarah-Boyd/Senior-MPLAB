//I2C FUNCTIONS----------------------------------------------------------------
//#define BRG      0x45
#define I2CBRG   I2C3BRG
#define I2CCONL  I2C3CONLbits
#define I2CCONH  I2C3CONHbits
#define I2CADDR  I2C3ADD
#define DATA_W   I2C3TRN
#define DATA_R   I2C3RCV
#define I2CSTAT  I2C3STATbits
#define MI2C_F   IFS8bits.MI2C3IF
#define MI2C_E   IEC8bits.MI2C3IE

//FLAGS
volatile int sending;
volatile int received;
volatile int stopped;
volatile int started;


void i2c_init(void);
void i2c_client_addr(int addr);
void i2c_start(void);
void i2c_rstart(void);
void i2c_rstart(void);
void i2c_stop(void);
void i2c_send(int data);
void i2c_ack(void);
void i2c_read(int data);

void i2c_init(void) {
    ANSELBbits.ANSELB7 = 0; //not analog!
    ANSELBbits.ANSELB2 = 0;
    
    //enable I2C
    I2CCONL.I2CEN = 1; //enable the third I2C module
    
    //set clock
    I2CBRG = 0x45;
    
    //I2C settings
    I2CCONL.A10M = 0; //client address is 7 bits
    
    //Enable MI2C Interrupt
    MI2C_E = 1; //enable the interrupt
    MI2C_F = 0; // lower the flag
    IPC35bits.MI2C3IP = 3;
    I2CCONH.PCIE = 1; //enable interrupt on stop condition
    I2CCONH.SCIE = 1; //enable interrupt on start condition
}

void i2c_start(void){
    started = 0;
    I2CCONL.SEN = 1; // assert a start condition
    __delay_ms(2)
    //while(started == 0); //wait for the start condition to complete
}

void i2c_rstart(void){
    started = 0;
    I2CCONL.RSEN = 1; //assert a repeated start
    __delay_ms(2)
    //while(started == 0);
}

void i2c_stop(void){
    I2CCONL.PEN = 1; //assert a stop condition
    while(I2CCONL.PEN == 1);
}

void i2c_send(int data) {
    sending = 1; //sending data so raise the flag
    DATA_W = data; //add data to register
    __delay_ms(2)
    //while(sending == 1); //wait until data transfer is complete
}

void i2c_ack(void){
    I2CCONL.ACKEN = 1; //send an acknowledge to the client
    while (I2CCONL.ACKEN == 1);
}

void i2c_read(int data) {
    received = 0; //sending data so raise the flag
    DATA_W = data; //add data to register
    __delay_ms(2)
    //while(received == 0); //wait until data transfer is complete
    received = 0;
}