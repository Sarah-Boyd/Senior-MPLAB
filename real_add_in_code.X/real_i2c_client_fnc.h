
//I2C DEFINITIONS

#define BRG     0x45
#define I2CCONL I2C3CONLbits
#define I2CCONH I2C3CONHbits
#define I2CADDR I2C3ADD
#define I2CMSK  I2C3MSK
#define I2CDATA I2C3TRN
#define I2CSTAT I2C3STATbits
#define I2CRCV  I2C3RCV
#define ADDRESS 0x18
#define SI2C_E  IEC8bits.SI2C3IE
#define SI2C_F  IFS8bits.SI2C3IF

//I2C VARIABLES
volatile float register_r;


void i2c_client_init(void);
void i2c_send(int data);

void i2c_client_init(void) {
    //enable I2C
    I2CCONL.I2CEN = 1; //enable the third I2C module
    
    //set clock
    I2C3BRG = BRG;
    
    //I2C settings
    I2CCONL.A10M = 0; //client address is 7 bits
    
    //set up slave settings
    I2CCONH.AHEN = 1; //disable address hold
    I2CCONH.DHEN = 0; //disable data hold
    
    //Enable SI2C Interrupt
    SI2C_E = 1; //enable the interrupt
    SI2C_F = 0; // lower the flag
    
    I2CCONH.PCIE = 0; //enable interrupt on stop condition
    I2CCONH.SCIE = 0; //enable interrupt on start condition
    
    I2CMSK  = 0;
    I2CADDR = ADDRESS;  // set I2C address as 0x70
}

void i2c_send(int data){
    I2C3TRN = data;
    I2CCONL.SCLREL = 1;
    while(I2CSTAT.TBF == 1);
}

void send_data(void){
    int temp;
    int ack;
    
    I2CCONL.SCLREL = 0;
    i2c_send(dataAN0);
    register_r = 0x00;
    ack = I2CSTAT.ACKSTAT;
    temp = I2CRCV;
    if(I2CSTAT.ACKSTAT == 0){
        i2c_send(dataAN1);
        register_r = 0x00;
        temp = I2CRCV;
    }
}