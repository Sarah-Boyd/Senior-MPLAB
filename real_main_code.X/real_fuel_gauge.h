


//FUEL GAUGE DATA
volatile float b_p;
volatile float b_v;

//I2C ADDRESSES
#define FUEL_W   0x6C
#define FUEL_R   0x6D

void power_on_reset(void){
    i2c_start();
    i2c_send(FUEL_W);
    i2c_send(0xFE);
    i2c_send(0x54);
    i2c_send(0x00);
    
    __delay_ms(1);
}

void config_fuelgauge(void){
	/////////write configuration register//////////////
	i2c_start();
	i2c_send(0x6C);    //write MAX
	i2c_send(0x0C);    //mode register
	i2c_send(0x97); 
	i2c_send(0x00);    //set alert to 32%
	//i2c_stop();	
    
    __delay_ms(1);
}

void reset_fuelgauge(void){
	/////////write configuration register//////////////
	i2c_start();
	i2c_send(0x6C);    //write MAX
	i2c_send(0xFE);
	i2c_send(0x54); 
	i2c_send(0x00);    //set alert to 32%
	i2c_stop();
	i2c_start();	
}

float read_battery_soc(void){		
    uint8_t xm, xl,xo;
	
    i2c_start();
    i2c_send(FUEL_W);
    i2c_send(0x04);
    i2c_rstart();
    received = 0;
    i2c_read(FUEL_R);
    I2CCONL.RCEN = 1; //enable a read
    while(I2CCONL.RCEN == 1);
    xm = DATA_R;
    I2CCONL.ACKDT = 0;
    i2c_ack();
    
    I2CCONL.RCEN = 1; //enable a read
    while(I2CCONL.RCEN == 1);
    xl = DATA_R;
    I2CCONL.ACKDT = 1;
    i2c_ack();
    i2c_stop();

    __delay_ms(1);
    
	
	xo = (0.003906)*xl + xm;
	return xo;
}

long read_config(void){
    uint8_t xl,xm,xo;
    
    i2c_start();
    i2c_send(FUEL_W);
    i2c_send(0x0C);
    i2c_rstart();
    i2c_read(FUEL_R);
    I2CCONL.RCEN = 1; //enable a read
    while(I2CCONL.RCEN == 1);
    xm = DATA_R;
    I2CCONL.ACKDT = 0; //ACK
    i2c_ack();
    
    I2CCONL.RCEN = 1; //enable a read
    while(I2CCONL.RCEN == 1);
    xl = DATA_R;
    I2CCONL.ACKDT = 1; //NAK
    i2c_ack();
    i2c_stop();
    
    xo = xl|(xm << 8);
    
    __delay_ms(1);
    
    return xo;
}

float read_battery_voltage(void){		
	uint8_t xm, xl, temp, xo;
    
    i2c_start();
    i2c_send(FUEL_W);
    i2c_send(0x02);
    i2c_rstart();
    i2c_read(FUEL_R);
    I2CCONL.RCEN = 1; //enable a read
    while(I2CCONL.RCEN == 1);
    xm = DATA_R;
    I2CCONL.ACKDT = 0; //ACK
    i2c_ack();
    
    I2CCONL.RCEN = 1; //enable a read
    while(I2CCONL.RCEN == 1);
    xl = DATA_R;
    I2CCONL.ACKDT = 1; //NAK
    i2c_ack();
    i2c_stop();
	
	temp = ((xl|(xm << 8)) >> 4);
	xo = 1.25* temp;
    
    __delay_ms(1);
	
	//returns A/D value in mV
	return xo;
}