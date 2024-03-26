
//IMU INITIALIZATION 
#define POWER_R  0x7D
#define POWER_M  0x0E
#define ACC_CR   0x40
#define ACC_CM   0xA8
#define GYR_CR   0x42
#define GYR_CM   0xA9
#define PW_CM    0x02
#define INIT_DATA 0x5E
#define PWR_CONF  0x7C
#define INT_CNTRL 0x59
#define INT_STATUS 0x21

//I2C ADDRESS
#define IMU_W    0xD0
#define IMU_R    0xD1
#define IMU_REG  0x0C


//IMU DATA
volatile int acc_x1;
volatile int acc_x2;
volatile int acc_x;
volatile int acc_y1;
volatile int acc_y2;
volatile int acc_y;
volatile int acc_z1;
volatile int acc_z2;
volatile int acc_z;
volatile int gyr_x1;
volatile int gyr_x2;
volatile int gyr_x;
volatile int gyr_y1;
volatile int gyr_y2;
volatile int gyr_y;
volatile int gyr_z1;
volatile int gyr_z2;
volatile int gyr_z;
volatile float imu_data[] = {0,0,0,0,0,0};

void imu_init(void);
void imu_check(void);
void config_imu(void);
void burst_write_config(void);
void imu_check_config();


void imu_check_config(){
    int chip_status;
    i2c_start();
    i2c_send(IMU_W);
    i2c_send(INT_STATUS);
    i2c_rstart();
    received = 0;
    i2c_read(IMU_R);
    I2CCONL.RCEN = 1; //enable a read
    while(I2CCONL.RCEN == 1);
    chip_status = DATA_R;
    I2CCONL.ACKDT = 1; //send a NACK
    i2c_ack();
    i2c_stop();
    
    __delay_ms(1);
}


void config_imu(void){
    __delay_ms(150);
   
    i2c_start();
    i2c_send(IMU_W);
    i2c_send(PWR_CONF);
    i2c_send(0x00); // Reset
    i2c_stop();
    __delay_ms(500);

    i2c_start();
    i2c_send(IMU_W);
    i2c_send(INT_CNTRL);
    i2c_send(0x00); // Power on
    i2c_stop();
    __delay_ms(100);
    
    burst_write_config();
    
    imu_check();
    imu_check_config();
    imu_init();
}


void burst_write_config(void){
    uint8_t byte_to_send;
    uint16_t k;
    i2c_start();
    i2c_send(IMU_W);
    k = 0;

    i2c_send(INIT_DATA);
    while( k < sizeof(bmi270_config_file)){
        byte_to_send = bmi270_config_file[k];
        i2c_send(byte_to_send);
        k = k + 1;
    }
    i2c_stop();
    __delay_ms(100);
    
    
    i2c_start();
    i2c_send(IMU_W);
    i2c_send(INT_CNTRL);
    i2c_send(0x01); // Reset
    i2c_stop();
    __delay_ms(500);
}

void read_imu(void){
    i2c_start();
    i2c_send(IMU_W);
    i2c_send(IMU_REG);
    i2c_rstart();
    received = 0;
    i2c_read(IMU_R);
    I2CCONL.RCEN = 1; //enable a read
    while(I2CCONL.RCEN == 1);
    acc_x1 = DATA_R;
    I2CCONL.ACKDT = 0; //send an ACK
    i2c_ack();

    I2CCONL.RCEN = 1; //enable a read
    while(I2CCONL.RCEN == 1);
    acc_x2 = DATA_R;
    I2CCONL.ACKDT = 0; //send an ACK
    i2c_ack();

    I2CCONL.RCEN = 1; //enable a read
    while(I2CCONL.RCEN == 1);
    acc_y1 = DATA_R;
    I2CCONL.ACKDT = 0; //send an ACK
    i2c_ack();
    
    I2CCONL.RCEN = 1; //enable a read
    while(I2CCONL.RCEN == 1);
    acc_y2 = DATA_R;
    I2CCONL.ACKDT = 0; //send an ACK
    i2c_ack();
   
    I2CCONL.RCEN = 1; //enable a read
    while(I2CCONL.RCEN == 1);
    acc_z1 = DATA_R;
    I2CCONL.ACKDT = 0; //send an ACK
    i2c_ack();
    
    I2CCONL.RCEN = 1; //enable a read
    while(I2CCONL.RCEN == 1);
    acc_z2 = DATA_R;
    I2CCONL.ACKDT = 0; //send an ACK
    i2c_ack();
    
    I2CCONL.RCEN = 1; //enable a read
    while(I2CCONL.RCEN == 1);
    gyr_x1 = DATA_R;
    I2CCONL.ACKDT = 0; //send an ACK
    i2c_ack();
    
    I2CCONL.RCEN = 1; //enable a read
    while(I2CCONL.RCEN == 1);
    gyr_x2 = DATA_R;
    I2CCONL.ACKDT = 0; //send an ACK
    i2c_ack();
    
    I2CCONL.RCEN = 1; //enable a read
    while(I2CCONL.RCEN == 1);
    gyr_y1 = DATA_R;
    I2CCONL.ACKDT = 0; //send an ACK
    i2c_ack();
    
    I2CCONL.RCEN = 1; //enable a read
    while(I2CCONL.RCEN == 1);
    gyr_y2 = DATA_R;
    I2CCONL.ACKDT = 0; //send an ACK
    i2c_ack();   
    
    I2CCONL.RCEN = 1; //enable a read
    while(I2CCONL.RCEN == 1);
    gyr_z1 = DATA_R;
    I2CCONL.ACKDT = 0; //send an ACK
    i2c_ack();
    
    //second byte
    I2CCONL.RCEN = 1; //enable a read
    while(I2CCONL.RCEN == 1);
    gyr_z2 = DATA_R;
    I2CCONL.ACKDT = 1; //send an NACK
    i2c_ack();
    i2c_stop();
    
    //put the data into an array
    imu_data[0] = acc_x2|(acc_x1 << 8);
    imu_data[1] = acc_y2|(acc_y1 << 8);
    imu_data[2] = acc_z2|(acc_z1 << 8);
    imu_data[3] = gyr_x2|(gyr_x1 << 8);
    imu_data[4] = gyr_y2|(gyr_y1 << 8);
    imu_data[5] = gyr_z2|(gyr_z1 << 8);
}

void imu_check(void){
    int chip_id;
    
    i2c_start();
    i2c_send(IMU_W);
    i2c_send(0x00);
    i2c_rstart();
    received = 0;
    i2c_read(IMU_R);
    I2CCONL.RCEN = 1; //enable a read
    while(I2CCONL.RCEN == 1);
    chip_id = DATA_R;
    I2CCONL.ACKDT = 1; //send a NACK
    i2c_ack();
    i2c_stop();
    
    __delay_ms(1);
}

void imu_init(void){
    //set up the power mode
    i2c_start();
    i2c_send(IMU_W);
    i2c_send(POWER_R);
    i2c_send(POWER_M);

    //setup the accelerometer mode
    i2c_start();
    i2c_send(IMU_W);
    i2c_send(ACC_CR);
    i2c_send(ACC_CM);
    
    //setup up the gyroscope mode
    i2c_start();
    i2c_send(IMU_W);
    i2c_send(GYR_CR);
    i2c_send(GYR_CM);

    //set up the power save mode
    i2c_start();
    i2c_send(IMU_W);
    i2c_send(PWR_CONF);
    i2c_send(PW_CM);
    
    __delay_ms(1);
}