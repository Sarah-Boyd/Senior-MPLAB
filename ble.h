// Global Variables
int ATFlag = 1;
int dataFlag = 1;
int conFlag = 1;
int conCount = 0;
 
// Function prototypes
void _ISR _U1RXInterrupt(void);
void Setup(void);
void SendCmd(char *command);
void SendData(char *command, char *data);
void ATResponse();
void HandleData();
void BLESetup(void);
 
void SendCmd(char *command){
    int len = strlen(command);
    for (int i=0; i<len; i++){
        U1TXREG = command[i]; // Send character
        while(U1STAHbits.UTXBE == 0){
        }
    }
    U1TXREG = 0x0D; // Send carriage return command
    while(U1STAHbits.UTXBE == 0){
    }
    U1TXREG = 0x0A; // Send line feed command
    while(U1STAHbits.UTXBE == 0){
    }
    ATFlag = 1;
    ATResponse();
}
 
void SendData(char *command, char *data){
    int comLen = strlen(command);
    int dataLen = strlen(data);
    for (int i=0; i<comLen; i++){
        U1TXREG = command[i]; // Send character
        while(U1STAHbits.UTXBE == 0){
        }
    }
    U1TXREG = 0x0D; // Send carriage return command
    while(U1STAHbits.UTXBE == 0){
    }
    U1TXREG = 0x0A; // Send line feed command
    while(U1STAHbits.UTXBE == 0){
    }
//    while (dataFlag != 0){
//        
//    }
    __delay_ms(1000);
    for (int i=0; i<dataLen; i++){
        U1TXREG = data[i]; // Send character
        while(U1STAHbits.UTXBE == 0){
        }
    }
    ATFlag = 1;
    ATResponse();
}
 
void ATResponse(){
    while (ATFlag != 0){
    }
    __delay_ms(1000);
}
 
void Setup(void){
    // Pin configurations
    TRISDbits.TRISD0 = 0; // set WIFI/BT enable to a digital output
    //TRISBbits.TRISB10 = 0; // set WIFI/BT reset to a digital output
    ANSELCbits.ANSELC0 = 0; //C0 is set as digital, this is AN_1
    TRISCbits.TRISC0 = 1; //C0 is set as a digital input
    // Map the WIFI/BT pins to UART1
    // Note that the schematic is backwards for the dsPIC: 
    // The dsPIC TX pin is labeled as the RX_WIFI pin (this is from the perspective of the ESP32)
    U1MODEbits.UARTEN = 0; // disable UART
    __builtin_write_RPCON(0x0000); // Unlock Registers
    RPINR18bits.U1RXR = 71; // Map UART1 RX pin to RX-WIFI/BT
    RPOR19bits.RP70R = 1; // Map UART1 TX pin to TX-WIFI/BT
    __builtin_write_RPCON(0x0800); // Lock Registers
    // Set up UART
    U1MODEHbits.STSEL = 0;   // Set # of stop bits to 1 
    U1MODEbits.MOD = 0;      // Set to be asynchronous 8-bit UART
    U1MODEHbits.BCLKMOD = 0; // use legacy divide-by-x counter for baud rate generation
    U1MODEbits.BRGH = 0;  // 0 is divide by 16, 1 is divide by 4
    U1BRGbits.BRG = 8; 
    // Set up RX interrupts
    U1STAHbits.URXISEL = 0; // transmit interrupt when RX buffer has a character
    //U1STAHbits.RIDLE = 0; // Trigger interrupt when UART RX Line is receiving something
    IEC0bits.U1RXIE = 1; // Enable RX interrupt - I think this is right
    // Enable WIFI/BT
    LATDbits.LATD0 = 1; // enable WIFI/BT pin 
    // Enable UART
    U1MODEbits.UARTEN = 1; // Enable UART
    U1MODEbits.UTXEN = 1; // Enable transmit bit
    U1MODEbits.URXEN = 1; // Enable receive bit 
    __delay_ms(5000);
}
 
void BLESetup(void){
    // set up all the BLE stuff
    SendCmd("AT+RST");              // restart module
    SendCmd("AT+CWMODE=0");         // set WIFI to NULL
    SendCmd("AT+BLEINIT=2");        // set ESP as server
    SendCmd("AT+BLEGATTSSRVCRE");   // create services
    SendCmd("AT+BLEGATTSSRVSTART"); // start services
    SendCmd("AT+BLEADDR?");          // get BLE address
    SendCmd("AT+BLEADVDATA=\"0201060A09457370726573736966030302A0\""); // set advertising data
    SendCmd("AT+BLEADVSTART");      // start advertising
    SendCmd("AT+BLEGATTSSRV?");     // get services
    SendCmd("AT+BLEGATTSCHAR?");    // get characteristics
}
 
void HandleData(){
    // send some data
    while (conFlag == 1){ // wait until connection is established
 
    }
    __delay_ms(5000)
    SendData("AT+BLEGATTSNTFY=0,1,6,2", "83"); // THIS WORKS!!!
}
