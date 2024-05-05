#include "TM4C123GH6PM.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "math.h"

//Function Prototypes
void Delay(unsigned long counter); // used to add delay
void HC05_init(void); // Initialize UART5 module for HC-05
char Bluetooth_Read(void); //Read data from Rx5 pin of TM4C123
void Bluetooth_Write(unsigned char data); // Transmit a character to HC-05 over Tx5 pin 
void Bluetooth_Write_String(char *str); // Transmit a string to HC-05 over Tx5 pin 
void readDHT11(uint8_t* humidity, uint8_t* temperature);
void Bluetooth_Write_Humidity(uint8_t humidity);
void Bluetooth_Write_Temperature(uint8_t temperature);

const uint8_t OledFont[][8] =
{
  {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
  {0x00,0x00,0x5F,0x00,0x00,0x00,0x00,0x00},
  {0x00,0x00,0x07,0x00,0x07,0x00,0x00,0x00},
  {0x00,0x14,0x7F,0x14,0x7F,0x14,0x00,0x00},
  {0x00,0x24,0x2A,0x7F,0x2A,0x12,0x00,0x00},
  {0x00,0x23,0x13,0x08,0x64,0x62,0x00,0x00},
  {0x00,0x36,0x49,0x55,0x22,0x50,0x00,0x00},
  {0x00,0x00,0x05,0x03,0x00,0x00,0x00,0x00},
  {0x00,0x1C,0x22,0x41,0x00,0x00,0x00,0x00},
  {0x00,0x41,0x22,0x1C,0x00,0x00,0x00,0x00},
  {0x00,0x08,0x2A,0x1C,0x2A,0x08,0x00,0x00},
  {0x00,0x08,0x08,0x3E,0x08,0x08,0x00,0x00},
  {0x00,0xA0,0x60,0x00,0x00,0x00,0x00,0x00},
  {0x00,0x08,0x08,0x08,0x08,0x08,0x00,0x00},
  {0x00,0x60,0x60,0x00,0x00,0x00,0x00,0x00},
  {0x00,0x20,0x10,0x08,0x04,0x02,0x00,0x00},
  {0x00,0x3E,0x51,0x49,0x45,0x3E,0x00,0x00},
  {0x00,0x00,0x42,0x7F,0x40,0x00,0x00,0x00},
  {0x00,0x62,0x51,0x49,0x49,0x46,0x00,0x00},
  {0x00,0x22,0x41,0x49,0x49,0x36,0x00,0x00},
  {0x00,0x18,0x14,0x12,0x7F,0x10,0x00,0x00},
  {0x00,0x27,0x45,0x45,0x45,0x39,0x00,0x00},
  {0x00,0x3C,0x4A,0x49,0x49,0x30,0x00,0x00},
  {0x00,0x01,0x71,0x09,0x05,0x03,0x00,0x00},
  {0x00,0x36,0x49,0x49,0x49,0x36,0x00,0x00},
  {0x00,0x06,0x49,0x49,0x29,0x1E,0x00,0x00},
  {0x00,0x00,0x36,0x36,0x00,0x00,0x00,0x00},
  {0x00,0x00,0xAC,0x6C,0x00,0x00,0x00,0x00},
  {0x00,0x08,0x14,0x22,0x41,0x00,0x00,0x00},
  {0x00,0x14,0x14,0x14,0x14,0x14,0x00,0x00},
  {0x00,0x41,0x22,0x14,0x08,0x00,0x00,0x00},
  {0x00,0x02,0x01,0x51,0x09,0x06,0x00,0x00},
  {0x00,0x32,0x49,0x79,0x41,0x3E,0x00,0x00},
  {0x00,0x7E,0x09,0x09,0x09,0x7E,0x00,0x00},
  {0x00,0x7F,0x49,0x49,0x49,0x36,0x00,0x00},
  {0x00,0x3E,0x41,0x41,0x41,0x22,0x00,0x00},
  {0x00,0x7F,0x41,0x41,0x22,0x1C,0x00,0x00},
  {0x00,0x7F,0x49,0x49,0x49,0x41,0x00,0x00},
  {0x00,0x7F,0x09,0x09,0x09,0x01,0x00,0x00},
  {0x00,0x3E,0x41,0x41,0x51,0x72,0x00,0x00},
  {0x00,0x7F,0x08,0x08,0x08,0x7F,0x00,0x00},
  {0x00,0x41,0x7F,0x41,0x00,0x00,0x00,0x00},
  {0x00,0x20,0x40,0x41,0x3F,0x01,0x00,0x00},
  {0x00,0x7F,0x08,0x14,0x22,0x41,0x00,0x00},
  {0x00,0x7F,0x40,0x40,0x40,0x40,0x00,0x00},
  {0x00,0x7F,0x02,0x0C,0x02,0x7F,0x00,0x00},
  {0x00,0x7F,0x04,0x08,0x10,0x7F,0x00,0x00},
  {0x00,0x3E,0x41,0x41,0x41,0x3E,0x00,0x00},
  {0x00,0x7F,0x09,0x09,0x09,0x06,0x00,0x00},
  {0x00,0x3E,0x41,0x51,0x21,0x5E,0x00,0x00},
  {0x00,0x7F,0x09,0x19,0x29,0x46,0x00,0x00},
  {0x00,0x26,0x49,0x49,0x49,0x32,0x00,0x00},
  {0x00,0x01,0x01,0x7F,0x01,0x01,0x00,0x00},
  {0x00,0x3F,0x40,0x40,0x40,0x3F,0x00,0x00},
  {0x00,0x1F,0x20,0x40,0x20,0x1F,0x00,0x00},
  {0x00,0x3F,0x40,0x38,0x40,0x3F,0x00,0x00},
  {0x00,0x63,0x14,0x08,0x14,0x63,0x00,0x00},
  {0x00,0x03,0x04,0x78,0x04,0x03,0x00,0x00},
  {0x00,0x61,0x51,0x49,0x45,0x43,0x00,0x00},
  {0x00,0x7F,0x41,0x41,0x00,0x00,0x00,0x00},
  {0x00,0x02,0x04,0x08,0x10,0x20,0x00,0x00},
  {0x00,0x41,0x41,0x7F,0x00,0x00,0x00,0x00},
  {0x00,0x04,0x02,0x01,0x02,0x04,0x00,0x00},
  {0x00,0x80,0x80,0x80,0x80,0x80,0x00,0x00},
  {0x00,0x01,0x02,0x04,0x00,0x00,0x00,0x00},
  {0x00,0x20,0x54,0x54,0x54,0x78,0x00,0x00},
  {0x00,0x7F,0x48,0x44,0x44,0x38,0x00,0x00},
  {0x00,0x38,0x44,0x44,0x28,0x00,0x00,0x00},
  {0x00,0x38,0x44,0x44,0x48,0x7F,0x00,0x00},
  {0x00,0x38,0x54,0x54,0x54,0x18,0x00,0x00},
  {0x00,0x08,0x7E,0x09,0x02,0x00,0x00,0x00},
  {0x00,0x18,0xA4,0xA4,0xA4,0x7C,0x00,0x00},
  {0x00,0x7F,0x08,0x04,0x04,0x78,0x00,0x00},
  {0x00,0x00,0x7D,0x00,0x00,0x00,0x00,0x00},
  {0x00,0x80,0x84,0x7D,0x00,0x00,0x00,0x00},
  {0x00,0x7F,0x10,0x28,0x44,0x00,0x00,0x00},
  {0x00,0x41,0x7F,0x40,0x00,0x00,0x00,0x00},
  {0x00,0x7C,0x04,0x18,0x04,0x78,0x00,0x00},
  {0x00,0x7C,0x08,0x04,0x7C,0x00,0x00,0x00},
  {0x00,0x38,0x44,0x44,0x38,0x00,0x00,0x00},
  {0x00,0xFC,0x24,0x24,0x18,0x00,0x00,0x00},
  {0x00,0x18,0x24,0x24,0xFC,0x00,0x00,0x00},
  {0x00,0x00,0x7C,0x08,0x04,0x00,0x00,0x00},
  {0x00,0x48,0x54,0x54,0x24,0x00,0x00,0x00},
  {0x00,0x04,0x7F,0x44,0x00,0x00,0x00,0x00},
  {0x00,0x3C,0x40,0x40,0x7C,0x00,0x00,0x00},
  {0x00,0x1C,0x20,0x40,0x20,0x1C,0x00,0x00},
  {0x00,0x3C,0x40,0x30,0x40,0x3C,0x00,0x00},
  {0x00,0x44,0x28,0x10,0x28,0x44,0x00,0x00},
  {0x00,0x1C,0xA0,0xA0,0x7C,0x00,0x00,0x00},
  {0x00,0x44,0x64,0x54,0x4C,0x44,0x00,0x00},
  {0x00,0x08,0x36,0x41,0x00,0x00,0x00,0x00},
  {0x00,0x00,0x7F,0x00,0x00,0x00,0x00,0x00},
  {0x00,0x41,0x36,0x08,0x00,0x00,0x00,0x00},
  {0x00,0x02,0x01,0x01,0x02,0x01,0x00,0x00},
  {0x00,0x02,0x05,0x05,0x02,0x00,0x00,0x00},
};
// Define OLED dimensions
#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define slaveaddress 0x3C
// Define command macros
#define OLED_SETCONTRAST 0x81
#define OLED_DISPLAYALLON_RESUME 0xA4
#define OLED_DISPLAYALLON 0xA5
#define OLED_NORMALDISPLAY 0xA6
#define OLED_INVERTDISPLAY 0xA7
#define OLED_DISPLAYOFF 0xAE
#define OLED_DISPLAYON 0xAF
#define OLED_SETDISPLAYOFFSET 0xD3
#define OLED_SETCOMPINS 0xDA
#define OLED_SETVCOMDETECT 0xDB
#define OLED_SETDISPLAYCLOCKDIV 0xD5
#define OLED_SETPRECHARGE 0xD9
#define OLED_SETMULTIPLEX 0xA8
#define OLED_SETLOWCOLUMN 0x00
#define OLED_SETHIGHCOLUMN 0x10
#define OLED_SETSTARTLINE 0x40
#define OLED_MEMORYMODE 0x20
#define OLED_COLUMNADDR 0x21
#define OLED_PAGEADDR   0x22
#define OLED_COMSCANINC 0xC0
#define OLED_COMSCANDEC 0xC8
#define OLED_SEGREMAP 0xA0
#define OLED_CHARGEPUMP 0x8D


// Function declarations
void OLED_Command( uint8_t temp);
void OLED_Data( uint8_t  temp);
void OLED_Init();
void OLED_YX(unsigned char Row, unsigned char Column); // warning! max 4 rows
void OLED_PutChar( char ch );
void OLED_Clear();
void OLED_Write_String( char *s );
void OLED_Write_Integer(uint8_t  i);
void OLED_Write_Float(float f);
void Delay_ms(int time_ms);

// Function prototypes initialize, tranmit and rea functions 
void I2C3_Init ( void );  
static int I2C_wait_till_done(void);
char I2C3_Write_Multiple(int slave_address, char slave_memory_address, int bytes_count, uint8_t* data);
char I2C3_Wr(int slaveAddr, char memAddr, uint8_t data);


int main(void)
{
    // initGPIO();
	I2C3_Init();
	OLED_Init();
	OLED_Clear();
    HC05_init(); // call HC05_init() to initialze UART5 of TM4C123GH6PM
	
	
	SYSCTL->RCGCGPIO |= 0x20;   /* enable clock to GPIOF */
    GPIOF->DIR |= 0x0E;         //set PF1, PF2 and PF3 as digital output pin
    GPIOF->DEN |= 0x0E;         // CON PF1, PF2 and PF3 as digital GPIO pins
    
	SYSCTL->RCGCGPIO |= (1 << 4);   /* Enable Clock to GPIOE */
	
    /* initialize PE3 for DHT11 data pin */
    GPIOE->AFSEL |= (1 << 3);       /* enable alternate function */
    GPIOE->DEN |= (1 << 3);         /* enable digital function */
    GPIOE->DIR |= (1 << 3);         /* set as output initially */
    GPIOE->DATA |= (1 << 3);        /* pull high initially */
 
	
		
	
	
    int humidity,temperature_dht;
	float temperature;
	unsigned int adc_value;
    float V_sensor;
		
	  
	
    /* Enable Clock to ADC0 and GPIO pins*/
    
    SYSCTL->RCGCADC |= (1<<0);    /* AD0 clock enable*/
    
    /* initialize PE2 for AIN0 input  */
    GPIOE->AFSEL |= (1<<2);       /* enable alternate function */
    GPIOE->DEN &= ~(1<<2);        /* disable digital function */
    GPIOE->AMSEL |= (1<<2);       /* enable analog function */
	  
   
    /* initialize sample sequencer3 */
    ADC0->ACTSS &= ~(1<<3);   /* disable SS3 during configuration */
    ADC0->EMUX &= ~0xF000;    /* software trigger conversion */
    ADC0->SSMUX3 = 0;         /* get input from channel 0 */
    ADC0->SSCTL3 |= (1<<1)|(1<<2); /* take one sample at a time, set flag at 1st sample */
    ADC0->ACTSS |= (1<<3);         /* enable ADC0 sequencer 3 */
    while ( 1 ) {
            readDHT11(&humidity, &temperature_dht);
            ADC0->PSSI |= (1<<3);        /* Enable SS3 conversion or start sampling data from AN0 */
			while((ADC0->RIS & 8) == 0) ;/* Wait untill sample conversion completed*/
			adc_value = ADC0->SSFIFO3;
			// Transform the raw adc reading to analog voltage
			V_sensor = ((adc_value * 3.3 )/ 4095)*1000 ;
			// Converting analog voltage to temperature reading in degree celscius
			temperature = (V_sensor - 1200) / 16;
			// Take average of both the temperature readings from DHT11 and LM35
			temperature = (temperature + temperature_dht)/2;
			// Display the values in OLED through I2C
			OLED_YX(0, 0 );
            OLED_Write_String("Temp:" );
            OLED_YX(0, 5);
            OLED_Write_Integer((int)(temperature));
								
            OLED_YX(1, 0 );
            OLED_Write_String( "Hum:" );
            OLED_YX(1, 4);
            OLED_Write_Integer((int)(humidity));				
			
            Delay_ms(100);
			//Clear the SS3 to take next sample fro processing
			ADC0->ISC = 8;
            
        
	    // Bluetooth displaying 
        if((UART5->FR & (1<<4))  == 0){ /* if Rx FIFO not empty */
            char c = Bluetooth_Read();
					// Bluetooth connection testing with red led on and off
            if( c=='A'){
                GPIOF->DATA |=(1<<1);
                Bluetooth_Write_String("RED LED ON\n");
            }
            else if( c=='B'){
                GPIOF->DATA &=~(1<<1);
                Bluetooth_Write_String("RED LED OFF\n");
            }
						//Actual function
            else if( c=='T'){
                
                Bluetooth_Write_Temperature(temperature);
							Bluetooth_Write_String("\n");
                
            }
            else if( c=='H'){
                
                Bluetooth_Write_Humidity(humidity);
							Bluetooth_Write_String("\n");
                
                
            }
        }
                
          
   }
}
// I2C intialization and GPIO alternate function configuration
void I2C3_Init ( void )
{
	SYSCTL->RCGCGPIO  |= 0x00000008 ; // Enable the clock for port D
	SYSCTL->RCGCI2C   |= 0x00000008 ; // Enable the clock for I2C 3
	GPIOD->DEN |= 0x03; // Assert DEN for port D
	// Configure Port D pins 0 and 1 as I2C 3
	GPIOD->AFSEL |= 0x00000003 ;
	GPIOD->PCTL |= 0x00000033 ;
	GPIOD->ODR |= 0x00000002 ; // SDA (PD1 ) pin as open darin
	I2C3->MCR  = 0x0010 ; // Enable I2C 3 master function
	/* Configure I2C 3 clock frequency
	(1 + TIME_PERIOD ) = SYS_CLK /(2*
	( SCL_LP + SCL_HP ) * I2C_CLK_Freq )
	TIME_PERIOD = 16 ,000 ,000/(2(6+4) *100000) - 1 = 7 */
	I2C3->MTPR  = 0x07 ;
}

/* wait untill I2C Master module is busy */
/*  and if not busy and no error return 0 */
static int I2C_wait_till_done(void)
{
    while(I2C3->MCS & 1);   /* wait until I2C master is not busy */
    return I2C3->MCS & 0xE; /* return I2C error code, 0 if no error*/
}


char I2C3_Wr(int slaveAddr, char memAddr, uint8_t data)
{

    char error;

    /* send slave address and starting address */
    I2C3->MSA = slaveAddr << 1;
    I2C3->MDR = memAddr;
    I2C3->MCS = 3;                      /* S-(saddr+w)-ACK-maddr-ACK */

    error = I2C_wait_till_done();       /* wait until write is complete */
    if (error) return error;

    /* send data */
    I2C3->MDR = data;
    I2C3->MCS = 5;                      /* -data-ACK-P */
    error = I2C_wait_till_done();       /* wait until write is complete */
    while(I2C3->MCS & 0x40);            /* wait until bus is not busy */
    error = I2C3->MCS & 0xE;
    if (error) return error;

    return 0;       /* no error */
}

// Receive one byte of data from I2C slave device
char I2C3_Write_Multiple(int slave_address, char slave_memory_address, int bytes_count, uint8_t* data)
{   
    char error;
    if (bytes_count <= 0)
        return -1;                  /* no write was performed */
    /* send slave address and starting address */
    I2C3->MSA = slave_address << 1;
    I2C3->MDR = slave_memory_address;
    I2C3->MCS = 3;                  /* S-(saddr+w)-ACK-maddr-ACK */

    error = I2C_wait_till_done();   /* wait until write is complete */
    if (error) return error;

    /* send data one byte at a time */
    while (bytes_count > 1)
    {
        I2C3->MDR = data++;             / write the next byte */
        I2C3->MCS = 1;                   /* -data-ACK- */
        error = I2C_wait_till_done();
        if (error) return error;
        bytes_count--;
    }
    
    /* send last byte and a STOP */
    I2C3->MDR = data++;                 / write the last byte */
    I2C3->MCS = 5;                       /* -data-ACK-P */
    error = I2C_wait_till_done();
    while(I2C3->MCS & 0x40);             /* wait until bus is not busy */
    if (error) return error;
    return 0;       /* no error */
}

//OLED
void OLED_Command( uint8_t temp){
    
	  I2C3_Wr(0x3C,0x00,temp);
}

/***************************
 * Function: void OLED_Data ( uint8_t temp)
 *
 * Returns: Nothing
 *
 * Description: sends data to the OLED
 * 
 **************************/

void OLED_Data( uint8_t temp){
 
	  I2C3_Wr(0x3C,0x40,temp);
	  
}

/***************************
 * Function: void OLED_Init ()
 *
 * Returns: Nothing
 *
 * Description: Initializes OLED
 * 
 **************************/

void OLED_Init() {
    
    OLED_Command(OLED_DISPLAYOFF);         // 0xAE
    OLED_Command(OLED_SETDISPLAYCLOCKDIV); // 0xD5
    OLED_Command(0x80);                    // the suggested ratio 0x80
    OLED_Command(OLED_SETMULTIPLEX);       // 0xA8
    OLED_Command(0x1F);
    OLED_Command(OLED_SETDISPLAYOFFSET);   // 0xD3
    OLED_Command(0x0);                        // no offset
    OLED_Command(OLED_SETSTARTLINE | 0x0); // line #0
    OLED_Command(OLED_CHARGEPUMP);         // 0x8D
    OLED_Command(0xAF);
    OLED_Command(OLED_MEMORYMODE);         // 0x20
    OLED_Command(0x00);                    // 0x0 act like ks0108
    OLED_Command(OLED_SEGREMAP | 0x1);
    OLED_Command(OLED_COMSCANDEC);
    OLED_Command(OLED_SETCOMPINS);         // 0xDA
    OLED_Command(0x02);
    OLED_Command(OLED_SETCONTRAST);        // 0x81
    OLED_Command(0x8F);
    OLED_Command(OLED_SETPRECHARGE);       // 0xd9
    OLED_Command(0xF1);
    OLED_Command(OLED_SETVCOMDETECT);      // 0xDB
    OLED_Command(0x40);
    OLED_Command(OLED_DISPLAYALLON_RESUME);// 0xA4
    OLED_Command(OLED_NORMALDISPLAY);      // 0xA6
    OLED_Command(OLED_DISPLAYON);          //--turn on oled panel

}

/***************************
 * Function: void OLED_YX(unsigned char Row, unsigned char Column)
 *
 * Returns: Nothing
 *
 * Description: Sets the X and Y coordinates
 * 
 **************************/

void OLED_YX(unsigned char Row, unsigned char Column)
{
    OLED_Command( 0xB0 + Row);
    OLED_Command( 0x00 + (8*Column & 0x0F) );
    OLED_Command( 0x10 + ((8*Column>>4)&0x0F) );
}

/***************************
 * Function: void OLED_PutChar(char ch)
 *
 * Returns: Nothing
 *
 * Description: Writes a character to the OLED
 * 
 **************************/

void OLED_PutChar(char ch )
{
    if ( ( ch < 32 ) || ( ch > 127 ) ){
        ch = ' ';
    }

   const uint8_t *base = &OledFont[ch - 32][0];

    uint8_t bytes[9];
    //bytes[0] = 0x40;
    memmove( bytes + 1, base, 8 );
       
		I2C3_Write_Multiple(0x3C,0x40,9,bytes);
    
}


/***************************
 * Function: void OLED_Clear()
 *
 * Returns: Nothing
 *
 * Description: Clears the OLED
 * 
 **************************/

void OLED_Clear()
{
    for ( uint16_t row = 0; row < 8; row++ ) {
        for ( uint16_t col = 0; col < 16; col++ ) {
            OLED_YX( row, col );
            OLED_PutChar(' ');
        }
    }
}


/***************************
 * Function:  void OLED_Write_String( char *s )
 * 
 * Returns: Nothing
 *
 * Description: Writes a string to the OLED
 * 
 **************************/

void OLED_Write_String( char *s )
{
    while (*s) OLED_PutChar( *s++);
}


/***************************
 * Function:  void OLED_Write_Integer ( uint8_t i )
 * 
 * Returns: Nothing
 *
 * Description: Writes an integer to the OLED
 * 
 **************************/

void OLED_Write_Integer(uint8_t  i)
{
     char s[20];
     sprintf( s, "%d", i );
     OLED_Write_String( s );
     OLED_Write_String( "     " );
}

/***************************
 * Function:  void OLED_Write_Float( float f )
 * 
 * Returns: Nothing
 *
 * Description: Writes a float to the OLED
 * 
 **************************/

void OLED_Write_Float(float f)
{
    char* buf11;
    int status;
    sprintf( buf11, "%f", f );
    OLED_Write_String(buf11);
    OLED_Write_String( "     " );
}
void Delay_ms(int time_ms)
{
    int i, j;
    for(i = 0 ; i < time_ms; i++)
        for(j = 0; j < 3180; j++)
            {}  /* excute NOP for 1ms */
}

void HC05_init(void)
{
	  SYSCTL->RCGCUART |= 0x20;  /* enable clock to UART5 */
    SYSCTL->RCGCGPIO |= 0x10;  /* enable clock to PORTE for PE4/Rx and RE5/Tx */
    Delay(1);
    /* UART0 initialization */
    UART5->CTL = 0;         /* UART5 module disbable */
    UART5->IBRD = 104;      /* for 9600 baud rate, integer = 104 */
    UART5->FBRD = 11;       /* for 9600 baud rate, fractional = 11*/
    UART5->CC = 0;          /select system clock/
    UART5->LCRH = 0x60;     /* data lenght 8-bit, not parity bit, no FIFO */
    UART5->CTL = 0x301;     /* Enable UART5 module, Rx and Tx */

    /* UART5 TX5 and RX5 use PE4 and PE5. Configure them digital and enable alternate function */
    GPIOE->DEN = 0x30;      /* set PE4 and PE5 as digital */
    GPIOE->AFSEL = 0x30;    /* Use PE4,PE5 alternate function */
    GPIOE->AMSEL = 0;    /* Turn off analg function*/
    GPIOE->PCTL = 0x00110000;     /* configure PE4 and PE5 for UART */
}
char Bluetooth_Read(void)  
{
    char data;
	  data = UART5->DR ;  	/* before giving it another byte */
    return (unsigned char) data; 
}

void Bluetooth_Write(unsigned char data)  
{
    while((UART5->FR & (1<<5)) != 0); /* wait until Tx buffer not full */
    UART5->DR = data;                  /* before giving it another byte */
}

void Bluetooth_Write_String(char *str)
{
  while(*str)
	{
		Bluetooth_Write(*(str++));
	}
}
void Bluetooth_Write_Temperature(uint8_t temperature)
{
    
    char temperature_str[4];

    // Convert numeric values to strings
    
    sprintf(temperature_str, "%d", temperature);

    // Send humidity and temperature values as strings
    
    Bluetooth_Write_String("Temperature: ");
    Bluetooth_Write_String(temperature_str);
    Bluetooth_Write_String("C");
}
void Bluetooth_Write_Humidity(uint8_t humidity)
{
    char humidity_str[4];
    

    // Convert numeric values to strings
    sprintf(humidity_str, "%d", humidity);
    

    // Send humidity and temperature values as strings
    Bluetooth_Write_String("Humidity: ");
    Bluetooth_Write_String(humidity_str);
    Bluetooth_Write_String("%");
}   


void Delay(unsigned long counter)
{
	unsigned long i = 0;
	
	for(i=0; i< counter; i++);
}



void readDHT11(uint8_t* humidity, uint8_t* temperature) {
    // Initialize variables to store the data
    uint8_t data[5] = {0};

    // Request data from the DHT11 sensor
    GPIOE->DIR |= (1 << 3);    // Set PE3 as output
    GPIOE->DATA &= ~(1 << 3);   // Pull the line low
    Delay_ms(18000);  // Delay for at least 18ms (DHT11 datasheet requirement)
    GPIOE->DATA |= (1 << 3);    // Release the line
    Delay_ms(40);     // Wait for the DHT11 response

    // Change pin to input for receiving data
    GPIOE->DIR &= ~(1 << 3);

    // Wait for the sensor to respond
    while ((GPIOE->DATA & (1 << 3)) != 0);

    // Wait for the sensor to finish the first response (80us low)
    while ((GPIOE->DATA & (1 << 3)) == 0);
    Delay_ms(80);

    // Read data from the sensor
    for (int i = 0; i < 5; ++i) {
        // Read 8 bits for each data byte
        for (int j = 7; j >= 0; --j) {
            // Wait for a falling edge (start of bit)
            while ((GPIOE->DATA & (1 << 3)) == 0);
            // Wait for a rising edge (end of bit)
            while ((GPIOE->DATA & (1 << 3)) != 0);
            // Record the time to determine if it's a 0 or 1
            Delay_ms(40);
            if (GPIOE->DATA & (1 << 3)) {
                // High pulse is longer, consider it a 1
                data[i] |= (1 << j);
            }
        }
    }

    // Check the checksum
    if (data[0] + data[1] + data[2] + data[3] == data[4]) {
        // Data is valid
        *humidity = data[0];
        *temperature = data[2];
    } else {
        // Data is invalid
        *humidity = 0;
        *temperature = 0;
    }
}