#ifndef _MY_CURS_H_

#define F_CPU 4000000UL
#define BAUDRATE 9600UL
#define BAUD_PRESCALLER (((F_CPU / (BAUDRATE * 16UL))) - 1)
#define TIMER_START 255-40+1
#define AREF_VOLTAGE 2
#define ANALOG_FACTOR 0.00489 // (AREF_VOLTAGE / 1023)

#define SETBIT(ADDRESS,BIT) (ADDRESS |= (1 << BIT)) 
#define CLEARBIT(ADDRESS,BIT) (ADDRESS &= ~(1 << BIT)) 
#define FLIPBIT(ADDRESS,BIT) (ADDRESS ^= (1 << BIT)) 
#define CHECKBIT(ADDRESS,BIT) (ADDRESS & (1 << BIT)) 
#define WRITEBIT(RADDRESS,RBIT,WADDRESS,WBIT) (CHECKBIT(RADDRESS,RBIT) ? SETBIT(WADDRESS,WBIT) : CLEARBIT(WADDRESS,WBIT))

#define LED0 0
#define LED1 1
#define LED2 2
#define LED3 3
#define LED4 4
#define LED5 5
#define LED6 6

#define STRING_LENGTH 27
#define LCD_MAX 15

/* Prototypes */
void UART_init( unsigned int baudrate );
unsigned char UART_receive( void );
void UART_transmit( unsigned char data );
void UART_transmit_string(char *str);
void timer0_init();
void timer1_init();
void adc_init();
void get_adc_vals(uint8_t*);
void uart_transmit_adc64(uint8_t [][8]);

void lcd_preparation_for_adc();
void setup();
void loop();
void greeting();
//void spi_init_master();

#endif //_MY_CURS_H_