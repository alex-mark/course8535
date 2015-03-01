#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdio.h>
#include <stddef.h>

#include "my_curs.h"
#include "lcd.h"
#include "eeprom.h"

uint16_t eeprom_pointer; //current pointer in eeprom
uint8_t eeprom_write_counter; //counting till 8 for reading a page

int main(void)
{	
	setup();	// Start initialization
	loop();		// Loop forever

	return 0;
}

void setup()
{
	
	cli();	// clear enable interrupts
	
	UART_init(BAUD_PRESCALLER); /* Set the baudrate */
	lcd_init(LCD_DISP_ON); 		  /* initialize lcd, display on, cursor off */

	greeting();		// send greeting message

	DDRC = 0xFF;	// all pins of PortC as output

	DDRA = 0x00;	// Configure PortA as input
					// PA0 is ADC0 input
						
	//timer0_init();	// initialization of Timer0
	
	adc_init();		// initialization of ADC
	
	eeprom_init();	//intialization of external eeprom
	spi_init();

	eeprom_pointer = get_eeprom_start_pointer();
	eeprom_write_counter = 0; // set number of writings to 0

	// read_page_eeprom(*page_adc_vals, 0, EEPROM_PAGE_SIZE);
	// uart_transmit_adc64(page_adc_vals);

	// read_page_eeprom(*page_adc_vals, 64, EEPROM_PAGE_SIZE);
	// uart_transmit_adc64(page_adc_vals);

	lcd_preparation_for_adc();
	timer1_init();  // initialization of Timer1
	//get_adc_vals(port_adc_vals);

    sei();	// set enable interrupts
}

void loop()
{	
	while(1){} // Loop forever
    	
}

void greeting()
{
	UART_transmit_string("                Hello!\r\n");
	UART_transmit_string("------------------------------------------\n\r");

	lcd_clrscr();
	lcd_puts("Hello!");
	_delay_ms(1000);
}

/* Initialize UART */
void UART_init( unsigned int baud_prescaler )
{
	/* Set the baud rate */
	UBRR = (unsigned char) baud_prescaler;
	
	/* Enable UART receiver and transmitter */
	UCR = (( 1 << RXEN ) | ( 1 << TXEN ) | ( 1 << TXB8 ) | (1 << RXB8)); 
}

/* Initialize Timer0 */
void timer0_init()
{
	//	set up timer with no prescaling
	TCCR0 |= (1 << CS00);
	
	//	set timer0 counter initial value to TIMER_START
	TCNT0 = TIMER_START;

	//	enable interrupt for timer0
	//TIMSK |= (1<<TOIE0);
}

void timer1_init()
{
	// set up timer with prescaler = 8 and CTC mode
    TCCR1B |= (1 << CTC1) | (1 << CS11);
  
    // initialize counter
    TCNT1 = 0;
  
    // initialize compare value
    OCR1A = 10000; // 6000 working fine
  
    // enable compare interrupt
    TIMSK |= (1 << OCIE1A);
}

void adc_init()		//initialization of ADC
{
	ADCSR |= (1 << ADEN);  // Enable ADC

	ADCSR |= (1 << ADPS2) | (1 << ADPS0); // Set ADC prescaler to 32 - 125KHz sample rate @ 4MHz

	ADMUX = 0; //Binary equivalent of										  
	// No MUX values needed to be changed to use ADC0

	//ADCSR |= (1 << ADFR);  // Set ADC to Free-Running Mode
	
	//ADCSR |= (1 << ADIE);  // Enable ADC Interrupt

	//ADCSR |= (1 << ADSC);  // Start A2D Conversions
}

// Getting 2 columns 4 lines with number of port
void lcd_preparation_for_adc()
{
	int i;
	char str[4] = "";
	
	lcd_clrscr();

	for (i = 0; i < 8; i++){
		//itoa(i, str, 10);	// Here 10 means binary
		//strcat(str, ": ");
		sprintf(str,"%d:", i); // Getting string with number of port
		
		if (i < 4){
			lcd_gotoxy(0,i);
		}else{
			lcd_gotoxy(8,i-4);
		}
		lcd_puts(str);
	}	
}

ISR (TIMER1_COMPA_vect)
{
	uint8_t page_adc_vals[8][8];
	uint8_t port_adc_vals[8];
	
	char buffer[50];

	// if true it's time to read from memory and send via uart
	if (eeprom_write_counter >= 8){
		if (eeprom_pointer >= 64){ // check for end of a memory
			read_page_eeprom(*page_adc_vals, eeprom_pointer - EEPROM_PAGE_SIZE, EEPROM_PAGE_SIZE);
		}else{
			read_page_eeprom(*page_adc_vals, EEPROM_SIZE - EEPROM_PAGE_SIZE, EEPROM_PAGE_SIZE);
		}
		
	}

	get_adc_vals(port_adc_vals);
	write_page_eeprom(eeprom_pointer, port_adc_vals, 8);

	if (eeprom_write_counter >= 8){ // continue to send via uart
		eeprom_write_counter = 0;
		sprintf(buffer, "eeprom_pointer %X\r\n", eeprom_pointer - EEPROM_PAGE_SIZE);
		UART_transmit_string(buffer);
		uart_transmit_adc64(page_adc_vals);
	}
	
	eeprom_write_counter++;
	if (eeprom_pointer < EEPROM_SIZE){ // check pointer for end of a memory
		eeprom_pointer += 8;
	}else{
		eeprom_pointer = get_eeprom_start_pointer();
	}
}

void get_adc_vals(uint8_t *port_adc_vals)
{
	char buffer[10];
	uint8_t adc_value;

	for (int i = 0; i < 8; i++){
		ADMUX = i;
		ADCSR |= (1 << ADSC);
		while(ADCSR & (1<<ADSC));
		adc_value = ADCW * 250 / 127; //equals mult 250 div 128 to get 250 mV
		port_adc_vals[i] = adc_value;

		//sprintf(buffer, "%d: %d\r\n", i, adc_value);
		//UART_transmit_string(buffer);

		sprintf(buffer, "%3d", adc_value);
		if (i < 4){
			lcd_gotoxy(2, i); // start writing after number of port "#:"
		}else{
			lcd_gotoxy(10, i - 4);
		}
		lcd_puts(buffer);
	}
}

// transmit 64 bytes (8x8) via uart
void uart_transmit_adc64(uint8_t page_adc_vals[][8])
{
	char buffer[50];
	for (int i = 0; i < 8; i++){
		for (int j = 0; j < 8; j++){
			sprintf(buffer, "%i: %3d mV  ", j, page_adc_vals[i][j]);
			UART_transmit_string(buffer);
			if (j == 3){
				UART_transmit_string("\r\n");
			}
		}
		if (i < 7){
			UART_transmit_string("\r\n------------------------------------------\r\n");
		}else{
			UART_transmit_string("\r\n******************************************\r\n\r\n");
		}
	}
}

/* Read and write functions */
unsigned char UART_receive( void )
{
	/* Wait for incomming data */
	while ( !(USR & (1<<RXC)) ) 	
		;			                
	/* Return the data */
	return UDR;
}

void UART_transmit(unsigned char data)
{
	/* Wait for empty transmit buffer */
	while ( !(USR & (1<<UDRE)) );

	/* Start transmittion */
	UDR = data; 		
}

void UART_transmit_string(char *str)
{
	int i;
	for (i = 0; i < strlen(str); i++){
		UART_transmit(str[i]);
	}
	//_delay_ms(500);
}