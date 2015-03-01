
/* ------------------------------------------------------------------------
**	 Module:	eeprom.c: 	
**	 Author:    Mike Hankey
**	 Hardware 	AVR ATmega328
**	 Software:	gcc 4.3.3 AVR Studio 4.18 Build 700
**
**	 DESCRIPTION: Handles eeprom communications
**						  
**    Version: 1.0
**
**    Copyright © 2010, Mike Hankey
**    All rights reserved. [BSD License]
**    http://www.JaxCoder.com/
** 
** Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are met:
** 1. Redistributions of source code must retain the above copyright
**    notice, this list of conditions and the following disclaimer.
** 2. Redistributions in binary form must reproduce the above copyright
**    notice, this list of conditions and the following disclaimer in the
**    documentation and/or other materials provided with the distribution.
** 3. All advertising materials mentioning features or use of this software
**    must display the following acknowledgement:
**    This product includes software developed by the <organization>.
** 4. Neither the name of the <organization> nor the
**    names of its contributors may be used to endorse or promote products
**    derived from this software without specific prior written permission.
**
** THIS SOFTWARE IS PROVIDED BY <COPYRIGHT HOLDER> ''AS IS'' AND ANY
** EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
** WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
** DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
** DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
** (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
** LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
** ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
** SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
** 
** ------------------------------------------------------------------------*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "my_curs.h"
#include "eeprom.h"

/* -- GetEEPROMStartPointer ----------------------------------------------
**
**	Description: Updates the avail pointer
**
**	Params:	None
**	Returns: word - start location in eeprom
** -----------------------------------------------------------------------*/
word get_eeprom_start_pointer()
{
	return eeprom_start;
}

/* -- GetEEPROMAvailPointer ----------------------------------------------
**
**	Description: Updates the avail pointer
**
**	Params:	None
**	Returns: word - Next available location in eeprom
** -----------------------------------------------------------------------*/
word get_eeprom_avail_pointer()
{
	return eeprom_avail;
}

/* -- UpdateEEPROMPointer ----------------------------------------------
**
**	Description: Updates the avail pointer
**
**	Params:	word - Offset
**	Returns: None
** -----------------------------------------------------------------------*/
void update_eeprom_pointer(word offset)
{
	eeprom_avail += offset;
}

/* -- eeprom_init --------------------------------------------------------
**
**	Description: Initialize/Reset eeprom and all pointers
**
**	Params:	None
**	Returns: None
** -----------------------------------------------------------------------*/
void eeprom_init()
{
	eeprom_start = 0;
	eeprom_avail = 0;
	eeprom_end = eeprom_start + EEPROM_SIZE;
}

/* -- spi_init -----------------------------------------------------------
**
**	Description: Initialize SPI interface
**
**	Params:	None
**	Returns: None
** -----------------------------------------------------------------------*/
void spi_init()
{
	DDRB |= _BV(SLAVESELECT) | _BV(SPICLOCK) | _BV(DATAOUT);
	PORTB |= _BV(SLAVESELECT);
  
	 // SPCR = 01010000
	 //interrupt disabled,spi enabled,msb 1st,master,clk low when idle,
	 //sample on leading edge of clk,system clock/4 rate (fastest)
	 SPCR = _BV(SPE) | _BV(MSTR);
}

/* -- read_eeprom -----------------------------------------------------------
**
**	Description: Read a byte from eeprom
**
**	Params:	word - address to read from
**	Returns: byte - data
** -----------------------------------------------------------------------*/
byte read_eeprom(word address)
{
	int data;

	if (address >= EEPROM_SIZE)
		return null;

	PORTB &= ~_BV(SLAVESELECT);

	spi_transfer(READ); //transmit read opcode
	word_transfer(address); // send MSByte and then LSByte
	data = spi_transfer(0xFF); //get data byte

	PORTB |= _BV(SLAVESELECT);
	PORTB &= ~_BV(DATAOUT);

	return data;
}

/* -- read_page_eeprom --------------------------------------------------
**
**	Description: Read data from eeprom
**
**	Params:	byte* - pointer to data buffer
**			word - address to read from
**			word - length of data to read
**	Returns: bool - true on success false otherwise
** -----------------------------------------------------------------------*/
bool read_page_eeprom(byte* pdata, word address, word len)
{
	if (address >= EEPROM_SIZE || len > EEPROM_PAGE_SIZE)
		return false;
	
	PORTB &= ~_BV(SLAVESELECT);

	spi_transfer(READ); //transmit read opcode
	word_transfer(address); // send MSByte and then LSByte

	for (int i = 0; i < len; i++)
		*pdata++ = spi_transfer(0xFF);


	PORTB |= _BV(SLAVESELECT);
	PORTB &= ~_BV(DATAOUT);

	return true;
}

/* -- write_epprom -------------------------------------------------------
**
**	Description: Write data to EEPROM
**
**	Params:	byte* - pointer to data buffer
**			word - address to write data
**			word - length of data
**	Returns: bool - true on success false otherwise
** -----------------------------------------------------------------------*/
bool write_eeprom(word address, byte data)
{
	if (address >= EEPROM_SIZE)
		return false;

	write_enable_eeprom();

	PORTB &= ~_BV(SLAVESELECT);

	spi_transfer(WRITE); //write instruction
	word_transfer(address); // send MSByte and then LSByte
    spi_transfer(data); //write data byte

	PORTB |= _BV(SLAVESELECT);
	PORTB &= ~_BV(DATAOUT);

	return true;
}

/* -- write_page_epprom --------------------------------------------------
**
**	Description: Write data to EEPROM
**
**	Params:	byte* - pointer to data buffer
**			word - address to write data
**			word - length of data
**	Returns: bool - true on success false otherwise
** -----------------------------------------------------------------------*/
bool write_page_eeprom(word address, byte* data, word len)
{
	if (address >= EEPROM_SIZE || len > EEPROM_PAGE_SIZE)
		return false;

	write_enable_eeprom();

	PORTB &= ~_BV(SLAVESELECT);
	spi_transfer(WRITE); //write instruction
	word_transfer(address); // send MSByte and then LSByte
  	for (int i = 0; i < len; i++){
	    spi_transfer(*(data)); //write data byte
	    data++;
	}
	PORTB |= _BV(SLAVESELECT);
	PORTB &= ~_BV(DATAOUT);

	return true;
}

/* -- write_enable_eeprom ------------------------------------------------
**
**	Description: Enable writting to the EEPROM
**
**	Params:	None
**	Returns: None
** -----------------------------------------------------------------------*/
void write_enable_eeprom()
{
	PORTB &= ~_BV(SLAVESELECT);

	spi_transfer(WREN); //write enable
	PORTB |= _BV(SLAVESELECT);
}

/* -- eeprom_copy ---------------------------------------------------------
**
**	Description: Copies len of eeprom to dest. memory
**
**	Params:	byte* - Destination address
**			word - eeprom source address
**			word - length of chunk to copy.
**	Returns: None
** -----------------------------------------------------------------------*/
bool eeprom_copy(byte* dest, word src, word len)
{
	if (src + len > EEPROM_SIZE || len > EEPROM_PAGE_SIZE)
		return false;

	read_page_eeprom(dest, src, len);

	return true;
}

/* -- spi_transfer -----------------------------------------------------------
**
**	Description: Transfers one byte of data via SPI
**
**	Params:	uint8_t	byte to send
**	Returns: None
** -----------------------------------------------------------------------*/
byte spi_transfer(byte data)
{
  SPDR = data;
  while (!(SPSR & (1<<SPIF)));	// Wait the end of the transmission

  return SPDR;                	// return the received byte
}

bool eeprom_page_erase(word address)
{
	if (address >= EEPROM_SIZE)
		return false;

	write_enable_eeprom();

	PORTB &= ~_BV(SLAVESELECT);
	spi_transfer(WRITE); //write instruction
	word_transfer(address); // send MSByte and then LSByte
  	for (int i = 0; i < EEPROM_PAGE_SIZE; i++){
	    spi_transfer(0x66); //write data byte
	}
	PORTB |= _BV(SLAVESELECT);
	PORTB &= ~_BV(DATAOUT);

	return true;
}

void eeprom_chip_erase()
{
	for (int j = 0; j < EEPROM_SIZE; j += EEPROM_PAGE_SIZE){
		//while(write_in_process_status()){};
		write_enable_eeprom();

		PORTB &= ~_BV(SLAVESELECT);
		spi_transfer(WRITE); //write instruction
		word_transfer(j); // send MSByte and then LSByte
	  	for (int i = 0; i < EEPROM_PAGE_SIZE; i++){
		    spi_transfer(0x66); //write data byte
		}
		PORTB |= _BV(SLAVESELECT);
		PORTB &= ~_BV(DATAOUT);
		_delay_ms(EEPROM_WRITE_TIME);
	}
}

int write_in_process_status(){
	// send read status comand, get status and send WIP bit
	PORTB &= ~_BV(SLAVESELECT);
	spi_transfer(RDSR);
	return (spi_transfer(0xFF) & 2);
	PORTB |= _BV(SLAVESELECT);
}

void word_transfer(word address)
{
	byte low_byte;
	byte high_byte;
	unsigned char *the_ptr;

	the_ptr = ((unsigned char *) &address);
	low_byte = *the_ptr;
	the_ptr++;
	high_byte = *((unsigned char *)the_ptr);
	spi_transfer(high_byte);
	spi_transfer(low_byte);
}

// void eeprom_sector_erase(word address)
// {
// 	write_enable_eeprom();

// 	PORTB &= ~_BV(SLAVESELECT);
// 	spi_transfer(SECTOR_ERASE); //write instruction

// 	PORTB |= _BV(SLAVESELECT);
// 	PORTB &= ~(_BV(SLAVESELECT) | _BV(DATAOUT) | _BV(SPICLOCK));
// }