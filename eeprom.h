
#ifndef _EEPROM_H_
#define _EEPROM_H_ 1

#define EEPROM_SIZE 		32768
#define EEPROM_PAGE_SIZE	64
#define EEPROM_PAGE_COUNT	EEPROM_SIZE/EEPROM_PAGE_SIZE
#define EEPROM_WRITE_TIME	5

#define SLAVESELECT PB4	//ss
#define DATAOUT PB5		//MOSI
#define DATAIN  PB6		//MISO 
#define SPICLOCK PB7	//sck

// opcodes
#define READ  3
#define WRITE 2
#define WREN  6
#define WRDI  4
#define RDSR  5
#define WRSR  1
//#define PAGE_ERASE 	  0x42 // Doesn't exist in 25LC256
//#define SECTOR_ERASE  0xDA
//#define CHIP_ERASE    0xC7

#define WIP   0


// define types
#define null 0
typedef enum {false, true} bool;
typedef uint8_t byte;
typedef uint16_t word;

volatile word eeprom_start;
volatile word eeprom_avail;
volatile word eeprom_end;

word get_eeprom_start_pointer();
word get_eeprom_avail_pointer();
void update_eeprom_pointer(word);

void eeprom_init();
byte read_eeprom(word);
bool read_page_eeprom(byte*, word, word);
bool write_eeprom(word, byte);
bool write_page_eeprom(word, byte*, word);
void write_enable_eeprom();
bool eeprom_copy(byte*, word, word);
bool eeprom_page_erase(word);
void eeprom_sector_erase(word);
void eeprom_chip_erase();

void spi_init();
byte spi_transfer(byte data);
void word_transfer(word address);
int write_in_process_status();

#endif //_EEPROM_H_
