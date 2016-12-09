// lab4_code.c 
// Jonathan Hardman
// 11.15.16

/******************LIBRARIES/DEFINES******************/
#include <stdlib.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <util/twi.h>
#include "hd44780.h"
#include "twi_master.h"
#include "uart_functions.h"
#include "si4734.h"

#define F_CPU 16000000 // cpu speed in hertz 
#define TRUE 1
#define FALSE 0
#define VOLUME 0x60

//RADIO SHIT
volatile enum radio_band current_radio_band = FM;

uint16_t eeprom_fm_freq;
uint16_t eeprom_am_freq;
uint16_t eeprom_sw_freq;
uint8_t  eeprom_volume;

uint16_t current_fm_freq = 9290; //94.5MHz, 200kHz steps
uint16_t current_am_freq;
uint16_t current_sw_freq;
uint8_t  current_volume;

//Used in debug mode for UART1
char uart1_tx_buf[40];      //holds string to send to crt
char uart1_rx_buf[40];      //holds string that recieves data from uart

volatile uint8_t STC_interrupt;  //flag bit to indicate tune or seek is done

//UART STUFF
char lcd_string_array[16]; //Holds the recieved data
volatile int rcv_rdy;	//Flag used to say when UART data recieved
char rx_char;			//Used to check char one at a time

//TWI STUFF
//delclare the 2 byte TWI read and write buffers (lm73_functions_skel.c)
extern uint8_t lm73_wr_buf[2];
extern uint8_t lm73_rd_buf[2];
uint16_t lm73_temp;		//a place to assemble the temperature from the lm73


/***********************FUNCTIONS**********************/

void TempUpdate(){
	
	static char buffer[16];
	static int charselect = 0;
	
	//read temperature data from LM73 (2 bytes)  (twi_start_rd())
	twi_start_rd(lm73_wr_buf[0], lm73_rd_buf, 2);  
	
	//now assemble the two bytes read back into one 16-bit value
	lm73_temp = lm73_rd_buf[0];//save high temperature byte into lm73_temp
	lm73_temp = lm73_temp<<8;//shift it into upper byte 
	lm73_temp |= lm73_rd_buf[1];//"OR" in the low temp byte to lm73_temp 
    //Change 16bit binary to integer decimal
	lm73_temp =  lm73_temp/128;
	itoa(lm73_temp, buffer, 10); //store into string buffer
	
	if (charselect == 0){
		set_cursor(0x02,0x04);
		char2lcd(buffer[0]);	//tens digit
	}else if(charselect == 1){
		set_cursor(0x02,0x05);
		char2lcd(buffer[1]);	//ones digit
	}else if(charselect == 2){}
	charselect = (charselect + 1) % 3;
}

//PORTE3 TCCR3
void tcnt3_init(void){
	//PORTE Pin3 as an output for PWM waveform
	DDRE  |= 0x0F;
	//8-bit Fast-PWM mode, no prescaling, 255 top, OCR3A comp (set on match) 
	TCCR3A |= (1<<COM3A1 | 1<<WGM30);
	TCCR3B |= (1<<WGM32 | 1<<CS30);
	OCR3A = VOLUME;
}

/***********************INTERRUPTS**********************/

ISR(INT7_vect){STC_interrupt = TRUE;}


ISR(USART0_RX_vect){
//USART0 RX complete interrupt
static  uint8_t  i;
  rx_char = UDR0;              //get character
  lcd_string_array[i++]=rx_char;  //store in array 
 //if entire string has arrived, set flag, reset index
  if(rx_char == '\0'){
    rcv_rdy=1; 
    lcd_string_array[--i]  = (' ');     //clear the count field
    lcd_string_array[i+1]  = (' ');
    lcd_string_array[i+2]  = (' ');
    i=0;  
  }
}

/********************MAIN************************/
int main()
{	//INITIALIZATION
	static int counter = 0;
	
	//Setup INT7 Interrupt
	EIMSK |= (1<<INT7);
	EICRB |= (1<<ISC71 | 1<<ISC70);
		
	//Setup UART
	uart_init();
	
	//Setup SPI 
	//Turn on SS, MOSI, SCLK and set port B pins 4-7 to outputs
	DDRB  |=  (1<<PB7 | 1<<PB6 | 1<<PB5 | 1<<PB4 | 1<<PB2 | 1<<PB1 | 1<<PB0);
	SPCR  |=  (1<<MSTR | 1<<SPE); 		//set up SPI mode
	SPSR  |=  (1<<SPI2X); 				//double speed operation
	DDRD  |=  (1<<PD6 | 1<<PD5);					//Used for encoders and bargraph
	
	//Setup TWI and LCD
	lcd_init();		//startup LCD display handler
	init_twi();		//initalize TWI (twi_master.h)
	
	//Setup Radio Board
	DDRE  |= 0x04; //Port E bit 2 is active high reset for radio 

	//hardware reset of Si4734
	PORTE &= ~(1<<PE7); //int2 initially low to sense TWI mode
	DDRE  |= 0x80;      //turn on Port E bit 7 to drive it low
	PORTE |=  (1<<PE2); //hardware reset Si4734 
	_delay_us(200);     //hold for 200us, 100us by spec         
	PORTE &= ~(1<<PE2); //release reset 
	_delay_us(30);      //5us required because of my slow I2C translators I suspect
                        //Si code in "low" has 30us delay...no explaination
	DDRE  &= ~(0x80);   //now Port E bit 7 becomes input from the radio interrupt
	
	//set LM73 mode for reading temperature by loading pointer register
	//this is done outside of the normal interrupt mode of operation 
	lm73_wr_buf[0] = 0x90; 	//load lm73_wr_buf[0] with temperature pointer address
	twi_start_wr(lm73_wr_buf[0], lm73_wr_buf, 2);	   //start the TWI write process (twi_start_wr())
	
	tcnt3_init();	//volume control
	
	//Turn on interrupts
	sei();
	
	clear_display();	//clean up the display
	char buf[16];
	itoa(current_fm_freq, buf, 10); //store into string buffer
	set_cursor(0x01, 0x04);
	string2lcd(buf);
	
	fm_pwr_up(); //powerup the radio as appropriate
	//current_fm_freq = 9990; //arg2, arg3: 99.9Mhz, 200khz steps
	fm_tune_freq(); //tune radio to frequency in current_fm_freq	
	
	/***************************MAIN LOOP**************************/
	while(1){//do forever
		
		//Print UART data when ready
		if (rcv_rdy == 1){
			home_line2();
			char2lcd(lcd_string_array[0]);	//tens digit
			char2lcd(lcd_string_array[1]);	//ones digit
			rcv_rdy = 0;
		}
		
		counter = (counter + 1) % 1000;
		if (counter == 0){
			lm73_wr_buf[0] = 0x90; 	//load lm73_wr_buf[0] with temperature pointer address
			twi_start_wr(lm73_wr_buf[0], lm73_wr_buf, 2);	   //start the TWI write process (twi_start_wr())
			TempUpdate();
		}
		_delay_us(50);
		
	}
	return 0;
}
