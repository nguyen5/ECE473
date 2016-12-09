//Huy Nguyen

#define TRUE 1
#define FALSE 0
#include <avr/io.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "uart_functions_m48.h"
#include "lm73_functions_skel.h"
#include "twi_master.h"

extern uint8_t lm73_wr_buf[2];
extern uint8_t lm73_rd_buf[2];
uint16_t lm73_temp;
char string_buffer[16];

//*****************************************************************************
void temp_read(){
    twi_start_rd(lm73_wr_buf[0], lm73_rd_buf, 2);    //read data from LM73
    lm73_temp = lm73_rd_buf[0];            //save high byte to lm73_temp
    lm73_temp = (lm73_temp << 8);          //shift to upper byte of lm73_temp
    lm73_temp |= (lm73_rd_buf[1]);         //save low byte to lm73_temp   
    lm73_temp = (lm73_temp >> 7);
    itoa(lm73_temp, string_buffer, 10);
}
//*****************************************************************************
int main(){
    uart_init();       //initialize uart 
    init_twi();        //initialize TWI 

    lm73_wr_buf[0] = 0x90;        //set address for lm73 
    twi_start_wr(lm73_wr_buf[0], lm73_wr_buf, 2);

    sei();

    while(1){
        _delay_ms(20);
        temp_read();        //read temperature from lm73
        _delay_ms(20);
        uart_puts(string_buffer);        //sent to mega128 via USART
        uart_putc('\0');
    }
}
