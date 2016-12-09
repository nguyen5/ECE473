// lab_code.c
// Huy Nguyen
// 10.26.16

#define CPU 16000000 // cpu speed in hertz 
#define TRUE 1
#define FALSE 0
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>  
#include "hd44780.h"
/* mode

mode 0: increase/decrease by 1    default. 
mode 1: increase/decresas by 2             button 0
mode 2: increase/decrease by 4             button 1
mode 3: stop                               button 0 and 1
*/

#define CW 1
#define CCW 2

static uint8_t button0 = 0, button1 = 0;
uint16_t val = 0;
uint8_t first = 1;
uint8_t inc_dec_value = 1;
uint8_t input;
//*****************************************************************************
// debounce funtion
int8_t debounce_switch(uint8_t button){
    static uint16_t state[8] = {0};
state[button] = (state[button] << 1) | (!bit_is_clear(PINA, button)) | 0xE000;
if (state[button] == 0xF000) return 1;
return 0;
}
//*****************************************************************************
uint8_t dec_to_7seg(uint8_t dec){ //Convert decimal number to display on 7-seg LED
switch(dec){
    case 0: return 0xC0;    //zero, turn on every segment but G
        break;
    case 1: return 0xF9;    //one, turn on only B and C segment
        break; 
    case 2: return 0xA4;    //two, turn on every segment but C and F
        break;
    case 3: return 0xB0;    //three, turn on every segment but E and F
        break;
    case 4: return 0x99;    //four, turn on every segment but A, D and E
        break;
    case 5: return 0x92;    //five, turn on every segment but B and E
        break;
    case 6: return 0x82;    //six, turn on every segment but B
        break;
    case 7: return 0xF8;    //seven, turn on only A,B and C segment
        break;
    case 8: return 0x80;    //eight, turn on all segment
        break;
    case 9: return 0x98;    //nine, turn on every segment but D and E
        break;
    }
return 0xFF;
} // dec_to_7seg
//******************************************************************************
void segment_sum(int pos){ //split the number into 4 digit and display 
    static uint8_t ones = 0;
    static uint8_t tens = 0;
    static uint8_t hundreds = 0;
    static uint8_t thousands = 0;
    
    if(val > (0 - 1023)){ val = 1023;}   //Rollover when decrease below 0
    else if(val > 1023){ val = 0;}       //Rollover when increase above 1023
    ones = val % 10;
    tens = (val % 10) / 10;
    hundreds = (val/100) % 10;
    thousands = (val/1000) % 10;
        
    DDRA = 0xFF;                         //Set PORTA as output
    
    _delay_ms(5);
    if(pos == 0){
        PORTB = (0<<4)|(0<<5)|(0<<6);   //select ones digit to display
        PORTA = dec_to_7seg(ones);
    }
    _delay_ms(5); 
    if(pos == 1){
        PORTB = (1<<4)|(0<<5)|(0<<6);   //select tens digit to display
        PORTA = dec_to_7seg(tens);
    }
    _delay_ms(5); 
    if(pos == 2){
        PORTB = (1<<4)|(1<<5)|(0<<6);   //select hundreds position to display
        PORTA = dec_to_7seg(hundreds);
    }
    _delay_ms(5); 
    if(pos == 3){
        PORTB = (0<<4)|(0<<5)|(1<<6);   //select thousands position to display
        PORTA = dec_to_7seg(thousands);
    }
    if(pos == 4){
        PORTB = (0<<4)|(1<<5)|(0<<6);
        if(alarm_en == 0){
            if((blink_flag == 1)|(set_time == 1)|(set_alarm == 1)){ 
                PORTA = 0xFC;
            }
        }
        else if(alarm_en == 1){
            if((blink_flag == 1)|(set_time == 1)|(set_alarm == 1)){
                PORTA = 0xF8;
            }
        }
    }
    _delay_ms(5);
}//segment_sum


///*****************************************************************************
uint8_t encodershit(uint8_t encoder){
    static uint8_t pre_enc1, pre_enc0;
    static uint8_t cur_enc;

    if(encoder == 0){                       //Masking the encoders bit
        cur_enc = (input & 0x03);}
    else if (encoder == 1){ 
        cur_enc = (input & 0x0C);
        cur_enc = cur_enc/4;
    }
      
    if(first == 1){                        //Check if the first turn 
        pre_enc0 = cur_enc;                //prevent further check
        pre_enc1 = cur_enc;
        first = 0; 
        return 0;
    }

    //Check positions of the encoder 
    if(encoder == 0){                           //First encoder                 
        if(cur_enc == pre_enc0){ 
            return 0;
        }
        if(pre_enc0 == 3){                 
            pre_enc0 = cur_enc;
            if(cur_enc == 1){ return CW;}       //CW if go from 3 to 1
            else if(cur_enc == 2){ return CCW;} //CCW if go from 3 to 2
        }
        pre_enc0 = cur_enc;
    }
    else if(encoder == 1){                      //Second encoder
        if(cur_enc == pre_enc1){ 
            return 0;
        }
        if(pre_enc1 == 3){
            pre_enc1 = cur_enc; 
            if(cur_enc == 1){ return CW;}       //CW if go from 3 to 1 
            else if(cur_enc == 2){ return CCW;} //CCW if go from 3 to 2 
        }
        pre_enc1 = cur_enc;
    } 
    return 0;
}
//*********************************************************************
void update_val(void){
    static int enc0, enc1;
    enc0 = encodershit(0);      //check for CW or CCW
    enc1 = encodershit(1);      //check for CW or CCW
    if(enc0 == CW) val = val + incd_dec_value;      //increase if CW
    else if(enc0 == CCW) val = val - inc_dec_value; //decrease if CCW
    if(enc1 == CW) val = val + inc_dec_value;       //increase if CW
    else if(enc1 == CCW) val = val - inc_dec_value; //decrease if CCW
} 
//*****************************************************************************
void bargraph(){
    uint8_t output = val;
    //convert mode into bit data for bargraph
    if(button0){ output |= 0x01;}
    else if(!button0){ output &= 0xFE;}
    if(button1){ output |= 0x02;}
    else if(!button1){ output &= 0xFD;}
    SPDR = output;
    while(bit_is_clear(SPSR, SPIF)){}
    //strobe the bargraph to transmit data
    PORTD |= (1<<PD5);
    PORTD &= ~(1<<PD5);
} 
//*****************************************************************************
ISR(TIMER0_OVF_vect){    
    uint8_t i;
    DDRA = 0x00;            //set PORTA as input
    PORTA = 0xFF;           //turn on pull up resistor
    PORTB = (1<<4)|(1<<5)|(1<<6);   //enable tri-state buffer
    for(i = 0; i < 8; i++){
        if(debounce_switch(i)){     //check buttons state
            if(i == 0){
                button0 = !button0; //toggle between 0 and 1 on button 0
            }
            else if(i == 1){
                button1 = !button1; //toggle between 0 and 1 on button 1
            }
            
            if(button0 && button1){ inc_dec_value = 0} //both mode chose, do nothing
            else if(button0 && !button1){ inc_dec_value = 2;}   //mode 1, increase by 2
            else if(!button0 && button1){ inc_dec_value = 4;}   //mode 2, increase by 4
            else if(!button0 && !button1){ inc_dec_value = 1;}  //mode 0, default increse by 1
        }
    }
    update_val();
    segment_sum();
    bargraph();         

}           

//*****************************************************************************
uint8_t main(){
    //setup SPI
    //Turn on SS, MOSI, SCLK and set PORTB pin 4-7 to outputs
    DDRB |= (1<<PB0)|(1<<PB1)|(1<<PB2)|(1<<PB4)|(1<<PB5)|(1<<PB6)|(1<<PB7);
    SPCR |= (1<<MSTR)|(1<<SPE);     //SPI mode
    SPSR |= (1<<SPI2X);             //double clock
    DDRD = (1<<PD6|1<<PD5);               //Set PORTD bit 2 to out put
    
    // initialize interrupt
    TIMSK |= (1<<TOIE0);
    TCCR0 |= (1<<CS02) | (1<<CS00);
    sei();
    
    while(1){
        DDRA = 0xFF;                //set PORTA as output
        PORTA = 0xFF;               //turn on pull up resistor
        
        //read and write SPI 
        PORTD = 0x00;
        PORTD = 0x40;
        while(bit_is_clear(SPSR, SPIF)){}
        input = SPDR; 
 
    }//while
    return 0;
}//main

