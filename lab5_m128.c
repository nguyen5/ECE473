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
#include "twi_master.h"
#include "lm73_functions_skel.h"
#include "uart_functions.h"
#include "si4734.h"
#include <stdlib.h>
#include <string.h>
/* mode

mode 0: increase/decrease by 1    default. 
mode 1: increase/decresas by 2             button 0
mode 2: increase/decrease by 4             button 1
mode 3: stop                               button 0 and 1
*/

#define CW 1
#define CCW 2

int cnt2 = 0;
int val = 0;
int alarm_val = 0;
int snooze_val = 0;
uint8_t ISR_count = 0;
uint8_t first = 1;
uint8_t set_time = 0;
uint8_t set_alarm = 0;
uint8_t blink_flag = 0;
uint8_t alarm_en = 0;
uint8_t play_sound = 0;
uint8_t radio_mode = 0;
uint8_t input;
uint16_t tone;
uint8_t show_radio = 0;
uint8_t just_another_flag = 0;
volatile enum radio_band current_radio_band = FM;
uint16_t eeprom_fm_freq;
uint16_t eeprom_am_freq;
uint16_t eeprom_sw_freq;
uint8_t eeprom_volume;

uint16_t temp_freq = 10630;
uint16_t current_fm_freq;
uint16_t current_am_freq;
uint16_t current_sw_freq;
uint8_t current_volume;
char uart1_tx_buf[40];
char uart1_rx_buf[40];
int Volume = 0x5F;
volatile uint8_t STC_interrupt;


int j = 0;
extern uint8_t lm73_wr_buf[2];
extern uint8_t lm73_rd_buf[2];
uint16_t lm73_temp;

volatile int rcv_rdy;
char rx_char;
char lcd_string_array[16];

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
    static uint8_t freq_tho = 0;
    if(set_alarm == 0){
        if(val < 0){ val = 1439;}   //Rollover when decrease below 0
        else if(val > 1439){ val = 0;}       //Rollover when increase above 1023
        ones = val % 10;
        tens = (val % 60) / 10;
        hundreds = (val/60) % 10;
        thousands = (val/600) % 10;
    }
    if(set_alarm == 1){
        if(alarm_val < 0){ alarm_val = 1439;}   //Rollover when decrease below 0
        else if(alarm_val > 1439){ alarm_val = 0;}       //Rollover when increase above 1023
        ones = alarm_val % 10;
        tens = (alarm_val % 60) / 10;
        hundreds = (alarm_val/60) % 10;
        thousands = (alarm_val/600) % 10;
    }
    if(temp_freq > 10790){ temp_freq = 8810;} //frequency roll over
    if(temp_freq < 8810){ temp_freq = 10790;} //frequency roll over
    if(radio_mode == 1 && just_another_flag == 1){      //setup to display frequency 
        ones = (temp_freq/10) % 10;                     //while tuning
        tens = (temp_freq/100) % 10;
        hundreds = (temp_freq/1000) % 10;
        thousands = (temp_freq/10000) % 10;
        freq_tho = (temp_freq/10000) % 10;
        blink_flag = 0;
    }
 
    DDRA = 0xFF;                         //Set PORTA as output


    if(pos == 0){
        PORTB = (0<<4)|(0<<5)|(0<<6);   //select ones digit to display
        PORTA = dec_to_7seg(ones);
    }
    if(pos == 1){
        PORTB = (1<<4)|(0<<5)|(0<<6);   //select tens digit to display
        if(radio_mode == 1 && just_another_flag == 1){
            PORTA = dec_to_7seg(tens);
            PORTA &= ~(1<<PA7);
        }
        else PORTA = dec_to_7seg(tens);
    }
    if(pos == 2){
        PORTB = (1<<4)|(1<<5)|(0<<6);   //select hundreds position to display
        PORTA = dec_to_7seg(hundreds);
    }
    if(pos == 3){
        PORTB = (0<<4)|(0<<5)|(1<<6);   //select thousands position to display
        if(just_another_flag == 1 && freq_tho == 0){
            PORTA = 0xFF;
            freq_tho = 1;
        }
        else PORTA = dec_to_7seg(thousands);
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
}//segment_sum
//*****************************************************************************
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
    if(first == 0){
        if(enc0 == CW){ 
            if(set_time == 1){ val += 1;}       //increase hours if CW
            else if(set_alarm == 1){ alarm_val += 1;}
            if((set_alarm == 0) && (set_time == 0) && (radio_mode == 1)){
                Volume += 10;                   //increase folume
                if(Volume < 150 && Volume > 0){ 
                    OCR3A = Volume;
                    if (Volume > 150){ Volume = 150;}
                }
            }
        snooze_val = alarm_val;
        }
        else if(enc0 == CCW){                   //decrease hours if CCW
            if(set_time == 1){ val -= 1;}
            else if(set_alarm == 1){ alarm_val -= 1;}
            if((set_alarm == 0) && (set_time == 0) && (radio_mode == 1)){
                Volume -= 10;                   //decrease volume
                if(Volume > 0 && Volume < 150){
                    OCR3A = Volume;             
                    if(Volume < 0){ Volume = 0;}
                }
            }
        snooze_val = alarm_val; 
        }
        if(enc1 == CW){                         //increase minutes if CW
            if(set_time == 1){ val += 60;}
            else if(set_alarm == 1){ alarm_val += 60;} 
            if((set_alarm == 0) && (set_time == 0) && (radio_mode == 1)){
                temp_freq += 20;                //increase frequency
                just_another_flag = 1;
            } 
        snooze_val = alarm_val;
        }
        else if(enc1 == CCW){                   //decrease minutes if CCW
            if(set_time == 1){ val -= 60;}
            else if(set_alarm == 1){ alarm_val -= 60;}   
            if((set_alarm == 0) && (set_time == 0) && (radio_mode == 1)){
                temp_freq -= 20;                //decrease frequency
                just_another_flag = 1;
            } 
        snooze_val = alarm_val; 
        }
    }
} 
//*****************************************************************************
void temp_read (){
    char buff[16];
    twi_start_rd(lm73_wr_buf[0], lm73_rd_buf, 2);   //read data from LM73
    lm73_temp = lm73_rd_buf[0];         //save high byte from LM73
    lm73_temp = (lm73_temp << 8);       //shift to upper byte of lm73_temp
    lm73_temp |= (lm73_rd_buf[1]);      //save low byte from LM73
   
    lm73_temp = (lm73_temp >> 7);
    itoa(lm73_temp, buff, 10);          //store into buffer at base 10 
    set_cursor(0x02, 0x00);
    string2lcd(buff);                   //print the temperature to LCD
}
//*****************************************************************************
void bargraph(){
    uint8_t output = val;
    SPDR = output;
    while(bit_is_clear(SPSR, SPIF)){}
    //strobe the bargraph to transmit data
    PORTD |= (1<<PD5);
    PORTD &= ~(1<<PD5);
}
//*****************************************************************************
ISR(INT7_vect){ STC_interrupt = TRUE;}
//*****************************************************************************
ISR(ADC_vect){
    //Dimming function interrupt
    static uint16_t adc_input;
    adc_input = (ADC/4);
    if(OCR2 < adc_input){ OCR2++;}
    else OCR2--;
} 
//*****************************************************************************
ISR(TIMER1_COMPA_vect){
    if((play_sound == 1) & (alarm_en == 1)){
        PORTC ^= (1<<PC0);      //flipping bit to make tone
    }
}
//*****************************************************************************
ISR(TIMER0_OVF_vect){    
    uint8_t i;
    static uint8_t second;
    static uint8_t state = 0x00;
    //setup clock
    ISR_count = (ISR_count + 1) % 489;
    cnt2 = (cnt2 + 1) % 100;
    if(ISR_count == 0){
        second = (second + 1) % 15;
        blink_flag = (blink_flag + 1) % 2;
        show_radio = (show_radio + 1) % 8;
        
        if((second == 0) & (set_time == 0)){
            val += 1;
            if((val == snooze_val) & (alarm_en == 1)){
                if(radio_mode == 1){ radio_mode = 0;}
                play_sound = 1;
            }
            else{
                if(val > snooze_val){ snooze_val = alarm_val;}
                play_sound = 0;
            }
        }
      
    }
   
    state = 0x00;           //reset state every ISR call
    PORTA = 0xFF;           //turn on pull up resistor
    DDRA = 0x00;            //set PORTA as input
    PORTB = (1<<4)|(1<<5)|(1<<6);   //enable tri-state buffer
    for(i = 0; i < 8; i++){
        if(debounce_switch(i)){     //check buttons state
            state |= (1<<i);
        }
    }   
    if(state == 0x00){}         //both mode chose, do nothing
    else if(state == 0x01){   //mode 1, increase by 2
        if(set_alarm == 0){ set_time = !set_time;} 
    }
    else if(state == 0x02){
        if(set_time == 0){ set_alarm = !set_alarm;}
    }
    else if(state == 0x04){ 
        // display alarm on/off on LCD
        alarm_en = !alarm_en;
        if(alarm_en){ 
            set_cursor(0x02, 0x0C);
            string2lcd("ON ");}
        else if(!alarm_en){ 
            set_cursor(0x02, 0x0C);
            string2lcd("OFF");
            play_sound = 0;
        }
    }        
    else if(state == 0x08){
        if(play_sound == 1){
            play_sound = 0;
            snooze_val += 3;
            if(snooze_val > 1439){ snooze_val = snooze_val % 1439;}
        }             
    }
    else if(state == 0x80){
        radio_mode = !radio_mode;
    }
    update_val();
    PORTA = 0xFF;               //turn on pull up resistor
    DDRA = 0xFF;                //set PORTA as output
    segment_sum(j);
    j = (j + 1) % 5;
}           
//*****************************************************************************
ISR(USART0_RX_vect){
    static uint8_t i;
    rx_char = UDR0;             //read USART for data from mega48
    lcd_string_array[i++] = rx_char;    //store data in string array
    if(rx_char == '\0'){        //set flag, reset index when complete receiving
        rcv_rdy = 1;
        lcd_string_array[--i] = (' ');
        lcd_string_array[i++] = (' ');
        lcd_string_array[i+2] = (' ');
        i = 0;
    }
}

//*****************************************************************************
void adc_init(void){
    // initialize adc for dimming feature
    DDRF &= ~(1<<PE7);
    PORTF = 0x00;
    ADMUX = (1<<REFS0|1<<MUX2|1<<MUX1|1<<MUX0);
    ADCSRA = (1<<ADEN|1<<ADSC|1<<ADFR|1<<ADIE|1<<ADPS2|1<<ADPS1|1<<ADPS0);
}
//*****************************************************************************
void tcnt1_init(void){
    // initialize timer/counter1 for creating tone
    DDRC |= (1<<PC0);
    TIMSK |= (1<<OCIE1A);     //enable interupt mask
    TCCR1B = (1<<WGM12|1<<CS12|0<<CS10);    //CTC mode, 1024 prescaling
    OCR1A = tone;
    
}                                                        
//*****************************************************************************
void tcnt3_init(void){
    //initialize timer/counter3
    // Fast PWM, no prescale, OCR3A compare set on match
    DDRE |= 0x0F;
    TCCR3A |= (1<<COM3A1|1<<WGM30);
    TCCR3B |= (1<<WGM32|1<<CS30);
    OCR3A = Volume;
}                        
//*****************************************************************************
uint8_t main(){
    static int cnt = 0;
 //   static int cnt2 = 0;
    static int cnt3 = 0;
    static int enc_check;
    static uint16_t prv_freq;
    tcnt1_init();
    tcnt3_init();

    EIMSK |= (1<<INT7);
    EICRB |= (1<<ISC71|1<<ISC70);

    //setup SPI
    //Turn on SS, MOSI, SCLK and set PORTB pin 4-7 to outputs
    DDRB |= (1<<PB0)|(1<<PB1)|(1<<PB2)|(1<<PB4)|(1<<PB5)|(1<<PB6)|(1<<PB7);
    SPCR |= (1<<MSTR)|(1<<SPE);     //SPI mode
    SPSR |= (1<<SPI2X);             //double clock
    DDRD = (1<<PD6|1<<PD5);               //Set PORTD bit 2 to out put
    
    // initialize interrupt
    TIMSK |= (1<<TOIE0);
    TCCR0 |= (1<<CS02) | (1<<CS00);
    //initialize timer/counter2
    TCCR2 |= (1<<WGM21|1<<WGM20|1<<CS20|1<<COM21|1<<COM20);
    OCR2 = 0xFF;

    uart_init();
    adc_init();
    lcd_init();
    init_twi();

    //initialize radio board
    DDRE |= 0x04;
    PORTE &= ~(1<<PE7);
    DDRE |= 0x80;
    PORTE |= (1<<PE2);
    _delay_us(200);
    PORTE &= ~(1<<PE2);
    _delay_us(30);
    DDRE &= ~(0x80);

    //setup twi buffer
    lm73_wr_buf[0] = 0x90;
    twi_start_wr(lm73_wr_buf[0], lm73_wr_buf, 2);
    sei();
    
    //LCD display interface
    clear_display();
    cursor_home();
    string2lcd("INT | EXT | ALR");
    home_line2();
    string2lcd("   C|    C| OFF");
    set_cursor(0x02, 0x02);
    char2lcd(0xDF);
    set_cursor(0x02, 0x08);
    char2lcd(0xDF);
    //turn on the radio board once
    fm_pwr_up();
    current_fm_freq = temp_freq;
    fm_tune_freq();
    
    while(1){
        //Print UART data to the LCD 
        if(rcv_rdy == 1){
            set_cursor(0x02, 0x06);
            char2lcd(lcd_string_array[0]);
            char2lcd(lcd_string_array[1]);
            rcv_rdy = 0;
        }   
        //**************************************
        if(radio_mode == 0){ 
            set_property(RX_VOLUME, 0x0000);  //mute the radio
        }
        else if(radio_mode == 1){
            set_property(RX_VOLUME, 0x003F);  //unmute the radio
        }  
        
        //check and display frequency when tuning
        enc_check = encodershit(1);
        prv_freq = current_fm_freq;
        if(enc_check != 0){} 
        else if((enc_check == 0)){
 //           cnt2 = (cnt2 + 1) % 50;
            cnt3 = (cnt3 + 1) % 200;
            if((cnt2 == 0) && (prv_freq != temp_freq)){
                prv_freq = temp_freq;
                current_fm_freq = prv_freq;
                fm_tune_freq();
                _delay_ms(50);
            }
            else if((cnt3 == 0) && (prv_freq == temp_freq)){
                just_another_flag = 0;        
            }
            
        }
        //update local temperature from the lm73
        cnt = (cnt + 1) % 20;
        if(cnt == 0){ temp_read();}
        SPDR = radio_mode;
        while(bit_is_clear(SPSR, SPIF)){}
        //strobe the bargraph to transmit data
        PORTD |= (1<<PD5);
        PORTD &= ~(1<<PD5);         
        PORTD = 0x00;
        PORTD = 0x40;
        while(bit_is_clear(SPSR, SPIF)){}
        input = SPDR; 
 
    }//while
    return 0;
}//main
                          
