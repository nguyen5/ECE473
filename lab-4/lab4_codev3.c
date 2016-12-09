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

//static uint8_t button0 = 0, button1 = 0;
int val = 0;
int alarm_val = 0;
int snooze_val = 0;

uint8_t first = 1;
uint8_t set_time = 0;
uint8_t set_alarm = 0;
uint8_t blink_flag = 0;
uint8_t alarm_en = 0;
uint8_t play_sound = 1;
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
    
    DDRA = 0xFF;                         //Set PORTA as output


    
//    _delay_ms(5);
    if(pos == 0){
        PORTB = (0<<4)|(0<<5)|(0<<6);   //select ones digit to display
        PORTA = dec_to_7seg(ones);
    }
//    _delay_ms(5); 
    if(pos == 1){
        PORTB = (1<<4)|(0<<5)|(0<<6);   //select tens digit to display
        PORTA = dec_to_7seg(tens);
    }
//    _delay_ms(5); 
    if(pos == 2){
        PORTB = (1<<4)|(1<<5)|(0<<6);   //select hundreds position to display
        PORTA = dec_to_7seg(hundreds);
    }
//    _delay_ms(5); 
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
//    _delay_ms(5);
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
    if(first == 0){
        if(enc0 == CW){ 
            if(set_time == 1){ val += 1;}       //increase if CW
            else if(set_alarm == 1){ alarm_val += 1;}
        snooze_val = alarm_val;
        }
        else if(enc0 == CCW){
            if(set_time == 1){ val -= 1;}
            else if(set_alarm == 1){ alarm_val -= 1;}
        snooze_val = alarm_val; 
        }
        if(enc1 == CW){
            if(set_time == 1){ val += 60;}
            else if(set_alarm == 1){ alarm_val += 60;}
        snooze_val = alarm_val;
        }
        else if(enc1 == CCW){
            if(set_time == 1){ val -= 60;}
            else if(set_alarm == 1){ alarm_val -= 60;}
        snooze_val = alarm_val; 
        }
    }
//    else if(enc0 == CCW) val = val - inc_dec_value; //decrease if CCW
//    if(enc1 == CW) val = val + inc_dec_value;       //increase if CW
//    else if(enc1 == CCW) val = val - inc_dec_value; //decrease if CCW
} 
//*****************************************************************************
void bargraph(){
    uint8_t output = val;
    //convert mode into bit data for bargraph
/*    if(button0){ output |= 0x01;}
    else if(!button0){ output &= 0xFE;}
    if(button1){ output |= 0x02;}
    else if(!button1){ output &= 0xFD;}
  */  SPDR = output;
    while(bit_is_clear(SPSR, SPIF)){}
    //strobe the bargraph to transmit data
    PORTD |= (1<<PD5);
    PORTD &= ~(1<<PD5);
}
//*****************************************************************************

//*****************************************************************************
ISR(ADC_vect){
    static uint16_t adc_input;
    adc_input = (ADC/4);
    if(OCR2 < adc_input){ OCR2++;}
    else OCR2--;
} 
//*****************************************************************************
/*ISR(TIMER3_COMPA_vect){
   PORTE ^=0b00000001;
}
*****************************************************************************    
ISR(TIMER3_OVF_vect){
    PORTE ^=0b00000001;
} */
//*****************************************************************************
ISR(TIMER1_COMPA_vect){
    if((play_sound == 1) & (alarm_en == 1)){
        PORTC ^= (1<<PC0);
//        PORTC |= (1<<PC0);
    }
}
//*****************************************************************************
ISR(TIMER0_OVF_vect){    
    uint8_t i;
    static uint8_t ISR_count, second;
    static uint8_t state = 0x00;
    //setup clock
    ISR_count = (ISR_count + 1) % 420;
    if(ISR_count == 0){
        second = (second + 1) % 15;
        blink_flag = (blink_flag + 1) % 2;
        if((second == 0) & (set_time == 0)){
            val += 1;
        }
    } 
    state = 0x00;
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
        //    set_time = (set_time + 1) % 2;}
    }
    else if(state == 0x02){
        if(set_time == 0){ set_alarm = !set_alarm;}
        //    set_alarm = (set_alarm + 1) % 2;}
    }
    else if(state == 0x04){ 
        alarm_en = !alarm_en;
        lcd_init();
        clear_display();
        cursor_home();
        if(alarm_en){ string2lcd("---ALARM ON---");}
        else if(!alarm_en){ string2lcd("---ALARM OFF---");}
        cursor_home();
    }
       // alarm_en = (alarm_en + 1) % 2;}

//    else if(!button0 && button1){ inc_dec_value = 4;}   //mode 2, increase by 4
//    else if(!button0 && !button1){ inc_dec_value = 1;}  //mode 0, default increse by 1
        
    update_val();
//    segment_sum();
    bargraph();         

}           
//*****************************************************************************
void adc_init(void){
    DDRF &= ~(1<<PE7);
    PORTF = 0x00;
    ADMUX = (1<<REFS0|1<<MUX2|1<<MUX1|1<<MUX0);
    ADCSRA = (1<<ADEN|1<<ADSC|1<<ADFR|1<<ADIE|1<<ADPS2|1<<ADPS1|1<<ADPS0);
}
//*****************************************************************************
void tcnt1_init(void){
    DDRC |= (1<<PC0);
    TIMSK |= (1<<OCIE1A);
    TCCR1B = (1<<WGM12|1<<CS12|0<<CS10);
    OCR1A = 0x0040;
}                                                        
//*****************************************************************************
void tcnt3_init(void){
    DDRE |= 0x0F;
    TCCR3A |= (1<<COM3A1|1<<WGM30);
    TCCR3B |= (1<<WGM32|1<<CS30);
    OCR3A = 0x8F;
//    ICR3 = 0xFF;
}
//*****************************************************************************
uint8_t main(){
    tcnt1_init();
    tcnt3_init();
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

    sei();
    adc_init();
    lcd_init();
    clear_display();
    cursor_home();
    string2lcd("---ALARM OFF---");
    cursor_home();
    int i = 0;
    while(1){
        DDRA = 0xFF;                //set PORTA as output
        PORTA = 0xFF;               //turn on pull up resistor
        segment_sum(i);
        i = (i + 1) % 5;
        //read and write SPI 
        PORTD = 0x00;
        PORTD = 0x40;
        while(bit_is_clear(SPSR, SPIF)){}
        input = SPDR; 
 
    }//while
    return 0;
}//main
                          
