// lab2_code.c
// Huy Nguyen
// 10.08.16

#define CPU 16000000 // cpu speed in hertz 
#define TRUE 1
#define FALSE 0
#include <avr/io.h>
#include <util/delay.h>

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
void segment_sum(uint16_t sum){ //split the number into 4 digit and display 
    static uint8_t ones = 0;
    static uint8_t tens = 0;
    static uint8_t hundreds = 0;
    static uint8_t thousands = 0;

    ones = (sum/1) % 10;
    tens = (sum/10) % 10;
    hundreds = (sum/100) % 10;
    thousands = (sum/1000) % 10;
    
    _delay_ms(20);
    if(sum >= 0){
        PORTB = (0<<4)|(0<<5)|(0<<6);   //select ones digit to display
        PORTA = dec_to_7seg(ones);
    }

    _delay_ms(20);
    if(sum > 9){
        PORTB = (1<<4)|(0<<5)|(0<<6);   //select tens digit to display
        PORTA = dec_to_7seg(tens);
    }

    _delay_ms(20);
    if(sum > 99){
        PORTB = (1<<4)|(1<<5)|(0<<6);   //select hundreds position to display
        PORTA = dec_to_7seg(hundreds);
    }

    _delay_ms(20);
    if(sum > 999){
        PORTB = (0<<4)|(0<<5)|(1<<6);   //select thousands position to display
        PORTA = dec_to_7seg(thousands);
    }
}//segment_sum
//*****************************************************************************
uint8_t main(){
    
    uint16_t sum = 0;
    DDRB = (1<<4)|(1<<5)|(1<<6)|(1<<7);         //set bits 4-7 of PORTB as outputs
    
    while(1){
        _delay_ms(2);                           //loop delay for debounce
        PORTB = (1<<4)|(1<<5)|(1<<6);           //enable tristate buffer
        DDRA = 0x00;                            //Set DDRA as input
        PORTA = 0xFF;                           //pullups PORTA

        _delay_ms(2);
        if(debounce_switch(0)){sum += 1;}       //add 1 when button 0 pressed
        if(debounce_switch(1)){sum += 2;}       //add 2 when button 1 pressed
        if(debounce_switch(2)){sum += 4;}       //add 4 when button 2 pressed
        if(debounce_switch(3)){sum += 8;}       //add 8 when button 3 pressed
        if(debounce_switch(4)){sum += 16;}      //add 16 when button 4 pressed
        if(debounce_switch(5)){sum += 32;}      //add 32 when button 5 pressed
        if(debounce_switch(6)){sum += 64;}      //add 64 when button 6 pressed
        if(debounce_switch(7)){sum += 128;}     //add 128 when button 7 pressed

        DDRA = 0xFF;                            //make PORTA an output
        _delay_ms(2);
        
        if(sum > 1023){sum = 1;}            //roll back to 1 if larger than 1023
        else segment_sum(sum);              //display value to the LED
    }//while
}//main

