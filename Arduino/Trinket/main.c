#include <avr/io.h>
#include <util/delay.h>


void delay(int us) {

    while(us > 500) {
        _delay_us(500);
        us -= 500;
    }

    while(us > 100) {
        _delay_us(100);
        us -= 100;
    }

    while(us > 50) {
        _delay_us(50);
        us -= 50;
    }

    _delay_us(25);
}

void led_on() {
    PORTB |= _BV(PB1);
}

void led_off() {
    PORTB &= ~(_BV(PB1));
}

void high() {
        PORTB |= _BV(PB2); 
}
void low() {
        PORTB &= ~(_BV(PB2));
}

void high_for(int us) {
    high();
    delay(us);
    low();
}

void pulse_for(int us_pulse_length, int ms_time) {
    int delayed = 0;

    for(;;) {
        high_for(us_pulse_length);
        _delay_ms(17);
        delayed += 17;
        
        if(delayed >= ms_time) break;
    }
}

int
main() {
    DDRB  |= _BV(PB1) | _BV(PB2);
    //TCCR1 = 0b01100101;

    pulse_for(2000, 10000);
    led_on();
    pulse_for(1000, 8000);
    led_off();
    pulse_for(1300, 10000);

    for(;;) {
        led_on();
        _delay_ms(250);
        led_off();
        _delay_ms(250);
    }

    /*
    OCR1A = 75;
    OCR1C = 153;
    */

    return 0;
}

