#include <avr/io.h>
#include <util/delay.h>


int main() {
    DDRB |= _BV(PB2) | _BV(PB1);

    TCCR0A = 0b00000000;
    TCCR0B = 0b
}

ISR(TIMER0_COMPA) {
    // Called every 20 ms
    PORTB ^= _BV(PB1);

    PORTB |= _BV(PB2);
    _delay_us(1500);
    PORTB &= ~(_BV(PB2));
}
