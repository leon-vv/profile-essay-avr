#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdbool.h>

// Picked from the ATMega328P datasheet (chapter 19: USART0)
#define F_CPU 16000000 // Clock speed
#define BAUD 9600
#include <util/setbaud.h>

#include "../Shared/fletcher.c"

void
usart_init()
{
    // See ATMega328P datasheet (chapter 20: Usart)
    // Set BAUD rate.
    UBRR0H = UBRRH_VALUE;
    UBRR0L = UBRRL_VALUE;

#if USE_2X
    UCSR0A |= _BV(U2X0);
#else
    UCSR0A &= ~(_BV(U2X0));
#endif

    // Enable receiver and transmitter
    UCSR0B = _BV(RXEN0) | _BV(TXEN0);

    // Use 1 stop bit and 8-bit data frames
    UCSR0C = _BV(USBS0) | _BV(UCSZ01) | _BV(UCSZ00);
}

void
usart_putchar(unsigned char c)
{
    loop_until_bit_is_set(UCSR0A, UDRE0);
    UDR0 = c;
}

unsigned char
usart_getchar()
{
    loop_until_bit_is_set(UCSR0A, RXC0);
    return UDR0;
}

void
usart_write_string(char *string)
{
    for(char *p = string; *p != '\0'; p++) usart_putchar(*p);
}

void
usart_write_number(unsigned number, unsigned size)
{
    if(size > 16) return;

    char buffer[64];
    sprintf(buffer, "decimal: %d, heximal: %X ", number, number);

    usart_write_string(buffer);
}

void
usart_write_state()
{
    usart_write_number(PORTD, sizeof(PORTD)*8);
    usart_write_number(DDRD, sizeof(PORTD)*8);
}

uint8_t inline
set_n_bit_equal_to(uint8_t value, uint8_t index, bool x)
{
    if((value & _BV(index)) && x == 0) return value & ~_BV(index);

    return value | (x << index);
}


void
led_on()
{
    PORTB |= _BV(PB5);
}

void
led_off()
{
    PORTB &= ~_BV(PB5);
}

volatile uint8_t counter = 48;

volatile uint8_t buffer[6] = {0b01010011, 0, 0, 0, 0, 0};
volatile uint8_t receiving[6] = {0, 0, 0, 0, 0, 0};

volatile int i = 0;

ISR(TIMER0_COMPA_vect) {

    if(i > 400) {
        PORTB ^= _BV(PB5);
        i = 0;
    }

    i += 1;

    if(counter >= 48) {
        // Update the data
        uint8_t checksum[2];

        for(int i = 1; i < 4; i++) {
            buffer[i] = usart_getchar();
        }

        // Increment buffer by the size of a byte to
        // skip the constant byte.
        compute_checksum(buffer + 1, checksum);

        buffer[4] = checksum[0];
        buffer[5] = checksum[1];

        //for(int i = 0; i < 6; i++) receiving[i] = 0;

        counter = 0;
    }
    
    uint8_t byte_index = counter / 8;
    uint8_t bit_index = 7 - (counter % 8);
    
    //Set the pin to the correct state.
    PORTD = set_n_bit_equal_to(PORTD, PD7, (buffer[byte_index] >> bit_index) & 1);

    /*
    _delay_us(100);
    
    ADCSRA |= _BV(ADSC); 
    loop_until_bit_is_clear(ADCSRA, ADSC);

    receiving[byte_index] = set_n_bit_equal_to(receiving[byte_index], bit_index, ADC > 500);
    */

    counter += 1;
}

int main() {
    _delay_ms(2000);

    // Enable transmitter output pin.
    DDRD |= _BV(DDD7);
    // Enable led output pin.
    DDRB |= _BV(DDB5);

    usart_init();

    // Run the 'TIM0_COMPA_vect' interrupt approximately every 5ms.
    TCCR0A = 0b00000010;
    TCCR0B = 0b00000101;
    OCR0A = 78;

    // Enable 'Timer/Counter0 Ouput Compare Match A Interrupt'
    TIMSK0 |= _BV(OCIE0A);

    // Enable interrupts.
    sei();

    // Configure ADC
    ADMUX = _BV(REFS0) | _BV(MUX1) | _BV(MUX0);
    ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);

    for(;;);
}

