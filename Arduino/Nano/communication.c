#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>

// Picked from the ATMega328P datasheet (chapter 19: USART0)
#define F_CPU 16000000 // Clock speed
#define BAUD 9600
#include <util/setbaud.h>

void
usart_init()
{
    // See ATMega328P datasheet (chapter 19: Usart)
    // Set BAUD rate
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
    if(size > 8) return;

    char buffer[64];
    sprintf(buffer, "decimal: %d, heximal: %X, binary: ", number, number);
    usart_write_string(buffer);

    static const char zero_one[2] = {'0', '1'};

    for(int i = size - 1; i >= 0; i--) {
        buffer[i] = zero_one[number & 1];
        number >>= 1;
    }

    buffer[size] = '\n';
    buffer[size+1] = '\0';
    usart_write_string(buffer);
}

void
usart_write_state()
{
    usart_write_number(PORTD, sizeof(PORTD)*8);
    usart_write_number(DDRD, sizeof(PORTD)*8);
}

enum pin_mode {
    INPUT,
    OUTPUT
};

void
pin_set_mode(int pin, enum pin_mode mode)
{
    switch(mode) {
        case INPUT:
            DDRA &= ~_BV(pin);
            PORTA |= _BV(pin);
            break;
        case OUTPUT:
            DDRD |= _BV(pin);
            break;
    }
}

enum pin_state {
    LOW = 0,
    HIGH = 1
};

void
pin_set_state(int pin, enum pin_state state)
{
    PORTD |= (1 & (int)state) << pin;
    usart_write_state();
}

int main() {
    usart_init();

    DDRD = 0;
    PORTD = 0;

    while(1) {
        char input = usart_getchar();
        int pin = input - (int)'0';

        if(pin >= 0 && pin < 8) {
            pin_set_mode(pin, OUTPUT);
            pin_set_state(pin, HIGH);
        }
    }

    return 0;
}
