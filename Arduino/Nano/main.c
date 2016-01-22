#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdbool.h>

// Picked from the ATMega328P datasheet (chapter 19: USART0)
#define F_CPU 16000000 // Clock speed
#define BAUD 19200
#include <util/setbaud.h>

#include "../Shared/fletcher.c"

void
usart_putchar(unsigned char c)
{
    loop_until_bit_is_set(UCSR0A, UDRE0);
    UDR0 = c;
}

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

	// Request data
	usart_putchar('D');
}

unsigned char
usart_getchar()
{
    loop_until_bit_is_set(UCSR0A, RXC0);
    return UDR0;
}

uint8_t inline
set_n_bit_equal_to(uint8_t value, uint8_t index, bool x)
{
    if((value & _BV(index)) && x == 0) return value & ~_BV(index);

    return value | (x << index);
}


volatile uint8_t counter = 48;

volatile uint8_t buffer[6] = {0b01010011, 0, 0, 0, 0, 0};
volatile uint8_t receiving[6] = {0, 0, 0, 0, 0, 0};

// Called every 2 ms.
ISR(TIMER0_COMPA_vect) {

    if(counter >= 48) {
    	// All the six bytes have been sent.
        // Request new bytes from the laptop.
        //
        uint8_t checksum[2];
        for(int i = 1; i < 4; i++) {
            buffer[i] = usart_getchar();
        }

		// Request data
		usart_putchar('D');

        // Increment buffer by the size of a byte to
        // skip the constant byte.
        compute_checksum(buffer + 1, checksum);

        buffer[4] = checksum[0];
        buffer[5] = checksum[1];

        counter = 0;
    }
    
    uint8_t byte_index = counter / 8;
    uint8_t bit_index = 7 - (counter % 8);
    
    // Set the pin to the correct state.
    // This pin is connected to the transmitter.
    PORTD = set_n_bit_equal_to(PORTD, PD7, (buffer[byte_index] >> bit_index) & 1);
    
    counter += 1;
}

int main() {
    _delay_ms(2000);

    // Enable transmitter output pin.
    DDRD |= _BV(DDD7);
    // Enable led output pin.
    DDRB |= _BV(DDB5);

    usart_init();

    // Run the 'TIM0_COMPA_vect' interrupt approximately every 2 ms.
    TCCR0A = 0b00000010;
    TCCR0B = 0b00000101;
    OCR0A = 32;

    // Enable 'Timer/Counter0 Ouput Compare Match A Interrupt'
    TIMSK0 |= _BV(OCIE0A);

    // Enable interrupts.
    sei();

    // Configure ADC
    ADMUX = _BV(REFS0) | _BV(MUX1) | _BV(MUX0);
    ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);

    for(;;);
}

