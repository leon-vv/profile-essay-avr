#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <util/atomic.h>
#include <util/delay.h>
#include <util/delay_basic.h>

#include <stdint.h>
#include <stdbool.h>

#include "../Shared/fletcher.c"

#define SERVO_SHORT_PULSE 650
#define SERVO_LONG_PULSE 2150
#define SERVO1_PIN PB2
#define SERVO2_PIN PB3

#define ESC_SHORT_PULSE 1100
#define ESC_LONG_PULSE 1600
#define ESC_PIN PB0

// An 8-bit value representing the state of the servo.
// If this value is 0 the servo shaft will be turned
// all the way to one side, if this value is 255
// the shaft will be turned the other way around.
//
struct control_value {
    uint8_t clock;
    const uint8_t pin;
};

// The first byte represents the length of the signal.
volatile struct control_value control_values[3] = {
	{ 0, ESC_PIN },
	{ 0, SERVO1_PIN },
	{ 0, SERVO2_PIN }
};


// Calculates how often the timer has to increment given
// a control value ('throttle').
uint16_t throttle_to_clock(int low, int high, uint8_t throttle)
{
	int time = low + (high - low) * ((float)throttle / 255);
	return (time / 32);
}

void
set_control_values(uint8_t esc, uint8_t servo1, uint8_t servo2) {
	control_values[0].clock =
		throttle_to_clock(ESC_SHORT_PULSE, ESC_LONG_PULSE, esc);

	control_values[1].clock =
		throttle_to_clock(SERVO_SHORT_PULSE, SERVO_LONG_PULSE, servo1);

	control_values[2].clock =
		throttle_to_clock(SERVO_SHORT_PULSE, SERVO_LONG_PULSE, servo2);
}


volatile uint8_t counter = 0;

ISR(TIM1_COMPA_vect) {
	// Start sending a pulse.
	PORTB |= _BV(control_values[counter].pin);
}

// Called every 6.5 ms.
// This time is chosen as the two servos and the esc
// expect a pulse every 20 ms. Since we loop through
// the three components every component has to wait
// 6.5 * 3 - p where p is the pulse length.
// Since the minimum value for p is 1 ms, a component
// will never have to wait more than 6.5 * 3 - 1 = 18.5 ms.
ISR(TIM1_COMPB_vect) {

	// Stop the signal to the previous component.
	PORTB &= ~_BV(control_values[counter].pin);

	counter = (counter + 1) % 3;
	
	// Configure how long the pulse to the next
	// component will be.
	OCR1A = 203 - control_values[counter].clock;
}


// Buffer will hold the bits while they are read
// from the receiver every 2 milliseconds.
volatile uint64_t stream_buffer = 0;

// Once the predefined constant byte is found
// in 'stream_buffer'. The contents of 'stream_buffer'
// will be written to 'state_buffer'. 'state_buffer'
// is checked periodically by the 'main' function.
// If it holds a valid data packet, the values of
// 'servo1', 'servo2' and 'esc' will be updated.
volatile uint64_t state_buffer = 0;
    
// Select the byte in 'buffer' at the given index.
uint8_t inline
select_byte(uint64_t buffer, uint8_t index)
{
    return (buffer >> (8 * index)) & 0xFF;
}


// As a security measure, if the arduino does not receive
// a single valid data packet during three seconds the motor
// will be shut off and the servo's will be put in their neutral position
// (the shaft will be turned 90 degrees).
// 'invalid_count' is incremented every 18 ms or so, thus the arduino
// should take action when 'invalid_count' passes 3000 / 18 = 157.

ISR(ADC_vect) {
	PORTB &= ~_BV(PB1);

	static uint16_t consequent_fails = 0;

	// ADC holds the value of the analog-to-digital conversion.
	// If this value is larger than 500 we will read the signal
	// as 1, else as 0.
    bool bit = ADC > 500;

	// All the bits are shifted to the left.
    stream_buffer <<= 1;
    // Then the bit is written to the buffer.
    stream_buffer |= bit;

    // Check if the constant byte is present.
    if(select_byte(stream_buffer, 5) == 0b01010011) {
    	// The constant byte is present,
    	// therefore the 'stream_buffer' might hold a
    	// valid data packet. We write the value of 'stream_buffer'
    	// to 'state_buffer' such that the 'main' function can inspect
    	// the data and update the control values if necessary.
    	state_buffer = stream_buffer;
    	stream_buffer = 0;
    	consequent_fails = 0; // BUG! See below.

    	return;
    }

	// Security measure: if for longer than 3 seconds
	// no valid data packet is received, we turn off the motor
	// and turn the servos 90 degrees.
	// Note that this security measure does not work properly
	// as the code contains a bug. 'consequent_fails' is reset above
	// when a correct constant byte is received. It should have been
	// reset when a data packet is validated.
	// We can no longer fix this, as the Arduino is already in the plane
	// and thus not reachable.
    if(++consequent_fails >= 1500) {
    	set_control_values(0, 125, 125);
    	consequent_fails = 0;
    }
}

EMPTY_INTERRUPT(TIMER0_COMPA_vect);

bool
validate_data_packet(uint64_t data_packet)
{
	// First the checksums are calculated.
    uint8_t checksum[2];

    uint8_t control_values[3] = {
        select_byte(data_packet, 4),
        select_byte(data_packet, 3),
        select_byte(data_packet, 2)
    };

    compute_checksum(control_values, checksum);

	// Then the checksums are compared to the checksums
	// in the data packet.
    if(checksum[0] == select_byte(data_packet, 1)
        && checksum[1] == select_byte(data_packet, 0)) {

        // If the checksums match we turn the led on
        // and update the control values.
		set_control_values(control_values[0], control_values[1], control_values[2]);
		PORTB |= _BV(PB1);

        return true;
    }
    
    return false;
}

int main() {
    // Use 16MHz cpu clock.
    clock_prescale_set(clock_div_1);

    // Set the pins as output.
    // The PB1 pin corresponds to the built-in led.
    DDRB |= _BV(SERVO1_PIN) | _BV(SERVO2_PIN) | _BV(ESC_PIN) | _BV(PB1);

    // The pin connected to the receiver is
    // configured as input by default.

    // Setup timer 0.
    // Timer 0 is used to trigger the analog to digital
    // conversions. This analog signal is received on pin PB4,
    // which is connected to the receiver.
    TCCR0A |= _BV(WGM01);
    TCCR0B |= _BV(CS02) | _BV(CS00);

    // The analog to digital conversion will be triggered
    // when the timer register is equal to OCR0A. When this analog to digital
    // conversion finishes, the 'ADC Conversion Complete' interrupt
    // will be fired (ADC_vect).
    //
    // The conversion will happen every 2 milliseconds.
    // This value can be calculated as follows:
    // the chip runs at a frequency of 16 MHz. By using a prescaler of
    // 1024 the timer register will be increased 16e6 / 1024 = 15625 times.
    // Thus it will be equal to OCR0A every (32 / 15626) * 1000 = 2.0479 milliseconds.
    OCR0A = 32;


	// Configure timer 1.
	// Select Clear Timer on Compare Match (CTC) mode. 
	// Select a prescaler of 512.
	TCCR1 |=  _BV(CTC1) | _BV(CS13) | _BV(CS11);
	// Every 6.5 ms.
	OCR1B = OCR1C = 203;



    // Configure the ADC (Analog to Digital Converter).
    // Use 5 volts as reference voltage.
    // Read analog input voltage from pin PB4.
    ADMUX |= _BV(MUX1);

    // Enable ADC.
    // Enable automatic conversion.
    // Enable ADC Conversion Complete Interrupt.
    // Select a prescaler of 128.
    ADCSRA |= _BV(ADEN) | _BV(ADATE) | _BV(ADIE) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);

    // A compare match of timer 0 with OCR0A
    // should lead to a conversion start.
    ADCSRB |= _BV(ADTS1) | _BV(ADTS0);

    // Enable 'Output Compare Interrupt'.
    TIMSK |= _BV(OCIE0A) | _BV(OCIE1A) | _BV(OCIE1B);
 
    // Enable interrupts globally.
    sei();

	// Loop indefinitely.
	for(;;) {
		uint64_t local_buffer;
		
		// Since the size of 'state_buffer' is larger than 8 bits,
		// it needs to be accessed atomically to ensure that it is not
		// mutated by the ADC interrupt handler during the access.
		// While the statements in the atomic block are run all interrupts
		// will be disabled.
		ATOMIC_BLOCK(ATOMIC_FORCEON) {
			local_buffer = state_buffer;
		}

		if(local_buffer != 0) {
			validate_data_packet(local_buffer);

			ATOMIC_BLOCK(ATOMIC_FORCEON) {
				state_buffer = 0;
			}
		}
	}
}

