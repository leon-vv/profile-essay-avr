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

#define SERVO_SHORT_PULSE 1000
#define SERVO_LONG_PULSE 2000
#define SERVO1_PIN PB2
#define SERVO2_PIN PB3

#define ESC_SHORT_PULSE 500
#define ESC_LONG_PULSE 1000
#define ESC_PIN PB0

int inline
delay_microseconds(unsigned us)
{
	// Since our chip runs at
    // 16 MHz we can convert a number of microseconds to the number of cpu cycles
    // as follows:
    // (time / 10^6) * 16 * 10^6 = 16 * time.
    // The '_delay_loop_2' function takes 4 cpu cycles per iteration.
    // Thus we should delay for (16 * time) / 4 = 4 * times iterations.

    _delay_loop_2(4 * us);
    return us;
}

// An 8-bit value representing the state of the servo.
// If this value is 0 the servo shaft will be turned
// all the way to one side, if this value is 255
// the shaft will be turned the other way around.
volatile uint8_t servo1 = 0;
volatile uint8_t servo2 = 0;
// Similarly, if 'esc' is 0 the motor will be turned off.
// If 'esc' is 255 the motor will be turning very fast.
volatile uint8_t esc = 0;
// The three 8-bit values above are called the "control values".


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
    
uint8_t inline
select_byte(uint64_t buffer, uint8_t index)
{
    return (buffer >> (8 * index)) & 0xFF;
}

ISR(ADC_vect) {
	// ADC holds the value of the analog-to-digital conversion.
	// If this value is larger than 500 we will read the signal
	// as 1, else as 0.
    bool bit = ADC > 500;

	// All the bits are shifted to the left.
    stream_buffer <<= 1;
    // Then the bit is written to the buffer.
    stream_buffer |= bit;

	// Turn the led off.
    PORTB &= ~_BV(PB1);

    // Check if the constant byte is present.
    if(select_byte(stream_buffer, 5) == 0b01010011) {
    	// The constant byte is present,
    	// therefore the 'stream_buffer' might hold a
    	// valid data packet. We write the value of 'stream_buffer'
    	// to 'state_buffer' such that the 'main' function can inspect
    	// the data and update the control values if necessary.
    	state_buffer = stream_buffer;
    	stream_buffer = 0;
    }
}

bool
update_control_values(uint64_t data_packet)
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

		// Turn the led on.
        PORTB |= _BV(PB1);

        esc = control_values[0];
        servo1 = control_values[1];
        servo2 = control_values[2];

        return true;
    }
    
    return false;
}

struct pulse {
    uint16_t delay;
    uint8_t pin;
};

uint16_t inline
map(uint16_t high, uint16_t low, uint8_t throttle)
{
    return low + (high * (throttle / 255.0));
}


EMPTY_INTERRUPT(TIMER0_COMPA_vect);

int
send_pulses() {
	// This function has the responsibility of sending a control pulse
	// to the servo's and the ESC.

    // A control pulse takes between 'SERVO_SHORT_PULSE' microseconds (off)
    // and 'SERVO_LONG_PULSE' microseconds (on).     // Therefore, the cpu should wait between
    // 16 * SERVO_LONG_PULSE and 16 * SERVO_SHORT_PULSE cycles.
    
    // We first translate the control variables 'esc' and 'servo'
    // whose values are between 0 and 255 to the amount of microseconds to wait.
    struct pulse pulses[3] = {
        { map(SERVO_LONG_PULSE, SERVO_SHORT_PULSE, servo1), SERVO1_PIN },
        { map(SERVO_LONG_PULSE, SERVO_SHORT_PULSE, servo2), SERVO2_PIN },
        { map(ESC_LONG_PULSE, ESC_SHORT_PULSE, esc), ESC_PIN }
    };


	int delayed = 0;

	for(int i = 0; i < 3; i++) {
		struct pulse current_pulse = pulses[i];

		// Start sending the pulse.
		PORTB |= _BV(current_pulse.pin);
		
		delayed += delay_microseconds(current_pulse.delay);

		// Stop sending the pulse.
		PORTB &= ~_BV(current_pulse.pin);
	}

	return delayed;
}

int main() {
    _delay_ms(2000);

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
    TIMSK |= _BV(OCIE0A);
 
    // Enable interrupts globally.
    sei();

	int invalid_count = 0;

	// Loop indefinitely.
	for(;;) {
		uint64_t local_buffer;
		
		// Since the size of 'state_buffer' is larger than 8 bits,
		// it needs to be accessed atomically to ensure that it is not
		// mutated by the ADC interrupt handler during the access.
		// While the statements in the atomic block are run interrupts
		// will be disabled.
		ATOMIC_BLOCK(ATOMIC_FORCEON) {
			local_buffer = state_buffer;
		}

		if(local_buffer != 0) {
			bool valid = update_control_values(local_buffer);

			if(valid) invalid_count = 0;

			ATOMIC_BLOCK(ATOMIC_FORCEON) {
				state_buffer = 0;		
			}
		}

		// As a security measure, if the arduino does not receive
		// a single valid data packet during three seconds the motor
		// will be shut off and the servo's will be put in their neutral position
		// (the shaft will be turned 90 degrees).
		// 'invalid_count' is incremented every 18 ms or so, thus the arduino
		// should take action when 'invalid_count' passes 3000 / 18 = 157.
		if(++invalid_count > 157) {
			esc = 0;	
			servo1 = 125;
			servo2 = 125;
			invalid_count = 0;
		}

		int delayed = send_pulses();

		delay_microseconds(18000 - delayed);
	}
}

