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

#define SERVO_SHORT_PULSE 530
#define SERVO_LONG_PULSE 2400
#define SERVO1_PIN PB2
#define SERVO2_PIN PB3

#define ESC_SHORT_PULSE 650
#define ESC_LONG_PULSE 1200
#define ESC_PIN PB0

#define RECEIVER_PIN PB3

void inline
delay_microseconds(unsigned us)
{
	// 4 cycles per iteration
	// 4 / 16e6 seconds per iteration
	// 4 / 16 microseconds per iteration
	// 4 iterations per microsecond
    _delay_loop_2(4 * us);
}

void inline
delay_milliseconds(unsigned ms)
{
    delay_microseconds(ms * 1000);
}

// An 8-bit value representing the state of the servo.
// If this value is 0 the servo shaft will be turned
// all the way to one side, if this value is 255
// the shaft will be turned the other way around.
volatile uint8_t servo1 = 0;
volatile uint8_t servo2 = 0;
volatile uint8_t esc = 0;

// Buffer will hold the bits as they are read in
// from the receiver.
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

    bool bit = ADC > 500;

    // The least significant bit is set corresponding
    // to whether the receiver is reading a signal.
    stream_buffer <<= 1;
    stream_buffer |= bit;

    PORTB &= ~_BV(PB1);

    // Check if the constant byte is present.
    if(select_byte(stream_buffer, 5) != 0b01010011) {
    	state_buffer = stream_buffer;
    	stream_buffer = 0;
    }
}

bool
update_control_values(uint64_t data_packet)
{
    // Error detection: the checksums should match.
    uint8_t checksum[2];

    uint8_t control_values[3] = {
        select_byte(data_packet, 4),
        select_byte(data_packet, 3),
        select_byte(data_packet, 2)
    };

    compute_checksum(control_values, checksum);

    if(checksum[0] == select_byte(data_packet, 1)
        && checksum[1] == select_byte(data_packet, 0)) {
        // Checksums match!

        PORTB |= _BV(PB1);

        esc = control_values[0];
        servo1 = control_values[1];
        servo2 = control_values[2];

        return true;
    }
    else return false;
}

struct pulse {
    uint16_t delay;
    uint8_t pin;
};

void inline
sort_pulses(struct pulse pulses[3]) {
    // Sort the array by the amount of microseconds to delay (ascending).
    // This function uses the 'bubble sort' algorithm.
    
    for(int i = 0; i < 2; i++) {
        for(int j = 0; j < 2; j++) {
            if(pulses[j].delay > pulses[j + 1].delay) {
                struct pulse tmp = pulses[j];
                pulses[j] = pulses[j + 1];
                pulses[j + 1] = tmp;
            }
        }
    }
}

uint16_t inline
map(uint16_t high, uint16_t low, uint8_t throttle)
{
    return low + (high * (throttle / 255.0));
}


EMPTY_INTERRUPT(TIMER0_COMPA_vect);

//ISR(TIMER1_COMPA_vect) {
void send_pulses() {
	// This function has the responsibility of sending a control pulse
	// to the servo's and the ESC.

    // A control pulse takes between 'SERVO_SHORT_PULSE' microseconds (off)
    // and 'SERVO_LONG_PULSE' microseconds (on). Since our chip runs at
    // 16 MHz we should wait between:
    // SERVO_LONG_CYCLES = (SERVO_LONG_PULSE / 10^6) * 16 * 10^6 = 16 * SERVO_LONG_PULSE and
    // SERVO_SHORT_CYCLES = (SERVO_SHORT_PULSE / 10^6) * 16 * 10^6 = 16 * SERVO_SHORT_PULSE cpu cycles.
    
    // We first translate the control variables 'esc' and 'servo'
    // whose values are between 0 and 255 to the amount of microseconds to wait.

    struct pulse pulses[3] = {
        { map(SERVO_LONG_PULSE, SERVO_SHORT_PULSE, servo1), SERVO1_PIN },
        { map(SERVO_LONG_PULSE, SERVO_SHORT_PULSE, servo2), SERVO2_PIN },
        { map(ESC_LONG_PULSE, ESC_SHORT_PULSE, esc), ESC_PIN }
    };

    sort_pulses(pulses);

    // Start sending the pulse.
    PORTB |= _BV(SERVO1_PIN) | _BV(SERVO2_PIN) | _BV(ESC_PIN);

    uint16_t delayed = 0;

    for(int i = 0; i < 3; i++) {
        struct pulse p = pulses[i];

		int to_delay = p.delay - delayed;

        delay_microseconds(to_delay);

        PORTB &= ~_BV(p.pin);

        delayed += to_delay;
    }
}

int main() {
    _delay_ms(2000);

    // Use 16MHz cpu clock.
    clock_prescale_set(clock_div_1);

    // Set the pins as output.
    DDRB |= _BV(SERVO1_PIN) | _BV(SERVO2_PIN) | _BV(ESC_PIN) | _BV(PB1);

    // The pin connected to the receiver is
    // configured as input by default.

    // Setup timer 0.
    // Timer 0 is used to trigger the analog to digital
    // conversions. This analog signal is received on pin PB4,
    // which is connected to the receiver.
    TCCR0A |= _BV(WGM01);
    TCCR0B |= _BV(CS02) | _BV(CS00);
    OCR0A = 16;

    // The analog to digital conversion will be triggered
    // when timer 0 is equal to OCR0A. When this analog to digital
    // conversion finishes, the 'ADC Conversion Complete' interrupt
    // will be fired.


/*
    // Setup timer 1.
    // Timer 1 is used to trigger an interrupt slightly more often than every 20 ms.
    // When this interrupt fires the servos and the ESC will receive a control pulse.
    
    // Select Clear Timer on Compare Match (CTC) mode. 
    // Select a prescaler of 2048.
    TCCR1 |= _BV(CTC1) | _BV(CS13) | _BV(CS12);
    OCR1A = OCR1C = 147;
   */


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

    // More specifically, enable 'Output Compare Interrupt'.
    TIMSK |= _BV(OCIE0A);
 
    // Enable interrupts.
    sei();

	int invalid_count = 0;

	for(;;) {
	
		uint64_t local_buffer;
		
		// Since the size of 'state_buffer' is larger than 8 bits,
		// it needs to be accessed atomically to ensure that it is not
		// mutated by the ADC interrupt handler during the access.
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
		// 'invalid_count' is incremented every 19 ms or so, thus the arduino
		// should take action when 'invalid_count' passes 3000 / 19 = 158.
		if(invalid_count > 158) {
			esc = 0;	
			servo1 = 125;
			servo2 = 125;
		}
		invalid_count += 1;

		send_pulses();
		_delay_ms(17);
	}
}

