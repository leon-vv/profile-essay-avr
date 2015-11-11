#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay_basic.h>
#include <stdint.h>

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
    _delay_loop_2(2 * us);
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
volatile uint64_t buffer = 0;
// As a security measure, if no valid data packet
// (correct constant byte and a correct checksum)
// is read, after 3 seconds the motor is turned off
// and the shaft of the servos are placed in the middle.
// Since every 5 ms the receiver is read, after
// 3000 / 5 = 600 consequent fails the 3 seconds have passed.
volatile uint16_t consequent_fails = 0;
    
uint8_t inline
select_byte(uint64_t buffer, uint8_t index)
{
    return (buffer >> (8 * index)) & 0xFF;
}

ISR(ADC_vect) {
    bool bit = ADC > 750;

    // The least significant bit is set corresponding
    // to whether the receiver is reading a signal.
    buffer <<= 1;
    buffer |= bit;

    // Check if the constant byte is present.
    if(select_byte(buffer, 5) == 0b01010011) {
        // Error detection: the checksums should match.
        uint8_t checksum[2];
        uint8_t control_values[3] = {
            select_byte(buffer, 4),
            select_byte(buffer, 3),
            select_byte(buffer, 2)
        };

        compute_checksum(control_values, checksum);

        if(checksum[0] == select_byte(buffer, 1)
            && checksum[1] == select_byte(buffer, 0)) {
            // Checksums match!
            esc = control_values[0];
            servo1 = control_values[1];
            servo2 = control_values[2];

            consequent_fails = 0;
        }
        else if(consequent_fails >= 600) {
            esc = 0;
            servo1 = 125;
            servo2 = 125;
        }
    }
}

/*
void inline
pulse_for(unsigned pulse_duration_us, int duration_ms, uint8_t pin)
{
    while(duration_ms > 0) {
        delay_milliseconds(18);
        duration_ms -= 18;

        PORTB |= _BV(pin);
        delay_microseconds(pulse_duration_us);
        PORTB &= ~_BV(pin);
    }
}

void
arm_esc() {
    // The ESC will only turn the motor on
    // if it receives short pulses at the start.
    // I think this is a security measure.
    pulse_for(ESC_SHORT_PULSE, 4000, ESC_PIN);
}
*/

struct pulse {
    uint16_t delay;
    uint8_t pin;
};

void inline
sort_pulses(struct pulse pulses[3]) {
    // Sort the array by the amount of microseconds to delay (ascending).
    // This function uses the 'bubble sort' algorithm.
    
    for(int i = 0; i < 2; i++) {
        // Two iterations
        for(int j = 0; j < 2; j++) {
            if(pulse[j].delay > pulse[j + 1].delay) {
                struct pulse tmp = pulse[j];
                pulse[j] = pulse[j + 1];
                pulse[j + 1] = tmp;
            }
        }
    }
}

uint16_t inline
map(uint16_t high, uint16_t low, uint8_t throttle)
{
    return low + (high * (throttle / 255.0));
}

ISR(TIM1_COMPA_vect) {
    // This interrupt will be triggered slightly more often than
    // every 20 ms. During this interrupt we send a control pulse
    // to the servo's and the ESC (which controls the motor).

    // A control pulse takes between 'SERVO_SHORT_PULSE' microseconds (off)
    // and 'SERVO_LONG_PULSE' microseconds (on). Since our chip runs at
    // 8 MHz we should wait between:
    // SERVO_LONG_CYCLES = (SERVO_LONG_PULSE / 10^6) * 8 * 10^6 = 8 * SERVO_LONG_PULSE and
    // SERVO_SHORT_CYCLES = (SERVO_SHORT_PULSE / 10^6) * 8 * 10^6 = 8 * SERVO_SHORT_PULSE cpu cycles.
    
    // We first translate the control variables 'esc' and 'servo'
    // whose values are between 0 adn 255 to the amount of microseconds to wait.

    struct pulse pulses[3] = {
        { map(SERVO_LONG_PULSE, SERVO_SHORT_PULSE, servo1), SERVO1_PIN },
        { map(SERVO_LONG_PULSE, SERVO_SHORT_PULSE, servo2), SERVO2_PIN },
        { map(ESC_LONG_PULSE, ESC_SHORT_PULSE, esc), ESC_PIN }
    };

    sort_pulses(pulses);

    // Start sending the pulse.
    PORTB |= _BV(SERVO_PIN) | _BV(SERVO1_PIN) | _BV(ESC_PIN);

    uint16_t delayed = 0;

    for(int i = 0; i < 3; i++) {
        struct pulse current = pulses[i];

        delay_microseconds(pulse.delay);

        PORTB &= ~_BV(pulse.pin);

        delayed += delay;
    }
}

int main() {

    // Set the pins as output.
    DDRB |= _BV(SERVO_PIN) | _BV(SERVO1_PIN) | _BV(ESC_PIN);

    // Enable interrupts.
    sei();

    // More specifically, enable 'Output Compare Interrupt'
    TIMSK |= _BV(4) | _BV(6);

    // Setup timer 0.
    // Timer 0 is used to trigger the analog to digital
    // conversions. This analog signal is received on pin PB4,
    // which is connected to the receiver.
    TCCR0A = 0b00000001;
    TCCR0B = 0b00000101;
    // The analog to digital conversion will be triggered
    // when timer 0 is equal to OCR0A. When this analog to digital
    // conversion finishes, the 'ADC Conversion Complete' interrupt
    // will be fired. Note that this value is precisely half of the
    // corresponding value on the Arduino Nano that is connected to the
    // laptop. This is because the CPU frequency of the Arduino Trinket
    // is half of the CPU frequency of the Arduino Nano.
    OCROA = 39;

    // Setup timer 1.
    // Timer 1 is used to trigger an interrupt slightly more often than every 20 ms.
    // When this interrupt fires the servos and the ESC will receive a control pulse.
    
    // Select Clear Timer on Compare Match (CTC) mode. 
    // Select a prescaler of 2048.
    TCCR1 = 0b10001100;
    TCCR1B = 0b00000101;
    OCR1A = OCR1C = 147;

    // Configure the ADC (Analog to Digital Converter).
    
    // Use 5 volts as reference voltage.
    // Read analog input voltage from pin PB4.
    ADMUX = 0b00000010;
    // Enable ADC.
    // Enable automatic conversion.
    // Enable ADC Conversion Complete Interrupt.
    // Select a prescaler of 128.
    ADCSRA = 0b00111111;
    // A compare match of timer 0 with OCR0B
    // should lead to a conversion start.
    ADCSRB = 0b00000101;
    OCR0B = 39;
    
    for(;;);
}

