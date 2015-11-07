#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay_basic.h>
#include <stdint.h>

#define SERVO_SHORT_PULSE 530
#define SERVO_LONG_PULSE 2400
#define SERVO_PIN PB2

#define ESC_SHORT_PULSE 650
#define ESC_LONG_PULSE 1200
#define ESC_PIN PB4

// An 8-bit value representing the state of the servo.
// If this value is 0 the servo shaft will be turned
// all the way to one side, if this value is 255
// the shaft will be turned the other way around.
volatile uint8_t servo = 200;
volatile uint8_t esc = 200;

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

void inline
sort_delay_array(uint32_t *array) {
    // Sort the array by the amount of microseconds to delay (ascending).
    // The amount of microseconds to delay is stored in the 16
    // least significant bits.
    
    uint16_t first = array[0];
    uint16_t second = array[1];

    if(first > second) {
        uint32_t tmp = array[0];
        array[0] = array[1];
        array[1] = tmp;
    }
}

ISR(TIM0_COMPA_vect) {
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
    uint16_t servo_us_delay = SERVO_LONG_PULSE * (servo/255.0);
    uint16_t esc_us_delay = ESC_LONG_PULSE * (esc/255.0);

    uint32_t delays[2] = {
        ((uint32_t)SERVO_PIN << 16) | servo_us_delay,
        ((uint32_t)ESC_PIN << 16) | esc_us_delay
    };

    sort_delay_array(delays);

    // Start sending the pulse.
    PORTB |= _BV(SERVO_PIN) | _BV(ESC_PIN);

    uint16_t delayed = 0;

    for(int i = 0; i < 2; i++) {
        uint32_t current = delays[i];
        
        unsigned delay = ((uint16_t)current) - delayed;

        delay_microseconds(delay);
        PORTB &= ~_BV(current >> 16);

        delayed += delay;
    }
}

int main() {
    // Set the pins as output.
    DDRB |= _BV(SERVO_PIN) | _BV(ESC_PIN);

    arm_esc(); 
    
    // Enable interrupts.
    sei();
    // More specifically, enable 'Output Compare Interrupt'
    TIMSK |= _BV(4);

    // Use CTC (clear timer on compare mode)
    // this means that the timer will be reset
    // whenever the timer is equal to OCRA.
    // Since we use a prescaler of 1024 and the 
    // frequency of the chip is 8e6, it will take
    // the counter OC0A / (8e6 / 1024) seconds to
    // trigger the TIMER0_COMAP interrupt.
    // Since we choose 0C0A to be 147,
    // this time will be just less than 20 ms.
    TCCR0A = 0b00000001;
    TCCR0B = 0b00000101;
    OCR0A = 147;

    for(;;);
}

