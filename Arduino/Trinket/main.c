#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/delay.h>
#include <util/delay_basic.h>
#include <stdint.h>

#define SERVO_SHORT_PULSE 500
#define SERVO_LONG_PULSE 2400
#define SERVO_SHORT_CYCLES (8 * SERVO_SHORT_PULSE)
#define SERVO_LONG_CYCLES (8 * SERVO_LONG_PULSE)
#define SERVO_PIN PB2

#define ESC_SHORT_PULSE 1000
#define ESC_LONG_PULSE 2000
#define ESC_SHORT_CYCLES (8 * ESC_SHORT_PULSE)
#define ESC_LONG_CYCLES (8 * ESC_LONG_PULSE)
#define ESC_PIN PB1

// An 8-bit value representing the state of the servo.
// If this value is 0 the servo shaft will be turned
// all the way to one side, if this value is 255
// the shaft will be turned the other way around.
volatile uint8_t servo = 200;
volatile uint8_t esc = 100;

void inline
set_on_for(unsigned iterations, uint8_t pin)
{
    PORTB |= _BV(pin);
    _delay_loop_2(iterations);
    PORTB &= ~_BV(pin);
}

void inline
pulse_for(unsigned pulse_duration, int duration, uint8_t pin)
{
    while(duration > 0) {
        _delay_ms(18);
        duration -= 18;

        set_on_for(pulse_duration * 2, pin);
    }
}

void
calibrate_esc() {
    pulse_for(1000, 5000, ESC_PIN);
    pulse_for(1500, 5000, ESC_PIN);
    pulse_for(2000, 5000, ESC_PIN);
}

void inline
sort_delay_array(uint32_t *array) {
    // Sort the array by the amount of iterations to delay (ascending).

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
    
    // So we first translate 'servo', which is a value between 0 and 255
    // to the necessary cycles.
    uint16_t servo_cycles_to_wait = SERVO_SHORT_CYCLES + (servo / 255.0) * (SERVO_LONG_CYCLES - SERVO_SHORT_CYCLES);
    uint16_t esc_cycles_to_wait = ESC_SHORT_CYCLES + (esc / 255.0) * (ESC_LONG_CYCLES - ESC_SHORT_CYCLES);

    // '_delay_loop_2' delayes the program by a number of iterations.
    // Every iteration takes four cycles, as documented on the following
    // page of the AVR Libc reference:nongnu.org/avr-libc/user-manual/group__util__delay__basic.html
    uint16_t servo_iter_to_wait = servo_cycles_to_wait / 4;
    uint16_t esc_iter_to_wait = esc_cycles_to_wait / 4;

    uint32_t delays[2] = {
        ((uint32_t)SERVO_PIN << 16) + servo_iter_to_wait,
        ((uint32_t)ESC_PIN << 16) + esc_iter_to_wait
    };

    sort_delay_array(delays);

    // Start sending the pulse.
    PORTB |= _BV(SERVO_PIN) | _BV(ESC_PIN);

    uint16_t delayed = 0;
    for(int i = 0; i < 2; i++) {
        uint32_t current = delays[i];
        
        unsigned delay = ((uint16_t)current) - delayed;

        set_on_for(delay, current >> 16); 

        delayed += delay;
    }
}

int main() {
    // Set the pins as output.
    DDRB |= _BV(SERVO_PIN) | _BV(ESC_PIN);

    calibrate_esc(); 
    
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

