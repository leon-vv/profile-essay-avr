#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

// An 8-bit value representing the state of the servo.
// If this value is 0 the servo shaft will be turned
// all the way to one side, if this value is 255
// the shaft will be turned the other way around.
volatile uint8_t servo = 125;

int main() {
    // Enable output on the PB2 and PB1 pins.
    DDRB |= _BV(PB2) | _BV(PB1);

    // Use CTC (clear timer on compare mode)
    // this means that the timer will be reset
    // whenever the timer is equal to OCRA.
    // Since we use a prescaler of 1024 and the 
    // frequency of the chip is 8e6, it will take
    // the counter OC0A / (8e6 / 1024) seconds to
    // trigger the TIMER0_COMAP interrupt.
    // Since we choose 0C0A to be 147,
    // this time will be just less than 20 ms.
    // It is slightly less on purpose:
    // we need te remaining time to actually send the pulses.
    TCCR0A = 0b00000001;
    TCCR0B = 0b00000101;
    OC0A = 147;
}

// Dynamic delay.
// Since the avr libc _delay_ms() and _delay_us() functions
// simply insert a number of irrelevant instructions to delay
// the execution of the program, the argument to these functions
// need to be a compile time constant. 
// This function divides the needed delay in smaller blocks and delays
// the program repeatedly by these blocks.
// Because the checks and the loops take some time, this delay function
// will not be very precise.
void delay(uint16_t microseconds) {

    // I love optimizing!
    // This is an array containing addresses.
    // These address correspond to the labels below.
    // Notice that the numbers are powers of two,
    // this is because they correspond to the individual bits
    // of 'microseconds'.
    const void *jump_table[14] = {
        &&wait32768,
        &&wait16384,
        &&wait8192,
        &&wait4096,
        &&wait2048,
        &&wait1024,
        &&wait512,
        &&wait256,
        &&wait128,
        &&wait64,
        &&wait32,
        &&wait16,
        &&wait8,
        &&wait4
    };
        

    // 3 is used instead of 0, since the overhead of the function
    // is likely greater than 3 microseconds. So trying to delay the last
    // three microseconds would just make this function more inaccurate.
    // Notice that the value 3 corresponds to binary number 11. So by skipping
    // the last three microseconds we also skip the last two bits, hence the jump_table
    // contains 14 addresses while 'microseconds' is a 16 bit number.
check:
    if(microseconds <= 3) return;
    // Count the number of leading zeros.
    // For example, the 8 bit value 00010010 has three leading zeros.
    // Notice that the number of leading zeros corresponds to the index
    // of the first 1 bit (an index starts at zero).
    int index = __bultin_clz(microseconds);
    
    // Credit goes to http://stackoverflow.com/a/6011206/1883402
    // for the following line.
    microseconds &= ~(0x800 >> index);
    goto jump_table[index];

wait32768:
    _delay_us(32768);
    goto check;
wait16384:
    _delay_us(16384);
    goto check;
wait8192:
    _delay_us(8192);
    goto check;
wait4096:
    _delay_us(4096);
    goto check;
wait2048:
    _delay_us(2048);
    goto check;
wait1024:
    _delay_us(1024);
    goto check;
wait512:
    _delay_us(512);
    goto check;
wait256:
    _delay_us(256);
    goto check;
wait128:
    _delay_us(128);
    goto check;
wait64:
    _delay_us(64);
    goto check;
wait32:
    _delay_us(32);
    goto check;
wait16:
    _delay_us(16);
    goto check;
wait8:
    _delay_us(8);
    goto check;
wait4:
    _delay_us(4);
    goto check;
}

ISR(TIMER0_COMPA) {
    // This interrupt will be triggered slightly more often than
    // every 20 ms.
    
}
