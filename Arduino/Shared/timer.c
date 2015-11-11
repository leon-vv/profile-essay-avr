
void
setup_timer_interrupt(uint8_t compare_value)
{
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
    OCR0A = compare_value;
}
