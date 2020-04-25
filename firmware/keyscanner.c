#include <avr/interrupt.h>
#include <util/atomic.h>
#include <util/delay.h>
#include <string.h>
#include "main.h"
#include DEBOUNCER
#include "wire-protocol.h"
#include "ringbuf.h"
#include "keyscanner.h"

debounce_t db[COUNT_OUTPUT];

uint8_t output_pin_map[COUNT_OUTPUT] = {PIN_ORDER_OUTPUT};


// do_scan gets set any time we should actually do a scan
volatile uint8_t do_scan = 1;


void keyscanner_set_interval(uint8_t interval) {
    OCR1A = interval;
}
uint8_t keyscanner_get_interval(void) {
    return OCR1A;
}

void keyscanner_init(void) {

    CONFIGURE_OUTPUT_PINS;

    CONFIGURE_INPUT_PINS;

    // Initialize our debouncer datastructure.
    memset(db, 0, sizeof(*db) * COUNT_OUTPUT);

    keyscanner_timer1_init();
}


void keyscanner_main(void) {
    uint8_t debounced_changes = 0;
    uint8_t pin_data;

    if (__builtin_expect(do_scan == 0, EXPECT_TRUE)) {
        return;
    }

    do_scan = 0;

    // For each enabled row...
    for (uint8_t output_pin = 0; output_pin < COUNT_OUTPUT; ++output_pin) {

        // Read pin data
        pin_data = PIN_INPUT;

        // Toggle the output we just read back off
        HIGH(PORT_OUTPUT, output_pin_map[output_pin]);

        // Toggle the output for the 'next' pin
        // We do this here to give the pin time to settle

        LOW(PORT_OUTPUT, output_pin_map[((output_pin+1) % COUNT_OUTPUT)]);

        // Debounce key state
        debounced_changes |= debounce(KEYSCANNER_CANONICALIZE_PINS(pin_data), db + output_pin);

    }

    // Most of the time there will be no new key events
    if (__builtin_expect(debounced_changes != 0, EXPECT_FALSE)) {
        keyscanner_record_state();
    }
}


inline void keyscanner_record_state (void) {

    // Snapshot the keystate to add to the ring buffer
    // Run this with interrupts off to make sure that
    // when we read from the ringbuffer, we always get
    // four bytes representing a single keyboard state.
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) ({
        for(int i =0 ; i< KEY_REPORT_SIZE_BYTES; i++) {
            ringbuf_append(db[i].state);
        }
    });

}

// initialize timer, interrupt and variable
void keyscanner_timer1_init(void) {

    // set up timer with prescaler = 256 and CTC mode
    TCCR1B |= _BV(WGM12)| _BV( CS12);

    // initialize counter
    TCNT1 = 0;

    // initialize compare value
    keyscanner_set_interval(KEYSCAN_INTERVAL_DEFAULT);

    // enable compare interrupt
    TIMSK1 |= _BV(OCIE1A);

    // enable global interrupts
    sei();
}

// interrupt service routine (ISR) for timer 1 A compare match
ISR(TIMER1_COMPA_vect) {
    do_scan = 1; // Yes! Let's do a scan
}
