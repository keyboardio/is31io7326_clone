#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include "main.h"
#include DEBOUNCER
#include "wire-protocol.h"
#include "ringbuf.h"
#include "keyscanner.h"

debounce_t db[COUNT_OUTPUT];

// do_scan_counter gets set any time we should actually do a scan
static volatile uint8_t do_scan_counter = 1;


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

    uint8_t last_scan_counter = do_scan_counter;
    if (__builtin_expect(last_scan_counter == 0, EXPECT_TRUE)) {
        return;
    }
    do_scan_counter = 0;

    // For each enabled row...
    for (uint8_t output_pin = 0; output_pin < COUNT_OUTPUT; ++output_pin) {

        REINIT_INPUT_PINS;

        // Toggle the output we want to check
        ACTIVATE_OUTPUT_PIN(output_pin);

        /* We need a no-op for synchronization. So says the datasheet
         * in Section 10.2.5 */
        asm volatile("nop\n\t");

        // Read pin data
        pin_data = PIN_INPUT;

        // Toggle the output we want to read back off
        DEACTIVATE_OUTPUT_PIN(output_pin);

        CLEANUP_INPUT_PINS;

        // Debounce key state
        debounced_changes += debounce(KEYSCANNER_CANONICALIZE_PINS(pin_data), db + output_pin);

    }

    // Most of the time there will be no new key events
    if (__builtin_expect(debounced_changes != 0, EXPECT_FALSE)) {
        RECORD_KEY_STATE;
    }
}

inline void keyscanner_record_state_rotate_ccw (void) {
    // The wire protocol expects data to be four rows of data, rather than 8 cols
    // of data. So we rotate it to match the original outputs
    uint8_t scan_data_as_rows[COUNT_OUTPUT]= {0};
    for(int i=0; i<COUNT_OUTPUT; ++i) {
        for(int j=0; j<COUNT_OUTPUT; ++j) {
            scan_data_as_rows[i] = (  ( (db[j].state & (1 << (7-i) ) ) >> (7-i) ) << j ) | scan_data_as_rows[i];
        }
    }

    DISABLE_INTERRUPTS({
        for(int i =7  ; i>= ( 8-KEY_REPORT_SIZE_BYTES); i--) {
            ringbuf_append(scan_data_as_rows[i]);
        }
    });


}

inline void keyscanner_record_state (void) {

    // Snapshot the keystate to add to the ring buffer
    // Run this with interrupts off to make sure that
    // when we read from the ringbuffer, we always get
    // four bytes representing a single keyboard state.
    DISABLE_INTERRUPTS({
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

// interrupt service routine (ISR) for timer 1 channel A compare match
ISR(TIMER1_COMPA_vect) {
    ++do_scan_counter; // Yes! Let's do a scan
}
