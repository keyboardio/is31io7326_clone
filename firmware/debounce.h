#pragma once

#include <stdint.h>

/*
each of these 8 bit variables are storing the state for 8 keys

so for key 0, the counter is represented by db0[0] and db1[0] 
and the state in state[0].
*/
typedef struct {
    uint8_t db0;    // counter bit 0
    uint8_t db1;    // counter bit 1
    uint8_t state;  // debounced state
} debounce_t;

/**
 * debounce --
 *    The debouncer is based on a stacked counter implementation, with each bit
 *    getting its own 2-bit counter. When a bit changes, a call to debounce
 *    will increment that bit's counter. When it overflows, the change is
 *    comitted to the final debounced state and the changed bit returned.
 *
 *    Because each key's counter and state are stored in this stacked way,
 *    8 keys are processed in parallel at each operation.
 *
 * args:
 *    sample - the current state
 *    debouncer - the state variables of the debouncer
 *
 * returns: bits that have changed in the final debounced state
 *
 * handy XOR truth table:   A   B   O
 *                          0 ^ 0 = 0
 *                          0 ^ 1 = 1
 *                          1 ^ 0 = 1
 *                          1 ^ 1 = 0
 * 
 * This is used below as a difference detector:
 *   if A ^ B is true, A and B are different.
 *
 * And a way to flip selected bits in a variable or register:
 *   Set B to 1, then A ^ B = !A
 */
static inline uint8_t debounce(uint8_t sample, debounce_t *debouncer) {
    uint8_t delta, changes;

    // Use xor to detect changes from last stable state:
    // if a key has changed, it's bit will be 1, otherwise 0
    delta = sample ^ debouncer->state;

    // Increment counters and reset any unchanged bits:
    // increment bit 1 for all changed keys
    debouncer->db1 = ((debouncer->db1) ^ (debouncer->db0)) & delta;
    // increment bit 0 for all changed keys
    debouncer->db0 = ~(debouncer->db0) & delta;

    // Calculate returned change set: if delta is still true
    // and the counter has wrapped back to 0, the key is changed.
    changes = delta & ~debouncer->db0 & ~debouncer->db1;

    // Update state: in this case use xor to flip any bit that is true in changes.
    debouncer->state ^= changes;

    return changes;
}
