
// All state machine configs use the same guard -- we only want one at a time.
#ifndef _DEBOUNCER_STATE_MACHINE_CONFIG_H
#define _DEBOUNCER_STATE_MACHINE_CONFIG_H

enum lifecycle_phases {
    OFF, TURNING_ON, LOCKED_ON, ON, TURNING_OFF, LOCKED_OFF,
    NOISY_SWITCH_OFF, NOISY_SWITCH_TURNING_ON, NOISY_SWITCH_LOCKED_ON, NOISY_SWITCH_ON, NOISY_SWITCH_TURNING_OFF, NOISY_SWITCH_LOCKED_OFF


};

lifecycle_phase_t lifecycle[] = {
    {
        // OFF -- during this phase, any 'off' value means that we should keep this key pressed
        // A single 'on' value means that we should start checking to see if it's really a key press
        //
        // IF we get an 'on' value, change the phase to 'TURNING_ON' to make sure it's not just
        // chatter
        //
        // Our timers are set to 0, but that doesn't matter because in the event that we overflow the timer
        // we just go back to the 'OFF' phase

        .next_phase = OFF,
        .expected_data = 0,
        .unexpected_data_phase = TURNING_ON,
        .change_output_on_expected_transition = 0,
        .timer = 1,
    },
    {
        // TURNING_ON-- during this phase, we believe that we've detected
        // a switch being turned on. We're now checking to see if it's
        // reading consistently as 'on' or if it was just a spurious "on" signal
        // as might happen if we saw key chatter
        //
        // If it was a spurious disconnection, mark the switch as noisy and go back to phase OFF
        //
        // If we get through the timer with no "off" signals, proceed to phase LOCKED_ON

        .next_phase = LOCKED_ON,
        .expected_data = 1,
        .unexpected_data_phase = OFF,
        .change_output_on_expected_transition = 0,
        .timer = 1,
    },
    {
        // LOCKED_ON -- during this phase, the key is on, no matter what value we read from the input
        // pin.
        //
        // If we see any 'off' signals, that indicates a short read or chatter.
        // In the event of unexpected data, stay in the LOCKED_ON phase, but don't reset the timer.

        .next_phase = ON,
        .expected_data = 1,
        .unexpected_data_phase = NOISY_SWITCH_LOCKED_ON,
        .change_output_on_expected_transition = CHANGE_OUTPUT,
        .timer = 10,
    },
    {
        // ON -- during this phase, any 'on' value means that we should keep this key pressed
        // A single 'off' value means that we should start checking to see if it's really a key release
        //
        // IF we get an 'off' value, change the phase to 'TURNING_OFF' to make sure it's not just
        // chatter
        //
        // Our timers are set to 0, but that doesn't matter because in the event that we overflow the timer
        // we just go back to the 'ON' phase
        .next_phase = ON,
        .expected_data = 1,
        .unexpected_data_phase = TURNING_OFF,
        .change_output_on_expected_transition = 0,
        .timer = 1,
    },
    {
        // TURNING_OFF -- during this phase, we believe that we've detected
        // a switch being turned off. We're now checking to see if it's
        // reading consistently as 'off' or if it was just a spurious "off" signal
        // as might happen if we saw key chatter
        //
        // If it was a spurious connection, mark the switch as noisy and go back to phase ON
        //
        // If we get through the timer with no "on" signals, proceed to phase LOCKED_OFF
        .next_phase = LOCKED_OFF,
        .expected_data = 0,
        .unexpected_data_phase = NOISY_SWITCH_ON,
        .change_output_on_expected_transition = 0,
        .timer = 16,  // release latency
    },
    {
        // LOCKED_OFF -- during this phase, the key is off, no matter what value we read from the input
        // pin.
        //
        // If we see any 'on' signals, that indicates a short read or chatter.
        // In the event of unexpected data, stay in the LOCKED_OFF phase, but don't reset the timer.
        .next_phase = OFF,
        .expected_data = 0,
        .unexpected_data_phase =  LOCKED_OFF,
        .change_output_on_expected_transition = CHANGE_OUTPUT,
        .timer = 1,
    },
    {
        // NOISY_SWITCH_OFF -- during this phase, any 'off' value means that we should keep this key pressed
        // A single 'on' value means that we should start checking to see if it's really a key press
        //
        // IF we get an 'on' value, change the phase to 'NOISY_SWITCH_TURNING_ON' to make sure it's not just
        // chatter
        //
        // Our timers are set to 0, but that doesn't matter because in the event that we overflow the timer
        // we just go back to the 'NOISY_SWITCH_OFF' phase

        .next_phase = NOISY_SWITCH_OFF,
        .expected_data = 0,
        .unexpected_data_phase = NOISY_SWITCH_TURNING_ON,
        .change_output_on_expected_transition = 0,
        .timer =1
    },
    {
        // NOISY_SWITCH_TURNING_ON-- during this phase, we believe that we've detected
        // a switch being turned on. We're now checking to see if it's
        // reading consistently as 'on' or if it was just a spurious "on" signal
        // as might happen if we saw key chatter
        //
        // If it was a spurious disconnection, mark the switch as noisy and go back to phase NOISY_SWITCH_OFF
        //
        // If we get through the timer with no "off" signals, proceed to phase NOISY_SWITCH_LOCKED_ON

        .next_phase = NOISY_SWITCH_LOCKED_ON,
        .expected_data = 1,
        .unexpected_data_phase = NOISY_SWITCH_OFF,
        .change_output_on_expected_transition = 0,
        .timer = 1
    },
    {
        // NOISY_SWITCH_LOCKED_ON -- during this phase, the key is on, no matter what value we read from the input
        // pin.
        //
        // If we see any 'off' signals, that indicates a short read or chatter.
        // In the event of unexpected data, stay in the NOISY_SWITCH_LOCKED_ON phase, but don't reset the timer.

        .next_phase = NOISY_SWITCH_ON,
        .expected_data = 1,
        .unexpected_data_phase = NOISY_SWITCH_LOCKED_ON,
        .change_output_on_expected_transition = CHANGE_OUTPUT,
        .timer = 100
    },
    {
        // NOISY_SWITCH_ON -- during this phase, any 'on' value means that we should keep this key pressed
        // A single 'off' value means that we should start checking to see if it's really a key release
        //
        // IF we get an 'off' value, change the phase to 'NOISY_SWITCH_TURNING_OFF' to make sure it's not just
        // chatter
        //
        // Our timers are set to 0, but that doesn't matter because in the event that we overflow the timer
        // we just go back to the 'ON' phase
        .next_phase = NOISY_SWITCH_ON,
        .expected_data = 1,
        .unexpected_data_phase = NOISY_SWITCH_TURNING_OFF,
        .change_output_on_expected_transition = 0,
        .timer = 1
    },
    {
        // NOISY_SWITCH_TURNING_OFF -- during this phase, we believe that we've detected
        // a switch being turned off. We're now checking to see if it's
        // reading consistently as 'off' or if it was just a spurious "off" signal
        // as might happen if we saw key chatter
        //
        // If it was a spurious connection, mark the switch as noisy and go back to phase ON
        //
        // If we get through the timer with no "on" signals, proceed to phase LOCKED_OFF
        .next_phase = NOISY_SWITCH_LOCKED_OFF,
        .expected_data = 0,
        .unexpected_data_phase = NOISY_SWITCH_ON,
        .change_output_on_expected_transition = 0,
        .timer =  59// release latency
    },
    {
        // NOISY_SWITCH_LOCKED_OFF -- during this phase, the key is off, no matter what value we read from the input
        // pin.
        //
        // If we see any 'on' signals, that indicates a short read or chatter.
        // In the event of unexpected data, stay in the LOCKED_OFF phase, but don't reset the timer.
        .next_phase = NOISY_SWITCH_OFF,
        .expected_data = 0,
        .unexpected_data_phase =  NOISY_SWITCH_LOCKED_OFF,
        .change_output_on_expected_transition = CHANGE_OUTPUT,
        .timer = 1
    }
};

#endif
