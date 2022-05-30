#include <util/crc16.h>
#include "Arduino.h"
extern "C" {
#include <twi.h>
};

#include "attiny88_flasher.h"

#define ELEMENTS(arr)  (sizeof(arr) / sizeof((arr)[0]))

#define ENDTRANS_SUCCESS 0
#define ENDTRANS_DATA_TOO_LONG 1
#define ENDTRANS_ADDR_NACK 2
#define ENDTRANS_DATA_NACK 3
#define ENDTRANS_ERROR 4

#define write_cmd(addr, data, sendStop) \
    twi_writeTo((addr), (data), ELEMENTS(data), true, (sendStop))

// Reset delay, in milliseconds. Also retry interval for initial
// commands to bootloader.
//
// The ATtinys in the Model 01 ship with fuses programmed for a
// 64 ms reset delay (actually 8192 watchdog timer cycles, so
// maybe 65ms to 70ms in practice, given temperature and voltage
// variations). Also, the original Model 01 bootloader times out
// in that same period. So delay once and retry with an interval
// a little less than that, to make sure we hit the bootloader
// before it times out.
//
// (We could delay slightly longer for the initial delay, but
// we would miss the bootloader timeout if someone changed
// the fuses to a shorter startup delay.)
#define RESET_DELAY 60

#define debug_print(...) Serial.print(__VA_ARGS__)
#define debug_println(...) Serial.println(__VA_ARGS__)

void debug_print_result(uint8_t result) {
    switch(result) {
	case ENDTRANS_SUCCESS:
		debug_print(F("OK"));
		break;
	case ENDTRANS_DATA_TOO_LONG:
		debug_print(F("DATA TOO LONG"));
		break;
	case ENDTRANS_ADDR_NACK:
		debug_print(F("ADDR NACK"));
		break;
	case ENDTRANS_DATA_NACK:
		debug_print(F("NACK"));
		break;
	case ENDTRANS_ERROR:
		debug_print(F("ERROR"));
		break;
	}
}

void debug_println_result(uint8_t result) {
    debug_print_result(result);
    debug_println();
}

#define LEFT_LEDS 0x58
#define RIGHT_LEDS (LEFT_LEDS | 0x03)

void turn_leds_off(uint8_t addr) {
    uint8_t leds_off[] = { 0x03, 0x00, 0x00, 0x00 };
    (void)write_cmd(addr, leds_off, true);
}

void setup() {
    // Turn on LED (and right hand) power, in case bootloader didn't
    // do that for us (e.g., not coming straight off an ATmega flash.)
    // That lets us explicitly turn them off, in case they're lit up
    // from power cycling or bootloader status updates.
    DDRC |= _BV(7);
    PORTC |= _BV(7);
    delay(3 * RESET_DELAY);
    twi_init();

    turn_leds_off(LEFT_LEDS);
    turn_leds_off(RIGHT_LEDS);
}

// The LEFT ATTiny has a reset pin directly connected to the ATMega
void reset_left_attiny() {
    // Hold the left ATTiny in reset,
    DDRC |= _BV(6);
    PORTC &= ~_BV(6);
    delay(30);
    DDRC &= ~_BV(6); // Turn the ATTiny back on
    delay(RESET_DELAY);
}

// The RIGHT ATTiny is on the other side of the wired i2c bus.
// Our best chance at resetting it is by toggling the current limiter.

void reset_right_attiny() {
    // Hold the left ATTiny in reset,
    DDRC |= _BV(7);
    PORTC &= ~_BV(7);
    delay(1000);
    PORTC |= _BV(7); // Turn the ATTiny back on
    delay(RESET_DELAY);
    // In case power cycling caused any to turn on
    turn_leds_off(LEFT_LEDS);
}

#define PROMPT_COLOR 0x00, 0x7f, 0x7f
void prompt_trigger(uint8_t addr) {
    uint8_t left_prompt0[] = { 0x04, 0, PROMPT_COLOR };
    uint8_t left_prompt1[] = { 0x04, 30, PROMPT_COLOR };
    uint8_t right_prompt0[] = { 0x04, 1, PROMPT_COLOR };
    uint8_t right_prompt1[] = { 0x04, 31, PROMPT_COLOR };
    debug_println(F("Exceeded retry count"));
    debug_print(F("Try holding down the bootloader keys on the "));
    if (addr == LEFT_ADDRESS) {
        debug_println(F("left side"));
        (void)write_cmd(LEFT_LEDS, left_prompt0, true);
        delay(1);
        (void)write_cmd(LEFT_LEDS, left_prompt1, true);
        delay(1000);
        reset_left_attiny();
    } else {
        debug_println(F("right side"));
        (void)write_cmd(RIGHT_LEDS, right_prompt0, true);
        delay(1);
        (void)write_cmd(RIGHT_LEDS, right_prompt1, true);
        delay(1000);
        reset_right_attiny();
    }
}

void leds_success(uint8_t addr) {
    uint8_t data[] = { 0x03, 0x00, 0x3f, 0x00 };
    (void)write_cmd(addr, data, true);
}

uint8_t read_crc16(byte addr, byte *version, uint16_t *crc16, uint16_t offset, uint16_t length) {
    uint8_t result = ENDTRANS_ADDR_NACK;


    // get version and CRC16
    uint8_t data[] = {
        (0x06),
        (uint8_t)(offset & 0xff), (uint8_t)(offset >> 8), // addr
        (uint8_t)(length & 0xff), (uint8_t)(length >> 8) // len
    };

    result = write_cmd(addr, data, true);
    if (result != ENDTRANS_SUCCESS) {
        return result;
    }

    uint8_t rxBuffer[3];

    // perform blocking read into buffer
    uint8_t read = twi_readFrom(addr, rxBuffer, ELEMENTS(rxBuffer), true);
    if (read == ENDTRANS_SUCCESS) {
    }
    if (read < ENDTRANS_DATA_NACK) {
        return 0xFF;
    }
    uint8_t v = rxBuffer[0];
    *version = v;
    uint8_t crc16_lo = rxBuffer[1];
    uint8_t crc16_hi = rxBuffer[2];
    *crc16 = (crc16_hi << 8) | crc16_lo;
    return result;
}


void get_version (byte addr) {
    uint8_t retries = 0;
    byte result = ENDTRANS_ADDR_NACK;

    while (result != ENDTRANS_SUCCESS) {
        debug_print(F("Reading CRC16: "));

        byte version;
        uint16_t crc16;
        result = read_crc16(addr, &version, &crc16, 0, firmware_length);
        debug_println_result(result);

        if (result != ENDTRANS_SUCCESS) {
            delay(RESET_DELAY);
            if (++retries < 4) {
              continue;
            }
            retries = 0;
            prompt_trigger(addr);
            continue;
        }
        debug_print(F("Version: "));
        debug_println(version);
        debug_print(F("Existing CRC16 of 0000-1FFF: "));
        debug_println(crc16, HEX);
    }
}



int erase_program(uint8_t addr) {
    // erase user space
    uint8_t data[] = { 0x04 };
    uint8_t result = write_cmd(addr, data, true);

    debug_print(F("Erasing: "));
    debug_println_result(result);
    if (result != ENDTRANS_SUCCESS) {
        _delay_ms(1000);
        debug_println(F("failed."));
        return -1;
    }
    return 0;

}


int write_firmware(uint8_t addr ) {

    uint8_t result = ENDTRANS_DATA_NACK;
    uint8_t o = 0;

    for (uint16_t i = 0; i < firmware_length; i += page_size) {
        debug_print(F("Writing page "));
        debug_print(offsets[o]);
        debug_print(F(": "));

        // write page addr
        uint8_t data[] = { 0x01, (uint8_t)(offsets[o] & 0xff), (uint8_t)(offsets[o] >> 8)};
        result = write_cmd(addr, data, true);
        debug_print_result(result);
    	debug_print(F(" - "));

        _delay_ms(DELAY);
        // got something other than ACK. Start over.
        if (result != ENDTRANS_SUCCESS) {
            debug_println();
            debug_println(F("Failed"));
            return -1;
        }

        // transmit each frame separately

        debug_print(F("Frame"));
        for (uint8_t frame = 0; frame < page_size / frame_size; frame++) {
            uint8_t data_counter =0;
            uint8_t data[frame_size +4] = {0};
            data[data_counter++] = 0x2; // continue page
            uint16_t crc16 = 0xffff;
            for (uint8_t j = frame * frame_size; j < (frame + 1) * frame_size; j++) {
                if (i + j < firmware_length) {
                    uint8_t b = pgm_read_byte(&firmware[i + j]);
                    data[data_counter++] = b;
                    crc16 = _crc16_update(crc16, b);
                } else {
                    data[data_counter++] = blank;
                    crc16 = _crc16_update(crc16, blank);
                }
            }
            // write the CRC16, little end first
            data[data_counter++] = (uint8_t)(crc16 & 0xff);
            data[data_counter++] = (uint8_t)(crc16 >> 8);
            data[data_counter++] = (0x00); // dummy end uint8_t

            result = write_cmd(addr, data, true);
            debug_print(F(" "));
            debug_print(frame);
            debug_print(F(" of 4: "));
            // got something other than NACK. Start over.
            if (result != ENDTRANS_DATA_NACK) {
                debug_println();
                debug_println(F("ERROR: Got something other than NACK"));
                return -1;
            }
	    debug_print(F("OK;"));
            delay(DELAY);
        }
        debug_println();
        o++;
    }
    return 0;
}


int verify_firmware(byte addr) {
    byte result = ENDTRANS_DATA_NACK;
    // verify firmware
    debug_println(F("Verifying install"));
    while (result != ENDTRANS_SUCCESS) {
        debug_print(F("CRC16 "));

        byte version;
        uint16_t crc16;
        // skip the first 4 bytes, are they were probably overwritten by the reset vector preservation
        result = read_crc16(addr, &version, &crc16, offsets[0] + 4, firmware_length - 4);

        debug_println_result(result);

        if (result != ENDTRANS_SUCCESS) {
            _delay_ms(100);
            continue;
        }
        debug_print(F("Version: "));
        debug_println(version);
        debug_print(F("CRC16 of "));
        debug_print(offsets[0] + 4, HEX);
        debug_print(F("-"));
        debug_print(offsets[0] + firmware_length, HEX);
        debug_print(F(": "));
        debug_println(crc16, HEX);
        // calculate our own CRC16
        uint16_t check_crc16 = 0xffff;
        for (uint16_t i = 4; i < firmware_length; i++) {
            check_crc16 = _crc16_update(check_crc16, pgm_read_byte(&firmware[i]));
        }
        if (crc16 != check_crc16) {
            debug_print(F("does not match: "));
            debug_print(check_crc16, HEX);
            return -1;
        }
        debug_println(F("OK"));
    }
    return 0;
}

byte update_attiny(byte addr) {
    debug_println(F("Communicating"));

    get_version(addr);

    int erased = erase_program(addr);
    if (erased == -1) {
        return 0;
    }

    int firmware_written = write_firmware(addr);
    if (firmware_written == -1) {
        debug_println(F("Write failed."));
        return 0;
    }

    int firmware_verified = verify_firmware(addr);
    if (firmware_verified == -1) {
        debug_println(F("Verify failed."));
        return 0;
    }

    debug_print(F("Resetting ATTiny "));

    // execute app
    uint8_t data[] = {0x03, 0x00};
    uint8_t result = write_cmd(addr, data, true);
    debug_println_result(result);
    debug_println(F("Done!"));

    return 1;
}

int left_written = 0;
int right_written = 0;

void loop() {
    delay(2000);

    // Update right side first, because resetting it will power-cycle
    // LEDs on both sides.
    if (right_written > 0) {
        debug_println(F("Done with right side."));
        leds_success(RIGHT_LEDS);
        // we're done
    } else {
        debug_println(F("Updating right side"));
        reset_right_attiny();
        right_written = update_attiny(RIGHT_ADDRESS);
        delay(3 * RESET_DELAY);
        leds_success(RIGHT_LEDS);
    }

    if (left_written > 0) {
        debug_println(F("Done with left side."));
        leds_success(LEFT_LEDS);
        // we're done
    } else {
        debug_println(F("Updating left side"));
        reset_left_attiny();
        left_written = update_attiny(LEFT_ADDRESS);
        delay(3 * RESET_DELAY);
        leds_success(LEFT_LEDS);
    }

    if (left_written && right_written  ) {
        debug_println(F("Both ATTiny MCUs have been flashed"));
        debug_println(F("It is now safe to reload the regular firmware"));
	delay(5000);
        return;
    }
}
