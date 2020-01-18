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

#define debug_print(...) Serial.print(__VA_ARGS__)

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

void setup() {
    delay(2000);
    twi_init();
}

// The LEFT ATTiny has a reset pin directly connected to the ATMega
void reset_left_attiny() {
    // Hold the left ATTiny in reset,
    DDRC |= _BV(6);
    PORTC &= ~_BV(6);
    delay(30);
    DDRC &= ~_BV(6); // Turn the ATTiny back on
}

// The RIGHT ATTiny is on the other side of the wired i2c bus.
// Our best chance at resetting it is by toggling the current limiter.

void reset_right_attiny() {
    // Hold the left ATTiny in reset,
    DDRC |= _BV(7);
    PORTC &= ~_BV(7);
    delay(1000);
    PORTC |= _BV(7); // Turn the ATTiny back on
}





uint8_t read_crc16(byte addr, byte *version, uint16_t *crc16, uint16_t offset, uint16_t length) {
    uint8_t result = ENDTRANS_ADDR_NACK;


// get version and CRC16 // addr (lo) // addr (hi) // len (lo) // len (hi)
    uint8_t data[] = { (0x06), (uint8_t)(offset & 0xff), (uint8_t)(offset >> 8), (uint8_t)(length & 0xff), (uint8_t)(length >> 8) };
    result = twi_writeTo(addr, data, ELEMENTS(data), true, true);


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

    byte result = ENDTRANS_ADDR_NACK;
    while (result != ENDTRANS_SUCCESS) {
        debug_print(F("Reading CRC16: "));

        byte version;
        uint16_t crc16;
        result = read_crc16(addr, &version, &crc16, 0, firmware_length);
        debug_print_result(result);
    	debug_print(F("\r\n"));

        if (result != ENDTRANS_SUCCESS) {
            _delay_ms(100);
            continue;
        }
        debug_print(F("Version: "));
        debug_print(version);
        debug_print(F("\r\nExisting CRC16 of 0000-1FFF: "));
        debug_print(crc16, HEX);
	debug_print(F("\r\n"));
    }

}



int erase_program(uint8_t addr) {
    // erase user space
    uint8_t data[] = { 0x04 };
    uint8_t result = twi_writeTo(addr, data, ELEMENTS(data), true, true);

    debug_print(F("Erasing: "));
    debug_print_result(result);
    debug_print(F("\r\n"));
    if (result != ENDTRANS_SUCCESS) {
        _delay_ms(1000);
        debug_print(F("failed.\r\n"));
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
        result = twi_writeTo(addr, data, ELEMENTS(data), true, true);
        debug_print_result(result);
    	debug_print(F(" - "));

        _delay_ms(DELAY);
        // got something other than ACK. Start over.
        if (result != ENDTRANS_SUCCESS) {
            debug_print(F("\r\nFailed\r\n"));
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

            result = twi_writeTo(addr, data, ELEMENTS(data), true, true);
            debug_print(F(" "));
            debug_print(frame);
            debug_print(F(" of 4: "));
            // got something other than NACK. Start over.
            if (result != ENDTRANS_DATA_NACK) {
                debug_print(F("\nERROR: Got something other than NACK\n") );
                return -1;
            }
	    debug_print(F("OK;"));
            delay(DELAY);
        }
	debug_print(F("\r\n"));
        o++;
    }
    return 0;
}


int verify_firmware(byte addr) {
    byte result = ENDTRANS_DATA_NACK;
    // verify firmware
    debug_print(F("Verifying install\r\n"));
    while (result != ENDTRANS_SUCCESS) {
        debug_print(F("CRC16 "));

        byte version;
        uint16_t crc16;
        // skip the first 4 bytes, are they were probably overwritten by the reset vector preservation
        result = read_crc16(addr, &version, &crc16, offsets[0] + 4, firmware_length - 4);

        debug_print_result(result);
    	debug_print(F("\r\n"));

        if (result != ENDTRANS_SUCCESS) {
            _delay_ms(100);
            continue;
        }
        debug_print(F("Version: "));
        debug_print(version);
        debug_print(F("\r\nCRC16 of "));
        debug_print(offsets[0] + 4, HEX);
        debug_print(F("-"));
        debug_print(offsets[0] + firmware_length, HEX);
        debug_print(F(": "));
        debug_print(crc16, HEX);
	debug_print(F("\r\n"));
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
        debug_print(F("OK\r\n"));
    }
    return 0;
}

byte update_attiny(byte addr) {
    debug_print(F("Communicating\r\n"));

    get_version(addr);

    int erased = erase_program(addr);

    if (erased == -1) {

        return 0;
    }

    int firmware_written = write_firmware(addr);
    if(firmware_written == -1) {
        debug_print(F("Write failed.\r\n"));
        return 0;
    }

    int firmware_verified = verify_firmware(addr);
    if(firmware_verified == -1) {
        debug_print(F("Verify failed.\r\n"));
        return 0;
    }

    debug_print(F("Resetting ATTiny "));

    // execute app
    uint8_t data[] = {0x03, 0x00};
    uint8_t result = twi_writeTo(addr, data, ELEMENTS(data), true, true);
    debug_print_result(result);
    debug_print(F("\r\n"));
    debug_print(F("Done!\r\n"));

    return 1;
}

int left_written = 0;
int right_written = 0;

void loop() {
    delay(2000);

    if (left_written > 0) {
        debug_print(F("Done with left side.\r\n"));
        // we're done
    } else {
    	debug_print(F("Updating left side\r\n"));
        reset_left_attiny();
        left_written = update_attiny(LEFT_ADDRESS);

    }

    if (right_written > 0) {
        debug_print(F("Done with right side.\r\n"));
        // we're done
    } else {
    	debug_print(F("Updating right side\r\n"));
        reset_right_attiny();
        right_written = update_attiny(RIGHT_ADDRESS);
    }

    if (left_written && right_written  ) {
        debug_print (F("Both ATTiny MCUs have been flashed\r\nIt is now safe to reload the regular firmware\r\n"));
	delay(5000);
        return;
    }


}


