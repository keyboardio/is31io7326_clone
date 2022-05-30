/*
 * Copyright (c) 2004, Atmel Corporation
 *
 * The sourcecode was orginally from Atmel Application Note
 * -- AVR311: TWI Slave Implementation --
 *
 * Changes to make it compile with the GNU C Compiler with the avr-libc were made by Bernhard Walle
 * Further changes made by Scott Perry
 *
 * -------------------------------------------------------------------------------------------------
 */
#include <avr/interrupt.h>
#include <util/twi.h>
#include "twi-slave.h"
#include "main.h"

static unsigned char TWI_buf[TWI_BUFFER_SIZE]; // Transceiver buffer. Set the size in the header file
static unsigned char TWI_msgSize  = 0;         // Number of bytes to be transmitted.

void (*TWI_Tx_Data_Callback)( unsigned char *, unsigned char * );
void (*TWI_Rx_Data_Callback)( unsigned char *, unsigned char );

/**
 * TWI flags explained:
 *
 * TWINT: TWI Interrupt Flag. It is set when the TWI finishes ANY bus operation and has to cleared (by writing a 1 to it) before a new operation can be started.
 * TWEA: TWI Enable Acknowledge; When the device receives data (as slave or as master), this bit has to be set if the next incoming byte should be ACKed and cleared for a NACK.
 * TWSTA: TWI Start; When a master has to generate a start condition, write this bit 1 together with TWEN and TWINT. The TWI hardware will generate a start condition and return the appropriate status code.
 * TWSTO: TWI Stop; Similar to TWSTA, but generates a Stop condition on the bus. TWINT is not set after generating a Stop condition.
 * TWWC: TWI Write Collision; Set by the TWI hardware when writing to the TWI Data Register TWDR while TWINT is high.
 * TWEN: Any bus operation only takes place when TWEN is written to 1 when accessing TWCR.
 * TWIE: TWI Interrupt Enable; If this bit is set, the CPU will jump to the TWI reset vector when a TWI interrupt occurs.
 * ---------------------------------------------------------------------------------------------- */

/**
 * Call this function to enable the TWI Tranceiver with the next ACK response.
 * ---------------------------------------------------------------------------------------------- */
static void TWI_Start_Transceiver( unsigned char ack ) {
    if (ack) {
        TWCR = _BV(TWEN)|            // Enable TWI-interface and release TWI pins.
               _BV(TWIE)|_BV(TWINT)| // Enable TWI Interupt and clear the flag to send byte
               _BV(TWEA);            // Send ACK after next reception
    } else {
        TWCR = _BV(TWEN)|            // Enable TWI-interface and release TWI pins.
               _BV(TWIE)|_BV(TWINT); // Enable TWI Interupt and clear the flag to send byte
        // Send NACK after next reception
    }
}

/**
 * Call this function to clear a bus error condition. Do NOT wait.
 * ---------------------------------------------------------------------------------------------- */
static void TWI_Stop( void ) {
    TWCR = _BV(TWEN)|            // Enable TWI-interface and release TWI pins
           _BV(TWIE)|_BV(TWINT)| // Enable TWI Interupt
           _BV(TWEA)|_BV(TWSTO); // Send ACK after next reception, stop bus
}

/**
 * Call this function to set up the TWI slave.
 * Remember to enable interrupts from the main application after initializing the TWI.
 * ---------------------------------------------------------------------------------------------- */
void TWI_Slave_Initialise( unsigned char TWI_ownAddress ) {
    TWAR = TWI_ownAddress << 1; // Set own TWI slave address. Reject TWI General Calls.

    TWDR = 0xFF;                // Default content = SDA released.
    LOW(TWSR, TWPS0);
    LOW(TWSR, TWPS1);
    TWI_Start_Transceiver(1);
}

/**
 * This function is the Interrupt Service Routine (ISR), and called when the TWI interrupt is
 * triggered; that is whenever a TWI event has occurred. This function should not be called
 * directly from the main application.
 * ---------------------------------------------------------------------------------------------- */
ISR(TWI_vect) {
    static unsigned char TWI_bufPtr;

    switch (TWSR) {
    case TW_ST_SLA_ACK:          // Own SLA+R has been received; ACK has been returned
    case TW_ST_ARB_LOST_SLA_ACK: // Arbitration lost; ACK has been returned
        TWI_bufPtr = 0;
        if (TWI_Tx_Data_Callback) {
            // Solicit data for reply via callback
            TWI_msgSize = TWI_BUFFER_SIZE;
            TWI_Tx_Data_Callback(TWI_buf, &TWI_msgSize);
        } else {
            TWI_msgSize = 0;
        }
        if (TWI_msgSize == 0) {
            TWI_msgSize = 1;
            TWI_buf[0] = 0x00;
        }

    // Intentional fall-through

    case TW_ST_DATA_ACK: // Data byte in TWDR has been transmitted; ACK has been received
        // TODO: TWI_msgSize == 0 — send immediate NACK ok?
        TWDR = TWI_buf[TWI_bufPtr++];
        TWI_Start_Transceiver(TWI_bufPtr < TWI_msgSize);
        break;

    case TW_ST_DATA_NACK:       // Data byte in TWDR has been transmitted; NOT ACK has been returned
    case TW_ST_LAST_DATA:       // Last data byte in TWDR has been transmitted (TWEA = �0�); ACK has been returned
        TWI_Start_Transceiver(1);
        break;

    case TW_SR_GCALL_ACK:          // General call address has been received; ACK has been returned
    case TW_SR_SLA_ACK:            // Own SLA+W has been received; ACK has been returned
    case TW_SR_ARB_LOST_SLA_ACK:   // Arbitration lost; ACK has been returned
    case TW_SR_ARB_LOST_GCALL_ACK: // Arbitration lost; ACK has been returned
        TWI_bufPtr = 0;
        TWI_Start_Transceiver(1);
        break;

    case TW_SR_DATA_ACK:       // Previously addressed with own SLA+W; data has been received; ACK has been returned
    case TW_SR_GCALL_DATA_ACK: // Previously addressed with general call; data has been received; ACK has been returned
        if (TWI_bufPtr < TWI_BUFFER_SIZE) {
            TWI_buf[TWI_bufPtr++] = TWDR;
            TWI_Start_Transceiver(1);
        } else {
            TWI_Start_Transceiver(0);
        }
        break;

    case TW_SR_STOP: // A STOP condition or repeated START condition has been received while still addressed as Slave
        // Clear the condition immediately, so we're less likely to miss
        // a START that immediately follows a STOP, and can still match
        // our address
        TWI_Start_Transceiver(1);
        // Either way, treat it as end of transmission
        if (TWI_Rx_Data_Callback) {
            TWI_Rx_Data_Callback(TWI_buf, TWI_bufPtr);
        }
        break;

    case TW_SR_DATA_NACK:       // Previously addressed with own SLA+W; data has been received; NOT ACK has been returned
    case TW_SR_GCALL_DATA_NACK: // Previously addressed with general call; data has been received; NOT ACK has been returned
        TWI_Start_Transceiver(0);
        break;

    case TW_NO_INFO:
        break;

    case TW_BUS_ERROR: // Bus error due to an illegal START or STOP condition
    default:
        TWI_Stop();
        break;
    }
}
