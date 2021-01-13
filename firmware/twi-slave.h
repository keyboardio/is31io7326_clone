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
#include "led-api.h"

#pragma once

/****************************************************************************
  TWI Status/Control register definitions
****************************************************************************/

#define TWI_BUFFER_SIZE 128      // Reserves memory for the drivers transceiver buffer
// 32 is the same as arduino's TX buffer for
//
// TWI

/****************************************************************************
  Callback definitions
****************************************************************************/

// Called to solicit data for transmission
extern void (*TWI_Tx_Data_Callback)( unsigned char *, unsigned char * );

// Called to provide received data
extern void (*TWI_Rx_Data_Callback)( unsigned char *, unsigned char );

/****************************************************************************
  Function definitions
****************************************************************************/

void TWI_Slave_Initialise( unsigned char );
