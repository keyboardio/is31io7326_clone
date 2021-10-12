#pragma once

#include <stdint.h>
#include "wire-protocol-constants.h"
#define TWI_BASE_ADDRESS     0x58


#define DEVICE_VERSION 4


// I²C driver functions
void twi_data_received( uint8_t *buf, uint8_t bufsiz);
void twi_data_requested( uint8_t *buf, uint8_t *bufsiz);

void twi_init(void);
