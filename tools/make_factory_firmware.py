#!/usr/bin/env python3

import sys
from intelhex import IntelHex

USER_FIRMWARE = 'firmware/main.hex'
BOOTLOADER_FIRMWARE = 'firmware/bootloader.hex'

try:
    user_hex = IntelHex(USER_FIRMWARE)
except FileNotFoundError:
    exit("User firmware %s must be built" % USER_FIRMWARE)
try:
    boot_hex = IntelHex(BOOTLOADER_FIRMWARE)
except FileNotFoundError:
    exit("Please copy %s to etc directory" % BOOTLOADER_FIRMWARE)

# Overwrite the reset vector to jump into the bootloader.
# This is an RJMP instruction to byte address 0x1c00.
user_hex[0:2] = [ 0xff, 0xcd ]
# Merge hex files, preserving the application interrupt vectors
user_hex.merge(boot_hex, overlap='ignore')
# Omit start address for consistency with older versions of this tool
user_hex.write_hex_file(sys.stdout, write_start_addr=False)
