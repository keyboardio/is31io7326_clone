#!/usr/bin/env python2.7

import os
import os.path
import sys
from collections import namedtuple

Line = namedtuple('Line', ['size', 'offset', 'kind', 'data', 'checksum'])

USER_FIRMWARE = 'firmware/main.hex'
BOOTLOADER_FIRMWARE = 'firmware/bootloader.hex'

def parse_line(line):
    """Parses an Intel HEX line into a Line tuple"""
    if len(line) < 11:
        exit("invalid Intel HEX line: %s" % line)
    if line[0] != ':':
        exit("invalid Intel HEX line: %s" % line)
    line = line[1:]
    try:
        int(line, 16)
    except ValueError:
        exit("invalid Intel HEX line: %s" % line)
    line = line.decode('hex')
    return Line(line[0], line[1:3], line[3], line[4:-1], line[-1])

def make_hex_line(line):
    return ':' + ''.join(line).encode('hex')

def make_hex(lines):
    return '\n'.join(make_hex_line(l) for l in lines)


def recompute_checksum(line):
    checksum = chr((-(sum([ord(x) for x in ''.join(line)]) & 0xff)) & 0xff)
    return Line(*(line[:-1] + (checksum,)))


if not os.path.exists(USER_FIRMWARE):
    exit("User firmware %s must be built" % USER_FIRMWARE)
if not os.path.exists(BOOTLOADER_FIRMWARE):
    exit("Please copy %s to etc directory" % BOOTLOADER_FIRMWARE)

with open(USER_FIRMWARE) as user_in:
    user_hex = user_in.read()
with open(BOOTLOADER_FIRMWARE) as boot_in:
    boot_hex = boot_in.read()

user_lines = user_hex.strip().split()
if user_lines[-1] != ':00000001FF':
    exit("Invalid user firmware")

user_lines = user_lines[:-1]

boot_lines = boot_hex.strip().split()
if boot_lines[-1] != ':00000001FF':
    exit("Invalid bootloader firmware")
boot_lines = boot_lines[:-1]

user_lines = [parse_line(l) for l in user_lines]
boot_lines = [parse_line(l) for l in boot_lines]

# if a 4-byte 0x0000 line is present in the user firmware, remove it, and make sure it points to the right place

def replace_zero_vector(lines):
    """Replace the zero vector with a jump to the bootloader, or append a new one"""
    zero_lines = [(i, l) for i, l in enumerate(lines) if l.offset == '\0\0' and l.kind == '\0']
    if not zero_lines:
        lines.append(recompute_checksum(Line(
            size='\x02',
            offset='\x00\x00',
            kind='\x00',
            # point to the bootloader at 0x1C00
            data='\xff\xcd',
            checksum='\x00',
        )))
        return lines
    index = zero_lines[0][0]
    if len(zero_lines) > 1:
        exit("Too many initial vector sections")
    zero_line = zero_lines[0][1]
    if ord(zero_line.size) < 2:
        exit("Too small of an initial vector line: %d" % len(zero_line.size))
    #if zero_line.data[:2] != '\x1b\xc0':
    #    exit("Unexpected jump in the initial vector line (expected 0x13 0xc0)")
    # replace it
    zero_line = recompute_checksum(Line(
        size=zero_line.size,
        offset='\x00\x00',
        kind=zero_line.kind,
        # point to the bootloader at 0x1C00
        data='\xff\xcd' + zero_line.data[2:],
        checksum='\x00',
    ))
    lines[index] = zero_line
    return lines

def delete_all_vectors(lines):
    """Delete the zero vector, if present"""
    remove = set()
    for i, l in enumerate(lines):
        offset = (ord(l.offset[0]) << 8) + ord(l.offset[1])
        if offset < 0x38:
            remove.add(i)
    return [l for i, l in enumerate(lines) if i not in remove]


user_lines= replace_zero_vector(user_lines)
boot_lines = delete_all_vectors(boot_lines)
new_lines = [
    # end record
    Line(size='\0', offset='\0\0', kind='\x01', data='', checksum='\xFF'),
]

sys.stdout.write(make_hex(user_lines + boot_lines + new_lines))
sys.stdout.write('\n')
