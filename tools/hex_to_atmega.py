#!/usr/bin/env python2.7

from intelhex import IntelHex
import sys

page_size = 64
frame_size = 16
memory_size = 8192
blank = 0xff
delay_ms = 1

template = open('etc/flash_firmware_h.template', 'r').read()

ih = IntelHex(sys.argv[1])

data = bytearray()
offsets = []
for i in range(ih.minaddr(), ih.maxaddr(), page_size):
    # Accumulate data bytes in page_size chunks, but only for addresses
    # that are present in contiguous segments.
    for (start, stop) in ih.segments():
        if i not in range(start, stop):
            continue
        offsets.append(i)
        # IntelHex will automatically fill missing bytes with 0xFF
        data += ih.tobinarray(start=i, size=page_size)

# Write page offsets, 8 per line
offsets_text = '\n'
for i in range(0, len(offsets), 8):
    line = ", ".join("0x%04x" % x for x in offsets[i:i+8])
    offsets_text += "    %s,\n" % line

# Write byte values, 8 per line
data_text = '\n'
for i in range(0, len(data), 8):
    line = ", ".join("0x%02x" %x for x in data[i:i+8])
    data_text += "    %s,\n" % line

print(template % (page_size, frame_size, blank, len(offsets), len(data), delay_ms, offsets_text, data_text))
