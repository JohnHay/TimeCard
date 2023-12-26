#!/bin/sh

kenv hint.spigen.0.at="spibus0"
kenv hint.spigen.0.clock="1000000"
kenv hint.spigen.0.cs="0"
kenv hint.spigen.0.mode="0"

# Read the device ID
# spi -f /dev/spigen0.0 -d r -c 3 -A -C 9f
# spi -f /dev/spigen0.0 -d r -c 20 -A -C 9e
#
# Last byte is a dummy byte
# Read data from the start
# spi -f /dev/spigen0.0 -d r -c 128 -A -C "03 00 00 00 00"
#
# Read the normal image at 0x400000
# spi -f /dev/spigen0.0 -d r -c 256 -A -C "03 40 00 00 00"
#
# jot -s " " -w "%03X" 1024 0 > tests/tstimg.txt
