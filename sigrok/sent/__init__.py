##
## OpenSENT - SAE J2716 SENT protocol decoder for libsigrokdecode / PulseView.
##
## SPDX-License-Identifier: MIT
##

'''
SENT (Single Edge Nibble Transmission, SAE J2716) is a unidirectional
automotive sensor protocol (throttle position, pedal, pressure, temperature,
mass air flow, ...). The information is in the time between falling edges:
a 56 tick sync/calibration pulse, a status nibble, 1..6 data nibbles and a
4-bit CRC, each nibble lasting 12 + value ticks, optionally followed by a
pause pulse. Bits 2 and 3 of the status nibble carry slow "serial"
messages (short 4/8-bit or enhanced 8/12- and 4/16-bit with CRC6).

This decoder measures every nibble against its own sync pulse, learns the
number of data nibbles and the CRC flavour (J2716-2010, legacy, status
included) and decodes short and enhanced serial messages.
'''

from .pd import Decoder
