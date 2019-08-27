#!/usr/bin/python

# Copyright (C) 2019  Adrien Boussicault <adrien.boussicault@labri.fr>
#
# This program is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either
# version 3 of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License along with this program.  If not, see
# <http://www.gnu.org/licenses/>.

import argparse

import serial
import sys
import time

parser = argparse.ArgumentParser(
    description='Execute the bootloader of a board associated with the device `DEVICE` .',
    formatter_class=argparse.ArgumentDefaultsHelpFormatter
)
parser.add_argument('--device', default='/dev/ttyACM0', help='The USB device')

args = parser.parse_args()

device = args.device

error = 0;
try:
    ser = serial.Serial(device, baudrate=115200, xonxoff=1)
    try:
        ser.write("\n".encode("ascii"))
        ser.flush()
        ser.write("jump_to_bootloader\n".encode("ascii"))
        ser.flush()
        time.sleep(1)
    except Exception as e:
        print('Impossible to send data in %s.' % device)
        error = 1
    finally:
        ser.close()
except Exception as e:
    print('Failed to open serial port %s for reset' % device)
    error = 2
sys.exit(error)
