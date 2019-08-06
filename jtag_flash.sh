#!/bin/bash

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

JLinkExe -device STM32F401RB -if JTAG -speed 4000 -jtagconf -1,-1 -CommanderScript tools/jlink_files/flash_board.jlink
