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

default_device=/dev/ttyACM0

usage (){
  echo "Usage: $0 [-d=DEVICE]"
  echo "  -d, --device : The USB device (default: ${default_device})"
}

DEVICE=${default_device}

for i in "$@"
do
case $i in
    -d=*|--device=*)
    DEVICE="${i#*=}"
    shift # past argument=value
    ;;
    *)
    usage
    exit 1
    ;;
esac
done

check_sucess () {
  if [ ! $1 -eq 0 ]
  then
    >&2 echo -n -e "\033[31m"
    >&2 echo "Failed to flash USB."
    >&2 echo -e "\033[0m"
    exit 1
  fi
}

display () {
  echo -n -e "\033[32m"
  echo -n $1
  echo -e "\033[0m"
}

display "We execute the bootloader of the board"
(
  cd tools/board_command 
  python ./execute_bootloader.py --device ${DEVICE}
)
check_sucess $?

sleep 1

bash ./first_flash_with_usb_and_reset_button.sh 
