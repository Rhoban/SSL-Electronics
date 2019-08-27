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

display_error () {
  >&2 echo -n -e "\033[31m"
  >&2 echo -n $1
  >&2 echo -e "\033[0m"
}

display () {
  echo -n -e "\033[32m"
  echo -n $1
  echo -e "\033[0m"
}

check_sucess () {
  if [ ! $1 -eq 0 ]
  then
    display_error "Failed to flash USB."
    exit 1
  fi
}


display "We re-enumearte the USB port"
(
  cd tools/usb/
  bash ./re_enumerate_usb.sh
)
check_sucess $?

sleep 1

display "We flash the board"
(
  line=$(dfu-util -l |grep -e "0483:df11" |grep -e "Internal Flash" |grep -e "/0x08000000/")
  if [ "${line}" == "" ]
  then
    display_error "The board is not in bootloader mode or is not a STM32 device."
    exit 1
  fi
  alt=$(echo "$line" |grep -o -e ", alt=[0-9]*, " |sed -e "s/, alt=\([0-9]*\), /\1/")
  expected_altsetting_for_intenal_flash=0
  if [ ! "$alt" == "${expected_altsetting_for_intenal_flash}" ]
  then
    display_error "The Altsetting of the DFU Interface is not ${expected_altsetting_for_intenal_flash}."
    display_error "We obtain ${alt} For that device."
    display_error ""
    display_error "this correspond to DFU line "
    display_error "$line"
    display_error ""
    display_error " (To list all dfu devices you can write 'dfu-util -l' in a command line)."
    display_error ""
    display_error "Any error can DEFINITIVELY DESTROY the device corresponding to tha Altsetting !!!!"
    read -p "Do you want to flash the device of that DFU Altsetting ? yes/no " answer
    if [ ! "$answer" == "yes" ]
    then
      display_error "We abort the flash process ."
      exit 1000
    fi
  fi
  dfu-util -s 0x08000000 -a $alt -d 0483:df11 -D build/brushless.bin
)
check_sucess $?

display "Flash is a success !"
