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

usage (){
  echo "Usage: $0 LOG_PATH"
}

if [ "$#" -ne 1 ]; then
  usage
  exit 1
fi

LOG=$1

(
  cd tools/logs/convert_binaries
  make > /dev/null
  ./convert_binary -test
)
if [ ! $? -eq 0 ]
then
  >&2 echo -n -e "\033[31m"
  >&2 echo "Fail to decode due to endianess problem. Adapt the code of concert_binary."
  >&2 echo -e "\033[0m"
  exit 1
fi

./tools/logs/convert_binaries/convert_binary $LOG
