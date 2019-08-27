#!/bin/bash

# Get usb device whose board is executing our firmware.
line=`lsusb|grep "f123:4567"`

if [ "$line" == "" ]; then
  # Get usb device whose board is executing the STM32 bootloader.
  line=`lsusb|grep "0483:df11"`
fi;

if [ "$line" == "" ]; then
    >&2 echo "Can't find brushless or STM32 device!"
    exit 1 
fi

bus=`echo $line|cut -d' ' -f2|head -c 3`
dev=`echo $line|cut -d' ' -f4|head -c 3`
file="/dev/bus/usb/$bus/$dev"
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
(
  cd $DIR
  make
)
$DIR/usbreset $file
err=$?
case $err in
  0) ;;
  3) >&2 echo "We ignore ioctl error. This error is not important. :)";;
  *) exit 1;;
esac
